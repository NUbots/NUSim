#include "module/SdkBridge/src/CameraPublisher.hpp"

#include <chrono>

#include "CameraInfo.h"
#include "CameraInfoPubSubTypes.h"
#include "Image.h"
#include "ImagePubSubTypes.h"

namespace k1sim::module::sdkbridge {

    namespace {

        // Not a real TF frame (the sim has none); NUbots reads the camera's pose from rt/head_pose instead
        constexpr const char* kFrameId = "head_camera";

        std_msgs::msg::dds_::Header_ make_header(const std::chrono::system_clock::time_point& stamp) {
            const auto since_epoch = stamp.time_since_epoch();
            const auto sec         = std::chrono::duration_cast<std::chrono::seconds>(since_epoch);
            std_msgs::msg::dds_::Header_ header;
            header.stamp().sec(static_cast<int32_t>(sec.count()));
            header.stamp().nanosec(
                static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(since_epoch - sec).count()));
            header.frame_id(kFrameId);
            return header;
        }

    }  // namespace

    CameraPublisher::CameraPublisher(DdsParticipant& dds, const std::string& image_topic) {
        using sensor_msgs::msg::dds_::CameraInfo_PubSubType;
        using sensor_msgs::msg::dds_::Image_PubSubType;

        image_writer_ = dds.create_writer<Image_PubSubType>(image_topic, DdsParticipant::image_writer_qos());
        info_writer_ =
            dds.create_writer<CameraInfo_PubSubType>(image_topic + "/camera_info", DdsParticipant::state_writer_qos());
    }

    void CameraPublisher::publish(const k1sim::message::CameraFrame& frame) {
        const auto header = make_header(frame.stamp);

        sensor_msgs::msg::dds_::Image_ image;
        image.header(header);
        image.height(frame.height);
        image.width(frame.width);
        image.encoding("rgb8");
        image.is_bigendian(0);
        image.step(frame.width * 3);
        image.data(frame.rgb);
        image_writer_->write(&image);

        // An ideal pinhole: no distortion, no rectification, so P is K with a zero fourth column
        sensor_msgs::msg::dds_::CameraInfo_ info;
        info.header(header);
        info.height(frame.height);
        info.width(frame.width);
        info.distortion_model("plumb_bob");
        info.d({0.0, 0.0, 0.0, 0.0, 0.0});
        info.k({frame.fx, 0.0, frame.cx, 0.0, frame.fy, frame.cy, 0.0, 0.0, 1.0});
        info.r({1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});
        info.p({frame.fx, 0.0, frame.cx, 0.0, 0.0, frame.fy, frame.cy, 0.0, 0.0, 0.0, 1.0, 0.0});
        info_writer_->write(&info);
    }

}  // namespace k1sim::module::sdkbridge
