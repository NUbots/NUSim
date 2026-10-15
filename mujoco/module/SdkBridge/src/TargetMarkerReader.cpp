#include "module/SdkBridge/src/TargetMarkerReader.hpp"

#include <fastdds/dds/subscriber/SampleInfo.hpp>
#include <string>

#include "Odometry.h"
#include "OdometryPubSubTypes.h"
#include "shared/k1/NUSimApi.hpp"
#include "shared/message/Commands.hpp"
#include "shared/sim/PlanarPose.hpp"

namespace k1sim::module::sdkbridge {

    using eprosima::fastdds::dds::DataReader;
    using eprosima::fastdds::dds::SampleInfo;
    using eprosima::fastrtps::types::ReturnCode_t;

    TargetMarkerReader::TargetMarkerReader(DdsParticipant& dds, NUClear::Reactor& reactor) : reactor_(reactor) {
        // RELIABLE like the ball command, so a "hidden" sent once isn't lost
        reader_ = dds.create_reader<nav_msgs::msg::dds_::Odometry_PubSubType>(k1sim::nusim::TOPIC_TARGET_MARKER,
                                                                              DdsParticipant::rpc_request_reader_qos(5),
                                                                              this);
    }

    void TargetMarkerReader::on_data_available(DataReader* reader) {
        if (reader != reader_) {
            return;
        }
        nav_msgs::msg::dds_::Odometry_ msg;
        SampleInfo info;
        while (reader_->take_next_sample(&msg, &info) == ReturnCode_t::RETCODE_OK) {
            if (!info.valid_data) {
                continue;
            }
            auto marker = std::make_unique<k1sim::message::TargetMarker>();
            if (msg.child_frame_id() == "hidden") {
                marker->visible = false;
                reactor_.emit(std::move(marker));
                continue;
            }
            const std::string frame = msg.header().frame_id();
            if (frame != "world" && frame != "robot") {
                reactor_.log<NUClear::LogLevel::WARN>("SdkBridge: target marker with unknown frame_id '",
                                                      frame,
                                                      "' (expected world or robot), ignored");
                continue;
            }
            const auto& p = msg.pose().pose().position();
            const auto& q = msg.pose().pose().orientation();
            marker->frame = frame == "robot" ? k1sim::message::TargetMarker::Frame::ROBOT
                                             : k1sim::message::TargetMarker::Frame::WORLD;
            marker->x     = p.x();
            marker->y     = p.y();
            marker->yaw   = k1sim::yaw_of({q.w(), q.x(), q.y(), q.z()});
            reactor_.emit(std::move(marker));
        }
    }

}  // namespace k1sim::module::sdkbridge
