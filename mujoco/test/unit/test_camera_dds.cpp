// A full-resolution frame is ~0.9 MB, far over one UDP datagram, so this checks it actually survives
// fragmentation on both transports and arrives whole, with a CameraInfo NUbots' K1Camera can use.
#include <chrono>
#include <cstdio>
#include <fastdds/dds/subscriber/SampleInfo.hpp>
#include <fastrtps/attributes/LibrarySettingsAttributes.h>
#include <fastrtps/xmlparser/XMLProfileManager.h>
#include <thread>

#include "CameraInfo.h"
#include "CameraInfoPubSubTypes.h"
#include "Image.h"
#include "ImagePubSubTypes.h"
#include "module/SdkBridge/src/CameraPublisher.hpp"
#include "module/SdkBridge/src/DdsParticipant.hpp"

int main() {
    using k1sim::module::sdkbridge::CameraPublisher;
    using k1sim::module::sdkbridge::DdsParticipant;
    using sensor_msgs::msg::dds_::CameraInfo_;
    using sensor_msgs::msg::dds_::CameraInfo_PubSubType;
    using sensor_msgs::msg::dds_::Image_;
    using sensor_msgs::msg::dds_::Image_PubSubType;
    using namespace eprosima::fastdds::dds;

    // Exercise transports rather than the same-process delivery shortcut.
    eprosima::fastrtps::LibrarySettingsAttributes settings;
    settings.intraprocess_delivery = eprosima::fastrtps::INTRAPROCESS_OFF;
    eprosima::fastrtps::xmlparser::XMLProfileManager::library_settings(settings);

    k1sim::message::CameraFrame frame;
    frame.stamp  = std::chrono::system_clock::now();
    frame.width  = 640;
    frame.height = 480;
    frame.fx     = 415.7;
    frame.fy     = 415.7;
    frame.cx     = 320.0;
    frame.cy     = 240.0;
    frame.rgb.resize(std::size_t(frame.width) * frame.height * 3);
    for (std::size_t i = 0; i < frame.rgb.size(); ++i) {
        frame.rgb[i] = static_cast<uint8_t>(i * 7 + i / 4096);
    }

    const std::string topic = "k1sim_test/camera";
    for (bool udp_only : {true, false}) {
        const char* transport = udp_only ? "UDP" : "UDP+SHM";
        // Keep test traffic away from the simulator's domain 0 and real robots.
        DdsParticipant publisher(udp_only ? 83 : 84, udp_only);
        DdsParticipant subscriber(udp_only ? 83 : 84, udp_only);
        CameraPublisher camera(publisher, topic);

        // K1Camera's QoS: best-effort images, reliable CameraInfo
        DataReaderQos image_qos      = DATAREADER_QOS_DEFAULT;
        image_qos.reliability().kind = BEST_EFFORT_RELIABILITY_QOS;
        auto* image_reader           = subscriber.create_reader<Image_PubSubType>(topic, image_qos, nullptr);
        auto* info_reader            = subscriber.create_reader<CameraInfo_PubSubType>(topic + "/camera_info",
                                                                            DdsParticipant::rpc_request_reader_qos(),
                                                                            nullptr);
        if (image_reader == nullptr || info_reader == nullptr) {
            std::fprintf(stderr, "Could not create test DDS readers\n");
            return 1;
        }

        bool got_image      = false;
        bool got_info       = false;
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while (std::chrono::steady_clock::now() < deadline && !(got_image && got_info)) {
            camera.publish(frame);
            std::this_thread::sleep_for(std::chrono::milliseconds(33));

            Image_ image;
            SampleInfo info;
            while (image_reader->take_next_sample(&image, &info) == ReturnCode_t::RETCODE_OK) {
                if (!info.valid_data) {
                    continue;
                }
                if (image.width() != frame.width || image.height() != frame.height || image.encoding() != "rgb8"
                    || image.step() != frame.width * 3 || image.data() != frame.rgb) {
                    std::fprintf(stderr, "Image arrived corrupted (%s)\n", transport);
                    return 1;
                }
                got_image = true;
            }

            CameraInfo_ camera_info;
            while (info_reader->take_next_sample(&camera_info, &info) == ReturnCode_t::RETCODE_OK) {
                if (!info.valid_data) {
                    continue;
                }
                const auto& k = camera_info.k();
                if (camera_info.width() != frame.width || camera_info.height() != frame.height || k[0] != frame.fx
                    || k[2] != frame.cx || k[4] != frame.fy || k[5] != frame.cy || k[8] != 1.0
                    || camera_info.d().size() != 5 || camera_info.d()[0] != 0.0) {
                    std::fprintf(stderr, "CameraInfo arrived wrong (%s)\n", transport);
                    return 1;
                }
                if (camera_info.header().stamp().sec() == 0) {
                    std::fprintf(stderr, "CameraInfo has no timestamp (%s)\n", transport);
                    return 1;
                }
                got_info = true;
            }
        }
        if (!got_image || !got_info) {
            std::fprintf(stderr,
                         "Camera delivery timed out (%s): image %s, camera_info %s\n",
                         transport,
                         got_image ? "ok" : "missing",
                         got_info ? "ok" : "missing");
            return 1;
        }
        std::printf("640x480 rgb8 Image + CameraInfo delivered (%s)\n", transport);
    }
    return 0;
}
