#ifndef K1SIM_MODULE_SDKBRIDGE_CAMERAPUBLISHER_HPP
#define K1SIM_MODULE_SDKBRIDGE_CAMERAPUBLISHER_HPP

#include <fastdds/dds/publisher/DataWriter.hpp>
#include <string>

#include "module/SdkBridge/src/DdsParticipant.hpp"
#include "shared/message/SimMessages.hpp"

// Publishes module::Camera's rendered frames the way the robot's camera driver does:
// sensor_msgs Image on the configured topic and its CameraInfo on `<topic>/camera_info`,
// the pair NUbots' input::K1Camera subscribes to. Layouts in module/SdkBridge/PROTOCOL.md §1.

namespace k1sim::module::sdkbridge {

    class CameraPublisher {
    public:
        CameraPublisher(DdsParticipant& dds, const std::string& image_topic);

        // Writes the frame as an rgb8 Image, then a CameraInfo with the same header, as a ROS
        // camera driver does once per frame.
        void publish(const k1sim::message::CameraFrame& frame);

    private:
        eprosima::fastdds::dds::DataWriter* image_writer_ = nullptr;
        eprosima::fastdds::dds::DataWriter* info_writer_  = nullptr;
    };

}  // namespace k1sim::module::sdkbridge

#endif  // K1SIM_MODULE_SDKBRIDGE_CAMERAPUBLISHER_HPP
