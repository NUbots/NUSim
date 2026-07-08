#ifndef K1SIM_MODULE_SDKBRIDGE_STATEPUBLISHER_HPP
#define K1SIM_MODULE_SDKBRIDGE_STATEPUBLISHER_HPP

#include <fastdds/dds/publisher/DataWriter.hpp>

#include "module/SdkBridge/src/DdsParticipant.hpp"
#include "shared/message/SimMessages.hpp"

// Builds and publishes the Booster SDK state topics from k1sim's internal
// SimStateUpdate. Field mapping verified in module/SdkBridge/PROTOCOL.md §1.

namespace k1sim::module::sdkbridge {

class StatePublisher {
public:
    // battery_soc comes from config/dds.yaml (constant, published at 1 Hz).
    StatePublisher(DdsParticipant& dds, double battery_soc);

    // Called at the LowState cadence (50 Hz, on<Trigger<SimStateUpdate>>): writes
    // rt/low_state and rt/odometer_state every call, and rt/fall_down whenever
    // fall_state changes or >=1s has elapsed since the last publish (keepalive).
    void publish(const k1sim::message::SimStateUpdate& update);

    // Called on<Every<1, std::chrono::seconds>>: writes the constant-SOC rt/battery_state.
    void publish_battery();

private:
    eprosima::fastdds::dds::DataWriter* low_state_writer_   = nullptr;
    eprosima::fastdds::dds::DataWriter* odometer_writer_    = nullptr;
    eprosima::fastdds::dds::DataWriter* fall_down_writer_   = nullptr;
    eprosima::fastdds::dds::DataWriter* battery_writer_     = nullptr;
    eprosima::fastdds::dds::DataWriter* button_event_writer_ = nullptr;  // created, never published

    double battery_soc_;

    bool have_last_fall_state_  = false;
    int last_fall_state_        = -1;
    double last_fall_publish_time_ = -1e9;
};

}  // namespace k1sim::module::sdkbridge

#endif  // K1SIM_MODULE_SDKBRIDGE_STATEPUBLISHER_HPP
