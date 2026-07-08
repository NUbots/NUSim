#include "module/SdkBridge/src/StatePublisher.hpp"

#include <cmath>

#include "BatteryState.h"
#include "BatteryStatePubSubTypes.h"
#include "ButtonEvent.h"
#include "ButtonEventPubSubTypes.h"
#include "FallDownState.h"
#include "FallDownStatePubSubTypes.h"
#include "LowState.h"
#include "LowStatePubSubTypes.h"
#include "Odometer.h"
#include "OdometerPubSubTypes.h"

#include "shared/k1/BoosterApi.hpp"
#include "shared/k1/JointIndex.hpp"

namespace k1sim::module::sdkbridge {

namespace {

// BaseState::quat is {w, x, y, z} (see shared/message/SimMessages.hpp). Planar yaw
// extraction for the Odometer's theta field.
double yaw_from_quat(const std::array<double, 4>& q) {
    const double w = q[0], x = q[1], y = q[2], z = q[3];
    return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

booster_interface::msg::dds_::MotorState_ to_motor_state(const k1sim::message::JointState& joint) {
    booster_interface::msg::dds_::MotorState_ m;
    m.mode(1);
    m.q(static_cast<float>(joint.q));
    m.dq(static_cast<float>(joint.dq));
    m.ddq(static_cast<float>(joint.ddq));
    m.tau_est(static_cast<float>(joint.tau));
    m.temperature(40);
    m.lost(0);
    m.reserve({0, 0});
    return m;
}

}  // namespace

StatePublisher::StatePublisher(DdsParticipant& dds, double battery_soc) : battery_soc_(battery_soc) {
    using booster_interface::msg::dds_::BatteryState_PubSubType;
    using booster_interface::msg::dds_::ButtonEventMsg_PubSubType;
    using booster_interface::msg::dds_::FallDownState_PubSubType;
    using booster_interface::msg::dds_::LowState_PubSubType;
    using booster_interface::msg::dds_::Odometer_PubSubType;

    low_state_writer_ =
        dds.create_writer<LowState_PubSubType>(k1sim::booster::TOPIC_LOW_STATE, DdsParticipant::state_writer_qos());
    odometer_writer_ = dds.create_writer<Odometer_PubSubType>(k1sim::booster::TOPIC_ODOMETER_STATE,
                                                               DdsParticipant::state_writer_qos());
    fall_down_writer_ =
        dds.create_writer<FallDownState_PubSubType>(k1sim::booster::TOPIC_FALL_DOWN, DdsParticipant::state_writer_qos());
    battery_writer_ = dds.create_writer<BatteryState_PubSubType>(k1sim::booster::TOPIC_BATTERY_STATE,
                                                                  DdsParticipant::state_writer_qos());
    // Created so the topic/type exist on the wire (a real subscriber could match),
    // but per the M4/M5 spec we never actually write to it — the sim has no buttons.
    button_event_writer_ = dds.create_writer<ButtonEventMsg_PubSubType>(k1sim::booster::TOPIC_BUTTON_EVENT,
                                                                         DdsParticipant::state_writer_qos());
}

void StatePublisher::publish(const k1sim::message::SimStateUpdate& update) {
    // --- rt/low_state ---
    booster_interface::msg::dds_::LowState_ low_state;
    low_state.imu_state().rpy({static_cast<float>(update.imu.rpy[0]), static_cast<float>(update.imu.rpy[1]),
                                static_cast<float>(update.imu.rpy[2])});
    low_state.imu_state().gyro({static_cast<float>(update.imu.gyro[0]), static_cast<float>(update.imu.gyro[1]),
                                 static_cast<float>(update.imu.gyro[2])});
    low_state.imu_state().acc({static_cast<float>(update.imu.acc[0]), static_cast<float>(update.imu.acc[1]),
                                static_cast<float>(update.imu.acc[2])});

    std::vector<booster_interface::msg::dds_::MotorState_> motors;
    motors.reserve(k1sim::JOINT_COUNT);
    for (std::size_t i = 0; i < k1sim::JOINT_COUNT; ++i) {
        motors.push_back(to_motor_state(update.joints[i]));
    }
    low_state.motor_state_serial(motors);
    low_state.motor_state_parallel(motors);  // K1 has no true parallel actuation — mirrored 1:1
    low_state_writer_->write(&low_state);

    // --- rt/odometer_state ---
    booster_interface::msg::dds_::Odometer_ odom;
    odom.x(static_cast<float>(update.base.x));
    odom.y(static_cast<float>(update.base.y));
    odom.theta(static_cast<float>(yaw_from_quat(update.base.quat)));
    odometer_writer_->write(&odom);

    // --- rt/fall_down --- publish on change, or every >=1s as a keepalive.
    const bool changed = !have_last_fall_state_ || last_fall_state_ != update.fall_state;
    const bool due_keepalive = (update.sim_time - last_fall_publish_time_) >= 1.0;
    if (changed || due_keepalive) {
        booster_interface::msg::dds_::FallDownState_ fall;
        fall.fall_down_state(
            static_cast<booster_interface::msg::dds_::FallDownStateType_>(update.fall_state));
        fall.is_recovery_available(true);
        fall_down_writer_->write(&fall);

        have_last_fall_state_  = true;
        last_fall_state_       = update.fall_state;
        last_fall_publish_time_ = update.sim_time;
    }
}

void StatePublisher::publish_battery() {
    booster_interface::msg::dds_::BatteryState_ battery;
    battery.voltage(0.0f);
    battery.current(0.0f);
    battery.soc(static_cast<float>(battery_soc_));
    battery.average_voltage(0.0f);
    battery_writer_->write(&battery);
}

}  // namespace k1sim::module::sdkbridge
