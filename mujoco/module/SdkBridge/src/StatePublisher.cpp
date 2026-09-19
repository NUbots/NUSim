#include "module/SdkBridge/src/StatePublisher.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>

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
#include "Odometry.h"
#include "OdometryPubSubTypes.h"
#include "Pose.h"
#include "PosePubSubTypes.h"
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

        // A world-frame vector expressed in the body frame of the {w, x, y, z} orientation q: R(q)^T v.
        std::array<double, 3> world_to_body(const std::array<double, 4>& q, const std::array<double, 3>& v) {
            const double w = q[0], x = q[1], y = q[2], z = q[3];
            // Rows of R^T are the columns of R
            const double r[3][3] = {{1 - 2 * (y * y + z * z), 2 * (x * y + w * z), 2 * (x * z - w * y)},
                                    {2 * (x * y - w * z), 1 - 2 * (x * x + z * z), 2 * (y * z + w * x)},
                                    {2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)}};
            return {r[0][0] * v[0] + r[0][1] * v[1] + r[0][2] * v[2],
                    r[1][0] * v[0] + r[1][1] * v[1] + r[1][2] * v[2],
                    r[2][0] * v[0] + r[2][1] * v[1] + r[2][2] * v[2]};
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
        using geometry_msgs::msg::dds_::Pose_PubSubType;
        using nav_msgs::msg::dds_::Odometry_PubSubType;

        low_state_writer_ =
            dds.create_writer<LowState_PubSubType>(k1sim::booster::TOPIC_LOW_STATE, DdsParticipant::state_writer_qos());
        odometer_writer_     = dds.create_writer<Odometer_PubSubType>(k1sim::booster::TOPIC_ODOMETER_STATE,
                                                                  DdsParticipant::state_writer_qos());
        ros_odometry_writer_ = dds.create_writer<Odometry_PubSubType>(k1sim::booster::TOPIC_ROS_ODOMETER,
                                                                      DdsParticipant::state_writer_qos());
        head_pose_writer_ =
            dds.create_writer<Pose_PubSubType>(k1sim::booster::TOPIC_HEAD_POSE, DdsParticipant::state_writer_qos());
        fall_down_writer_ = dds.create_writer<FallDownState_PubSubType>(k1sim::booster::TOPIC_FALL_DOWN,
                                                                        DdsParticipant::state_writer_qos());
        battery_writer_   = dds.create_writer<BatteryState_PubSubType>(k1sim::booster::TOPIC_BATTERY_STATE,
                                                                     DdsParticipant::state_writer_qos());
        // Created so the topic/type exist on the wire (a real subscriber could match),
        // but per the M4/M5 spec we never actually write to it — the sim has no buttons.
        button_event_writer_ = dds.create_writer<ButtonEventMsg_PubSubType>(k1sim::booster::TOPIC_BUTTON_EVENT,
                                                                            DdsParticipant::state_writer_qos());
    }

    void StatePublisher::publish(const k1sim::message::SimStateUpdate& update) {
        // --- rt/low_state ---
        booster_interface::msg::dds_::LowState_ low_state;
        low_state.imu_state().rpy({static_cast<float>(update.imu.rpy[0]),
                                   static_cast<float>(update.imu.rpy[1]),
                                   static_cast<float>(update.imu.rpy[2])});
        low_state.imu_state().gyro({static_cast<float>(update.imu.gyro[0]),
                                    static_cast<float>(update.imu.gyro[1]),
                                    static_cast<float>(update.imu.gyro[2])});
        low_state.imu_state().acc({static_cast<float>(update.imu.acc[0]),
                                   static_cast<float>(update.imu.acc[1]),
                                   static_cast<float>(update.imu.acc[2])});

        std::vector<booster_interface::msg::dds_::MotorState_> motors;
        motors.reserve(k1sim::JOINT_COUNT);
        for (std::size_t i = 0; i < k1sim::JOINT_COUNT; ++i) {
            motors.push_back(to_motor_state(update.joints[i]));
        }
        low_state.motor_state_serial(motors);
        // motor_state_parallel mirrors the real firmware's layout: the 12 leg motors only
        // (JointIndexK1 kLeftHipPitch..kCrankDownRight => vector indices 0..11), with the
        // crank slots carrying the serial-equivalent ankle pitch/roll values (the sim has
        // no true parallel actuation). Clients (e.g. NUbots HardwareIO) index this vector
        // as joint_index - kLeftHipPitch.
        std::vector<booster_interface::msg::dds_::MotorState_> legs(motors.begin() + JointIndexK1::LeftHipPitch,
                                                                    motors.end());
        low_state.motor_state_parallel(legs);
        low_state_writer_->write(&low_state);

        // --- rt/odometer_state ---
        booster_interface::msg::dds_::Odometer_ odom;
        odom.x(static_cast<float>(update.base.x));
        odom.y(static_cast<float>(update.base.y));
        odom.theta(static_cast<float>(yaw_from_quat(update.base.quat)));
        odometer_writer_->write(&odom);

        // --- rt/odom --- The same base odometry with its velocity: pose in "odom" (the world), twist
        // in the body frame "base_link", per the ROS nav_msgs convention (twist in child_frame_id).
        // The sim's base state is ground truth, so the covariances are left zero.
        nav_msgs::msg::dds_::Odometry_ ros_odom;
        const auto stamp = std::chrono::system_clock::now().time_since_epoch();
        const auto sec   = std::chrono::duration_cast<std::chrono::seconds>(stamp);
        ros_odom.header().stamp().sec(static_cast<int32_t>(sec.count()));
        ros_odom.header().stamp().nanosec(
            static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(stamp - sec).count()));
        ros_odom.header().frame_id("odom");
        ros_odom.child_frame_id("base_link");
        ros_odom.pose().pose().position().x(update.base.x);
        ros_odom.pose().pose().position().y(update.base.y);
        ros_odom.pose().pose().position().z(update.base.z);
        ros_odom.pose().pose().orientation().w(update.base.quat[0]);
        ros_odom.pose().pose().orientation().x(update.base.quat[1]);
        ros_odom.pose().pose().orientation().y(update.base.quat[2]);
        ros_odom.pose().pose().orientation().z(update.base.quat[3]);
        const auto linear  = world_to_body(update.base.quat, update.base.lin_vel);
        const auto angular = world_to_body(update.base.quat, update.base.ang_vel);
        ros_odom.twist().twist().linear().x(linear[0]);
        ros_odom.twist().twist().linear().y(linear[1]);
        ros_odom.twist().twist().linear().z(linear[2]);
        ros_odom.twist().twist().angular().x(angular[0]);
        ros_odom.twist().twist().angular().y(angular[1]);
        ros_odom.twist().twist().angular().z(angular[2]);
        ros_odometry_writer_->write(&ros_odom);

        // --- rt/head_pose --- The head frame in the yaw-only base footprint frame, which
        // K1Sensors composes with the odometry above to place the camera and torso in the world.
        if (update.head.valid) {
            geometry_msgs::msg::dds_::Pose_ pose;
            pose.position().x(update.head.position[0]);
            pose.position().y(update.head.position[1]);
            pose.position().z(update.head.position[2]);
            pose.orientation().w(update.head.quat[0]);
            pose.orientation().x(update.head.quat[1]);
            pose.orientation().y(update.head.quat[2]);
            pose.orientation().z(update.head.quat[3]);
            head_pose_writer_->write(&pose);
        }

        // --- rt/fall_down --- publish on change, or every >=1s as a keepalive.
        const bool changed       = !have_last_fall_state_ || last_fall_state_ != update.fall_state;
        const bool due_keepalive = (update.sim_time - last_fall_publish_time_) >= 1.0;
        if (changed || due_keepalive) {
            if (changed) {
                std::fprintf(stderr, "StatePublisher: fall_state -> %d (t=%.2f)\n", update.fall_state, update.sim_time);
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

        StatePublisher::StatePublisher(DdsParticipant & dds, double battery_soc) : battery_soc_(battery_soc) {
            using booster_interface::msg::dds_::BatteryState_PubSubType;
            using booster_interface::msg::dds_::ButtonEventMsg_PubSubType;
            using booster_interface::msg::dds_::FallDownState_PubSubType;
            using booster_interface::msg::dds_::LowState_PubSubType;
            using booster_interface::msg::dds_::Odometer_PubSubType;
            using geometry_msgs::msg::dds_::Pose_PubSubType;

            low_state_writer_ = dds.create_writer<LowState_PubSubType>(k1sim::booster::TOPIC_LOW_STATE,
                                                                       DdsParticipant::state_writer_qos());
            odometer_writer_  = dds.create_writer<Odometer_PubSubType>(k1sim::booster::TOPIC_ODOMETER_STATE,
                                                                      DdsParticipant::state_writer_qos());
            head_pose_writer_ =
                dds.create_writer<Pose_PubSubType>(k1sim::booster::TOPIC_HEAD_POSE, DdsParticipant::state_writer_qos());
            fall_down_writer_ = dds.create_writer<FallDownState_PubSubType>(k1sim::booster::TOPIC_FALL_DOWN,
                                                                            DdsParticipant::state_writer_qos());
            battery_writer_   = dds.create_writer<BatteryState_PubSubType>(k1sim::booster::TOPIC_BATTERY_STATE,
                                                                         DdsParticipant::state_writer_qos());
            // Created so the topic/type exist on the wire (a real subscriber could match),
            // but per the M4/M5 spec we never actually write to it — the sim has no buttons.
            button_event_writer_ = dds.create_writer<ButtonEventMsg_PubSubType>(k1sim::booster::TOPIC_BUTTON_EVENT,
                                                                                DdsParticipant::state_writer_qos());
        }

        void StatePublisher::publish(const k1sim::message::SimStateUpdate& update) {
            // --- rt/low_state ---
            booster_interface::msg::dds_::LowState_ low_state;
            low_state.imu_state().rpy({static_cast<float>(update.imu.rpy[0]),
                                       static_cast<float>(update.imu.rpy[1]),
                                       static_cast<float>(update.imu.rpy[2])});
            low_state.imu_state().gyro({static_cast<float>(update.imu.gyro[0]),
                                        static_cast<float>(update.imu.gyro[1]),
                                        static_cast<float>(update.imu.gyro[2])});
            low_state.imu_state().acc({static_cast<float>(update.imu.acc[0]),
                                       static_cast<float>(update.imu.acc[1]),
                                       static_cast<float>(update.imu.acc[2])});

            std::vector<booster_interface::msg::dds_::MotorState_> motors;
            motors.reserve(k1sim::JOINT_COUNT);
            for (std::size_t i = 0; i < k1sim::JOINT_COUNT; ++i) {
                motors.push_back(to_motor_state(update.joints[i]));
            }
            low_state.motor_state_serial(motors);
            // motor_state_parallel mirrors the real firmware's layout: the 12 leg motors only
            // (JointIndexK1 kLeftHipPitch..kCrankDownRight => vector indices 0..11), with the
            // crank slots carrying the serial-equivalent ankle pitch/roll values (the sim has
            // no true parallel actuation). Clients (e.g. NUbots HardwareIO) index this vector
            // as joint_index - kLeftHipPitch.
            std::vector<booster_interface::msg::dds_::MotorState_> legs(motors.begin() + JointIndexK1::LeftHipPitch,
                                                                        motors.end());
            low_state.motor_state_parallel(legs);
            low_state_writer_->write(&low_state);

            // --- rt/odometer_state ---
            booster_interface::msg::dds_::Odometer_ odom;
            odom.x(static_cast<float>(update.base.x));
            odom.y(static_cast<float>(update.base.y));
            odom.theta(static_cast<float>(yaw_from_quat(update.base.quat)));
            odometer_writer_->write(&odom);

            // --- rt/head_pose --- The head frame in the yaw-only base footprint frame, which
            // K1Sensors composes with the odometry above to place the camera and torso in the world.
            if (update.head.valid) {
                geometry_msgs::msg::dds_::Pose_ pose;
                pose.position().x(update.head.position[0]);
                pose.position().y(update.head.position[1]);
                pose.position().z(update.head.position[2]);
                pose.orientation().w(update.head.quat[0]);
                pose.orientation().x(update.head.quat[1]);
                pose.orientation().y(update.head.quat[2]);
                pose.orientation().z(update.head.quat[3]);
                head_pose_writer_->write(&pose);
            }

            // --- rt/fall_down --- publish on change, or every >=1s as a keepalive.
            const bool changed       = !have_last_fall_state_ || last_fall_state_ != update.fall_state;
            const bool due_keepalive = (update.sim_time - last_fall_publish_time_) >= 1.0;
            if (changed || due_keepalive) {
                if (changed) {
                    std::fprintf(stderr,
                                 "StatePublisher: fall_state -> %d (t=%.2f)\n",
                                 update.fall_state,
                                 update.sim_time);
                }
                booster_interface::msg::dds_::FallDownState_ fall;
                fall.fall_down_state(static_cast<booster_interface::msg::dds_::FallDownStateType_>(update.fall_state));
                fall.is_recovery_available(true);
                fall_down_writer_->write(&fall);

                have_last_fall_state_   = true;
                last_fall_state_        = update.fall_state;
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
