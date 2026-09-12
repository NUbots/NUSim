#ifndef K1SIM_SHARED_K1_BOOSTERAPI_HPP
#define K1SIM_SHARED_K1_BOOSTERAPI_HPP

// Booster SDK wire-protocol constants the sim must speak. Values verified against
// the SDK commit NUbots_K1 pins (booster_robotics_sdk @ 324946e7): b1_api_const.hpp,
// b1_loco_api.hpp, common/robot_shared.hpp. Do not change without re-verifying there.

namespace k1sim::booster {

// DDS topics ("rt/" prefix = ROS2 rmw naming, so ROS2 clients interop directly)
inline constexpr const char* TOPIC_LOW_STATE      = "rt/low_state";
inline constexpr const char* TOPIC_JOINT_CTRL     = "rt/joint_ctrl";
inline constexpr const char* TOPIC_ODOMETER_STATE = "rt/odometer_state";
inline constexpr const char* TOPIC_FALL_DOWN      = "rt/fall_down";
inline constexpr const char* TOPIC_BATTERY_STATE  = "rt/battery_state";
inline constexpr const char* TOPIC_BUTTON_EVENT   = "rt/button_event";
inline constexpr const char* TOPIC_RPC_REQUEST    = "rt/LocoApiTopicReq";
inline constexpr const char* TOPIC_RPC_RESPONSE   = "rt/LocoApiTopicResp";
// The controller's full ROS odometry (nav_msgs Odometry: pose and twist), kTopicRosOdometer. Only
// in the SDK from the 1.7.0 firmware (booster_robotics_sdk @ d5d8f7ae).
inline constexpr const char* TOPIC_ROS_ODOMETER   = "rt/odom";
// Not an SDK constant: the robot's head pose (geometry_msgs Pose), the topic NUbots'
// K1Sensors subscribes to (K1Sensors.yaml head_pose.topic).
inline constexpr const char* TOPIC_HEAD_POSE      = "rt/head_pose";

// LocoApiId values carried in RpcReqMsg.header JSON {"api_id": <int>}
enum ApiId : int {
    CHANGE_MODE               = 2000,  // body {"mode": <int>}
    MOVE                      = 2001,  // body {"vx":,"vy":,"vyaw":}
    ROTATE_HEAD               = 2004,  // body {"pitch":,"yaw":}
    WAVE_HAND                 = 2005,
    ROTATE_HEAD_WITH_DIRECTION = 2006,
    LIE_DOWN                  = 2007,  // empty body
    GET_UP                    = 2008,  // empty body
    GET_MODE                  = 2017,  // response body {"mode": <int>}
    GET_UP_WITH_MODE          = 2025,  // body {"mode": <int>} — what NUbots' BoosterGetUp uses
    VISUAL_KICK               = 2038,  // what NUbots' BoosterVisualKick uses
};

// booster::robot::RobotMode
enum RobotMode : int {
    UNKNOWN = -1,
    DAMPING = 0,  // motors damp, robot collapses if unsupported
    PREPARE = 1,  // stand on both feet
    WALKING = 2,  // velocity-command locomotion
    CUSTOM  = 3,  // low-level rt/joint_ctrl control
    SOCCER  = 4,  // soccer locomotion — NUbots enters this immediately at startup
};

// RobotMode name for the viewers' status text
inline const char* mode_name(int mode) {
    switch (mode) {
        case DAMPING: return "DAMPING";
        case PREPARE: return "PREPARE";
        case WALKING: return "WALKING";
        case CUSTOM: return "CUSTOM";
        case SOCCER: return "SOCCER";
        default: return "?";
    }
}

// FallDownState.fall_down_state (matches NUbots' BoosterFallDownState enum)
enum FallState : int {
    IS_READY      = 0,
    IS_FALLING    = 1,
    HAS_FALLEN    = 2,
    IS_GETTING_UP = 3,
};

}  // namespace k1sim::booster

#endif  // K1SIM_SHARED_K1_BOOSTERAPI_HPP
