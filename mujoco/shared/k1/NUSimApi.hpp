#ifndef K1SIM_SHARED_K1_NUSIMAPI_HPP
#define K1SIM_SHARED_K1_NUSIMAPI_HPP

// NUSim-only DDS topics: simulator ground truth and test control. These are NOT part of the
// Booster SDK surface (see BoosterApi.hpp) and a real robot never publishes or listens on them.
// All of them carry nav_msgs/Odometry, which both NUSim and NUbots_K1 already generate, so no new
// IDL is needed on either side. Semantics are pinned in module/SdkBridge/PROTOCOL.md §6.

namespace k1sim::nusim {

    // Ground-truth ball state, published at the state cadence (50 Hz) while the scene has a ball.
    // pose.position = ball centre, world frame; twist.linear/angular = centre velocity and spin,
    // world frame; header.stamp = wall clock when the physics snapshot was taken.
    inline constexpr const char* TOPIC_GT_BALL = "rt/nusim/gt/ball";

    // Ground-truth robot base (Trunk) state, same cadence and conventions as TOPIC_GT_BALL
    // (unlike rt/odom, whose twist is body frame per the ROS convention).
    inline constexpr const char* TOPIC_GT_ROBOT = "rt/nusim/gt/robot";

    // Where the ball will cross, from rolling it ahead in a copy of the scene without the robot
    // (shared/sim/BallForecast.hpp), same cadence. header.stamp = wall clock when it crosses (the
    // snapshot's when it doesn't); child_frame_id = "crossing", or "none" if it doesn't cross within
    // BALL_FORECAST_HORIZON; pose.position/twist.linear = ball centre and its velocity as it crosses.
    // ..._ROBOT: the robot's frontal plane, crossed from in front, frame_id "robot" (the yaw-only frame
    // at the Trunk's ground projection at the snapshot, as for ball commands).
    // ..._GOAL: the first goal line the ball leaves the field over, frame_id "world".
    inline constexpr const char* TOPIC_GT_BALL_CROSSING_ROBOT = "rt/nusim/gt/ball_crossing/robot";
    inline constexpr const char* TOPIC_GT_BALL_CROSSING_GOAL  = "rt/nusim/gt/ball_crossing/goal";
    inline constexpr double BALL_FORECAST_HORIZON             = 5.0;  // s

    // Place the ball and set its velocity. header.frame_id selects the frame of pose/twist:
    // "world", or "robot" (yaw-only frame at the Trunk's ground projection, x forward, z up).
    // pose.position.z < 0 rests the ball on the floor at its radius. child_frame_id "rolling"
    // derives the spin for rolling without slipping from twist.linear and ignores twist.angular.
    inline constexpr const char* TOPIC_BALL_COMMAND = "rt/nusim/ball_command";

}  // namespace k1sim::nusim

#endif  // K1SIM_SHARED_K1_NUSIMAPI_HPP
