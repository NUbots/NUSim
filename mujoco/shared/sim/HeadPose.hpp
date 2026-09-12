#ifndef K1SIM_SHARED_SIM_HEADPOSE_HPP
#define K1SIM_SHARED_SIM_HEADPOSE_HPP

#include <array>
#include <cmath>
#include <mujoco/mujoco.h>

namespace k1sim {

// Head pose in the yaw-only base footprint frame, the frame the real robot's rt/head_pose
// is expressed in. NUbots' K1Sensors composes it with its yaw-only odometry (Hwr) to
// recover the true world pose, including torso tilt when fallen.
struct FootprintPose {
    std::array<double, 3> position{};
    std::array<double, 4> quat{1, 0, 0, 0};  // w,x,y,z
};

// The real robot's head frame sits this far above the Head_pitch joint (the Head_2 body
// origin) along the head's z-axis, near the head's centre of mass (Head_2 inertial z 0.0805).
// NUbots' K1Sensors removes it again with Hhp (translation [0, 0, -0.08], tuned on the robot),
// so publishing the bare Head_2 origin would put NUbots' torso this much too low.
inline constexpr double HEAD_FRAME_ABOVE_PITCH = 0.08;

// Hrh = (translate(base_x, base_y, 0) * rotz(base_yaw))^-1 * Hwh * translate(0, 0, HEAD_FRAME_ABOVE_PITCH),
// from the Head_2 body's world pose (xpos/xquat) and the root free joint's qpos (xy + wxyz quat).
inline FootprintPose head_in_footprint(const mjtNum head_p[3],
                                       const mjtNum head_q[4],
                                       const mjtNum base_xy[2],
                                       const mjtNum base_q[4]) {
    const mjtNum yaw = std::atan2(2.0 * (base_q[0] * base_q[3] + base_q[1] * base_q[2]),
                                  1.0 - 2.0 * (base_q[2] * base_q[2] + base_q[3] * base_q[3]));
    const mjtNum axis[3]{0, 0, 1};
    mjtNum neg_yaw_q[4];
    mju_axisAngle2Quat(neg_yaw_q, axis, -yaw);
    const mjtNum rel_w[3]{head_p[0] - base_xy[0], head_p[1] - base_xy[1], head_p[2]};
    mjtNum rel_p[3];
    mju_rotVecQuat(rel_p, rel_w, neg_yaw_q);
    mjtNum rel_q[4];
    mju_mulQuat(rel_q, neg_yaw_q, head_q);

    const mjtNum up_h[3]{0, 0, HEAD_FRAME_ABOVE_PITCH};
    mjtNum up_r[3];
    mju_rotVecQuat(up_r, up_h, rel_q);

    FootprintPose pose;
    pose.position = {rel_p[0] + up_r[0], rel_p[1] + up_r[1], rel_p[2] + up_r[2]};
    pose.quat     = {rel_q[0], rel_q[1], rel_q[2], rel_q[3]};
    return pose;
}

}  // namespace k1sim

#endif  // K1SIM_SHARED_SIM_HEADPOSE_HPP
