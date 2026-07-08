#ifndef K1SIM_SHARED_K1_JOINTINDEX_HPP
#define K1SIM_SHARED_K1_JOINTINDEX_HPP

#include <array>
#include <cstddef>

namespace k1sim {

// Booster JointIndexK1 order: the index each joint occupies in the SDK's
// LowState::motor_state_serial. This is the T1 JointIndex with the waist removed.
// The parallel-ankle crank indices (15/16, 21/22 on T1) carry the *serial*
// ankle pitch/roll values here, which map 1:1 onto the MJCF's serial ankle joints.
enum JointIndexK1 : std::size_t {
    HeadYaw = 0,
    HeadPitch,
    LeftShoulderPitch,
    LeftShoulderRoll,
    LeftElbowPitch,
    LeftElbowYaw,
    RightShoulderPitch,
    RightShoulderRoll,
    RightElbowPitch,
    RightElbowYaw,
    LeftHipPitch,
    LeftHipRoll,
    LeftHipYaw,
    LeftKneePitch,
    LeftAnklePitch,
    LeftAnkleRoll,
    RightHipPitch,
    RightHipRoll,
    RightHipYaw,
    RightKneePitch,
    RightAnklePitch,
    RightAnkleRoll,
};

inline constexpr std::size_t JOINT_COUNT = 22;

// Joint *and* actuator names in the vendored booster_assets K1_22dof.xml,
// in JointIndexK1 order (verified identical to booster_deploy's K1_CFG.joint_names).
inline constexpr std::array<const char*, JOINT_COUNT> JOINT_NAMES = {
    "AAHead_yaw",
    "Head_pitch",
    "ALeft_Shoulder_Pitch",
    "Left_Shoulder_Roll",
    "Left_Elbow_Pitch",
    "Left_Elbow_Yaw",
    "ARight_Shoulder_Pitch",
    "Right_Shoulder_Roll",
    "Right_Elbow_Pitch",
    "Right_Elbow_Yaw",
    "Left_Hip_Pitch",
    "Left_Hip_Roll",
    "Left_Hip_Yaw",
    "Left_Knee_Pitch",
    "Left_Ankle_Pitch",
    "Left_Ankle_Roll",
    "Right_Hip_Pitch",
    "Right_Hip_Roll",
    "Right_Hip_Yaw",
    "Right_Knee_Pitch",
    "Right_Ankle_Pitch",
    "Right_Ankle_Roll",
};

}  // namespace k1sim

#endif  // K1SIM_SHARED_K1_JOINTINDEX_HPP
