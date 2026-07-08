"""K1 joint names in JointIndexK1 order — the single ordering the sim, the deploy
contract, and training all share. Mirrors mujoco/shared/k1/JointIndex.hpp JOINT_NAMES
(keep in sync). Legs are the contiguous slice [10, 22)."""

JOINT_NAMES = [
    "AAHead_yaw",              # 0
    "Head_pitch",             # 1
    "ALeft_Shoulder_Pitch",   # 2
    "Left_Shoulder_Roll",     # 3
    "Left_Elbow_Pitch",       # 4
    "Left_Elbow_Yaw",         # 5
    "ARight_Shoulder_Pitch",  # 6
    "Right_Shoulder_Roll",    # 7
    "Right_Elbow_Pitch",      # 8
    "Right_Elbow_Yaw",        # 9
    "Left_Hip_Pitch",         # 10  ← leg start
    "Left_Hip_Roll",          # 11
    "Left_Hip_Yaw",           # 12
    "Left_Knee_Pitch",        # 13
    "Left_Ankle_Pitch",       # 14
    "Left_Ankle_Roll",        # 15
    "Right_Hip_Pitch",        # 16
    "Right_Hip_Roll",         # 17
    "Right_Hip_Yaw",          # 18
    "Right_Knee_Pitch",       # 19
    "Right_Ankle_Pitch",      # 20
    "Right_Ankle_Roll",       # 21
]
