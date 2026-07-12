#include "module/Locomotion/src/backends/PolicyBackend.hpp"

#ifdef K1_WITH_ONNX

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <stdexcept>

#include "shared/util/Config.hpp"

namespace k1sim::module::backends {

namespace {

constexpr double kHeadPitchMin = -0.3;
constexpr double kHeadPitchMax = 1.0;
constexpr double kHeadYawLimit = 0.785;
constexpr double kPi           = 3.141592653589793;
constexpr double kTwoPi        = 6.283185307179586;

// Resolves policy.path (relative to the mujoco/ source root, like every other
// config path) and fatal-throws if the file is missing.
std::string resolve_and_validate_path(const YAML::Node& policy_cfg) {
    const std::string configured = policy_cfg["path"].as<std::string>("");
    if (configured.empty()) {
        throw std::runtime_error(
            "config/locomotion.yaml: backend: policy requires policy.path to point at an .onnx file");
    }
    const std::filesystem::path resolved = config::resolve_path(configured);
    if (!std::filesystem::exists(resolved)) {
        std::fprintf(stderr, "FATAL: PolicyBackend: policy.path '%s' (resolved '%s') does not exist\n",
                     configured.c_str(), resolved.string().c_str());
        throw std::runtime_error("PolicyBackend: onnx model file not found: " + resolved.string());
    }
    return resolved.string();
}

Ort::SessionOptions make_session_options() {
    Ort::SessionOptions opts;
    opts.SetIntraOpNumThreads(1);
    opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    return opts;
}

// Loads a 22-element sequence from policy.<key>; fatal-throws on wrong length so a
// half-edited config can't silently skew the obs/action mapping.
std::array<double, JOINT_COUNT> load_joint_array(const YAML::Node& policy_cfg, const char* key) {
    const YAML::Node node = policy_cfg[key];
    if (!node || node.size() != JOINT_COUNT) {
        throw std::runtime_error(std::string("config/locomotion.yaml: policy.") + key + " must be a list of "
                                 + std::to_string(JOINT_COUNT) + " values (JointIndexK1 order)");
    }
    std::array<double, JOINT_COUNT> out{};
    for (std::size_t i = 0; i < JOINT_COUNT; ++i) {
        out[i] = node[i].as<double>();
    }
    return out;
}

}  // namespace

PolicyBackend::PolicyBackend(const ModelMap& map,
                              PdController pd,
                              std::array<double, JOINT_COUNT> /*ready_pose — unused; the policy
                              tracks policy.default_pose (the training keyframe) instead*/,
                              const YAML::Node& policy_cfg)
    : map_(map)
    , pd_(std::move(pd))
    , default_pose_(load_joint_array(policy_cfg, "default_pose"))
    , action_scale_joint_(load_joint_array(policy_cfg, "action_scale_joint"))
    , gait_frequency_(policy_cfg["gait_frequency"].as<double>(1.5))
    , stand_threshold_(policy_cfg["stand_threshold"].as<double>(0.01))
    , inference_divisor_(std::max(1, policy_cfg["inference_divisor"].as<int>(20)))
    , env_(ORT_LOGGING_LEVEL_WARNING, "k1sim_policy_backend")
    , session_(env_, resolve_and_validate_path(policy_cfg).c_str(), make_session_options())
    , mem_info_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeCPU)) {
    // The policy runs against the gains it was trained with, not gains.yaml (which
    // is tuned for the stand/kinematic controllers).
    pd_.kp = load_joint_array(policy_cfg, "kp");
    pd_.kd = load_joint_array(policy_cfg, "kd");
    const double global_scale = policy_cfg["action_scale"].as<double>(1.0);
    for (double& s : action_scale_joint_) {
        s *= global_scale;
    }
}

void PolicyBackend::reset(const mjModel* /*m*/, mjData* /*d*/) {
    step_counter_ = 0;  // forces an inference on the very next update()
    phase_        = {0.0, kPi};
    last_action_.fill(0.0);
}

std::array<float, OBS_DIM> PolicyBackend::build_obs(const mjModel* /*m*/, mjData* d, const LocoCommand& cmd) const {
    std::array<float, OBS_DIM> obs{};
    std::size_t idx = 0;

    // [0:3] base linear velocity, body frame (velocimeter at the imu site; falls
    // back to the root qvel rotated into the body frame if the sensor is absent).
    const double* quat = &d->qpos[map_.root_qpos_adr + 3];  // wxyz, body->world
    if (map_.sens_linvel >= 0) {
        obs[idx++] = static_cast<float>(d->sensordata[map_.sens_linvel + 0]);
        obs[idx++] = static_cast<float>(d->sensordata[map_.sens_linvel + 1]);
        obs[idx++] = static_cast<float>(d->sensordata[map_.sens_linvel + 2]);
    }
    else {
        double inv_quat[4];
        mju_negQuat(inv_quat, quat);
        double linvel_body[3];
        mju_rotVecQuat(linvel_body, &d->qvel[map_.root_dof_adr], inv_quat);
        obs[idx++] = static_cast<float>(linvel_body[0]);
        obs[idx++] = static_cast<float>(linvel_body[1]);
        obs[idx++] = static_cast<float>(linvel_body[2]);
    }

    // [3:6] base angular velocity (gyro), body frame.
    if (map_.sens_gyro >= 0) {
        obs[idx++] = static_cast<float>(d->sensordata[map_.sens_gyro + 0]);
        obs[idx++] = static_cast<float>(d->sensordata[map_.sens_gyro + 1]);
        obs[idx++] = static_cast<float>(d->sensordata[map_.sens_gyro + 2]);
    }
    else {
        idx += 3;
    }

    // [6:9] projected gravity: world "down" in the body frame (unit vector).
    {
        double inv_quat[4];
        mju_negQuat(inv_quat, quat);
        const double down_world[3] = {0.0, 0.0, -1.0};
        double grav_body[3];
        mju_rotVecQuat(grav_body, down_world, inv_quat);
        obs[idx++] = static_cast<float>(grav_body[0]);
        obs[idx++] = static_cast<float>(grav_body[1]);
        obs[idx++] = static_cast<float>(grav_body[2]);
    }

    // [9:12] command [vx, vy, vyaw] — passed through as-is (the standing gate
    // only pins the phase observation, matching the playground task).
    obs[idx++] = static_cast<float>(cmd.vx);
    obs[idx++] = static_cast<float>(cmd.vy);
    obs[idx++] = static_cast<float>(cmd.vyaw);

    // [12:34] q - default_pose, [34:56] dq (unscaled), all 22 joints.
    for (std::size_t j = 0; j < JOINT_COUNT; ++j) {
        obs[idx++] = static_cast<float>(d->qpos[map_.qpos_adr[j]] - default_pose_[j]);
    }
    for (std::size_t j = 0; j < JOINT_COUNT; ++j) {
        obs[idx++] = static_cast<float>(d->qvel[map_.dof_adr[j]]);
    }
    // [56:78] previous action (raw network output).
    for (std::size_t k = 0; k < ACT_DIM; ++k) {
        obs[idx++] = static_cast<float>(last_action_[k]);
    }

    // [78:82] two-foot gait phase [cos(ph0), cos(ph1), sin(ph0), sin(ph1)];
    // pinned to [pi, pi] while standing (cos = -1, sin = 0).
    const double speed = std::sqrt(cmd.vx * cmd.vx + cmd.vy * cmd.vy + cmd.vyaw * cmd.vyaw);
    const std::array<double, 2> ph = speed >= stand_threshold_ ? phase_ : std::array<double, 2>{kPi, kPi};
    obs[idx++] = static_cast<float>(std::cos(ph[0]));
    obs[idx++] = static_cast<float>(std::cos(ph[1]));
    obs[idx++] = static_cast<float>(std::sin(ph[0]));
    obs[idx++] = static_cast<float>(std::sin(ph[1]));
    return obs;
}

void PolicyBackend::run_inference(const mjModel* m, mjData* d, const LocoCommand& cmd) {
    auto obs = build_obs(m, d, cmd);
    const std::array<int64_t, 2> obs_shape{1, static_cast<int64_t>(obs.size())};
    Ort::Value obs_tensor =
        Ort::Value::CreateTensor<float>(mem_info_, obs.data(), obs.size(), obs_shape.data(), obs_shape.size());

    static constexpr const char* kInputNames[]  = {"obs"};
    // "continuous_actions" is what the playground brax-to-ONNX export names the
    // output tensor (see docs/OBS_ACTION_CONTRACT.md).
    static constexpr const char* kOutputNames[] = {"continuous_actions"};
    auto outputs = session_.Run(Ort::RunOptions{nullptr}, kInputNames, &obs_tensor, 1, kOutputNames, 1);

    const float* action_data = outputs.front().GetTensorData<float>();
    for (std::size_t k = 0; k < ACT_DIM; ++k) {
        last_action_[k] = static_cast<double>(action_data[k]);
    }
}

void PolicyBackend::update(const mjModel* m, mjData* d, const LocoCommand& cmd) {
    if (step_counter_ % inference_divisor_ == 0) {
        run_inference(m, d, cmd);
        // Advance the gait phase once per inference (50 Hz), wrapped to [-pi, pi).
        // It advances regardless of the command; standing only pins the *observed*
        // phase (build_obs), matching the playground deployment reference
        // (mujoco_playground experimental/sim2sim).
        const double dt = static_cast<double>(inference_divisor_) * m->opt.timestep;
        for (double& ph : phase_) {
            ph = std::fmod(ph + kTwoPi * dt * gait_frequency_ + kPi, kTwoPi) - kPi;
        }
    }
    ++step_counter_;

    // Target = training default pose + per-joint-scaled action for all 22 joints;
    // overwrite the head with the RotateHead command (the sim owns the head).
    std::array<double, JOINT_COUNT> q_ref{};
    for (std::size_t j = 0; j < JOINT_COUNT; ++j) {
        q_ref[j] = default_pose_[j] + action_scale_joint_[j] * last_action_[j];
    }
    q_ref[JointIndexK1::HeadPitch] = std::clamp(cmd.head_pitch, kHeadPitchMin, kHeadPitchMax);
    q_ref[JointIndexK1::HeadYaw]   = std::clamp(cmd.head_yaw, -kHeadYawLimit, kHeadYawLimit);

    std::array<double, JOINT_COUNT> dq_ref{};
    std::array<double, JOINT_COUNT> tau_ff{};
    tau_ff[JointIndexK1::HeadPitch] = d->qfrc_bias[map_.dof_adr[JointIndexK1::HeadPitch]];
    tau_ff[JointIndexK1::HeadYaw]   = d->qfrc_bias[map_.dof_adr[JointIndexK1::HeadYaw]];

    pd_.apply(m, d, map_, q_ref, dq_ref, tau_ff);
}

}  // namespace k1sim::module::backends

#endif  // K1_WITH_ONNX
