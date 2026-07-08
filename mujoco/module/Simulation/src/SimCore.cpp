#include "module/Simulation/src/SimCore.hpp"

#include <algorithm>
#include <cmath>
#include <ctime>
#include <stdexcept>

#include "shared/k1/BoosterApi.hpp"
#include "shared/util/Config.hpp"

namespace k1sim {

namespace {

// ZYX (yaw-pitch-roll) Euler extraction from a wxyz quaternion — the conventional
// roll/pitch/yaw decomposition used by the Booster wire format's imu_state.rpy.
void quat_to_rpy(const std::array<double, 4>& q, std::array<double, 3>& rpy) {
    const double w = q[0];
    const double x = q[1];
    const double y = q[2];
    const double z = q[3];

    const double sinr_cosp = 2.0 * (w * x + y * z);
    const double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    rpy[0]                 = std::atan2(sinr_cosp, cosr_cosp);

    double sinp = 2.0 * (w * y - z * x);
    sinp        = std::clamp(sinp, -1.0, 1.0);
    rpy[1]      = std::asin(sinp);

    const double siny_cosp = 2.0 * (w * z + x * y);
    const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    rpy[2]                 = std::atan2(siny_cosp, cosy_cosp);
}

double to_seconds(const timespec& t) {
    return static_cast<double>(t.tv_sec) + static_cast<double>(t.tv_nsec) * 1e-9;
}

timespec add_seconds(timespec t, double seconds) {
    const double total_nsec = static_cast<double>(t.tv_nsec) + seconds * 1e9;
    auto sec_adjust         = static_cast<long long>(std::floor(total_nsec / 1e9));
    t.tv_sec += sec_adjust;
    t.tv_nsec = static_cast<long>(total_nsec - static_cast<double>(sec_adjust) * 1e9);
    return t;
}

}  // namespace

SimCore::SimCore(Config config, StateCallback on_state) : config_(std::move(config)), on_state_(std::move(on_state)) {
    pd_.kp = config_.kp;
    pd_.kd = config_.kd;
}

SimCore::~SimCore() {
    stop();
    unload();
}

void SimCore::load_model() {
    const std::string resolved = config::resolve_path(config_.model_path).string();

    char error[1024] = {0};
    m_ = mj_loadXML(resolved.c_str(), nullptr, error, sizeof(error));
    if (m_ == nullptr) {
        throw std::runtime_error("mj_loadXML failed for '" + resolved + "': " + error);
    }

    // Throws if any joint/actuator is missing or there is no free root joint.
    map_ = ModelMap::build(m_);

    d_ = mj_makeData(m_);
    if (d_ == nullptr) {
        mj_deleteModel(m_);
        m_ = nullptr;
        throw std::runtime_error("mj_makeData failed for '" + resolved + "'");
    }

    const int ready_key = mj_name2id(m_, mjOBJ_KEY, "ready");
    if (ready_key >= 0) {
        mj_resetDataKeyframe(m_, d_, ready_key);
        for (std::size_t i = 0; i < JOINT_COUNT; ++i) {
            ready_target_[i] = m_->key_qpos[ready_key * m_->nq + map_.qpos_adr[i]];
        }
    }
    else {
        ready_target_ = config_.ready_pose_fallback;
    }

    // Populate derived quantities (xquat, sensordata, ...) for the reset pose before the
    // physics thread's first mj_step; harmless if nothing reads them this early.
    mj_forward(m_, d_);
}

void SimCore::start() {
    bool expected = false;
    if (!running_.compare_exchange_strong(expected, true, std::memory_order_acq_rel)) {
        return;  // already running
    }
    thread_ = std::thread(&SimCore::physics_loop, this);
}

void SimCore::stop() {
    running_.store(false, std::memory_order_release);
    if (thread_.joinable()) {
        thread_.join();
    }
}

void SimCore::unload() {
    if (d_ != nullptr) {
        mj_deleteData(d_);
        d_ = nullptr;
    }
    if (m_ != nullptr) {
        mj_deleteModel(m_);
        m_ = nullptr;
    }
}

std::unique_ptr<message::SimStateUpdate> SimCore::make_snapshot(uint64_t steps) const {
    auto s       = std::make_unique<message::SimStateUpdate>();
    s->sim_time  = d_->time;
    s->step_count = steps;

    for (std::size_t i = 0; i < JOINT_COUNT; ++i) {
        auto& j = s->joints[i];
        j.q     = d_->qpos[map_.qpos_adr[i]];
        j.dq    = d_->qvel[map_.dof_adr[i]];
        j.ddq   = d_->qacc[map_.dof_adr[i]];
        j.tau   = d_->actuator_force[map_.act_id[i]];
    }

    // IMU orientation + rpy.
    std::array<double, 4> quat{1, 0, 0, 0};
    if (map_.sens_quat >= 0) {
        for (int k = 0; k < 4; ++k) {
            quat[k] = d_->sensordata[map_.sens_quat + k];
        }
    }
    else if (map_.root_body_id >= 0) {
        for (int k = 0; k < 4; ++k) {
            quat[k] = d_->xquat[4 * map_.root_body_id + k];
        }
    }
    s->imu.quat = quat;
    quat_to_rpy(quat, s->imu.rpy);

    if (map_.sens_gyro >= 0) {
        for (int k = 0; k < 3; ++k) {
            s->imu.gyro[k] = d_->sensordata[map_.sens_gyro + k];
        }
    }
    else if (map_.root_body_id >= 0) {
        // Fallback: cvel's angular part is world-axis-aligned; rotate into the body's local
        // frame with the body rotation matrix (R^T * world = mju_mulMatTVec3).
        const mjtNum* rmat  = d_->xmat + 9 * map_.root_body_id;
        const mjtNum world[3] = {d_->cvel[6 * map_.root_body_id + 0],
                                  d_->cvel[6 * map_.root_body_id + 1],
                                  d_->cvel[6 * map_.root_body_id + 2]};
        mjtNum local[3];
        mju_mulMatTVec3(local, rmat, world);
        s->imu.gyro = {local[0], local[1], local[2]};
    }

    if (map_.sens_acc >= 0) {
        for (int k = 0; k < 3; ++k) {
            s->imu.acc[k] = d_->sensordata[map_.sens_acc + k];
        }
    }
    else if (map_.root_body_id >= 0) {
        // Fallback approximation: the reading a stationary accelerometer would show under
        // gravity alone (R^T * (0,0,+g)); ignores true linear acceleration (no cacc bookkeeping
        // without the real sensor). Good enough for a defensive path that isn't exercised by
        // the vendored model, which always carries the real sensor.
        const mjtNum* rmat = d_->xmat + 9 * map_.root_body_id;
        const mjtNum g     = -m_->opt.gravity[2];
        const mjtNum world[3] = {0, 0, g};
        mjtNum local[3];
        mju_mulMatTVec3(local, rmat, world);
        s->imu.acc = {local[0], local[1], local[2]};
    }

    // Base pose/velocity (free root joint). qpos: [x y z qw qx qy qz]; qvel: [vx vy vz wx wy wz]
    // with the linear part in world frame and the angular part in the body's local frame.
    const int qadr = map_.root_qpos_adr;
    const int vadr = map_.root_dof_adr;
    s->base.x       = d_->qpos[qadr + 0];
    s->base.y       = d_->qpos[qadr + 1];
    s->base.z       = d_->qpos[qadr + 2];
    s->base.quat    = {d_->qpos[qadr + 3], d_->qpos[qadr + 4], d_->qpos[qadr + 5], d_->qpos[qadr + 6]};
    s->base.lin_vel = {d_->qvel[vadr + 0], d_->qvel[vadr + 1], d_->qvel[vadr + 2]};
    {
        const mjtNum* rmat = d_->xmat + 9 * map_.root_body_id;
        const mjtNum local[3] = {d_->qvel[vadr + 3], d_->qvel[vadr + 4], d_->qvel[vadr + 5]};
        mjtNum world[3];
        mju_mulMatVec3(world, rmat, local);
        s->base.ang_vel = {world[0], world[1], world[2]};
    }

    StepController* ctrl = controller_.load(std::memory_order_acquire);
    if (ctrl != nullptr) {
        s->mode        = ctrl->mode();
        s->fall_state  = ctrl->fall_state();
        s->getting_up  = ctrl->getting_up();
    }
    else {
        s->mode       = booster::RobotMode::PREPARE;
        s->fall_state = booster::FallState::IS_READY;
        s->getting_up = false;
    }

    s->measured_rtf = measured_rtf_.load(std::memory_order_relaxed);
    return s;
}

void SimCore::physics_loop() {
    const double dt        = m_->opt.timestep;
    const bool free_run     = config_.rtf <= 0.0;
    const double period     = free_run ? 0.0 : dt / config_.rtf;
    const auto publish_every = static_cast<uint64_t>(config_.state_publish_divisor > 0 ? config_.state_publish_divisor : 0);

    timespec deadline{};
    clock_gettime(CLOCK_MONOTONIC, &deadline);

    double window_wall_start   = to_seconds(deadline);
    uint64_t window_step_start = 0;
    uint64_t steps             = 0;

    while (running_.load(std::memory_order_acquire)) {
        std::unique_ptr<message::SimStateUpdate> snapshot;
        {
            std::lock_guard<std::mutex> lock(mutex_);

            StepController* ctrl = controller_.load(std::memory_order_acquire);
            if (ctrl != nullptr) {
                ctrl->step(m_, d_);
            }
            else {
                pd_.apply(m_, d_, map_, ready_target_);
            }
            mj_step(m_, d_);
            ++steps;
            step_count_.store(steps, std::memory_order_relaxed);

            if (publish_every > 0 && steps % publish_every == 0) {
                snapshot = make_snapshot(steps);
            }
        }  // release the mutex before emitting/pacing

        if (snapshot && on_state_) {
            on_state_(std::move(snapshot));
        }

        if (!free_run) {
            deadline = add_seconds(deadline, period);
            timespec now_ts{};
            clock_gettime(CLOCK_MONOTONIC, &now_ts);
            const double behind = to_seconds(now_ts) - to_seconds(deadline);
            if (behind > config_.resync_threshold) {
                deadline = now_ts;
                dropped_deadlines_.fetch_add(1, std::memory_order_relaxed);
            }
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &deadline, nullptr);
        }

        timespec wall_now{};
        clock_gettime(CLOCK_MONOTONIC, &wall_now);
        const double wall_elapsed = to_seconds(wall_now) - window_wall_start;
        if (wall_elapsed >= 1.0) {
            const double sim_elapsed = static_cast<double>(steps - window_step_start) * dt;
            measured_rtf_.store(sim_elapsed / wall_elapsed, std::memory_order_relaxed);
            window_wall_start  = to_seconds(wall_now);
            window_step_start  = steps;
        }
    }
}

}  // namespace k1sim
