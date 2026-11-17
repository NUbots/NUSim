#include "module/SkillPlayer/src/SkillPlayer.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <stdexcept>

#include "module/SkillPlayer/src/SkillPolicy.hpp"
#include "shared/k1/BoosterApi.hpp"
#include "shared/message/Commands.hpp"
#include "shared/message/InputMessages.hpp"
#include "shared/sim/FreeBody.hpp"
#include "shared/util/Config.hpp"

namespace k1sim::module {

    using message::KeyEvent;
    using message::SimHandles;
    using message::SimStateUpdate;
    using skill::Skill;
    using skill::SkillPolicy;

    namespace {

        constexpr double CONTROL_DT = 0.02;  // the policies' step; SimStateUpdate arrives at this rate
        constexpr const char* KEY_HELP =
            "No ball: W S walk, A D turn  |  Ball: W A S D dribble, X release  |  Shift faster\n"
            "Q pass from the ball  |  E ball at feet  |  Space hold to charge, W A S D aim, release to kick\n"
            "R stand up  |  P pause";

        // Ball-skill PD gains from the controller spec (section 4), for files whose metadata gains
        // are placeholders. The same values as the earlier exports' metadata.
        const skill::Gains SPEC_BALL_GAINS{
            {3.948, 3.948, 3.948, 3.948, 3.948, 3.948, 3.948, 3.948, 3.948, 3.948, 30.201,
             21.448, 17.846, 60.402, 35.692, 35.692, 30.201, 21.448, 17.846, 60.402, 35.692, 35.692},
            {0.251, 0.251, 0.251, 0.251, 0.251, 0.251, 0.251, 0.251, 0.251, 0.251, 3.605,
             2.560, 2.130, 4.807, 4.260, 4.260, 3.605, 2.560, 2.130, 4.807, 4.260, 4.260}};

        double smoothstep(double x) {
            x = std::clamp(x, 0.0, 1.0);
            return x * x * (3.0 - 2.0 * x);
        }

        double quat_yaw(const mjtNum* q) {
            return std::atan2(2.0 * (q[0] * q[3] + q[1] * q[2]), 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]));
        }

        // "Q", "space", "shift" or any single letter -> a KeyEvent code; 0 if it is not a key.
        int key_code(std::string name) {
            std::transform(name.begin(), name.end(), name.begin(), [](unsigned char c) { return std::toupper(c); });
            if (name == "SPACE") {
                return message::KEY_SPACE;
            }
            if (name == "SHIFT") {
                return message::KEY_LEFT_SHIFT;
            }
            return name.size() == 1 ? static_cast<unsigned char>(name[0]) : 0;
        }

    }  // namespace

    struct SkillPlayer::Policies {
        Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "k1sim-skills"};
        std::unique_ptr<SkillPolicy> walk, receive, dribble, kick, kick_moving;
    };

    SkillPlayer::~SkillPlayer() = default;

    SkillPlayer::SkillPlayer(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        const YAML::Node cfg = config::load("skills.yaml");

        // Policies load here, before the sim starts, so a missing or mismatched file stops the run
        // with its reason instead of leaving a robot nothing is driving.
        policies_ = std::make_unique<Policies>();

        skill::Gains ball_gains = SPEC_BALL_GAINS;
        if (const auto node = cfg["ball_skill_gains"]; node && node["kp"] && node["kd"]) {
            for (std::size_t i = 0; i < JOINT_COUNT; ++i) {
                ball_gains.kp[i] = node["kp"][i].as<double>();
                ball_gains.kd[i] = node["kd"][i].as<double>();
            }
        }

        auto load = [&](const char* name, bool required, bool walk = false) -> std::unique_ptr<SkillPolicy> {
            const std::string file = cfg["policies"][name].as<std::string>("");
            if (file.empty()) {
                if (required) {
                    throw std::runtime_error(std::string("skills.yaml: policies.") + name + " is not set");
                }
                return nullptr;
            }
            const auto path = config::resolve_path(file);
            if (!std::filesystem::exists(path)) {
                throw std::runtime_error("skills.yaml: policies." + std::string(name) + " -> " + path.string()
                                         + " does not exist (copy the policies in with ./b skills sync)");
            }
            auto policy = walk ? std::make_unique<SkillPolicy>(policies_->env, path.string(), skill::WALK_OBS_SIZE)
                               : std::make_unique<SkillPolicy>(policies_->env, path.string(), skill::OBS_SIZE, ball_gains);
            log<NUClear::LogLevel::INFO>("SkillPlayer:",
                                         name,
                                         "policy",
                                         path.string(),
                                         policy->gains_from_file() ? "" : "(placeholder gains in the file; using ball_skill_gains)");
            return policy;
        };
        policies_->walk        = load("walk", false, true);
        policies_->receive     = load("receive", true);
        policies_->dribble     = load("dribble", true);
        policies_->kick        = load("kick", true);
        policies_->kick_moving = load("kick_moving", false);

        auto range = [](const YAML::Node& node, Range fallback) {
            if (node && node.IsSequence() && node.size() == 2) {
                return Range{node[0].as<double>(), node[1].as<double>()};
            }
            if (node && node.IsScalar()) {
                return Range{node.as<double>(), node.as<double>()};
            }
            return fallback;
        };

        skill::Config logic;
        const auto walk             = cfg["walk"];
        logic.walk_speed            = walk["speed"].as<double>(logic.walk_speed);
        logic.walk_fast_speed       = walk["fast_speed"].as<double>(logic.walk_fast_speed);
        logic.walk_turn_rate        = walk["turn_rate"].as<double>(logic.walk_turn_rate);
        logic.walk_back_speed       = walk["back_speed"].as<double>(logic.walk_back_speed);
        logic.walk_side_speed       = walk["side_speed"].as<double>(logic.walk_side_speed);
        logic.walk_turn_creep       = walk["turn_creep"].as<double>(logic.walk_turn_creep);
        const std::string side      = walk["side_keys"].as<std::string>("turn");
        if (side != "turn" && side != "strafe") {
            throw std::runtime_error("skills.yaml: walk.side_keys is turn or strafe");
        }
        logic.walk_side_keys = side == "strafe" ? skill::Config::SideKeys::STRAFE : skill::Config::SideKeys::TURN;
        logic.walk_pickup_distance  = walk["pickup_distance"].as<double>(logic.walk_pickup_distance);
        logic.walk_enabled          = policies_->walk != nullptr;
        logic.start_walking         = walk["start_walking"].as<bool>(true);
        walk_action_delay_          = walk["action_delay"].as<double>(walk_action_delay_);
        const auto dribble       = cfg["dribble"];
        logic.dribble_speed      = dribble["speed"].as<double>(logic.dribble_speed);
        logic.dribble_fast_speed = dribble["fast_speed"].as<double>(logic.dribble_fast_speed);
        const std::string frame  = dribble["frame"].as<std::string>("view");
        if (frame != "view" && frame != "field" && frame != "robot") {
            throw std::runtime_error("skills.yaml: dribble.frame is view, field or robot");
        }
        logic.frame = frame == "field"   ? skill::Config::Frame::FIELD
                      : frame == "robot" ? skill::Config::Frame::ROBOT
                                         : skill::Config::Frame::VIEW;
        const auto shoot         = cfg["shoot"];
        logic.kick_speed_min     = shoot["min_speed"].as<double>(logic.kick_speed_min);
        logic.kick_speed_max     = shoot["max_speed"].as<double>(logic.kick_speed_max);
        logic.charge_time        = shoot["charge_time"].as<double>(logic.charge_time);
        logic.aim_rate           = shoot["aim_rate"].as<double>(logic.aim_rate);
        logic.aim_goal           = shoot["aim"].as<std::string>("goal") != "heading";
        if (shoot["goal"] && shoot["goal"].size() == 2) {
            logic.goal = {shoot["goal"][0].as<double>(), shoot["goal"][1].as<double>()};
        }
        logic.kick_reach = shoot["reach"].as<double>(logic.kick_reach);
        logic.kick_cone  = shoot["cone_deg"].as<double>(logic.kick_cone * 180.0 / 3.14159265358979323846) * 3.14159265358979323846 / 180.0;
        const auto trap  = cfg["receive"];
        logic.settle_time       = trap["settle_time"].as<double>(logic.settle_time);
        logic.takeover_distance = trap["takeover_distance"].as<double>(logic.takeover_distance);
        logic.takeover_speed    = trap["takeover_speed"].as<double>(logic.takeover_speed);
        logic_                  = skill::Logic(logic);

        const auto pass     = cfg["pass"];
        pass_distance_      = range(pass["distance"], pass_distance_);
        pass_bearing_       = range(pass["bearing"], pass_bearing_);
        pass_offset_        = range(pass["offset"], pass_offset_);
        pass_arrival_       = range(pass["arrival_speed"], pass_arrival_);
        pass_rolling_decel_ = pass["rolling_decel"].as<double>(pass_rolling_decel_);
        pass_from_ball_     = pass["from"].as<std::string>("ball") != "spawn";
        pass_min_distance_  = pass["min_distance"].as<double>(pass_min_distance_);

        drop_distance_    = cfg["drop_distance"].as<double>(drop_distance_);
        action_delay_     = cfg["action_delay"].as<double>(action_delay_);
        log_period_       = cfg["log_period"].as<double>(log_period_);
        blend_time_       = cfg["blend_time"].as<double>(blend_time_);
        fall_reset_after_ = cfg["fall_reset_after"].as<double>(fall_reset_after_);

        const auto camera = cfg["camera"];
        if (camera["offset"] && camera["offset"].size() == 3) {
            camera_offset_ = {camera["offset"][0].as<double>(),
                              camera["offset"][1].as<double>(),
                              camera["offset"][2].as<double>()};
        }
        const double deg = 3.14159265358979323846 / 180.0;
        camera_hfov_     = camera["hfov_deg"].as<double>(90.0) * deg;
        camera_vfov_     = camera["vfov_deg"].as<double>(60.0) * deg;
        camera_range_    = camera["range"].as<double>(camera_range_);

        // Scripted key presses, for running without a keyboard: [time, press|release|tap, key].
        for (const auto& entry : cfg["script"]) {
            const double time        = entry[0].as<double>();
            const std::string action = entry[1].as<std::string>();
            const int code           = key_code(entry[2].as<std::string>());
            if (code == 0 || (action != "press" && action != "release" && action != "tap")) {
                throw std::runtime_error("skills.yaml: script entries are [time, press|release|tap, key]");
            }
            if (action != "release") {
                script_.push_back({time, code, true});
            }
            if (action != "press") {
                script_.push_back({action == "tap" ? time + 0.1 : time, code, false});
            }
        }
        std::stable_sort(script_.begin(), script_.end(), [](const ScriptEvent& a, const ScriptEvent& b) {
            return a.time < b.time;
        });

        on<Trigger<SimHandles>, Sync<SkillPlayer>>().then([this](const SimHandles& handles) {
            map_       = ModelMap::build(handles.model);
            head_body_ = mj_name2id(handles.model, mjOBJ_BODY, "Head_2");
            ball_body_ = mj_name2id(handles.model, mjOBJ_BODY, "ball");
            ball_geom_ = mj_name2id(handles.model, mjOBJ_GEOM, "ball");
            if (head_body_ < 0) {
                throw std::runtime_error("SkillPlayer: the model has no Head_2 body to carry the camera");
            }
            if (!freebody::valid_free_body_geom(handles.model, ball_body_, ball_geom_)) {
                log<NUClear::LogLevel::WARN>("SkillPlayer: this field has no ball, so there is nothing to play with");
                ball_body_ = ball_geom_ = -1;
            }
            goals_.clear();
            for (const char* name : {"goal_pos_x", "goal_neg_x"}) {
                const int body = mj_name2id(handles.model, mjOBJ_BODY, name);
                const int post = mj_name2id(handles.model, mjOBJ_GEOM, (std::string(name) + "_post1").c_str());
                const int bar  = mj_name2id(handles.model, mjOBJ_GEOM, (std::string(name) + "_bar").c_str());
                if (body < 0 || post < 0 || bar < 0) {
                    continue;
                }
                const mjtNum* m_pos = handles.model->body_pos + 3 * body;
                const double radius = handles.model->geom_size[3 * post];
                Goal goal;
                goal.x          = m_pos[0];
                goal.y          = m_pos[1];
                goal.half_width = std::abs(handles.model->geom_pos[3 * post + 1]) - radius;
                goal.height     = handles.model->geom_pos[3 * bar + 2] - radius;
                goal.dir        = goal.x >= 0.0 ? 1 : -1;
                goals_.push_back(goal);
            }
            rtf_   = handles.measured_rtf;
            data_  = handles.data;
            mutex_ = handles.mutex;
            model_ = handles.model;
            log<NUClear::LogLevel::INFO>("SkillPlayer ready --", KEY_HELP);
        });

        on<Trigger<SimStateUpdate>, Sync<SkillPlayer>>().then([this](const SimStateUpdate& state) { tick(state); });

        on<Trigger<message::ViewDirection>, Sync<SkillPlayer>>().then(
            [this](const message::ViewDirection& view) { view_yaw_ = view.yaw; });

        on<Trigger<KeyEvent>, Sync<SkillPlayer>>().then([this](const KeyEvent& event) { key(event.key, event.down); });
    }

    void SkillPlayer::key(int code, bool down) {
        switch (code) {
            case 'W': keys_.forward = down; break;
            case 'S': keys_.back = down; break;
            case 'A': keys_.left = down; break;
            case 'D': keys_.right = down; break;
            case message::KEY_LEFT_SHIFT:
            case message::KEY_RIGHT_SHIFT: keys_.fast = down; break;
            case 'Q': pass_pressed_ = pass_pressed_ || down; break;
            case 'E': drop_pressed_ = drop_pressed_ || down; break;
            case 'X': release_pressed_ = release_pressed_ || down; break;
            case message::KEY_SPACE:
                (down ? shoot_down_ : shoot_up_) = true;
                keys_.shoot                      = down;
                break;
            case 'R':
                if (down) {
                    emit(std::make_unique<message::RobotResetRequest>());
                }
                break;
            default: break;
        }
    }

    void SkillPlayer::restart() {
        phase_          = Phase::WAITING;
        mode_asked_     = -1.0;
        history_primed_ = false;
        last_action_    = {};
        walk_last_action_ = {};
        have_target_    = false;
        running_        = nullptr;
        last_ball_      = {0.5, 0.0, -0.8, 0.0};
        pass_pressed_ = drop_pressed_ = shoot_down_ = shoot_up_ = release_pressed_ = false;
        logic_.reset();
        decision_     = {};
        logged_skill_ = logic_.skill();
    }

    SkillPlayer::Sample SkillPlayer::read_state() {
        Sample s;
        std::lock_guard<std::mutex> lock(*mutex_);
        const mjtNum* root = data_->qpos + map_.root_qpos_adr;  // xyz + wxyz
        const mjtNum* quat = root + 3;

        // Trunk frame, as mjlab's root_link_ang_vel_b and projected_gravity_b: a free joint's
        // angular velocity is already in the body frame.
        const mjtNum* w = data_->qvel + map_.root_dof_adr + 3;
        s.gyro          = {w[0], w[1], w[2]};
        mjtNum inverse[4];
        mju_negQuat(inverse, quat);
        const mjtNum down[3] = {0.0, 0.0, -1.0};
        mjtNum gravity[3];
        mju_rotVecQuat(gravity, down, inverse);
        s.gravity = {gravity[0], gravity[1], gravity[2]};

        for (std::size_t i = 0; i < JOINT_COUNT; ++i) {
            s.q[i]  = data_->qpos[map_.qpos_adr[i]];
            s.dq[i] = data_->qvel[map_.dof_adr[i]];
        }

        s.world.robot_x = root[0];
        s.world.robot_y = root[1];
        s.world.yaw      = quat_yaw(quat);
        s.world.view_yaw = view_yaw_;

        s.ball_camera = last_ball_;
        s.ball_camera[3] = 0.0;
        if (ball_body_ >= 0) {
            const auto ball     = freebody::geom_centre_state(model_, data_, ball_body_, ball_geom_);
            s.world.ball_valid  = true;
            s.world.ball_x      = ball.position[0];
            s.world.ball_y      = ball.position[1];
            s.world.ball_z      = ball.position[2];
            s.world.ball_speed  = std::hypot(ball.lin_vel[0], ball.lin_vel[1], ball.lin_vel[2]);
            s.world.ball_radius = model_->geom_size[3 * ball_geom_];

            // The ball as the head camera would report it: the Head_2 frame (x forward, y left,
            // z up) moved to the front of the head. Out of view, the last sighting is repeated
            // with the flag down.
            const mjtNum* head_pos = data_->xpos + 3 * head_body_;
            const mjtNum* head_mat = data_->xmat + 9 * head_body_;
            mjtNum camera[3], relative[3], local[3];
            mju_mulMatVec3(camera, head_mat, camera_offset_.data());
            for (int k = 0; k < 3; ++k) {
                relative[k] = ball.position[k] - (head_pos[k] + camera[k]);
            }
            mju_mulMatTVec3(local, head_mat, relative);
            const double distance  = mju_norm3(local);
            const double azimuth   = std::atan2(local[1], local[0]);
            const double elevation = std::atan2(local[2], std::max(local[0], 1e-6));
            if (local[0] > 0.0 && std::abs(azimuth) < camera_hfov_ / 2.0 && std::abs(elevation) < camera_vfov_ / 2.0
                && distance < camera_range_) {
                s.ball_camera = {local[0], local[1], local[2], 1.0};
            }
        }
        return s;
    }

    double SkillPlayer::sample_range(const Range& range) {
        return std::uniform_real_distribution<double>(std::min(range.lo, range.hi), std::max(range.lo, range.hi))(rng_);
    }

    // Rolls a ground pass at the robot, as mjlab's launch_pass does in training: aimed `offset` to
    // its side and launched fast enough to arrive at `arrival_speed` after rolling friction has
    // slowed it. With pass_from_ball_ the ball is rolled from wherever it is now; otherwise it is
    // first put `distance` away at `bearing` off the robot's heading. Returns a note when the pass
    // is outside what receive was trained on (from in front, up to about 4 m away, within 45
    // degrees), or empty.
    std::string SkillPlayer::launch_pass(const Sample& sample) {
        const auto& w = sample.world;
        const double c = std::cos(w.yaw), s = std::sin(w.yaw);
        double sx = 0.0, sy = 0.0;  // ball start, the robot's yaw frame
        if (pass_from_ball_) {
            const auto ball = w.ball_local();
            sx = ball[0];
            sy = ball[1];
        }
        else {
            const double distance = sample_range(pass_distance_);
            const double bearing  = sample_range(pass_bearing_);
            sx                    = distance * std::cos(bearing);
            sy                    = distance * std::sin(bearing);
        }
        const double offset  = sample_range(pass_offset_);
        const double arrival = sample_range(pass_arrival_);

        const double px = -sx, py = offset - sy;
        const double length = std::max(std::hypot(px, py), 1e-6);
        const double speed  = std::sqrt(arrival * arrival + 2.0 * pass_rolling_decel_ * length);
        const double vx = speed * px / length, vy = speed * py / length;

        auto command        = std::make_unique<message::BallCommand>();
        command->frame      = message::BallCommand::Frame::WORLD;
        command->position   = {w.robot_x + c * sx - s * sy, w.robot_y + s * sx + c * sy, -1.0};  // z < 0: on the floor
        command->velocity   = {c * vx - s * vy, s * vx + c * vy, 0.0};
        command->rolling    = true;
        emit(command);

        const double distance = std::hypot(sx, sy);
        const double bearing  = std::atan2(sy, sx);
        log<NUClear::LogLevel::INFO>("SkillPlayer: pass from",
                                     distance,
                                     "m at bearing",
                                     bearing,
                                     "rad, aimed",
                                     offset,
                                     "m to the side, launched at",
                                     speed,
                                     "m/s to arrive at",
                                     arrival);
        if (std::abs(bearing) > pass_trained_bearing_) {
            return std::abs(bearing) > 1.5708 ? "pass from behind the robot: receive was trained on passes from in front"
                                              : "pass from wide of the robot: receive was trained within 45 degrees";
        }
        if (distance > pass_trained_distance_) {
            return "long pass: receive was trained on passes from up to 4 m";
        }
        return "";
    }

    SkillPolicy& SkillPlayer::policy_for(const skill::Decision& decision) {
        switch (decision.skill) {
            case Skill::WALK: return policies_->walk ? *policies_->walk : *policies_->receive;
            case Skill::DRIBBLE: return *policies_->dribble;
            case Skill::KICK:
                return decision.moving_kick && policies_->kick_moving ? *policies_->kick_moving : *policies_->kick;
            case Skill::RECEIVE: break;
        }
        return *policies_->receive;
    }

    void SkillPlayer::send_targets(const std::array<double, JOINT_COUNT>& q, const skill::Gains& gains, double delay) {
        auto command      = std::make_unique<message::LowCmdMessage>();
        command->cmd_type = 1;  // serial
        command->motors.resize(JOINT_COUNT);
        for (std::size_t i = 0; i < JOINT_COUNT; ++i) {
            command->motors[i].q  = static_cast<float>(q[i]);
            command->motors[i].kp = static_cast<float>(gains.kp[i]);
            command->motors[i].kd = static_cast<float>(gains.kd[i]);
        }
        last_target_ = q;
        have_target_ = true;
        // The ball skills were trained with the servo command reaching the motor 10 to 40 ms late;
        // sending it at once makes them over-correct. The wait is in wall time, so it is scaled by
        // how fast the sim is running.
        const double rtf = rtf_ != nullptr ? rtf_->load(std::memory_order_relaxed) : 1.0;
        if (delay > 0.0 && rtf > 0.05) {
            emit<Scope::DELAY>(command,
                               std::chrono::duration_cast<NUClear::clock::duration>(
                                   std::chrono::duration<double>(delay / rtf)));
        }
        else {
            emit(command);
        }
    }

    void SkillPlayer::tick(const SimStateUpdate& state) {
        if (model_ == nullptr || state.identity.model_index != 0) {
            return;
        }
        // Snapshots keep arriving while the sim is paused; only a new physics step is a control step.
        const bool rewound = have_state_ && (state.step_count < last_step_ || state.reset_count != reset_seen_);
        if (have_state_ && !rewound && state.step_count == last_step_) {
            return;
        }
        if (rewound || !have_state_) {
            if (state.step_count < last_step_) {
                script_next_ = 0;  // the whole world was reset, clock included
            }
            restart();
        }
        have_state_ = true;
        last_step_  = state.step_count;
        reset_seen_ = state.reset_count;

        while (script_next_ < script_.size() && script_[script_next_].time <= state.sim_time) {
            key(script_[script_next_].key, script_[script_next_].down);
            ++script_next_;
        }

        const Sample sample = read_state();
        last_ball_          = sample.ball_camera;
        check_goal(sample, state.sim_time);
        // The pose and gains the robot eases into at start-up: the policy it starts with.
        const SkillPolicy& base = logic_.skill() == Skill::WALK ? *policies_->walk : *policies_->receive;

        // The servo targets are only followed in CUSTOM mode; ask until the robot is in it.
        if (state.mode != booster::CUSTOM) {
            if (phase_ != Phase::WAITING) {
                restart();
            }
            if (mode_asked_ < 0.0 || state.sim_time - mode_asked_ > 0.25 || state.sim_time < mode_asked_) {
                mode_asked_ = state.sim_time;
                emit(std::make_unique<message::ModeChangeRequest>(message::ModeChangeRequest{booster::CUSTOM, 1}));
            }
            publish_status(state.sim_time, false);
            return;
        }

        if (phase_ == Phase::WAITING) {
            phase_       = Phase::BLENDING;
            phase_start_ = state.sim_time;
            blend_from_  = sample.q;
        }

        if (phase_ != Phase::FALLEN && state.fall_state == booster::HAS_FALLEN) {
            phase_       = Phase::FALLEN;
            phase_start_ = state.sim_time;
            log<NUClear::LogLevel::WARN>("SkillPlayer: the robot fell over (no policy gets it up; R stands it back up)");
        }

        switch (phase_) {
            case Phase::WAITING: break;
            case Phase::BLENDING: {
                // Ease from the pose the robot spawned in to the pose the policies are centred on.
                const double x = blend_time_ > 0.0 ? (state.sim_time - phase_start_) / blend_time_ : 1.0;
                const double s = smoothstep(x);
                std::array<double, JOINT_COUNT> target{};
                for (std::size_t i = 0; i < JOINT_COUNT; ++i) {
                    target[i] = blend_from_[i] + s * (base.default_pose[i] - blend_from_[i]);
                }
                send_targets(target, base.gains, 0.0);
                if (x >= 1.0) {
                    phase_ = Phase::PLAYING;
                    log<NUClear::LogLevel::INFO>(logic_.skill() == Skill::WALK
                                                 ? "SkillPlayer: policies in control, standing in walk (W walks, Q passes)"
                                                 : "SkillPlayer: policies in control, waiting for a pass (Q)");
                }
                break;
            }
            case Phase::PLAYING:
                play(state, sample);
                publish_status(state.sim_time, decision_.aiming, &sample);
                return;
            case Phase::FALLEN: {
                // Go limp rather than let a policy thrash on the ground.
                skill::Gains limp;
                limp.kd.fill(1.0);
                send_targets(sample.q, limp, 0.0);
                if (fall_reset_after_ > 0.0 && state.sim_time - phase_start_ > fall_reset_after_) {
                    phase_start_ = state.sim_time;
                    emit(std::make_unique<message::RobotResetRequest>());
                }
                break;
            }
        }
        publish_status(state.sim_time, false);
    }

    void SkillPlayer::play(const SimStateUpdate& state, const Sample& sample) {
        if (pass_pressed_) {
            if (!sample.world.ball_valid) {
                note_       = "no ball on this field";
                note_until_ = state.sim_time + 2.0;
            }
            else if (pass_from_ball_ && sample.world.ball_distance() < pass_min_distance_) {
                note_       = "the ball is already at the robot's feet";
                note_until_ = state.sim_time + 2.0;
            }
            else {
                const std::string warning = launch_pass(sample);
                logic_.on_pass();
                if (!warning.empty()) {
                    note_       = warning;
                    note_until_ = state.sim_time + 3.0;
                }
            }
        }
        if (drop_pressed_ && sample.world.ball_valid) {
            logic_.on_pass();  // receive it, from walk too
            // A ball at rest at the robot's feet, to dribble or shoot without a pass first.
            auto command        = std::make_unique<message::BallCommand>();
            command->frame      = message::BallCommand::Frame::ROBOT;
            command->position   = {drop_distance_, 0.0, -1.0};
            command->robot_body = "Trunk";
            emit(command);
        }
        if (shoot_down_ && !logic_.on_shoot_press(sample.world)) {
            note_       = logic_.skill() == Skill::WALK ? "walk up to the ball first" : "the ball is not in front of the robot";
            note_until_ = state.sim_time + 2.0;
        }
        if (shoot_up_) {
            logic_.on_shoot_release(sample.world);
        }
        if (release_pressed_) {
            logic_.on_release();
        }
        pass_pressed_ = drop_pressed_ = shoot_down_ = shoot_up_ = release_pressed_ = false;

        decision_ = logic_.tick(keys_, sample.world, CONTROL_DT);
        if (decision_.skill != logged_skill_) {
            log<NUClear::LogLevel::INFO>("SkillPlayer: t =",
                                         state.sim_time,
                                         skill::name(logged_skill_),
                                         "->",
                                         skill::name(decision_.skill),
                                         decision_.skill == Skill::KICK && decision_.moving_kick ? "(ball moving)" : "");
            logged_skill_ = decision_.skill;
        }
        SkillPolicy& policy  = policy_for(decision_);
        const bool walking   = decision_.skill == Skill::WALK && policies_->walk != nullptr;
        const bool was_walk  = running_ != nullptr && running_ == policies_->walk.get();
        if (running_ != nullptr && walking != was_walk && have_target_) {
            // Switching between walk and a ball skill changes gains, default pose, action scale and
            // input layout in the same step (spec section 4). The new policy starts from the current
            // state, and its "previous action" is the joint target last sent, in its own convention.
            std::array<float, JOINT_COUNT>& previous = walking ? walk_last_action_ : last_action_;
            for (std::size_t i = 0; i < JOINT_COUNT; ++i) {
                previous[i] = static_cast<float>((last_target_[i] - policy.default_pose[i]) / policy.action_scale[i]);
            }
            history_primed_ = false;
        }
        running_ = &policy;

        const auto previous_action = walking ? walk_last_action_ : last_action_;
        const auto action          = walking ? run_walk(policy, sample) : run_ball_skill(policy, sample);
        for (std::size_t i = 10; i < JOINT_COUNT; ++i) {  // legs
            jitter_sum_ += std::abs(policy.action_scale[i] * static_cast<double>(action[i] - previous_action[i]));
        }
        ++jitter_count_;
        std::array<double, JOINT_COUNT> target{};
        for (std::size_t i = 0; i < JOINT_COUNT; ++i) {
            target[i] = policy.default_pose[i] + policy.action_scale[i] * static_cast<double>(action[i]);
        }
        send_targets(target, policy.gains, walking ? walk_action_delay_ : action_delay_);

        if (state.sim_time >= log_next_ || state.sim_time < log_next_ - 1.0) {
            log_next_ = state.sim_time + log_period_;
            char line[360];
            std::snprintf(line,
                          sizeof(line),
                          "SKILL t=%.1f %s cmd=(%.2f, %.2f) twist=(%.2f, %.2f, %.2f) robot=(%.2f, %.2f, yaw %.2f) z=%.2f ball=(%.2f, %.2f) "
                          "speed=%.2f seen=%d cam=(%.2f, %.2f, %.2f) head=(%.2f, %.2f) local_cmd=(%.2f, %.2f) leg_target_step=%.4f",
                          state.sim_time,
                          skill::name(decision_.skill),
                          decision_.command_world[0],
                          decision_.command_world[1],
                          decision_.twist[0],
                          decision_.twist[1],
                          decision_.twist[2],
                          sample.world.robot_x,
                          sample.world.robot_y,
                          sample.world.yaw,
                          state.base.z,
                          sample.world.ball_x,
                          sample.world.ball_y,
                          sample.world.ball_speed,
                          static_cast<int>(sample.ball_camera[3]),
                          sample.ball_camera[0],
                          sample.ball_camera[1],
                          sample.ball_camera[2],
                          sample.q[JointIndexK1::HeadYaw],
                          sample.q[JointIndexK1::HeadPitch],
                          decision_.command_local[0],
                          decision_.command_local[1],
                          jitter_count_ ? jitter_sum_ / (12.0 * jitter_count_) : 0.0);
            jitter_sum_   = 0.0;
            jitter_count_ = 0;
            log<NUClear::LogLevel::INFO>(line);
        }
    }

    std::array<float, JOINT_COUNT> SkillPlayer::run_ball_skill(SkillPolicy& policy, const Sample& sample) {
        Frame frame;
        for (int k = 0; k < 3; ++k) {
            frame.gyro[k]    = static_cast<float>(sample.gyro[k]);
            frame.gravity[k] = static_cast<float>(sample.gravity[k]);
        }
        for (std::size_t i = 0; i < JOINT_COUNT; ++i) {
            frame.q[i]  = static_cast<float>(sample.q[i] - policy.default_pose[i]);
            frame.dq[i] = static_cast<float>(sample.dq[i]);
        }
        frame.action  = last_action_;
        frame.command = {static_cast<float>(decision_.command_local[0]), static_cast<float>(decision_.command_local[1])};
        for (int k = 0; k < 4; ++k) {
            frame.ball[k] = static_cast<float>(sample.ball_camera[k]);
        }
        if (!history_primed_) {
            history_.fill(frame);
            history_primed_ = true;
        }
        else {
            history_[0] = history_[1];
            history_[1] = history_[2];
            history_[2] = frame;
        }

        // Term by term, each as its three steps oldest first.
        std::vector<float> obs;
        obs.reserve(skill::OBS_SIZE);
        auto put = [&](auto member) {
            for (const Frame& step : history_) {
                obs.insert(obs.end(), (step.*member).begin(), (step.*member).end());
            }
        };
        put(&Frame::gyro);
        put(&Frame::gravity);
        put(&Frame::q);
        put(&Frame::dq);
        put(&Frame::action);
        put(&Frame::command);
        put(&Frame::ball);
        last_action_ = policy.infer(obs);
        return last_action_;
    }

    // The walk policy's 75 numbers, current step only: gyro, gravity, joint angles from ITS default
    // pose, joint speeds, its own previous output, twist (vx, vy, wz).
    std::array<float, JOINT_COUNT> SkillPlayer::run_walk(SkillPolicy& policy, const Sample& sample) {
        std::vector<float> obs;
        obs.reserve(skill::WALK_OBS_SIZE);
        for (int k = 0; k < 3; ++k) {
            obs.push_back(static_cast<float>(sample.gyro[k]));
        }
        for (int k = 0; k < 3; ++k) {
            obs.push_back(static_cast<float>(sample.gravity[k]));
        }
        for (std::size_t i = 0; i < JOINT_COUNT; ++i) {
            obs.push_back(static_cast<float>(sample.q[i] - policy.default_pose[i]));
        }
        for (std::size_t i = 0; i < JOINT_COUNT; ++i) {
            obs.push_back(static_cast<float>(sample.dq[i]));
        }
        obs.insert(obs.end(), walk_last_action_.begin(), walk_last_action_.end());
        for (int k = 0; k < 3; ++k) {
            obs.push_back(static_cast<float>(decision_.twist[k]));
        }
        walk_last_action_ = policy.infer(obs);
        return walk_last_action_;
    }

    // A goal: the whole ball past the goal line, between the posts and under the bar. Latched until
    // the ball is back on the field side of a goal line, so one goal is celebrated once.
    void SkillPlayer::check_goal(const Sample& sample, double sim_time) {
        const auto& w = sample.world;
        if (!w.ball_valid) {
            return;
        }
        bool behind_a_line = false;
        for (const Goal& goal : goals_) {
            const double past = goal.dir * (w.ball_x - goal.x);
            behind_a_line     = behind_a_line || past > 0.0;
            if (!ball_in_goal_ && past > w.ball_radius && std::abs(w.ball_y - goal.y) < goal.half_width
                && w.ball_z < goal.height) {
                ball_in_goal_ = true;
                auto scored    = std::make_unique<message::GoalScored>();
                scored->x      = goal.x;
                scored->y      = goal.y;
                scored->width  = 2.0 * goal.half_width;
                scored->height = goal.height;
                scored->dir    = goal.dir;
                emit(scored);
                note_       = "GOAL!";
                note_until_ = sim_time + 2.5;
                log<NUClear::LogLevel::INFO>("SkillPlayer: GOAL in the", goal.dir > 0 ? "+x" : "-x", "goal at t =", sim_time);
            }
        }
        if (!behind_a_line) {
            ball_in_goal_ = false;
        }
    }

    void SkillPlayer::publish_status(double sim_time, bool force, const Sample* sample) {
        auto status  = std::make_unique<message::SkillStatus>();
        status->keys = KEY_HELP;
        if (phase_ == Phase::PLAYING && decision_.aiming && sample != nullptr) {
            const auto& cfg   = logic_.config();
            status->aiming    = true;
            status->ball_x    = sample->world.ball_x;
            status->ball_y    = sample->world.ball_y;
            status->aim_angle = decision_.aim_angle;
            status->charge    = (decision_.aim_speed - cfg.kick_speed_min)
                                / std::max(cfg.kick_speed_max - cfg.kick_speed_min, 1e-6);
        }
        char text[160];
        switch (phase_) {
            case Phase::WAITING:
            case Phase::BLENDING:
                status->skill  = "STARTING";
                status->detail = "getting into the ready stance";
                break;
            case Phase::FALLEN:
                status->skill  = "FALLEN";
                status->detail = fall_reset_after_ > 0.0 ? "standing the robot back up shortly (R: now)"
                                                         : "R stands the robot back up";
                break;
            case Phase::PLAYING:
                status->skill = skill::name(decision_.skill);
                if (decision_.aiming) {
                    status->skill = "AIMING";
                    std::snprintf(text,
                                  sizeof(text),
                                  "%.1f m/s -- hold for more, W A S D to aim, release to shoot",
                                  decision_.aim_speed);
                    status->detail = text;
                }
                else if (decision_.skill == Skill::WALK) {
                    const auto& t = decision_.twist;
                    if (t[0] != 0.0 || t[1] != 0.0 || t[2] != 0.0) {
                        std::snprintf(text, sizeof(text), "walking: %.1f m/s forward, %.1f m/s left, turning %.1f rad/s", t[0], t[1], t[2]);
                        status->detail = text;
                    }
                    else {
                        status->detail = "standing; W A S D walk, Q plays a pass, E a ball at the feet";
                    }
                }
                else if (decision_.skill == Skill::RECEIVE) {
                    status->detail = "waiting for a pass (Q), then trapping it; W walks if the ball is away";
                }
                else if (decision_.skill == Skill::KICK) {
                    std::snprintf(text,
                                  sizeof(text),
                                  "shot at %.1f m/s; W to walk after it, Q for a new pass",
                                  std::hypot(decision_.command_world[0], decision_.command_world[1]));
                    status->detail = text;
                }
                else if (decision_.command_world[0] != 0.0 || decision_.command_world[1] != 0.0) {
                    std::snprintf(text,
                                  sizeof(text),
                                  "ball to go (%.1f, %.1f) m/s",
                                  decision_.command_world[0],
                                  decision_.command_world[1]);
                    status->detail = text;
                }
                else {
                    status->detail = "holding the ball";
                }
                break;
        }
        if (sim_time < note_until_ && sim_time > note_until_ - 2.5) {
            status->detail += "  [" + note_ + "]";
        }
        const std::string all = status->skill + status->detail;
        if (force || all != status_text_ || sim_time - status_sent_ > 1.0 || sim_time < status_sent_) {
            status_text_ = all;
            status_sent_ = sim_time;
            emit(status);
        }
    }

}  // namespace k1sim::module
