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
#include "shared/util/RobotRoster.hpp"

namespace k1sim::module {

    using message::KeyEvent;
    using message::RobotStatesUpdate;
    using message::SimHandles;
    using message::SimStateUpdate;
    using skill::Skill;
    using skill::SkillPolicy;

    namespace {

        constexpr double CONTROL_DT = 0.02;  // the policies' step; SimStateUpdate arrives at this rate
        constexpr const char* KEY_HELP =
            "No ball: W S walk, A D turn  |  Ball: W A S D dribble, X release  |  Shift faster\n"
            "Q call for a pass (as it comes: hold a direction = first touch, Q = first-time pass, Space = first-time shot)  |  E ball at feet\n"
            "Space hold to charge a shot, C an air shot, W A S D aim, release to kick  |  R stand up  |  P pause";
        constexpr const char* RONDO_HELP =
            "Rondo: pass it round the triangle, keep it from the robot in the middle. The keys play whoever the ball is going to\n"
            "Space pass (aimed at the open teammate, tap = right power, hold = harder; W A S D re-aim)  |  As it comes: Q pass first time, Space hit it first time, hold a direction = first touch\n"
            "W A S D dribble / walk  |  X release  |  E ball at feet  |  R new round  |  P pause";
        constexpr const char* DEFEND_HELP =
            "Rondo, defending: you're the robot in the middle. The three pass it round and dribble away from you; touch the ball to win it\n"
            "W walk (Shift 2 m/s)  |  S back  |  A D turn  |  R new round  |  P pause";
        constexpr const char* SOLO_HELP =
            "Rondo vs solo eyes: no communication. Each of the three knows only what its own camera sees (its own coloured track)\n"
            "and decides from that alone. Come at the one with the ball from outside its view. Touch the ball to win it\n"
            "W walk (Shift 2 m/s)  |  S back  |  A D turn  |  R new round  |  P pause";
        constexpr const char* EKF_HELP =
            "Rondo vs EKF: the three only know where you are from what their cameras see, fused in one shared EKF (orange).\n"
            "Hide behind one of them, or stay out of their views, and they pass into you. Touch the ball to win it\n"
            "W walk (Shift 2 m/s)  |  S back  |  A D turn  |  R new round  |  P pause";

        constexpr double PI = 3.14159265358979323846;

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
        std::unique_ptr<SkillPolicy> walk, receive, dribble, kick, kick_moving, kick_air, first_touch, first_pass;
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
        policies_->kick_air    = load("kick_air", false);
        policies_->first_touch = load("first_touch", false);
        policies_->first_pass  = load("first_pass", false);

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
        logic.air_enabled = policies_->kick_air != nullptr;
        const auto touch                = cfg["first_touch"];
        logic.first_touch_enabled       = policies_->first_touch != nullptr;
        logic.first_touch_speed         = touch["speed"].as<double>(logic.first_touch_speed);
        logic.first_touch_fast_speed    = touch["fast_speed"].as<double>(logic.first_touch_fast_speed);
        logic.first_touch_window        = touch["window"].as<double>(logic.first_touch_window);
        logic.first_touch_timeout       = touch["timeout"].as<double>(logic.first_touch_timeout);
        const auto first_pass           = cfg["first_pass"];
        logic.first_pass_enabled        = policies_->first_pass != nullptr;
        logic.first_pass_speed          = first_pass["speed"].as<double>(logic.first_pass_speed);
        logic.first_pass_fast_speed     = first_pass["fast_speed"].as<double>(logic.first_pass_fast_speed);
        logic.first_shot_speed          = first_pass["shot_speed"].as<double>(logic.first_shot_speed);
        logic.kick_cone  = shoot["cone_deg"].as<double>(logic.kick_cone * 180.0 / 3.14159265358979323846) * 3.14159265358979323846 / 180.0;
        const auto trap  = cfg["receive"];
        logic.settle_time       = trap["settle_time"].as<double>(logic.settle_time);
        logic.takeover_distance = trap["takeover_distance"].as<double>(logic.takeover_distance);
        logic.takeover_speed    = trap["takeover_speed"].as<double>(logic.takeover_speed);
        const auto rondo        = cfg["rondo"];
        logic.pass_arrival_speed = rondo["arrival_speed"].as<double>(logic.pass_arrival_speed);
        logic.rolling_decel      = cfg["pass"]["rolling_decel"].as<double>(logic.rolling_decel);
        logic.target_snap        = rondo["snap_deg"].as<double>(10.0) * PI / 180.0;
        logic.target_key_snap    = rondo["key_snap_deg"].as<double>(30.0) * PI / 180.0;
        logic_config_            = logic;
        if (rondo["centre"] && rondo["centre"].size() == 2) {
            rondo_.centre = {rondo["centre"][0].as<double>(), rondo["centre"][1].as<double>()};
        }
        rondo_.side           = rondo["side"].as<double>(rondo_.side);
        rondo_.defender_speed = rondo["defender_speed"].as<double>(rondo_.defender_speed);
        rondo_.defender_turn  = rondo["defender_turn_rate"].as<double>(rondo_.defender_turn);
        rondo_.defender_delay = rondo["defender_delay"].as<double>(rondo_.defender_delay);
        rondo_.defender_lead  = rondo["defender_lead"].as<double>(rondo_.defender_lead);
        rondo_.defender_accel = rondo["defender_accel"].as<double>(rondo_.defender_accel);
        rondo_.defender_close_speed = rondo["defender_close_speed"].as<double>(rondo_.defender_close_speed);
        rondo_.support_speed     = rondo["support_speed"].as<double>(rondo_.support_speed);
        rondo_.support_tolerance = rondo["support_tolerance"].as<double>(rondo_.support_tolerance);
        rondo_.lane_clear        = rondo["lane_clear"].as<double>(rondo_.lane_clear);
        rondo_.support_shift     = rondo["support_shift"].as<double>(rondo_.support_shift);
        rondo_.ball_ahead     = rondo["ball_ahead"].as<double>(rondo_.ball_ahead);
        rondo_.out_margin     = rondo["out_margin"].as<double>(rondo_.out_margin);
        rondo_.next_round     = rondo["next_round"].as<double>(rondo_.next_round);
        rondo_.receive_reach  = rondo["receive_reach"].as<double>(rondo_.receive_reach);
        rondo_.chase_speed    = rondo["chase_speed"].as<double>(rondo_.chase_speed);
        if (rondo["ai_hold"] && rondo["ai_hold"].size() == 2) {
            rondo_.ai_hold = {rondo["ai_hold"][0].as<double>(), rondo["ai_hold"][1].as<double>()};
        }
        rondo_.ai_pressure   = rondo["ai_pressure"].as<double>(rondo_.ai_pressure);
        rondo_.ai_lane_open  = rondo["ai_lane_open"].as<double>(rondo_.ai_lane_open);
        rondo_.ai_first_time = rondo["ai_first_time"].as<double>(rondo_.ai_first_time);
        rondo_.forced_win_passes = rondo["forced_win_passes"].as<int>(rondo_.forced_win_passes);
        const auto ekf = cfg["rondo_ekf"];
        tracking_.side              = ekf["side"].as<double>(tracking_.side);
        tracking_.rate              = ekf["rate"].as<double>(tracking_.rate);
        tracking_.range             = ekf["range"].as<double>(tracking_.range);
        tracking_.sigma_bearing     = ekf["sigma_bearing_deg"].as<double>(tracking_.sigma_bearing * 180.0 / PI) * PI / 180.0;
        tracking_.sigma_range       = ekf["sigma_range"].as<double>(tracking_.sigma_range);
        tracking_.sigma_range_per_m = ekf["sigma_range_per_m"].as<double>(tracking_.sigma_range_per_m);
        tracking_.miss              = ekf["miss"].as<double>(tracking_.miss);
        tracking_.lost_sigma        = ekf["lost_sigma"].as<double>(tracking_.lost_sigma);
        tracking_.aspect            = ekf["aspect"].as<double>(tracking_.aspect);
        tracking_.truth_marker      = ekf["truth_marker"].as<bool>(tracking_.truth_marker);
        skill::EkfConfig ekf_config;
        ekf_config.accel_sigma      = ekf["accel_sigma"].as<double>(ekf_config.accel_sigma);
        ekf_config.velocity_tau     = ekf["velocity_tau"].as<double>(ekf_config.velocity_tau);
        ekf_config.init_speed_sigma = ekf["init_speed_sigma"].as<double>(ekf_config.init_speed_sigma);
        ekf_config.gate             = ekf["gate"].as<double>(ekf_config.gate);
        ekf_config.reinit_after     = ekf["reinit_after"].as<int>(ekf_config.reinit_after);
        ekf_                        = skill::DefenderEkf(ekf_config);
        ekf_config_                 = ekf_config;

        const std::string drill = cfg["drill"].as<std::string>("free");
        if (drill != "free" && drill != "rondo" && drill != "rondo_defend" && drill != "rondo_ekf" && drill != "rondo_solo") {
            throw std::runtime_error("skills.yaml: drill is free, rondo, rondo_defend, rondo_ekf or rondo_solo");
        }
        drill_wanted_ = drill == "rondo"          ? Drill::RONDO
                        : drill == "rondo_defend" ? Drill::RONDO_DEFEND
                        : drill == "rondo_ekf"    ? Drill::RONDO_EKF
                        : drill == "rondo_solo"   ? Drill::RONDO_SOLO
                                                  : Drill::FREE;
        if (const auto park = cfg["park"]; park && park.IsSequence()) {
            rondo_.park.clear();
            for (const auto& spot : park) {
                rondo_.park.push_back({spot[0].as<double>(), spot[1].as<double>()});
            }
        }

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

        const auto contact     = cfg["ball_contact"];
        match_ball_contact_    = contact["match_mjlab"].as<bool>(match_ball_contact_);
        ball_contact_friction_ = contact["friction"].as<double>(ball_contact_friction_);
        if (contact["solref"] && contact["solref"].size() == 2) {
            ball_contact_solref_ = {contact["solref"][0].as<double>(), contact["solref"][1].as<double>()};
        }

        const auto camera = cfg["camera"];
        if (camera["offset"] && camera["offset"].size() == 3) {
            camera_offset_ = {camera["offset"][0].as<double>(),
                              camera["offset"][1].as<double>(),
                              camera["offset"][2].as<double>()};
        }
        const double deg = 3.14159265358979323846 / 180.0;
        camera_hfov_     = camera["hfov_deg"].as<double>(106.2) * deg;
        camera_vfov_     = camera["vfov_deg"].as<double>(95.0) * deg;
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
            const mjModel* m = handles.model;
            ball_body_       = mj_name2id(m, mjOBJ_BODY, "ball");
            ball_geom_       = mj_name2id(m, mjOBJ_GEOM, "ball");
            if (!freebody::valid_free_body_geom(m, ball_body_, ball_geom_)) {
                log<NUClear::LogLevel::WARN>("SkillPlayer: this field has no ball, so there is nothing to play with");
                ball_body_ = ball_geom_ = -1;
            }
            goals_.clear();
            for (const char* name : {"goal_pos_x", "goal_neg_x"}) {
                const int body = mj_name2id(m, mjOBJ_BODY, name);
                const int post = mj_name2id(m, mjOBJ_GEOM, (std::string(name) + "_post1").c_str());
                const int bar  = mj_name2id(m, mjOBJ_GEOM, (std::string(name) + "_bar").c_str());
                if (body < 0 || post < 0 || bar < 0) {
                    continue;
                }
                const mjtNum* m_pos = m->body_pos + 3 * body;
                const double radius = m->geom_size[3 * post];
                Goal goal;
                goal.x          = m_pos[0];
                goal.y          = m_pos[1];
                goal.half_width = std::abs(m->geom_pos[3 * post + 1]) - radius;
                goal.height     = m->geom_pos[3 * bar + 2] - radius;
                goal.dir        = goal.x >= 0.0 ? 1 : -1;
                goals_.push_back(goal);
            }
            rtf_          = handles.measured_rtf;
            placed_robot_ = handles.placed_robot;

            // One agent per robot in the scene: the first is free play's, the rest stand aside
            // until a drill wants them.
            agents_.clear();
            for (const auto& robot : config::robot_roster()) {
                const std::string prefix = robot_prefix(robot.model_index);
                Agent agent;
                agent.robot_id  = robot.robot_id;
                agent.trunk     = prefix + "Trunk";
                agent.map       = ModelMap::build(m, prefix);
                agent.head_body = mj_name2id(m, mjOBJ_BODY, (prefix + "Head_2").c_str());
                if (agent.head_body < 0) {
                    throw std::runtime_error("SkillPlayer: the model has no " + prefix + "Head_2 body to carry the camera");
                }
                // Its head camera: the one on a body of this robot, "head" by preference.
                for (int c = 0; c < m->ncam; ++c) {
                    if (m->body_rootid[m->cam_bodyid[c]] == agent.map.root_body_id) {
                        const char* name = mj_id2name(m, mjOBJ_CAMERA, c);
                        if (agent.camera < 0 || (name != nullptr && std::string(name).find("head") != std::string::npos)) {
                            agent.camera = c;
                        }
                    }
                }
                agent.ekf          = skill::DefenderEkf(ekf_config_);
                agent.logic        = skill::Logic(logic_config_);
                agent.logged_skill = agent.logic.skill();
                agent.role         = agents_.empty() ? Role::PLAYER : Role::PARKED;
                agents_.push_back(std::move(agent));
            }
            if (agents_.empty()) {
                throw std::runtime_error("SkillPlayer: no robots in the roster");
            }

            // Where a placed robot stands: the "ready" keyframe's pose, as the scene spawns it.
            const ModelMap& first = agents_.front().map;
            ready_joints_         = policies_->receive->default_pose;
            if (const int key = mj_name2id(m, mjOBJ_KEY, "ready"); key >= 0) {
                const mjtNum* qpos = m->key_qpos + static_cast<std::size_t>(key) * m->nq;
                ready_z_           = qpos[first.root_qpos_adr + 2];
                for (std::size_t i = 0; i < JOINT_COUNT; ++i) {
                    ready_joints_[i] = qpos[first.qpos_adr[i]];
                }
            }

            std::lock_guard<std::mutex> lock(*handles.mutex);
            // Free play's starting spots for the first robot and the ball, as the scene put them.
            const mjtNum* root = handles.data->qpos + first.root_qpos_adr;
            start_pose_        = {root[0], root[1], quat_yaw(root + 3)};
            if (ball_body_ >= 0) {
                const int joint = m->body_jntadr[ball_body_];
                const mjtNum* q = handles.data->qpos + m->jnt_qposadr[joint];
                ball_start_     = {q[0] + m->geom_pos[3 * ball_geom_], q[1] + m->geom_pos[3 * ball_geom_ + 1]};
            }
            // Match the foot-ball contact the policies were trained on (mjlab's explicit foot-ball pairs,
            // corrected 2026-11-18): the ball has priority in its contacts with the robot, so its own
            // friction and solref decide them. Ball-floor contacts come from the scene's explicit
            // <pair> and are untouched.
            if (match_ball_contact_ && ball_geom_ >= 0) {
                auto* mm = const_cast<mjModel*>(m);
                mm->geom_friction[3 * ball_geom_]     = ball_contact_friction_;
                mm->geom_friction[3 * ball_geom_ + 1] = 0.005;
                mm->geom_friction[3 * ball_geom_ + 2] = 1e-4;
                mm->geom_solref[2 * ball_geom_]       = ball_contact_solref_[0];
                mm->geom_solref[2 * ball_geom_ + 1]   = ball_contact_solref_[1];
                log<NUClear::LogLevel::INFO>("SkillPlayer: ball-robot contact set to friction",
                                             ball_contact_friction_,
                                             "solref",
                                             ball_contact_solref_[0],
                                             ball_contact_solref_[1],
                                             "(as mjlab trained the policies)");
            }
            data_  = handles.data;
            mutex_ = handles.mutex;
            model_ = m;
            log<NUClear::LogLevel::INFO>("SkillPlayer ready with", agents_.size(), "robot(s) --", KEY_HELP);
        });

        on<Trigger<RobotStatesUpdate>, Sync<SkillPlayer>>().then([this](const RobotStatesUpdate& states) { tick(states); });

        on<Trigger<message::ViewDirection>, Sync<SkillPlayer>>().then(
            [this](const message::ViewDirection& view) { view_yaw_ = view.yaw; });

        on<Trigger<KeyEvent>, Sync<SkillPlayer>>().then([this](const KeyEvent& event) { key(event.key, event.down); });

        on<Trigger<message::SkillDrill>, Sync<SkillPlayer>>().then([this](const message::SkillDrill& request) {
            if (request.name == "rondo" || request.name == "rondo_defend" || request.name == "rondo_ekf" || request.name == "rondo_solo"
                || request.name == "free") {
                drill_wanted_  = request.name == "rondo"          ? Drill::RONDO
                                 : request.name == "rondo_defend" ? Drill::RONDO_DEFEND
                                 : request.name == "rondo_ekf"    ? Drill::RONDO_EKF
                                 : request.name == "rondo_solo"   ? Drill::RONDO_SOLO
                                                                  : Drill::FREE;
                drill_pending_ = true;
            }
            else {
                log<NUClear::LogLevel::WARN>("SkillPlayer: no drill called", request.name);
            }
        });
    }

    void SkillPlayer::key(int code, bool down) {
        switch (code) {
            case 'W':
                keys_.forward   = down;
                masked_.forward = masked_.forward && down;
                break;
            case 'S':
                keys_.back   = down;
                masked_.back = masked_.back && down;
                break;
            case 'A':
                keys_.left   = down;
                masked_.left = masked_.left && down;
                break;
            case 'D':
                keys_.right   = down;
                masked_.right = masked_.right && down;
                break;
            case message::KEY_LEFT_SHIFT:
            case message::KEY_RIGHT_SHIFT: keys_.fast = down; break;
            case 'Q': pass_pressed_ = pass_pressed_ || down; break;
            case 'E': drop_pressed_ = drop_pressed_ || down; break;
            case 'C': (down ? air_down_ : air_up_) = true; break;
            case 'X': release_pressed_ = release_pressed_ || down; break;
            case message::KEY_SPACE:
                (down ? shoot_down_ : shoot_up_) = true;
                keys_.shoot                      = down;
                break;
            case 'R': reset_pressed_ = reset_pressed_ || down; break;
            default: break;
        }
    }

    // The keys as the robot being played should see them: direction keys still held from before
    // the control last switched don't count until they are let go.
    skill::Keys SkillPlayer::held_keys() const {
        skill::Keys keys = keys_;
        keys.forward     = keys.forward && !masked_.forward;
        keys.back        = keys.back && !masked_.back;
        keys.left        = keys.left && !masked_.left;
        keys.right       = keys.right && !masked_.right;
        return keys;
    }

    void SkillPlayer::note(const std::string& text, double sim_time, double seconds) {
        note_       = text;
        note_until_ = sim_time + seconds;
    }

    void SkillPlayer::restart(Agent& agent) {
        agent.phase            = Phase::WAITING;
        agent.mode_asked       = -1.0;
        agent.history_primed   = false;
        agent.last_action      = {};
        agent.walk_last_action = {};
        agent.have_target      = false;
        agent.running          = nullptr;
        agent.last_ball        = {0.5, 0.0, -0.8, 0.0};
        agent.logic.reset();
        agent.decision     = {};
        agent.logged_skill = agent.logic.skill();
        if (&agent == &agents_[active_]) {
            pass_pressed_ = drop_pressed_ = shoot_down_ = shoot_up_ = release_pressed_ = air_down_ = air_up_ = false;
        }
    }

    SkillPlayer::Sample SkillPlayer::read_state(Agent& agent) {
        Sample s;
        std::lock_guard<std::mutex> lock(*mutex_);
        const ModelMap& map = agent.map;
        const mjtNum* root  = data_->qpos + map.root_qpos_adr;  // xyz + wxyz
        const mjtNum* quat  = root + 3;

        // Trunk frame, as mjlab's root_link_ang_vel_b and projected_gravity_b: a free joint's
        // angular velocity is already in the body frame.
        const mjtNum* w = data_->qvel + map.root_dof_adr + 3;
        s.gyro          = {w[0], w[1], w[2]};
        mjtNum inverse[4];
        mju_negQuat(inverse, quat);
        const mjtNum down[3] = {0.0, 0.0, -1.0};
        mjtNum gravity[3];
        mju_rotVecQuat(gravity, down, inverse);
        s.gravity = {gravity[0], gravity[1], gravity[2]};

        for (std::size_t i = 0; i < JOINT_COUNT; ++i) {
            s.q[i]  = data_->qpos[map.qpos_adr[i]];
            s.dq[i] = data_->qvel[map.dof_adr[i]];
        }

        s.world.robot_x  = root[0];
        s.world.robot_y  = root[1];
        s.world.yaw      = quat_yaw(quat);
        s.world.view_yaw = view_yaw_;

        s.ball_camera    = agent.last_ball;
        s.ball_camera[3] = 0.0;
        if (ball_body_ >= 0) {
            const auto ball    = freebody::geom_centre_state(model_, data_, ball_body_, ball_geom_);
            s.world.ball_valid = true;
            s.world.ball_x     = ball.position[0];
            s.world.ball_y     = ball.position[1];
            s.world.ball_z     = ball.position[2];
            s.world.ball_speed = std::hypot(ball.lin_vel[0], ball.lin_vel[1], ball.lin_vel[2]);
            s.world.ball_vx    = ball.lin_vel[0];
            s.world.ball_vy    = ball.lin_vel[1];
            // A first touch: a contact between the ball and a body of this robot right now, or (since a
            // touch can begin and end between two 20 ms control steps) a jump in the ball's velocity
            // near the robot that rolling friction can't explain.
            for (int i = 0; i < data_->ncon && !s.ball_contact; ++i) {
                const mjContact& c = data_->contact[i];
                const int other    = c.geom[0] == ball_geom_ ? c.geom[1] : c.geom[1] == ball_geom_ ? c.geom[0] : -1;
                s.ball_contact     = other >= 0 && model_->body_rootid[model_->geom_bodyid[other]] == map.root_body_id;
            }
            s.world.ball_touched = s.ball_contact;
            const double jump =
                std::hypot(ball.lin_vel[0] - agent.last_ball_vel[0], ball.lin_vel[1] - agent.last_ball_vel[1]);
            if (jump > touch_velocity_jump_ && s.world.ball_distance() < 0.6) {
                s.world.ball_touched = true;
                // Which robot was it? Every robot near the ball sees the same jump; the one that
                // touched it is the one still (nearly) against it.
                for (int g = 0; g < model_->ngeom; ++g) {
                    if (model_->body_rootid[model_->geom_bodyid[g]] == map.root_body_id) {
                        s.ball_gap = std::min(s.ball_gap, static_cast<double>(mj_geomDistance(model_, data_, ball_geom_, g, 0.3, nullptr)));
                    }
                }
            }
            agent.last_ball_vel = {ball.lin_vel[0], ball.lin_vel[1]};
            s.world.ball_radius = model_->geom_size[3 * ball_geom_];

            // The ball as the head camera would report it: the Head_2 frame (x forward, y left,
            // z up) moved to the front of the head. Out of view, the last sighting is repeated
            // with the flag down.
            const mjtNum* head_pos = data_->xpos + 3 * agent.head_body;
            const mjtNum* head_mat = data_->xmat + 9 * agent.head_body;
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
    std::string SkillPlayer::launch_pass(Agent& agent) {
        const auto& w  = agent.sample.world;
        const double c = std::cos(w.yaw), s = std::sin(w.yaw);
        double sx = 0.0, sy = 0.0;  // ball start, the robot's yaw frame
        if (pass_from_ball_) {
            const auto ball = w.ball_local();
            sx              = ball[0];
            sy              = ball[1];
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
        pass_angle_     = w.yaw + std::atan2(vy, vx);  // the way it rolls, world frame

        auto command      = std::make_unique<message::BallCommand>();
        command->frame    = message::BallCommand::Frame::WORLD;
        command->position = {w.robot_x + c * sx - s * sy, w.robot_y + s * sx + c * sy, -1.0};  // z < 0: on the floor
        command->velocity = {c * vx - s * vy, s * vx + c * vy, 0.0};
        command->rolling  = true;
        emit(command);

        const double distance = std::hypot(sx, sy);
        const double bearing  = std::atan2(sy, sx);
        log<NUClear::LogLevel::INFO>("SkillPlayer: pass to robot",
                                     agent.robot_id,
                                     "from",
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
                if (decision.air && policies_->kick_air) {
                    return *policies_->kick_air;
                }
                return decision.moving_kick && policies_->kick_moving ? *policies_->kick_moving : *policies_->kick;
            case Skill::FIRST_TOUCH: return policies_->first_touch ? *policies_->first_touch : *policies_->receive;
            case Skill::FIRST_PASS: return policies_->first_pass ? *policies_->first_pass : *policies_->receive;
            case Skill::RECEIVE: break;
        }
        return *policies_->receive;
    }

    void SkillPlayer::send_targets(Agent& agent,
                                   const std::array<double, JOINT_COUNT>& q,
                                   const skill::Gains& gains,
                                   double delay) {
        auto command      = std::make_unique<message::LowCmdMessage>();
        command->cmd_type = 1;  // serial
        command->robot_id = agent.robot_id;
        command->motors.resize(JOINT_COUNT);
        for (std::size_t i = 0; i < JOINT_COUNT; ++i) {
            command->motors[i].q  = static_cast<float>(q[i]);
            command->motors[i].kp = static_cast<float>(gains.kp[i]);
            command->motors[i].kd = static_cast<float>(gains.kd[i]);
        }
        agent.last_target = q;
        agent.have_target = true;
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

    void SkillPlayer::tick(const RobotStatesUpdate& states) {
        if (model_ == nullptr || states.robots.empty() || agents_.empty()) {
            return;
        }
        // Snapshots keep arriving while the sim is paused; only a new physics step is a control step.
        const auto& first       = states.robots.front();
        const bool world_reset  = have_state_ && first.step_count < last_step_;
        if (have_state_ && !world_reset && first.step_count == last_step_) {
            return;
        }
        if (world_reset) {
            // The whole world was reset, clock included: the script starts over and the drill is
            // set up again (the reset put every robot back where the scene spawns it).
            script_next_   = 0;
            drill_wanted_  = drill_pending_ ? drill_wanted_ : drill_;
            drill_pending_ = true;
        }
        have_state_ = true;
        last_step_  = first.step_count;
        sim_time_   = first.sim_time;

        while (script_next_ < script_.size() && script_[script_next_].time <= sim_time_) {
            key(script_[script_next_].key, script_[script_next_].down);
            ++script_next_;
        }

        if (reset_pressed_) {
            reset_pressed_ = false;
            if (rondo()) {
                drill_wanted_  = drill_;
                drill_pending_ = true;
            }
            else {
                emit(std::make_unique<message::RobotResetRequest>(message::RobotResetRequest{agents_[active_].robot_id}));
            }
        }
        if (drill_pending_) {
            drill_pending_ = false;
            start_drill(drill_wanted_, sim_time_);
        }

        std::vector<const SimStateUpdate*> robot_states(agents_.size(), nullptr);
        for (std::size_t i = 0; i < agents_.size(); ++i) {
            Agent& agent = agents_[i];
            for (const auto& robot : states.robots) {
                if (robot.identity.robot_id == agent.robot_id) {
                    robot_states[i] = &robot;
                }
            }
            if (robot_states[i] == nullptr) {
                continue;
            }
            if (world_reset || (agent.have_state && robot_states[i]->reset_count != agent.reset_seen)) {
                restart(agent);
            }
            agent.have_state = true;
            agent.reset_seen = robot_states[i]->reset_count;
            agent.sample     = read_state(agent);
            agent.last_ball  = agent.sample.ball_camera;
        }

        if (drill_ == Drill::FREE) {
            check_goal(agents_[active_].sample, sim_time_);
        }
        else {
            update_tracking(sim_time_);
            update_rondo(sim_time_);
        }
        if (!tracking() && track_sent_) {
            track_sent_ = false;
            emit(std::make_unique<message::DefenderTrack>());  // inactive: the viewer drops the camera views
        }
        for (std::size_t i = 0; i < agents_.size(); ++i) {
            if (robot_states[i] != nullptr) {
                step_agent(agents_[i], *robot_states[i]);
            }
        }
        publish_status(sim_time_, agents_[active_].decision.aiming);
    }

    void SkillPlayer::step_agent(Agent& agent, const SimStateUpdate& state) {
        const double t = state.sim_time;
        // The pose and gains the robot eases into at start-up: the policy it starts with.
        const bool walk_first = policies_->walk && (agent.role == Role::DEFENDER || agent.logic.skill() == Skill::WALK);
        const SkillPolicy& base = walk_first ? *policies_->walk : *policies_->receive;

        // The servo targets are only followed in CUSTOM mode; ask until the robot is in it.
        if (state.mode != booster::CUSTOM) {
            if (agent.phase != Phase::WAITING) {
                restart(agent);
            }
            if (agent.mode_asked < 0.0 || t - agent.mode_asked > 0.25 || t < agent.mode_asked) {
                agent.mode_asked = t;
                emit(std::make_unique<message::ModeChangeRequest>(message::ModeChangeRequest{booster::CUSTOM, agent.robot_id}));
            }
            return;
        }

        if (agent.phase == Phase::WAITING) {
            agent.phase       = Phase::BLENDING;
            agent.phase_start = t;
            agent.blend_from  = agent.sample.q;
        }

        // (Not in the first moments after a reset, while the fall detector catches up.)
        const bool settled = agent.phase == Phase::PLAYING || t - agent.phase_start > 0.3;
        if (agent.phase != Phase::FALLEN && settled && state.fall_state == booster::HAS_FALLEN) {
            agent.phase       = Phase::FALLEN;
            agent.phase_start = t;
            log<NUClear::LogLevel::WARN>("SkillPlayer: robot", agent.robot_id, "fell over (no policy gets it up; it is stood back up shortly)");
        }

        switch (agent.phase) {
            case Phase::WAITING: break;
            case Phase::BLENDING: {
                // Ease from the pose the robot spawned in to the pose the policies are centred on.
                const double x = blend_time_ > 0.0 ? (t - agent.phase_start) / blend_time_ : 1.0;
                const double s = smoothstep(x);
                std::array<double, JOINT_COUNT> target{};
                for (std::size_t i = 0; i < JOINT_COUNT; ++i) {
                    target[i] = agent.blend_from[i] + s * (base.default_pose[i] - agent.blend_from[i]);
                }
                send_targets(agent, target, base.gains, 0.0);
                if (x >= 1.0) {
                    agent.phase         = Phase::PLAYING;
                    agent.playing_since = t;
                    if (agent.start_with_ball) {
                        agent.start_with_ball = false;
                        agent.logic.on_drop();  // the ball was put at its feet: receive it
                    }
                    if (drill_ == Drill::FREE && agent.role == Role::PLAYER) {
                        log<NUClear::LogLevel::INFO>(agent.logic.skill() == Skill::WALK
                                                         ? "SkillPlayer: policies in control, standing in walk (W walks, Q passes)"
                                                         : "SkillPlayer: policies in control, waiting for a pass (Q)");
                    }
                }
                break;
            }
            case Phase::PLAYING:
                if (&agent == &agents_[active_] && agent.role != Role::DEFENDER) {
                    apply_inputs(agent, t);
                }
                run_policy(agent, state);
                break;
            case Phase::FALLEN: {
                // Go limp rather than let a policy thrash on the ground.
                skill::Gains limp;
                limp.kd.fill(1.0);
                send_targets(agent, agent.sample.q, limp, 0.0);
                if (fall_reset_after_ > 0.0 && t - agent.phase_start > fall_reset_after_) {
                    agent.phase_start = t;
                    if (agent.homed && placed_robot_) {
                        std::lock_guard<std::mutex> lock(*mutex_);
                        place_locked(agent, agent.home[0], agent.home[1], agent.home[2]);
                        mj_forward(model_, data_);
                    }
                    else {
                        emit(std::make_unique<message::RobotResetRequest>(message::RobotResetRequest{agent.robot_id}));
                    }
                }
                break;
            }
        }
    }

    // The one-off keys (Q, E, Space, C, X) for the robot being played.
    void SkillPlayer::apply_inputs(Agent& agent, double sim_time) {
        skill::Logic& logic = agent.logic;
        const auto& world   = agent.sample.world;
        const skill::Keys keys = held_keys();
        if (pass_pressed_ && logic.pass_incoming()) {
            // Q with a pass rolling in: pass it on first time.
            if (!logic.on_first_pass(keys, world)) {
                note("no first-time pass policy", sim_time);
            }
        }
        else if (pass_pressed_) {
            if (!world.ball_valid) {
                note("no ball on this field", sim_time);
            }
            else if (pass_from_ball_ && world.ball_distance() < pass_min_distance_) {
                note("the ball is already at the robot's feet", sim_time);
            }
            else {
                const std::string warning = launch_pass(agent);
                logic.on_pass(pass_angle_);
                if (!warning.empty()) {
                    note(warning, sim_time, 3.0);
                }
            }
        }
        if (drop_pressed_ && world.ball_valid) {
            logic.on_drop();  // receive it, from walk too
            // A ball at rest at the robot's feet, to dribble or shoot without a pass first.
            auto command        = std::make_unique<message::BallCommand>();
            command->frame      = message::BallCommand::Frame::ROBOT;
            command->position   = {drop_distance_, 0.0, -1.0};
            command->robot_body = agent.trunk;
            emit(command);
        }
        if ((shoot_down_ || air_down_) && logic.pass_incoming()) {
            // The shoot key with a pass rolling in: shoot it first time. There is no first-time air
            // shot, so C does the same on the ground.
            if (!logic.on_first_shot(keys, world)) {
                note("no first-time pass policy to shoot it first time with", sim_time);
            }
            else if (air_down_) {
                note("no first-time air shot: shooting it first time on the ground", sim_time);
            }
            shoot_down_ = air_down_ = false;
        }
        if (shoot_down_ && !logic.on_shoot_press(world)) {
            note(logic.skill() == Skill::WALK ? "walk up to the ball first" : "the ball is not in front of the robot", sim_time);
        }
        if (shoot_up_) {
            logic.on_shoot_release(world);
        }
        if (air_down_ && !logic.on_shoot_press(world, true)) {
            note(!logic.config().air_enabled    ? "no air-shot policy"
                 : logic.skill() == Skill::WALK ? "walk up to the ball first"
                                                : "the ball is not in front of the robot",
                 sim_time);
        }
        if (air_up_) {
            logic.on_shoot_release(world, true);
        }
        if (release_pressed_) {
            logic.on_release();
        }
        pass_pressed_ = drop_pressed_ = shoot_down_ = shoot_up_ = release_pressed_ = air_down_ = air_up_ = false;
    }

    void SkillPlayer::run_policy(Agent& agent, const SimStateUpdate& state) {
        const double t      = state.sim_time;
        const std::size_t index = static_cast<std::size_t>(&agent - agents_.data());
        const bool played   = index == active_;
        const auto was      = agent.decision.twist;
        if (agent.role == Role::DEFENDER) {
            if (played) {
                // rondo_defend: walked with the keys, never a ball skill (touching the ball wins it).
                agent.decision       = skill::Decision{};
                agent.decision.skill = Skill::WALK;
                agent.decision.twist = agent.logic.walk_twist(held_keys());
            }
            else {
                agent.decision = defender_decision(agent, t);
            }
        }
        else {
            const skill::Keys keys = played ? held_keys()
                                     : defending() && agent.role == Role::TEAMMATE && index == owner_ ? computer_keys(agent, t)
                                                                                                      : skill::Keys{};
            agent.decision = agent.logic.tick(keys, agent.sample.world, CONTROL_DT);
        }
        if (rondo() && agent.role == Role::TEAMMATE && !played && agent.decision.skill == Skill::WALK) {
            // Off the ball: get to where it can be passed to (support_twist), standing still between
            // rounds. The computer's teammate with the ball, if it has lost it, walks to it.
            const std::array<double, 3> want = round_over_at_ >= 0.0               ? std::array<double, 3>{}
                                               : defending() && index == owner_ ? chase_twist(agent)
                                                                                : support_twist(agent);
            agent.decision.twist = ease_twist(was, want);
        }
        const skill::Decision& decision = agent.decision;
        if (decision.skill != agent.logged_skill) {
            log<NUClear::LogLevel::INFO>("SkillPlayer: t =",
                                         t,
                                         "robot",
                                         agent.robot_id,
                                         skill::name(agent.logged_skill),
                                         "->",
                                         skill::name(decision.skill),
                                         decision.skill == Skill::KICK && decision.moving_kick ? "(ball moving)" : "");
            agent.logged_skill = decision.skill;
        }
        SkillPolicy& policy = policy_for(decision);
        const bool walking  = decision.skill == Skill::WALK && policies_->walk != nullptr;
        const bool was_walk = agent.running != nullptr && agent.running == policies_->walk.get();
        if (agent.running != nullptr && walking != was_walk && agent.have_target) {
            // Switching between walk and a ball skill changes gains, default pose, action scale and
            // input layout in the same step (spec section 4). The new policy starts from the current
            // state, and its "previous action" is the joint target last sent, in its own convention.
            std::array<float, JOINT_COUNT>& previous = walking ? agent.walk_last_action : agent.last_action;
            for (std::size_t i = 0; i < JOINT_COUNT; ++i) {
                previous[i] = static_cast<float>((agent.last_target[i] - policy.default_pose[i]) / policy.action_scale[i]);
            }
            agent.history_primed = false;
        }
        agent.running = &policy;

        const auto previous_action = walking ? agent.walk_last_action : agent.last_action;
        const auto action          = walking ? run_walk(agent, policy) : run_ball_skill(agent, policy);
        for (std::size_t i = 10; i < JOINT_COUNT; ++i) {  // legs
            agent.jitter_sum += std::abs(policy.action_scale[i] * static_cast<double>(action[i] - previous_action[i]));
        }
        ++agent.jitter_count;
        std::array<double, JOINT_COUNT> target{};
        for (std::size_t i = 0; i < JOINT_COUNT; ++i) {
            target[i] = policy.default_pose[i] + policy.action_scale[i] * static_cast<double>(action[i]);
        }
        send_targets(agent, target, policy.gains, walking ? walk_action_delay_ : action_delay_);

        // One SKILL line a second for each robot in play (free play: just the one).
        if (agent.role == Role::PARKED) {
            return;
        }
        if (t < agent.log_next && t >= agent.log_next - 1.0 - log_period_) {
            return;
        }
        agent.log_next = t + log_period_;
        const Sample& sample = agent.sample;
        char line[400];
        std::snprintf(line,
                      sizeof(line),
                      "SKILL t=%.1f robot=%d %s cmd=(%.2f, %.2f) twist=(%.2f, %.2f, %.2f) robot=(%.2f, %.2f, yaw %.2f) z=%.2f "
                      "ball=(%.2f, %.2f, %.2f) speed=%.2f seen=%d cam=(%.2f, %.2f, %.2f) head=(%.2f, %.2f) local_cmd=(%.2f, %.2f) "
                      "leg_target_step=%.4f",
                      t,
                      agent.robot_id,
                      skill::name(decision.skill),
                      decision.command_world[0],
                      decision.command_world[1],
                      decision.twist[0],
                      decision.twist[1],
                      decision.twist[2],
                      sample.world.robot_x,
                      sample.world.robot_y,
                      sample.world.yaw,
                      state.base.z,
                      sample.world.ball_x,
                      sample.world.ball_y,
                      sample.world.ball_z,
                      sample.world.ball_speed,
                      static_cast<int>(sample.ball_camera[3]),
                      sample.ball_camera[0],
                      sample.ball_camera[1],
                      sample.ball_camera[2],
                      sample.q[JointIndexK1::HeadYaw],
                      sample.q[JointIndexK1::HeadPitch],
                      decision.command_local[0],
                      decision.command_local[1],
                      agent.jitter_count ? agent.jitter_sum / (12.0 * agent.jitter_count) : 0.0);
        agent.jitter_sum   = 0.0;
        agent.jitter_count = 0;
        log<NUClear::LogLevel::INFO>(line);
    }

    std::array<float, JOINT_COUNT> SkillPlayer::run_ball_skill(Agent& agent, SkillPolicy& policy) {
        const Sample& sample = agent.sample;
        Frame frame;
        for (int k = 0; k < 3; ++k) {
            frame.gyro[k]    = static_cast<float>(sample.gyro[k]);
            frame.gravity[k] = static_cast<float>(sample.gravity[k]);
        }
        for (std::size_t i = 0; i < JOINT_COUNT; ++i) {
            frame.q[i]  = static_cast<float>(sample.q[i] - policy.default_pose[i]);
            frame.dq[i] = static_cast<float>(sample.dq[i]);
        }
        frame.action  = agent.last_action;
        frame.command = {static_cast<float>(agent.decision.command_local[0]),
                         static_cast<float>(agent.decision.command_local[1])};
        for (int k = 0; k < 4; ++k) {
            frame.ball[k] = static_cast<float>(sample.ball_camera[k]);
        }
        auto& history = agent.history;
        if (!agent.history_primed) {
            history.fill(frame);
            agent.history_primed = true;
        }
        else {
            history[0] = history[1];
            history[1] = history[2];
            history[2] = frame;
        }

        // Term by term, each as its three steps oldest first.
        std::vector<float> obs;
        obs.reserve(skill::OBS_SIZE);
        auto put = [&](auto member) {
            for (const Frame& step : history) {
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
        agent.last_action = policy.infer(obs);
        return agent.last_action;
    }

    // The walk policy's 75 numbers, current step only: gyro, gravity, joint angles from ITS default
    // pose, joint speeds, its own previous output, twist (vx, vy, wz).
    std::array<float, JOINT_COUNT> SkillPlayer::run_walk(Agent& agent, SkillPolicy& policy) {
        const Sample& sample = agent.sample;
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
        obs.insert(obs.end(), agent.walk_last_action.begin(), agent.walk_last_action.end());
        for (int k = 0; k < 3; ++k) {
            obs.push_back(static_cast<float>(agent.decision.twist[k]));
        }
        agent.walk_last_action = policy.infer(obs);
        return agent.walk_last_action;
    }

    // ---------------------------------------------------------------------------------------------
    // Drills

    void SkillPlayer::place_locked(Agent& agent, double x, double y, double yaw) {
        const ModelMap& map = agent.map;
        mjtNum* root        = data_->qpos + map.root_qpos_adr;
        root[0]             = x;
        root[1]             = y;
        root[2]             = ready_z_;
        root[3]             = std::cos(yaw / 2.0);  // yaw-only quaternion, wxyz
        root[4]             = 0.0;
        root[5]             = 0.0;
        root[6]             = std::sin(yaw / 2.0);
        for (int i = 0; i < 6; ++i) {
            data_->qvel[map.root_dof_adr + i]           = 0.0;
            data_->qfrc_applied[map.root_dof_adr + i]   = 0.0;
            data_->qacc_warmstart[map.root_dof_adr + i] = 0.0;
        }
        for (std::size_t i = 0; i < JOINT_COUNT; ++i) {
            data_->qpos[map.qpos_adr[i]]           = ready_joints_[i];
            data_->qvel[map.dof_adr[i]]            = 0.0;
            data_->qacc_warmstart[map.dof_adr[i]]  = 0.0;
            data_->ctrl[map.act_id[i]]             = 0.0;
        }
        placed_robot_(agent.robot_id);  // clears its controller; it eases back into the policies
    }

    void SkillPlayer::place_ball_locked(double x, double y) {
        if (ball_body_ < 0) {
            return;
        }
        const int joint = model_->body_jntadr[ball_body_];
        mjtNum* q       = data_->qpos + model_->jnt_qposadr[joint];
        mjtNum* v       = data_->qvel + model_->jnt_dofadr[joint];
        const mjtNum* g = model_->geom_pos + 3 * ball_geom_;
        q[0]            = x - g[0];
        q[1]            = y - g[1];
        q[2]            = model_->geom_size[3 * ball_geom_] - g[2];
        q[3]            = 1.0;
        q[4] = q[5] = q[6] = 0.0;
        std::fill_n(v, 6, 0.0);
    }

    std::array<double, 2> SkillPlayer::rondo_spot(int k) const {
        // The corners of an equilateral triangle about the centre, the first nearest the camera's
        // starting side (-y).
        const double radius = rondo_side() / std::sqrt(3.0);
        const double angle  = -PI / 2.0 + k * 2.0 * PI / 3.0;
        return {rondo_.centre[0] + radius * std::cos(angle), rondo_.centre[1] + radius * std::sin(angle)};
    }

    void SkillPlayer::start_drill(Drill drill, double sim_time) {
        if (placed_robot_ == nullptr) {
            return;
        }
        const Drill before = drill_;
        if (drill != Drill::FREE) {
            const char* why = agents_.size() < 4  ? "the rondo needs 4 robots: start with ./b skills --robots 4"
                              : !policies_->walk  ? "the rondo needs the walk policy (policies.walk)"
                              : ball_body_ < 0    ? "no ball on this field"
                                                  : nullptr;
            if (why != nullptr) {
                log<NUClear::LogLevel::WARN>("SkillPlayer:", why);
                note(why, sim_time, 4.0);
                if (rondo()) {
                    drill = Drill::FREE;  // can't go on with it
                }
                else {
                    return;
                }
            }
        }
        if (drill != Drill::FREE) {
            drill_ = drill;
            if (before != drill) {
                passes_ = best_ = steals_ = lost_ = 0;
                rondo_holder_ = 0;
            }
            start_rondo_round(sim_time);
            // Free camera, looking over the triangle from behind the first teammate.
            auto camera       = std::make_unique<message::CameraRequest>();
            camera->mode      = 0;
            camera->place     = true;
            camera->x         = rondo_.centre[0];
            camera->y         = rondo_.centre[1];
            camera->distance  = 2.6 * rondo_side();
            camera->azimuth   = 90.0;
            camera->elevation = -55.0;
            if (tracking()) {
                // The tracking rondos: close and steep, 1 m behind the teammate on the halfway line,
                // looking over it into the triangle.
                const auto spot     = rondo_spot(0);
                const double length = std::max(std::hypot(rondo_.centre[0] - spot[0], rondo_.centre[1] - spot[1]), 1e-6);
                const double ux = (rondo_.centre[0] - spot[0]) / length, uy = (rondo_.centre[1] - spot[1]) / length;
                const double behind = 1.0, height = 3.0, ahead = 0.6 * length, flat = behind + ahead;  // about 46 deg down
                camera->x         = spot[0] + ahead * ux;
                camera->y         = spot[1] + ahead * uy;
                camera->distance  = std::hypot(flat, height);
                camera->azimuth   = std::atan2(uy, ux) * 180.0 / PI;
                camera->elevation = -std::atan2(height, flat) * 180.0 / PI;
            }
            emit(camera);
            log<NUClear::LogLevel::INFO>(sharing()      ? "SkillPlayer: rondo vs the EKF: you play the robot in the middle, the three track you with their cameras;"
                                         : tracking()  ? "SkillPlayer: rondo vs solo eyes: you play the robot in the middle, each of the three tracks you with its own camera alone;"
                                         : defending() ? "SkillPlayer: rondo, defending: you play the robot in the middle;"
                                                       : "SkillPlayer: rondo:",
                                         "triangle of side", rondo_side(), "m about (", rondo_.centre[0], ",", rondo_.centre[1], ")");
            return;
        }

        // Free play: the first robot and the ball back where the scene started them, the rest
        // standing on the touchline.
        drill_         = Drill::FREE;
        active_        = 0;
        round_over_at_ = -1.0;
        pass_live_     = false;
        masked_        = {};
        {
            std::lock_guard<std::mutex> lock(*mutex_);
            for (std::size_t i = 0; i < agents_.size(); ++i) {
                Agent& agent = agents_[i];
                agent.logic.set_pass_targets({});
                agent.start_with_ball = false;
                agent.idle_since      = -1.0;
                if (i == 0) {
                    agent.role  = Role::PLAYER;
                    agent.homed = false;
                    place_locked(agent, start_pose_[0], start_pose_[1], start_pose_[2]);
                    continue;
                }
                const std::size_t k = i - 1;
                const auto spot = k < rondo_.park.size() ? rondo_.park[k]
                                                         : std::array<double, 2>{rondo_.park.empty() ? 2.5 : rondo_.park.back()[0] + double(k - rondo_.park.size() + 1),
                                                                                 rondo_.park.empty() ? -5.0 : rondo_.park.back()[1]};
                agent.role  = Role::PARKED;
                agent.homed = true;
                agent.home  = {spot[0], spot[1], spot[1] < 0.0 ? PI / 2.0 : -PI / 2.0};  // facing the field
                place_locked(agent, agent.home[0], agent.home[1], agent.home[2]);
            }
            place_ball_locked(ball_start_[0], ball_start_[1]);
            mj_forward(model_, data_);
        }
        for (auto& agent : agents_) {
            restart(agent);
        }
        if (before != Drill::FREE) {
            auto camera  = std::make_unique<message::CameraRequest>();
            camera->mode = 2;  // behind the robot, as free play starts
            emit(camera);
            note("free play", sim_time);
        }
    }

    void SkillPlayer::start_rondo_round(double sim_time) {
        round_over_at_    = -1.0;
        pass_live_        = false;
        pass_handled_     = false;
        defender_stopped_ = false;
        passes_           = 0;
        round_start_      = sim_time;
        ekf_.reset();
        track_time_       = sim_time;
        for (std::size_t k = 0; k < std::min<std::size_t>(3, agents_.size()); ++k) {
            // Staggered, so the three cameras' frames don't all land on one control step.
            agents_[k].next_detect = sim_time + double(k) / (3.0 * std::max(tracking_.rate, 1e-3));
            agents_[k].detected_at = -1e9;
            agents_[k].sight       = {};
            agents_[k].ekf.reset();
        }
        const int holder  = rondo_holder_ % 3;
        std::array<double, 2> ball{};
        {
            std::lock_guard<std::mutex> lock(*mutex_);
            for (std::size_t i = 0; i < agents_.size(); ++i) {
                Agent& agent          = agents_[i];
                agent.homed           = true;
                agent.start_with_ball = false;
                agent.idle_since      = sim_time;
                agent.computer_since  = -1.0;
                agent.receiving_since = -1.0;
                agent.computer_release = false;
                if (i < 3) {
                    const auto spot = rondo_spot(static_cast<int>(i));
                    agent.role      = Role::TEAMMATE;
                    agent.home      = {spot[0], spot[1], std::atan2(rondo_.centre[1] - spot[1], rondo_.centre[0] - spot[0])};
                    if (static_cast<int>(i) == holder) {
                        agent.start_with_ball = true;
                        ball = {spot[0] + rondo_.ball_ahead * std::cos(agent.home[2]),
                                spot[1] + rondo_.ball_ahead * std::sin(agent.home[2])};
                    }
                }
                else if (i == 3) {
                    // In the middle, facing the teammate with the ball.
                    const auto spot = rondo_spot(holder);
                    agent.role      = Role::DEFENDER;
                    agent.home      = {rondo_.centre[0], rondo_.centre[1],
                                       std::atan2(spot[1] - rondo_.centre[1], spot[0] - rondo_.centre[0])};
                }
                else {
                    const std::size_t k = i - 4;
                    const auto spot     = k < rondo_.park.size() ? rondo_.park[k] : std::array<double, 2>{2.5 + double(k), -5.0};
                    agent.role          = Role::PARKED;
                    agent.home          = {spot[0], spot[1], spot[1] < 0.0 ? PI / 2.0 : -PI / 2.0};
                }
                place_locked(agent, agent.home[0], agent.home[1], agent.home[2]);
            }
            place_ball_locked(ball[0], ball[1]);
            mj_forward(model_, data_);
        }
        for (auto& agent : agents_) {
            restart(agent);
        }
        owner_  = static_cast<std::size_t>(holder);
        active_ = defending() ? 3 : owner_;
        agents_[owner_].idle_since = -1.0;
        masked_ = held_keys();  // whatever is held now waits to be let go
        masked_.fast = masked_.shoot = false;
        pass_pressed_ = drop_pressed_ = shoot_down_ = shoot_up_ = release_pressed_ = air_down_ = air_up_ = false;
        log<NUClear::LogLevel::INFO>("SkillPlayer: rondo round, robot", agents_[owner_].robot_id, "has the ball");
    }

    // The teammate with the ball (or the one it is rolling to, or the one going to fetch it) becomes
    // `to`. In the passing rondo the keys go with it; keys still held wait until let go.
    void SkillPlayer::set_owner(std::size_t to, double sim_time) {
        if (to == owner_) {
            return;
        }
        agents_[owner_].idle_since     = sim_time;
        agents_[owner_].computer_since = -1.0;
        owner_                         = to;
        agents_[to].idle_since         = -1.0;
        if (!defending()) {
            active_       = to;
            masked_       = keys_;
            masked_.fast  = masked_.shoot = false;
            pass_pressed_ = drop_pressed_ = shoot_down_ = shoot_up_ = release_pressed_ = air_down_ = air_up_ = false;
        }
    }

    void SkillPlayer::end_round(const std::string& text, double sim_time) {
        pass_live_        = false;
        defender_stopped_ = true;
        round_over_at_    = sim_time + rondo_.next_round;
        note(text, sim_time, rondo_.next_round + 0.5);
        log<NUClear::LogLevel::INFO>("SkillPlayer: t =", sim_time, "rondo:", text);
    }

    // Nobody has the ball under control or on its way to them: it is rolling loose or lying somewhere.
    bool SkillPlayer::ball_loose() const {
        const Skill skill = agents_[owner_].logic.skill();
        return !pass_live_ && (skill == Skill::WALK || skill == Skill::KICK || skill == Skill::FIRST_PASS);
    }

    // The defender touched the ball: a contact now, or a jump in the ball's velocity with the defender
    // against it (within 10 cm) and nearer it than any teammate. Bumping into a robot doesn't count,
    // nor a teammate's touch with the defender close by.
    bool SkillPlayer::defender_touched() const {
        const Sample& d = agents_[3].sample;
        if (d.ball_contact) {
            return true;
        }
        if (!d.world.ball_touched || d.ball_gap > 0.1) {
            return false;
        }
        for (std::size_t k = 0; k < 3; ++k) {
            if (agents_[k].sample.ball_contact || agents_[k].sample.ball_gap < d.ball_gap) {
                return false;
            }
        }
        return true;
    }

    // rondo_ekf: each teammate's head camera looks for the defender, and every detection goes into
    // the one EKF the three share. A camera sees the defender when its trunk is in the image (the
    // model's head camera, the K1's lens), within tracking_.range, and not hidden: a ray to its
    // trunk or its head reaches it before any other robot or the ball. A camera in view still misses
    // it now and then (tracking_.miss). A detection is a range, from the robot's size in the image,
    // so poor and worse further off, and a bearing, from where it is in the image; each camera
    // reports at tracking_.rate. Every teammate knows where its own camera is (no self-localisation
    // error). The viewer gets the track and what each camera sees, every control step.
    void SkillPlayer::update_tracking(double sim_time) {
        if (!tracking() || agents_.size() < 4) {
            return;
        }
        if (track_time_ >= 0.0 && sim_time > track_time_) {
            const double dt = std::min(sim_time - track_time_, 0.5);
            if (sharing()) {
                ekf_.predict(dt);
            }
            else {
                for (std::size_t k = 0; k < 3; ++k) {
                    agents_[k].ekf.predict(dt);
                }
            }
        }
        track_time_ = sim_time;

        const Agent& defender = agents_[3];
        const int target_root = defender.map.root_body_id;
        auto track            = std::make_unique<message::DefenderTrack>();
        track->active         = true;
        track->aspect         = tracking_.aspect;
        std::normal_distribution<double> unit(0.0, 1.0);
        std::uniform_real_distribution<double> chance(0.0, 1.0);
        {
            std::lock_guard<std::mutex> lock(*mutex_);
            const mjtNum* trunk = data_->xpos + 3 * target_root;
            const mjtNum* head  = data_->xpos + 3 * defender.head_body;
            track->truth_x      = trunk[0];
            track->truth_y      = trunk[1];
            for (std::size_t k = 0; k < 3; ++k) {
                Agent& mate = agents_[k];
                auto& sight = mate.sight;
                sight          = {};
                sight.robot_id = mate.robot_id;
                sight.camera   = mate.camera;
                const int c    = mate.camera;
                if (c >= 0) {
                    const mjtNum* cp  = data_->cam_xpos + 3 * c;
                    const mjtNum* cm  = data_->cam_xmat + 9 * c;
                    const double tanh = std::tan(0.5 * model_->cam_fovy[c] * PI / 180.0);
                    // A world point in the image: (u, v) in half image heights about the centre
                    // (MuJoCo cameras look down -z, x right, y up); false if behind the camera.
                    auto project = [&](const mjtNum p[3], double& u, double& v) {
                        mjtNum rel[3] = {p[0] - cp[0], p[1] - cp[1], p[2] - cp[2]}, local[3];
                        mju_mulMatTVec3(local, cm, rel);
                        const double depth = -local[2];
                        if (depth < 0.05) {
                            return false;
                        }
                        u = local[0] / depth / tanh;
                        v = local[1] / depth / tanh;
                        return true;
                    };
                    // Hidden from the camera: something other than the defender (and the camera's own
                    // robot) is hit first on the way to p.
                    auto hidden = [&](const mjtNum p[3]) {
                        mjtNum ray[3]         = {p[0] - cp[0], p[1] - cp[1], p[2] - cp[2]};
                        const double distance = mju_normalize3(ray);
                        int geom              = -1;
                        const double hit = mj_ray(model_, data_, cp, ray, nullptr, 1, mate.head_body, &geom, nullptr);
                        if (geom < 0 || hit < 0.0 || hit > distance - 0.05) {
                            return false;
                        }
                        const int root = model_->body_rootid[model_->geom_bodyid[geom]];
                        return root != target_root && root != mate.map.root_body_id && root != 0;
                    };
                    double uc = 0.0, vc = 0.0;
                    const double ground = std::hypot(trunk[0] - cp[0], trunk[1] - cp[1]);
                    const bool in_view  = project(trunk, uc, vc) && std::abs(uc) < tracking_.aspect && std::abs(vc) < 1.0
                                         && ground < tracking_.range;
                    if (in_view) {
                        const bool ball_or_robot_between = hidden(trunk) && hidden(head);
                        sight.occluded = ball_or_robot_between;
                        sight.seen     = !ball_or_robot_between;
                    }
                    if (sight.seen || sight.occluded) {
                        // Its box in the image: the head's top, the feet, and the trunk's width either side.
                        const double hx = -(trunk[1] - cp[1]) / std::max(ground, 1e-6), hy = (trunk[0] - cp[0]) / std::max(ground, 1e-6);
                        const mjtNum points[4][3] = {{head[0], head[1], head[2] + 0.12},
                                                     {trunk[0], trunk[1], 0.0},
                                                     {trunk[0] + 0.18 * hx, trunk[1] + 0.18 * hy, trunk[2]},
                                                     {trunk[0] - 0.18 * hx, trunk[1] - 0.18 * hy, trunk[2]}};
                        sight.u0 = sight.v0 = 1e9;
                        sight.u1 = sight.v1 = -1e9;
                        for (const auto& p : points) {
                            double u = 0.0, v = 0.0;
                            if (project(p, u, v)) {
                                sight.u0 = std::min(sight.u0, u);
                                sight.u1 = std::max(sight.u1, u);
                                sight.v0 = std::min(sight.v0, v);
                                sight.v1 = std::max(sight.v1, v);
                            }
                        }
                    }
                    if (sight.seen && sim_time >= mate.next_detect) {
                        mate.next_detect = sim_time + 1.0 / std::max(tracking_.rate, 1e-3);
                        if (chance(rng_) >= tracking_.miss) {
                            skill::Detection z;
                            z.cam_x         = cp[0];
                            z.cam_y         = cp[1];
                            z.sigma_bearing = tracking_.sigma_bearing;
                            z.sigma_range   = tracking_.sigma_range + tracking_.sigma_range_per_m * ground;
                            z.range   = std::max(0.2, ground + z.sigma_range * unit(rng_));
                            z.bearing = std::atan2(trunk[1] - cp[1], trunk[0] - cp[0]) + z.sigma_bearing * unit(rng_);
                            ekf_of(mate).update(z);
                            mate.detected_at = sim_time;
                            mate.sight.det_x = z.cam_x + z.range * std::cos(z.bearing);
                            mate.sight.det_y = z.cam_y + z.range * std::sin(z.bearing);
                            mate.sight.cam_x = z.cam_x;
                            mate.sight.cam_y = z.cam_y;
                        }
                    }
                    else if (!sight.seen) {
                        mate.next_detect = std::max(mate.next_detect, sim_time);
                    }
                }
                track->views.push_back(sight);
            }
        }
        // The last detection each camera made, carried in its view for the viewer.
        for (std::size_t k = 0; k < 3; ++k) {
            auto& view  = track->views[k];
            auto& last  = agents_[k].last_detection;
            if (agents_[k].detected_at == sim_time) {
                last = {view.det_x, view.det_y, view.cam_x, view.cam_y};
            }
            view.age      = sim_time - agents_[k].detected_at;
            view.detected = view.age < 1e8;
            view.det_x    = last[0];
            view.det_y    = last[1];
            view.cam_x    = last[2];
            view.cam_y    = last[3];
        }
        // The track(s): the shared one, or each teammate's own.
        track->shared = sharing();
        for (std::size_t k = 0; k < (sharing() ? 1u : 3u); ++k) {
            const skill::DefenderEkf& e = sharing() ? ekf_ : agents_[k].ekf;
            message::DefenderTrack::Track out;
            out.valid = e.valid();
            if (out.valid) {
                const auto& x = e.state();
                const auto& P = e.covariance();
                out.x     = x[0];
                out.y     = x[1];
                out.vx    = x[2];
                out.vy    = x[3];
                out.cxx   = P[0][0];
                out.cxy   = P[0][1];
                out.cyy   = P[1][1];
                out.sigma = e.position_sigma();
                out.error = std::hypot(x[0] - track->truth_x, x[1] - track->truth_y);
            }
            track->tracks.push_back(out);
        }
        if (!tracking_.truth_marker) {
            track->truth_x = track->truth_y = 0.0;
        }
        if (sim_time >= track_log_next_ || track_log_next_ - sim_time > 10.0) {
            track_log_next_ = sim_time + 2.0 * log_period_;
            int seeing = 0, hidden = 0;
            for (const auto& view : track->views) {
                seeing += view.seen ? 1 : 0;
                hidden += view.occluded ? 1 : 0;
            }
            std::string tracks;
            char one[64];
            for (const auto& t : track->tracks) {
                if (t.valid) {
                    std::snprintf(one, sizeof(one), "%s%.2f m off (sigma %.2f)", tracks.empty() ? "" : " / ", t.error, t.sigma);
                }
                else {
                    std::snprintf(one, sizeof(one), "%sno track", tracks.empty() ? "" : " / ");
                }
                tracks += one;
            }
            log<NUClear::LogLevel::INFO>("SkillPlayer: t =", sim_time, sharing() ? "EKF (shared): seen by" : "EKFs (solo): seen by",
                                         seeing, "hidden from", hidden, "|", tracks);
        }
        track_sent_ = true;
        emit(track);
    }

    std::array<double, 2> SkillPlayer::defender_xy(const Agent& agent) const {
        if (!tracking()) {
            const auto& dw = agents_[3].sample.world;
            return {dw.robot_x, dw.robot_y};
        }
        const skill::DefenderEkf& e = ekf_of(agent);
        if (e.valid() && e.position_sigma() < tracking_.lost_sigma) {
            return {e.state()[0], e.state()[1]};
        }
        return rondo_.centre;  // no idea: assume the middle
    }

    void SkillPlayer::update_rondo(double sim_time) {
        if (agents_.size() < 4) {
            return;
        }
        if (round_over_at_ >= 0.0) {
            if (sim_time >= round_over_at_) {
                rondo_holder_ = (rondo_holder_ + 1) % 3;
                start_rondo_round(sim_time);
            }
            return;
        }
        Agent& owner   = agents_[owner_];
        Agent& defender = agents_[3];
        const auto& w  = owner.sample.world;  // the ball is the same in every robot's sample
        if (!w.ball_valid) {
            return;
        }
        char text[160];
        const char* plural = passes_ == 1 ? "" : "es";

        // The defender wins it: its body touched the ball.
        if (defender.phase == Phase::PLAYING && defender_touched()) {
            ++steals_;
            if (defending()) {
                std::snprintf(text, sizeof(text), "you won it after %d pass%s in %.1f s!", passes_, plural, sim_time - round_start_);
            }
            else {
                std::snprintf(text, sizeof(text), "intercepted after %d pass%s! new round shortly", passes_, plural);
            }
            end_round(text, sim_time);
            return;
        }
        // Out of the rondo altogether. Defending, before forced_win_passes passes that is your win
        // too: you pressed them into losing it.
        const double radius = rondo_side() / std::sqrt(3.0);
        if (std::hypot(w.ball_x - rondo_.centre[0], w.ball_y - rondo_.centre[1]) > radius + rondo_.out_margin) {
            if (defending() && passes_ < rondo_.forced_win_passes) {
                ++steals_;
                std::snprintf(text, sizeof(text), "you forced it out after %d pass%s in %.1f s: a win!", passes_, plural,
                              sim_time - round_start_);
                end_round(text, sim_time);
                return;
            }
            ++lost_;
            if (defending()) {
                std::snprintf(text, sizeof(text), "the ball got away after %d pass%s: no winner (under %d would have been yours). New round shortly",
                              passes_, plural, rondo_.forced_win_passes);
            }
            else {
                std::snprintf(text, sizeof(text), "the ball got away after %d pass%s! new round shortly", passes_, plural);
            }
            end_round(text, sim_time);
            return;
        }
        // A pass arriving: the teammate it was played to touches it (not after giving up on it).
        if (pass_live_ && owner.logic.skill() == Skill::WALK) {
            pass_live_ = false;
        }
        if (pass_live_ && w.ball_touched) {
            pass_live_ = false;
            ++passes_;
            best_ = std::max(best_, passes_);
            log<NUClear::LogLevel::INFO>("SkillPlayer: t =", sim_time, "pass", passes_, "completed to robot", owner.robot_id);
        }
        else if (ball_loose()) {
            // Another teammate got to it first: it's theirs (a pass only counts for the one it was played to).
            for (std::size_t k = 0; k < 3; ++k) {
                if (k != owner_ && agents_[k].sample.world.ball_touched) {
                    set_owner(k, sim_time);
                    break;
                }
            }
        }
        // A pass leaving the teammate with the ball. If it will roll to a teammate (near enough to its
        // feet, before it stops) that teammate receives it, and in the passing rondo the keys go with
        // it. If it won't, nobody stands waiting in receive: the teammate nearest where it will stop
        // walks, to go and get it.
        const auto& d = agents_[owner_].decision;
        const bool kicked = d.skill == Skill::KICK || (d.skill == Skill::FIRST_PASS && d.touched);
        if (!kicked) {
            pass_handled_ = false;
        }
        const double speed = std::hypot(w.ball_vx, w.ball_vy);
        if (kicked && !pass_handled_ && speed > 1.0 && w.ball_distance() > 0.35) {
            pass_handled_ = true;
            const double ux = w.ball_vx / speed, uy = w.ball_vy / speed;
            const double roll = speed * speed / (2.0 * logic_config_.rolling_decel);
            std::size_t receiver = owner_, nearest = owner_;
            double nearest_along = 1e9, nearest_stop = 1e9;
            for (std::size_t k = 0; k < 3; ++k) {
                const auto& tw     = agents_[k].sample.world;
                const double rx    = tw.robot_x - w.ball_x, ry = tw.robot_y - w.ball_y;
                const double along = rx * ux + ry * uy, side = std::abs(-rx * uy + ry * ux);
                if (k != owner_ && along > 0.4 && along < roll + 0.5 && side < rondo_.receive_reach && along < nearest_along) {
                    nearest_along = along;
                    receiver      = k;
                }
                const double stop = std::hypot(w.ball_x + ux * roll - tw.robot_x, w.ball_y + uy * roll - tw.robot_y);
                if (stop < nearest_stop) {
                    nearest_stop = stop;
                    nearest      = k;
                }
            }
            const int from = agents_[owner_].robot_id;
            if (receiver != owner_) {
                log<NUClear::LogLevel::INFO>("SkillPlayer: t =", sim_time, "pass from robot", from, "to robot", agents_[receiver].robot_id);
                set_owner(receiver, sim_time);
                agents_[receiver].logic.on_pass(std::atan2(uy, ux));
                pass_live_ = true;
            }
            else {
                log<NUClear::LogLevel::INFO>("SkillPlayer: t =", sim_time, "pass from robot", from, "reaches nobody: robot",
                                             agents_[nearest].robot_id, "goes for it");
                set_owner(nearest, sim_time);
                agents_[nearest].logic.on_release();
                if (!defending()) {
                    note("off target: walk to the ball", sim_time);
                }
            }
        }
        // Teammates without the ball: once it is well away, back to standing in walk, from which
        // receive is a trained hand-over.
        for (std::size_t k = 0; k < 3; ++k) {
            Agent& mate = agents_[k];
            if (k != owner_ && mate.idle_since >= 0.0 && sim_time - mate.idle_since > 1.0
                && mate.logic.skill() != Skill::WALK && mate.sample.world.ball_distance() > 1.5) {
                mate.logic.on_release();
            }
        }
        // From here on the computer plays the three, so each knows only where it believes the
        // defender is (in the tracking rondos, its track: shared, or its own) and keeps lanes clear of
        // that by its doubt too.
        std::array<std::array<double, 2>, 3> seen_by{};
        std::array<double, 3> doubt_of{};
        for (std::size_t k = 0; k < 3; ++k) {
            seen_by[k]               = defender_xy(agents_[k]);
            const skill::DefenderEkf& e = ekf_of(agents_[k]);
            doubt_of[k]              = tracking() && e.valid() ? std::min(e.position_sigma(), 1.0) : 0.0;
        }
        // Where each teammate off the ball should be: its corner of the triangle, unless the
        // defender is standing in the lane from the ball to it; then stepped sideways off the
        // corner, away from the defender, until the lane is clear (by up to support_shift).
        for (std::size_t k = 0; k < 3; ++k) {
            Agent& mate    = agents_[k];
            mate.support   = {mate.home[0], mate.home[1]};
            const double lx = mate.home[0] - w.ball_x, ly = mate.home[1] - w.ball_y;
            const double length = std::hypot(lx, ly);
            if (k == owner_ || length < 0.5) {
                continue;
            }
            const double ux = lx / length, uy = ly / length;  // along the lane
            const auto& seen   = seen_by[k];
            const double doubt = doubt_of[k];
            const double ax = seen[0] - w.ball_x, ay = seen[1] - w.ball_y;
            const double along = ax * ux + ay * uy;
            const double side  = -ax * uy + ay * ux;  // the defender's offset left of the lane
            if (along > 0.3 && along < length - 0.3 && std::abs(side) < rondo_.lane_clear + doubt) {
                // Turn the lane about the ball until the defender is lane_clear off it, at its distance.
                const double need  = std::copysign(rondo_.lane_clear + doubt, side) - side;  // how far the lane must move at the defender
                const double shift = std::clamp(-need * length / along, -rondo_.support_shift, rondo_.support_shift);
                mate.support = {mate.home[0] - uy * shift, mate.home[1] + ux * shift};
            }
        }

        // Each teammate's pass targets: the other two, the one the defender is further from the
        // passing lane of first. The lane starts at the ball when it's at the teammate's feet, else
        // at the teammate (a pass on its way to it).
        for (std::size_t k = 0; k < 3; ++k) {
            const auto& me    = agents_[k].sample.world;
            const bool at_feet = me.ball_distance() < logic_config_.kick_reach;
            const double fx   = at_feet ? w.ball_x : me.robot_x;
            const double fy   = at_feet ? w.ball_y : me.robot_y;
            const auto& seen   = seen_by[k];
            const double doubt = doubt_of[k];
            std::vector<std::pair<double, std::array<double, 2>>> lanes;
            for (std::size_t j = 0; j < 3; ++j) {
                if (j == k) {
                    continue;
                }
                const std::array<double, 2> to{agents_[j].sample.world.robot_x, agents_[j].sample.world.robot_y};
                const double lx = to[0] - fx, ly = to[1] - fy;
                const double length2 = std::max(lx * lx + ly * ly, 1e-6);
                const double u = std::clamp(((seen[0] - fx) * lx + (seen[1] - fy) * ly) / length2, 0.0, 1.0);
                lanes.push_back({std::hypot(fx + u * lx - seen[0], fy + u * ly - seen[1]) - doubt, to});
            }
            std::sort(lanes.begin(), lanes.end(), [](const auto& a, const auto& b) { return a.first > b.first; });
            agents_[k].open_lane = lanes.front().first;
            std::vector<std::array<double, 2>> targets;
            for (const auto& lane : lanes) {
                targets.push_back(lane.second);
            }
            agents_[k].logic.set_pass_targets(std::move(targets));
        }
    }

    // rondo_defend: what the computer "holds down" for the teammate with the ball. It passes to the
    // open teammate once it has held the ball a moment, or at once with the defender close, if a lane
    // is open and the ball sits squarely in front; pressed with no lane open, it dribbles away from
    // the defender, bending back towards its corner. A pass coming in with the defender near the
    // receiver is passed on first time. A kick that went nowhere, or a trap that never settled, is
    // picked up with the dribble; a ball that got away is walked to (chase_twist).
    skill::Keys SkillPlayer::computer_keys(Agent& agent, double sim_time) {
        skill::Logic& logic = agent.logic;
        const auto& w       = agent.sample.world;
        const auto dp       = defender_xy(agent);  // where it believes the defender is
        const double pressure = std::hypot(dp[0] - w.ball_x, dp[1] - w.ball_y);
        const bool open       = agent.open_lane > rondo_.ai_lane_open;
        const skill::Keys none{};
        auto steer = [](double angle) {
            skill::Keys keys;
            keys.steer = true;
            keys.angle = angle;
            return keys;
        };
        if (agent.computer_release) {  // a tap of the pass key: pressed last step, released now
            agent.computer_release = false;
            logic.on_shoot_release(w);
            return none;
        }
        if (agent.phase != Phase::PLAYING) {
            return none;
        }
        const Skill skill = logic.skill();
        if (skill != Skill::DRIBBLE) {
            agent.computer_since = -1.0;
        }
        if (skill != Skill::RECEIVE || logic.pass_incoming()) {
            agent.receiving_since = -1.0;
        }
        else if (agent.receiving_since < 0.0) {
            agent.receiving_since = sim_time;
        }
        if (skill == Skill::RECEIVE && logic.pass_incoming()) {
            const double to_me = std::hypot(dp[0] - w.robot_x, dp[1] - w.robot_y), distance = w.ball_distance();
            if (to_me < rondo_.ai_first_time && open && distance > 0.8 && distance < 2.5) {
                logic.on_first_pass(none, w);
            }
            return none;
        }
        const auto& cfg = logic.config();
        if ((skill == Skill::DRIBBLE || skill == Skill::RECEIVE) && w.ball_distance() > cfg.takeover_distance
            && w.ball_speed < cfg.trap_speed) {
            logic.on_release();  // lost it: walk to it
            return none;
        }
        if (skill == Skill::RECEIVE && agent.receiving_since >= 0.0 && sim_time - agent.receiving_since > 1.5
            && w.ball_speed < 0.5) {
            // Trapped but never quite settled where receive hands over: take it on with the dribble.
            return steer(std::atan2(w.ball_y - w.robot_y, w.ball_x - w.robot_x));
        }
        if ((skill == Skill::KICK || (skill == Skill::FIRST_PASS && logic.touched())) && w.ball_speed < 0.5) {
            // A kick or first-time pass that went nowhere: once the lockout is over, a direction picks
            // the ball back up if it is near, or walks.
            return steer(std::atan2(agent.home[1] - w.ball_y, agent.home[0] - w.ball_x));
        }
        if (skill != Skill::DRIBBLE) {
            return none;
        }
        if (agent.computer_since < 0.0) {
            agent.computer_since = sim_time;
            agent.computer_hold  = sample_range(rondo_.ai_hold);
        }
        const double held = sim_time - agent.computer_since;
        const auto ball   = w.ball_local();
        const bool squared = ball[0] > 0.1 && std::abs(std::atan2(ball[1], ball[0])) < 0.5 && w.ball_speed < 0.4;
        if (open && squared && logic.can_kick(w) && (held > agent.computer_hold || pressure < rondo_.ai_pressure)) {
            if (logic.on_shoot_press(w)) {
                agent.computer_release = true;
            }
            return none;
        }
        if (pressure < rondo_.ai_pressure + 0.4) {
            const double ax = w.ball_x - dp[0], ay = w.ball_y - dp[1], an = std::max(std::hypot(ax, ay), 1e-6);
            const double hx = agent.home[0] - w.ball_x, hy = agent.home[1] - w.ball_y, hn = std::max(std::hypot(hx, hy), 1e-6);
            const double pull = std::clamp(hn - 0.5, 0.0, 1.5);
            return steer(std::atan2(ay / an + pull * hy / hn, ax / an + pull * hx / hn));
        }
        return none;
    }

    // A teammate going to get a loose ball: turn to it and walk at it, slowing as it gets there so the
    // walk can pick it up (walking up to a ball in front hands over to receive).
    std::array<double, 3> SkillPlayer::chase_twist(const Agent& agent) const {
        const auto& w = agent.sample.world;
        const double dx = w.ball_x - w.robot_x, dy = w.ball_y - w.robot_y, distance = std::hypot(dx, dy);
        const double err   = skill::wrap_angle(std::atan2(dy, dx) - w.yaw);
        const double creep = logic_config_.walk_turn_creep;
        const double top   = std::min(rondo_.chase_speed, 0.3 + 0.5 * distance);
        return {std::abs(err) < PI / 2.0 ? std::max(creep, top * std::cos(err) * std::cos(err)) : creep, 0.0,
                std::clamp(2.0 * err, -logic_config_.walk_turn_rate, logic_config_.walk_turn_rate)};
    }

    // The defender walks at the ball as fast as the walk goes, aiming where the ball will be by the
    // time it gets there (up to defender_lead s ahead), turning towards it as it goes. It eases off
    // close to the ball, so it doesn't run through the robot holding it, and speeds up and slows
    // down at defender_accel: the walk falls if its command jumps from 2 m/s to standing.
    skill::Decision SkillPlayer::defender_decision(const Agent& agent, double sim_time) {
        skill::Decision out;
        out.skill     = Skill::WALK;
        const auto& w = agent.sample.world;
        std::array<double, 3> want{};  // standing still
        if (!defender_stopped_ && round_over_at_ < 0.0 && w.ball_valid
            && sim_time - agent.playing_since >= rondo_.defender_delay) {
            const double distance = w.ball_distance();
            const double lead     = std::min(rondo_.defender_lead, distance / std::max(rondo_.defender_speed, 0.1));
            const double tx = w.ball_x + w.ball_vx * lead, ty = w.ball_y + w.ball_vy * lead;
            const double error = skill::wrap_angle(std::atan2(ty - w.robot_y, tx - w.robot_x) - w.yaw);
            const double creep = logic_config_.walk_turn_creep;  // the walk does not turn standing still
            const double top   = std::min(rondo_.defender_speed, rondo_.defender_close_speed + 1.5 * distance);
            want[2]            = std::clamp(2.5 * error, -rondo_.defender_turn, rondo_.defender_turn);
            want[0]            = std::abs(error) < PI / 2.0 ? std::max(creep, top * std::cos(error) * std::cos(error)) : creep;
        }
        out.twist = ease_twist(agent.decision.twist, want);
        return out;
    }

    // A walk command moved from `was` towards `want` no faster than defender_accel (m/s^2) and
    // 4 rad/s^2: the walk falls if its command jumps, from 2 m/s to standing say.
    std::array<double, 3> SkillPlayer::ease_twist(const std::array<double, 3>& was, const std::array<double, 3>& want) const {
        const double dv = rondo_.defender_accel * CONTROL_DT;
        const double dw = 4.0 * CONTROL_DT;
        std::array<double, 3> out{was[0] + std::clamp(want[0] - was[0], -dv, dv),
                                  was[1] + std::clamp(want[1] - was[1], -dv, dv),
                                  was[2] + std::clamp(want[2] - was[2], -dw, dw)};
        if (std::abs(out[0]) < 1e-3 && std::abs(out[1]) < 1e-3 && std::abs(out[2]) < 1e-3) {
            out = {};
        }
        return out;
    }

    // An off-the-ball teammate's walk command: to its support spot (update_rondo), turned to face
    // the ball so a pass comes from in front, as receive was trained. Far off, it turns and walks
    // there; within a metre it steps there sideways and backwards while keeping the ball in view.
    // Hysteresis keeps it from fidgeting once there.
    std::array<double, 3> SkillPlayer::support_twist(Agent& agent) {
        const auto& w = agent.sample.world;
        std::array<double, 3> want{};
        const double dx = agent.support[0] - w.robot_x, dy = agent.support[1] - w.robot_y;
        const double distance = std::hypot(dx, dy);
        const double face     = w.ball_valid ? std::atan2(w.ball_y - w.robot_y, w.ball_x - w.robot_x) : agent.home[2];
        const double face_err = skill::wrap_angle(face - w.yaw);
        const double creep    = logic_config_.walk_turn_creep;  // the walk does not turn standing still
        const double turn     = logic_config_.walk_turn_rate;
        if (distance > rondo_.support_tolerance) {
            agent.repositioning = true;
        }
        else if (distance < 0.2) {
            agent.repositioning = false;
        }
        if (agent.repositioning) {
            const double c = std::cos(w.yaw), s = std::sin(w.yaw);
            const double lx = c * dx + s * dy, ly = -s * dx + c * dy;  // robot frame
            if (distance < 1.0 && lx > -0.4) {
                want[0] = std::clamp(1.2 * lx, -logic_config_.walk_back_speed, rondo_.support_speed);
                want[1] = std::clamp(1.2 * ly, -logic_config_.walk_side_speed, logic_config_.walk_side_speed);
                want[2] = std::clamp(1.5 * face_err, -turn, turn);
            }
            else {
                const double err = skill::wrap_angle(std::atan2(dy, dx) - w.yaw);
                want[2]          = std::clamp(2.0 * err, -turn, turn);
                want[0] = std::abs(err) < PI / 2.0 ? std::max(creep, rondo_.support_speed * std::cos(err) * std::cos(err)) : creep;
            }
            return want;
        }
        if (std::abs(face_err) > 0.45) {
            agent.turning = true;
        }
        else if (std::abs(face_err) < 0.2) {
            agent.turning = false;
        }
        if (agent.turning) {
            want[2] = std::clamp(1.5 * face_err, -turn, turn);
            want[0] = creep;
        }
        return want;
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
                ball_in_goal_  = true;
                auto scored    = std::make_unique<message::GoalScored>();
                scored->x      = goal.x;
                scored->y      = goal.y;
                scored->width  = 2.0 * goal.half_width;
                scored->height = goal.height;
                scored->dir    = goal.dir;
                emit(scored);
                note("GOAL!", sim_time, 2.5);
                log<NUClear::LogLevel::INFO>("SkillPlayer: GOAL in the", goal.dir > 0 ? "+x" : "-x", "goal at t =", sim_time);
            }
        }
        if (!behind_a_line) {
            ball_in_goal_ = false;
        }
    }

    void SkillPlayer::publish_status(double sim_time, bool force) {
        const Agent& agent       = agents_[active_];
        const auto& decision     = agent.decision;
        const auto& sample       = agent.sample;
        const bool rondo         = this->rondo();
        auto status              = std::make_unique<message::SkillStatus>();
        status->keys             = sharing() ? EKF_HELP : tracking() ? SOLO_HELP : defending() ? DEFEND_HELP : rondo ? RONDO_HELP : KEY_HELP;
        status->drill            = sharing()    ? "rondo_ekf"
                                   : tracking() ? "rondo_solo"
                                   : defending() ? "rondo_defend"
                                   : rondo       ? "rondo"
                                                 : "free";
        status->robot_body       = agent.trunk;
        status->marker           = rondo;
        const bool loaded = (decision.skill == Skill::FIRST_TOUCH || decision.skill == Skill::FIRST_PASS) && !decision.touched;
        if (agent.phase == Phase::PLAYING && (decision.aiming || loaded)) {
            const auto& cfg   = agent.logic.config();
            status->aiming    = true;
            status->aim_angle = decision.aim_angle;
            if (loaded) {
                // The loaded first touch, as an arrow from the robot's feet.
                status->ball_x = sample.world.robot_x;
                status->ball_y = sample.world.robot_y;
                status->charge = decision.skill == Skill::FIRST_PASS ? 0.6 : 0.0;
            }
            else {
                status->ball_x = sample.world.ball_x;
                status->ball_y = sample.world.ball_y;
                status->charge = (decision.aim_speed - cfg.kick_speed_min)
                                 / std::max(cfg.kick_speed_max - cfg.kick_speed_min, 1e-6);
            }
        }
        char text[200];
        switch (agent.phase) {
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
                status->skill = skill::name(decision.skill);
                if (decision.aiming) {
                    status->skill = decision.air ? "AIMING (AIR)" : "AIMING";
                    std::snprintf(text,
                                  sizeof(text),
                                  "%.1f m/s -- hold for more, W A S D to aim, release to %s",
                                  decision.aim_speed,
                                  rondo ? "pass" : "shoot");
                    status->detail = text;
                }
                else if (decision.skill == Skill::FIRST_TOUCH || decision.skill == Skill::FIRST_PASS) {
                    const bool pass = decision.skill == Skill::FIRST_PASS;
                    const bool shot = pass && agent.logic.first_shot();
                    if (shot) {
                        status->skill = "FIRST-TIME SHOT";
                    }
                    std::snprintf(text,
                                  sizeof(text),
                                  decision.touched ? "%s at %.1f m/s, done" : "%s loaded: %.1f m/s off the touch; W A S D re-aim",
                                  shot ? "shot" : pass ? "pass" : "touch",
                                  decision.aim_speed);
                    status->detail = text;
                }
                else if (decision.skill == Skill::RECEIVE && decision.pass_incoming) {
                    status->detail = rondo ? "pass coming: Q to pass it on first time, Space to hit it first time, hold a "
                                             "direction for a first touch, or let it come"
                                           : "pass coming: hold a direction for a first touch, Q to pass it first time, "
                                             "Space to shoot it first time, or let it come";
                }
                else if (decision.skill == Skill::WALK) {
                    const auto& t = decision.twist;
                    if (t[0] != 0.0 || t[1] != 0.0 || t[2] != 0.0) {
                        std::snprintf(text, sizeof(text), "walking: %.1f m/s forward, %.1f m/s left, turning %.1f rad/s", t[0], t[1], t[2]);
                        status->detail = text;
                    }
                    else {
                        status->detail = rondo ? "standing; W A S D walk to the ball" : "standing; W A S D walk, Q plays a pass, E a ball at the feet";
                    }
                }
                else if (decision.skill == Skill::RECEIVE) {
                    status->detail = "waiting for a pass (Q), then trapping it; W walks if the ball is away";
                }
                else if (decision.skill == Skill::KICK) {
                    if (decision.air) {
                        status->skill = "AIR SHOT";
                    }
                    std::snprintf(text,
                                  sizeof(text),
                                  "shot at %.1f m/s; W to walk after it, Q for a new pass",
                                  std::hypot(decision.command_world[0], decision.command_world[1]));
                    status->detail = text;
                }
                else if (decision.command_world[0] != 0.0 || decision.command_world[1] != 0.0) {
                    std::snprintf(text,
                                  sizeof(text),
                                  "ball to go (%.1f, %.1f) m/s",
                                  decision.command_world[0],
                                  decision.command_world[1]);
                    status->detail = text;
                }
                else {
                    status->detail = rondo ? "holding the ball; Space to pass" : "holding the ball";
                }
                break;
        }
        if (defending() && agent.phase == Phase::PLAYING) {
            const auto& t = decision.twist;
            status->skill = "DEFENDER";
            if (t[0] != 0.0 || t[2] != 0.0) {
                std::snprintf(text, sizeof(text), "walking: %.1f m/s, turning %.1f rad/s", t[0], t[2]);
                status->detail = text;
            }
            else {
                status->detail = "W A S D walk at the ball; touch it to win it";
            }
        }
        if (defending()) {
            status->skill = (sharing() ? "RONDO VS EKF: " : tracking() ? "RONDO VS SOLO EYES: " : "RONDO (DEFENDING): ") + status->skill;
            std::snprintf(text, sizeof(text), "  |  PASSES %d  |  won %d, got away %d", passes_, steals_, lost_);
            status->detail += text;
        }
        if (tracking()) {
            int seeing = 0;
            for (std::size_t k = 0; k < 3; ++k) {
                seeing += agents_[k].sight.seen ? 1 : 0;
            }
            const auto& dw = agents_[3].sample.world;
            auto off = [&](const skill::DefenderEkf& e) {
                char one[24];
                if (e.valid()) {
                    std::snprintf(one, sizeof(one), "%.1f m", std::hypot(e.state()[0] - dw.robot_x, e.state()[1] - dw.robot_y));
                }
                else {
                    std::snprintf(one, sizeof(one), "?");
                }
                return std::string(one);
            };
            std::snprintf(text, sizeof(text), "  |  seen by %d of 3, ", seeing);
            status->detail += text;
            status->detail += sharing() ? "their shared track is " + off(ekf_) + " off"
                                        : "their own tracks are " + off(agents_[0].ekf) + " / " + off(agents_[1].ekf) + " / "
                                              + off(agents_[2].ekf) + " off";
        }
        else if (rondo) {
            std::snprintf(text, sizeof(text), "RONDO, robot %d: ", agent.robot_id);
            status->skill = text + status->skill;
            std::snprintf(text, sizeof(text), "  |  PASSES IN A ROW %d (best %d)  |  intercepted %d, lost %d", passes_, best_, steals_, lost_);
            status->detail += text;
        }
        if (sim_time < note_until_ && sim_time > note_until_ - 10.0) {
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
