// MatchReferee against the real M-Field scene and config/match.yaml, through the real GameController
// library: robot identity, the side mapping (which end is whose goal) and a game's progress.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <memory>
#include <mujoco/mujoco.h>
#include <sstream>
#include <string>
#include <vector>

#include "module/MatchReferee/src/HoldController.hpp"
#include "module/MatchReferee/src/MatchRefereeLogic.hpp"
#include "module/SdkBridge/src/OdometryModel.hpp"
#include "module/Simulation/src/SimCore.hpp"
#include "shared/util/Config.hpp"

using namespace k1sim;
using namespace k1sim::module::matchreferee;

namespace {

    int failures = 0;

#define CHECK(condition)                                                             \
    do {                                                                             \
        if (!(condition)) {                                                          \
            std::fprintf(stderr, "FAIL %s:%d %s\n", __FILE__, __LINE__, #condition); \
            ++failures;                                                              \
        }                                                                            \
    } while (0)

    constexpr std::uint8_t HOME = 125;
    constexpr std::uint8_t AWAY = 126;

    std::string competition(const MatchConfig& cfg) {
        std::ifstream file(std::string(K1SIM_GC_PARAMS_DIR) + "/" + cfg.competition + ".yaml");
        std::stringstream content;
        content << file.rdbuf();
        return content.str();
    }

    // A --game <n> --on-field-positions scene, not stepped: tests move things directly.
    struct Match {
        explicit Match(int n, bool hold = false, bool skip_phases = false) {
            auto sim_cfg = config::load("simulation.yaml");
            auto gains   = config::load("gains.yaml");
            SimCore::Config cfg;
            config::Game game = config::game(sim_cfg, n, true);
            cfg.model_path    = config::field_scene(sim_cfg, game.field);
            cfg.robots        = static_cast<int>(game.spawns.size());
            cfg.spawns        = game.spawns;
            std::vector<RobotIdentity> roster;
            for (int i = 0; i < 2 * n; ++i) {
                roster.push_back({i + 1, i, HOME + i / n, i % n + 1, i});
            }
            cfg.identities = roster;
            std::array<double, JOINT_COUNT> ready{};
            for (std::size_t i = 0; i < JOINT_COUNT; ++i) {
                cfg.kp[i]                  = gains["kp"][i].as<double>();
                cfg.kd[i]                  = gains["kd"][i].as<double>();
                cfg.ready_pose_fallback[i] = gains["ready_pose"][i].as<double>();
                ready[i]                   = cfg.ready_pose_fallback[i];
            }
            sim = std::make_unique<SimCore>(std::move(cfg));
            sim->load_model();

            MatchConfig match          = load_config(config::load("match.yaml"));
            match.auto_start_after     = -1.0;
            match.hold_robots          = hold;
            match.skip_kick_off_phases = skip_phases;
            logic                      = std::make_unique<MatchRefereeLogic>(match,
                                                        with_penalty_time(competition(match), match.penalty_seconds),
                                                        sim->model(),
                                                        roster,
                                                        ready);
        }

        // Runs the game for `seconds`, in the module's 20 ms updates and 500 ms control messages.
        void run(double seconds) {
            const int ticks = static_cast<int>(std::lround(seconds / 0.02));
            for (int i = 0; i < ticks; ++i) {
                time += 0.02;
                mj_kinematics(sim->model(), sim->data());
                logic->update(sim->data(), time, 20);
                if (++tick % 25 == 0) {
                    logic->report_changes(logic->control_message());
                }
            }
        }

        // Runs until `state`, or gives up after `limit` seconds.
        bool run_until(gc::State state, double limit) {
            for (double t = 0.0; t < limit; t += 0.1) {
                run(0.1);
                if (logic->control_message().state == state) {
                    return true;
                }
            }
            return false;
        }

        void place_ball(double x, double y, double z = 0.0785) {
            const mjModel* m = sim->model();
            k1sim::module::supervisor::place_free_body_by_geom_center(m,
                                                                      sim->data(),
                                                                      mj_name2id(m, mjOBJ_BODY, "ball"),
                                                                      mj_name2id(m, mjOBJ_GEOM, "ball"),
                                                                      x,
                                                                      y,
                                                                      z);
        }

        std::unique_ptr<SimCore> sim;
        std::unique_ptr<MatchRefereeLogic> logic;
        double time = 0.0;
        int tick    = 0;
    };

    void test_identity() {
        Match match(3);
        const auto& players = match.logic->players();
        CHECK(players.size() == 6);
        mj_kinematics(match.sim->model(), match.sim->data());
        for (std::size_t i = 0; i < players.size(); ++i) {
            const bool home = i < 3;
            CHECK((players[i].side == RoboCup::Side::home) == home);
            CHECK(players[i].identity.player_id == static_cast<int>(i % 3) + 1);
            // Each team's bodies are in its own half, which is what ties a number to a robot.
            const double x = match.sim->data()->xpos[3 * players[i].root_body];
            CHECK(home ? x < 0.0 : x > 0.0);
            // The contact lookup relies on a body's root being its Trunk, for attached copies too.
            CHECK(match.sim->model()->body_rootid[players[i].root_body] == players[i].root_body);
            // Every robot needs its own head camera for module::MatchCameras.
            const std::string camera = robot_prefix(players[i].identity.model_index) + "head";
            CHECK(mj_name2id(match.sim->model(), mjOBJ_CAMERA, camera.c_str()) >= 0);
        }
        CHECK(match.logic->expected_players().size() == 6);

        const auto packet = match.logic->control_message();
        // The team defending the left goal (+x) comes first: that's away.
        CHECK(packet.teams[0].team_id == AWAY);
        CHECK(packet.teams[1].team_id == HOME);
        CHECK(packet.teams[0].goalkeeper == 2);
        CHECK(packet.teams[1].goalkeeper == 2);
        CHECK(packet.state == gc::State::INITIAL);
    }

    // Packet numbers count messages sent, so running the game mustn't use them up.
    void test_packet_number() {
        Match match(1);
        const std::uint8_t first = match.logic->control_message().packet_number;
        for (int i = 0; i < 100; ++i) {
            match.logic->update(match.sim->data(), i * 0.02, 20);
        }
        CHECK(match.logic->control_message().packet_number == static_cast<std::uint8_t>(first + 1));
    }

    void test_game_reaches_playing() {
        Match match(1);
        match.logic->start();
        CHECK(match.logic->control_message().state == gc::State::READY);
        CHECK(match.run_until(gc::State::SET, 60.0));
        CHECK(match.run_until(gc::State::PLAYING, 60.0));
    }

    // Straight to Playing with the ball on the centre mark, at the start and after a goal.
    void test_skip_kick_off_phases() {
        Match match(1, true, true);
        const auto ball_x = [&match] {
            mj_kinematics(match.sim->model(), match.sim->data());
            return match.sim->data()->geom_xpos[3 * mj_name2id(match.sim->model(), mjOBJ_GEOM, "ball")];
        };
        match.place_ball(1.0, 1.0);
        match.logic->start();
        match.logic->start();  // as auto-start does on every update
        match.run(0.1);
        CHECK(match.logic->control_message().state == gc::State::PLAYING);
        CHECK(std::abs(ball_x()) < 1e-6);

        match.run(15.0);
        match.place_ball(7.3, 0.0);
        match.run(0.1);
        const auto packet = match.logic->control_message();
        CHECK(MatchRefereeLogic::score(packet, HOME) == 1);
        CHECK(packet.state == gc::State::PLAYING);
        match.run(0.6);
        CHECK(match.logic->messages().find("Goal! Home 1 - 0 Away") != std::string::npos);
        CHECK(std::abs(ball_x()) < 1e-6);
    }

    // A penalty lasts penalty_seconds however often it happens, and the label says why.
    void test_penalty_time() {
        Match match(1, true, true);
        match.logic->start();
        match.run(1.0);
        const int root = match.logic->players()[0].map.root_qpos_adr;
        for (int offence = 0; offence < 3; ++offence) {
            match.sim->data()->qpos[root + 1] = 7.0;  // well off the field
            match.run(1.0);
            const auto packet = match.logic->control_message();
            const auto& robot = packet.teams[1].players[0];
            CHECK(robot.penalty_state == gc::PenaltyState::LEAVING_THE_FIELD);
            CHECK(robot.penalised_time_left <= 10);
            CHECK(match.logic->held()[0]);
            CHECK(match.logic->messages().find("Home 1 left the field") != std::string::npos);
            match.run(11.0);
            CHECK(match.logic->control_message().teams[1].players[0].penalty_state == gc::PenaltyState::UNPENALISED);
            CHECK(match.logic->messages().find("Home 1 back in play") != std::string::npos);
            CHECK(match.logic->robot_labels() == std::vector<std::string>({"Home 1", "Away 1"}));
        }
    }

    // Past the +x goal line is the goal away defends, so it must score for home, and vice versa.
    void test_goal_side(double x, std::uint8_t scorer, std::uint8_t other) {
        Match match(1);
        match.logic->start();
        CHECK(match.run_until(gc::State::PLAYING, 120.0));
        // Long enough for the kick-off restriction to end, so that the goal counts.
        match.run(15.0);
        match.logic->report_changes(match.logic->control_message());
        match.place_ball(x, 0.0);
        match.time += 0.02;
        mj_kinematics(match.sim->model(), match.sim->data());
        match.logic->update(match.sim->data(), match.time, 20);
        const auto packet = match.logic->control_message();
        const auto events = match.logic->report_changes(packet);
        CHECK(MatchRefereeLogic::score(packet, scorer) == 1);
        CHECK(MatchRefereeLogic::score(packet, other) == 0);
        const std::string score = scorer == HOME ? "GOAL: home 1 - 0 away" : "GOAL: home 0 - 1 away";
        CHECK(std::find(events.begin(), events.end(), score) != events.end());
    }

    // Robots wait for the kick-off, are held until Playing and released during it.
    void test_hold_through_kick_off() {
        Match match(1, true, true);
        const auto all = [&match](bool value) {
            const auto& held = match.logic->held();
            return std::all_of(held.begin(), held.end(), [value](bool h) { return h == value; });
        };
        match.run(1.0);
        CHECK(all(true));
        CHECK(match.logic->primary_label() == "Start");

        match.logic->primary_action();
        CHECK(match.run_until(gc::State::PLAYING, 20.0));
        match.run(1.0);
        CHECK(all(false));
        CHECK(match.logic->primary_label() == "Pause");

        match.logic->primary_action();
        match.run(1.0);
        CHECK(all(true));
        CHECK(match.logic->primary_label() == "Resume");
        CHECK(match.logic->summary(2).find("PAUSED") != std::string::npos);
        CHECK(match.logic->summary(2).find("PLAYING") == std::string::npos);
        CHECK(match.logic->messages().find("s  Paused") != std::string::npos);
        match.logic->primary_action();
        match.run(1.0);
        CHECK(all(false));
    }

    struct CountingController : StepController {
        void step(const mjModel* /*m*/, mjData* d) override {
            ++steps;
            d->ctrl[0] = 12.5;
        }
        int mode() const override {
            return 3;
        }
        int fall_state() const override {
            return 4;
        }
        bool getting_up() const override {
            return true;
        }
        int steps = 0;
    };

    void test_hold_controller() {
        Match match(1);
        const mjModel* m = match.sim->model();
        mjData* d        = match.sim->data();
        auto gains       = config::load("gains.yaml");
        std::array<double, JOINT_COUNT> ready{};
        for (std::size_t i = 0; i < JOINT_COUNT; ++i) {
            ready[i] = gains["ready_pose"][i].as<double>();
        }
        const double stand = 0.555;
        const ModelMap map = ModelMap::build(m);
        auto inner         = std::make_shared<CountingController>();
        HoldController hold(inner, "", ready, stand);

        // Caught mid-step and leaning: moved upright into the ready pose without falling, facing the
        // same way, while the player's controller keeps running.
        double* root   = d->qpos + map.root_qpos_adr;
        const double x = root[0];
        d->qpos[map.qpos_adr[10]] += 0.4;
        root[3]       = std::cos(0.1);
        root[4]       = std::sin(0.1);
        root[5]       = 0.0;
        root[6]       = 0.0;
        double lowest = root[2];
        for (int i = 0; i < 1500; ++i) {
            hold.step(m, d);
            mj_step(m, d);
            lowest = std::min(lowest, root[2]);
        }
        CHECK(inner->steps == 1500);
        CHECK(lowest > 0.5);
        CHECK(std::abs(d->qpos[map.qpos_adr[10]] - ready[10]) < 1e-3);
        CHECK(std::abs(root[2] - stand) < 1e-3);
        CHECK(std::abs(root[4]) < 1e-3);
        CHECK(std::abs(root[0] - x) < 1e-3);
        CHECK(hold.mode() == 3 && hold.fall_state() == 4 && hold.getting_up());

        // Two held robots overlapping, as positioning them can leave them, are parted and don't get
        // fired apart on release.
        {
            Match pair(1);
            const mjModel* pm = pair.sim->model();
            mjData* pd        = pair.sim->data();
            const ModelMap a  = ModelMap::build(pm);
            const ModelMap b  = ModelMap::build(pm, "sub01_");
            HoldController hold_a(inner, "", ready, stand);
            HoldController hold_b(inner, "sub01_", ready, stand);
            pd->qpos[b.root_qpos_adr]     = pd->qpos[a.root_qpos_adr] + 0.08;
            pd->qpos[b.root_qpos_adr + 1] = pd->qpos[a.root_qpos_adr + 1];
            const auto deepest            = [&] {
                double depth = 0.0;
                for (int i = 0; i < pd->ncon; ++i) {
                    const int r1 = pm->body_rootid[pm->geom_bodyid[pd->contact[i].geom[0]]];
                    const int r2 = pm->body_rootid[pm->geom_bodyid[pd->contact[i].geom[1]]];
                    if (r1 != r2 && r1 != 0 && r2 != 0) {
                        depth = std::min(depth, pd->contact[i].dist);
                    }
                }
                return depth;
            };
            for (int i = 0; i < 1500; ++i) {
                hold_a.step(pm, pd);
                hold_b.step(pm, pd);
                mj_step(pm, pd);
            }
            CHECK(deepest() > -0.005);
            hold_a.set_held(false);
            hold_b.set_held(false);
            double highest = 0.0;
            for (int i = 0; i < 300; ++i) {
                hold_a.step(pm, pd);
                hold_b.step(pm, pd);
                mj_step(pm, pd);
                highest = std::max({highest, pd->qpos[a.root_qpos_adr + 2], pd->qpos[b.root_qpos_adr + 2]});
            }
            CHECK(highest < 0.6);
        }

        // A fresh hold on a robot that's down leaves it to its player, so that it can get up.
        const double z = root[2];
        HoldController fallen(inner, "", ready, stand);
        root[2] = 0.1;
        fallen.step(m, d);
        CHECK(d->ctrl[0] == 12.5);
        CHECK(root[2] == 0.1);
        root[2] = z;

        // Released: the player's commands go through.
        hold.set_held(false);
        hold.step(m, d);
        CHECK(d->ctrl[0] == 12.5);

        // A world reset drops the frozen pose, or it would drag the robot back.
        hold.set_held(true);
        hold.step(m, d);
        hold.reset();
        root[0] += 1.0;
        const double moved = root[0];
        hold.step(m, d);
        CHECK(root[0] == moved);
    }

    // Through mjcb_control like the module, to show contacts are complete at that point in mj_step.
    MatchRefereeLogic* step_logic = nullptr;
    void on_step(const mjModel* /*m*/, mjData* d) {
        step_logic->record_contacts(d);
    }

    void test_touch_is_recorded() {
        Match match(2);
        mjData* d          = match.sim->data();
        const mjModel* m   = match.sim->model();
        const auto& player = match.logic->players()[3];  // away player 2: an attached copy, not the main robot
        mj_kinematics(m, d);
        CHECK(match.logic->touching() == std::vector<bool>(4, false));

        // Drop the ball onto the robot.
        const double* pos = d->xpos + 3 * player.root_body;
        match.place_ball(pos[0], pos[1], pos[2] + 0.6);
        step_logic   = match.logic.get();
        mjcb_control = on_step;
        for (int i = 0; i < 1000 && !match.logic->touching()[3]; ++i) {
            mj_step(m, d);
        }
        mjcb_control = nullptr;
        CHECK(match.logic->touching() == std::vector<bool>({false, false, false, true}));
    }

    // A robot that starts facing -x and walks forward has walked forward, not backward.
    void test_odometry_from_start() {
        k1sim::module::sdkbridge::OdometryModel::Config cfg;
        cfg.from_start = true;
        k1sim::module::sdkbridge::OdometryModel odometry(cfg);
        odometry.update(0.0, 2.0, 1.0, M_PI, 0.0, 0.0, 0.0);
        const auto result = odometry.update(1.0, 1.0, 1.0, M_PI, -1.0, 0.0, 0.0);
        CHECK(std::abs(result.x - 1.0) < 1e-9);
        CHECK(std::abs(result.y) < 1e-9);
        CHECK(std::abs(result.yaw) < 1e-9);
    }

    void test_roster() {
        MatchRoster roster({{1, 1}, {2, 1}}, 4.0);

        CHECK(roster.on_status({1, 1, false}, 0.0).at(0) == "online: team 1 player 1");
        CHECK(roster.on_status({1, 1, false}, 1.0).empty());

        // The check that catches a robot started with the wrong ID.
        CHECK(roster.on_status({3, 2, false}, 1.0).at(0) == "UNEXPECTED player online: team 2 player 3");

        CHECK(roster.on_status({1, 1, false}, 3.0).empty());
        CHECK(roster.expire(4.5).empty());
        CHECK(roster.expire(5.5).at(0) == "offline: team 2 player 3");
        CHECK(roster.expire(7.5).at(0) == "offline: team 1 player 1");

        std::array<std::uint8_t, STATUS_MESSAGE_SIZE> bytes{'R', 'G', 'r', 't', 4, 3, 1, 1};
        const auto status = parse_status(bytes.data(), bytes.size());
        CHECK(status && status->player == 3 && status->team == 1 && status->fallen);
        CHECK(!parse_status(bytes.data(), bytes.size() - 1));
        bytes[4] = 3;
        CHECK(!parse_status(bytes.data(), bytes.size()));
    }

}  // namespace

int main() {
    test_roster();
    test_odometry_from_start();
    test_identity();
    test_packet_number();
    test_game_reaches_playing();
    test_goal_side(7.3, HOME, AWAY);
    test_goal_side(-7.3, AWAY, HOME);
    test_skip_kick_off_phases();
    test_penalty_time();
    test_hold_through_kick_off();
    test_hold_controller();
    test_touch_is_recorded();
    if (failures == 0) {
        std::puts("test_match_referee: PASS");
    }
    return failures == 0 ? 0 : 1;
}
