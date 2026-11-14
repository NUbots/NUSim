#ifndef K1SIM_MODULE_MATCHREFEREE_MATCHREFEREELOGIC_HPP
#define K1SIM_MODULE_MATCHREFEREE_MATCHREFEREELOGIC_HPP

#include <GameController.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <functional>
#include <memory>
#include <mujoco/mujoco.h>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "module/MatchReferee/src/MatchRefereeConfig.hpp"
#include "module/MatchReferee/src/MatchRoster.hpp"
#include "module/Supervisor/src/GameControllerPacket.hpp"
#include "module/Supervisor/src/SupervisorPlacement.hpp"
#include "shared/k1/JointIndex.hpp"
#include "shared/sim/ModelMap.hpp"
#include "shared/sim/RobotIdentity.hpp"

// The game itself: owns the GameController and automatic referee, feeds them the MuJoCo world and
// carries out what they ask for. Free of NUClear and sockets so it can be driven directly against
// a loaded model, like SupervisorLogic.
namespace k1sim::module::matchreferee {

    namespace gc = k1sim::module::supervisor::gc;

    // A roster robot, with what the game needs to find and place it.
    struct MatchPlayer {
        RobotIdentity identity;
        RoboCup::Side side = RoboCup::Side::home;
        int root_body      = -1;
        ModelMap map;
    };

    class MatchRefereeLogic {
    public:
        static constexpr std::size_t MAX_MESSAGES = 6;

        // `competition_yaml` is the content of a GameController params.yaml, `roster` lists the first
        // team then the second (see RobotRoster.hpp). `m` must outlive this.
        MatchRefereeLogic(MatchConfig cfg,
                          const std::string& competition_yaml,
                          const mjModel* m,
                          const std::vector<RobotIdentity>& roster,
                          std::array<double, JOINT_COUNT> ready)
            : cfg_(std::move(cfg)), m_(m), ready_(ready) {
            home_team_ = static_cast<std::uint8_t>(roster.front().team_id);
            away_team_ = static_cast<std::uint8_t>(roster.back().team_id);

            ball_body_ = mj_name2id(m, mjOBJ_BODY, "ball");
            ball_geom_ = mj_name2id(m, mjOBJ_GEOM, "ball");
            if (ball_body_ < 0 || ball_geom_ < 0) {
                throw std::runtime_error("MatchReferee: the scene has no ball");
            }

            player_of_root_.assign(m->nbody, -1);
            for (std::size_t i = 0; i < roster.size(); ++i) {
                const RobotIdentity& identity = roster[i];
                const int body                = mj_name2id(m, mjOBJ_BODY, identity.body().c_str());
                if (body < 0) {
                    throw std::runtime_error("MatchReferee: no robot body '" + identity.body() + "' in the model");
                }
                players_.push_back({identity,
                                    identity.team_id == home_team_ ? RoboCup::Side::home : RoboCup::Side::away,
                                    body,
                                    ModelMap::build(m, robot_prefix(identity.model_index))});
                player_of_root_[body] = static_cast<int>(i);
            }
            touching_.assign(players_.size(), false);
            held_.assign(players_.size(), cfg_.hold_robots);

            competition_.reset(
                check(RoboCup::gc_competition_params_new(reinterpret_cast<const std::uint8_t*>(competition_yaml.data()),
                                                         competition_yaml.size())));

            const RoboCup::TeamParams home{home_team_, RoboCup::Color::blue, RoboCup::Color::blue};
            const RoboCup::TeamParams away{away_team_, RoboCup::Color::red, RoboCup::Color::red};
            // The left goal is at +x in the autoreferee's frame, and home starts in the -x half.
            // A simulation doesn't need the delay that covers a human referee's reaction time.
            params_.reset(check(RoboCup::gc_params_new(competition_.get(),
                                                       &home,
                                                       &away,
                                                       cfg_.kick_off_side,
                                                       RoboCup::SideMapping::homeDefendsRightGoal,
                                                       RoboCup::TestParams{true, false})));
            game_.reset(check(RoboCup::gc_new(params_.get())));

            RoboCup::AutorefereeConfig referee{};
            referee.features                 = cfg_.features;
            referee.field                    = cfg_.field;
            referee.timeUntilSwitchToSet     = static_cast<std::uint64_t>(cfg_.time_until_switch_to_set * 1000.0);
            referee.timeUntilSwitchToPlaying = static_cast<std::uint64_t>(cfg_.time_until_switch_to_playing * 1000.0);
            referee.penaltyPlacementDistance = cfg_.penalty_placement_distance;
            referee.userptr                  = this;
            referee.placeBall                = &MatchRefereeLogic::on_place_ball;
            referee.placePlayer              = &MatchRefereeLogic::on_place_player;
            referee.whistle                  = &MatchRefereeLogic::on_whistle;
            autoreferee_.reset(check(RoboCup::gc_autoreferee_new(&referee)));

            // With --on-field-positions the second robot of a team is the goalkeeper.
            if (players_.size() >= 4) {
                for (const auto side : {RoboCup::Side::home, RoboCup::Side::away}) {
                    apply(RoboCup::gc_action_select_goalkeeper(side, 2));
                }
            }
            else {
                // One a side: the GameController's default goalkeeper is player 1, and a lone player told
                // it is the goalkeeper never leaves its goal. Name player 2, who isn't there, instead.
                for (const auto side : {RoboCup::Side::home, RoboCup::Side::away}) {
                    apply(RoboCup::gc_action_select_goalkeeper(side, 2));
                }
            }
        }

        // Records which players touch the ball. Must run every physics step, with the sim mutex
        // held (the mjcb_control callback does both): contact in a kick can be over between two
        // update() calls.
        void record_contacts(const mjData* d) {
            for (int i = 0; i < d->ncon; ++i) {
                const mjContact& contact = d->contact[i];
                const int body1          = m_->geom_bodyid[contact.geom[0]];
                const int body2          = m_->geom_bodyid[contact.geom[1]];
                if (body1 != ball_body_ && body2 != ball_body_) {
                    continue;
                }
                const int player = player_of_root_[m_->body_rootid[body1 == ball_body_ ? body2 : body1]];
                if (player >= 0) {
                    touching_[player] = true;
                }
            }
        }

        // Advances the game by `dt_ms` of sim time. Caller holds the sim mutex. Returns what to log.
        std::vector<std::string> update(mjData* d, double sim_time, std::uint64_t dt_ms) {
            d_ = d;
            RoboCup::gc_seek(game_.get(), dt_ms);
            advance_kick_off(d, sim_time);
            remove_incapable(d, sim_time);

            for (std::size_t i = 0; i < players_.size(); ++i) {
                if (touching_[i]) {
                    RoboCup::gc_autoreferee_ball_contact(autoreferee_.get(),
                                                         game_.get(),
                                                         players_[i].side,
                                                         static_cast<std::uint8_t>(players_[i].identity.player_id));
                    touching_[i] = false;
                }
            }

            if (!started_ && cfg_.auto_start_after >= 0.0 && sim_time >= cfg_.auto_start_after) {
                start();
            }

            const RoboCup::AutorefereeWorld world = observe(d);
            RoboCup::gc_autoreferee_update(autoreferee_.get(), game_.get(), &world);
            d_ = nullptr;

            // One step per update, so that the autoreferee sees Ready and places the ball.
            if (cfg_.skip_kick_off_phases && !manual_set_play_) {
                // A game started with the Start button takes every kick-off the same way, so the one
                // after a goal also waits in Ready for the robots to walk back before Set and Playing.
                const auto now = peek();
                if (staged_kick_offs_ && now && now->state == gc::State::READY && now->set_play == gc::SetPlay::NONE) {
                    manual_set_play_ = true;
                    kick_off_        = KickOff::READY;
                    phase_started_   = -1.0;
                    events_.push_back("kick-off: Ready until the robots are back in position");
                }
                else if (!RoboCup::gc_apply(game_.get(),
                                            RoboCup::gc_action_wait_for_set_play(),
                                            RoboCup::ActionSource::autoreferee)) {
                    RoboCup::gc_apply(game_.get(),
                                      RoboCup::gc_action_free_set_play(),
                                      RoboCup::ActionSource::autoreferee);
                }
            }

            std::vector<std::string> events = std::move(events_);
            events_.clear();
            return events;
        }

        // Starts the kick-off, taking the game from Initial to Ready. Does nothing after the first time.
        void start() {
            if (started_) {
                return;
            }
            started_                 = true;
            const RoboCup::Side side = cfg_.kick_off_side;
            apply(RoboCup::gc_action_start_set_play(&side, RoboCup::SetPlay::kickOff));
        }

        // Kick-off by hand, one phase at a time: ready() takes the game from Initial to Ready and holds
        // it there, set() moves on to Set, and release_set_play() starts play.
        void ready() {
            if (started_) {
                return;
            }
            manual_set_play_ = true;
            start();
        }

        // The Start button: Ready now, then Set and Playing by themselves (see advance_kick_off()).
        void start_kick_off() {
            if (started_) {
                return;
            }
            ready();
            staged_kick_offs_ = true;
            kick_off_         = KickOff::READY;
            phase_started_    = -1.0;
        }
        void set() {
            if (manual_set_play_) {
                apply(RoboCup::gc_action_wait_for_set_play());
            }
        }

        // Calls a set play by hand (the viewer's Testing Room). It stays in force, instead of being
        // waved straight through like a kick-off, until release_set_play().
        bool call_set_play(RoboCup::Side side, RoboCup::SetPlay set_play) {
            if (!apply(RoboCup::gc_action_start_set_play(&side, set_play))) {
                return false;
            }
            manual_set_play_ = true;
            return true;
        }

        // The Playing button: kicks off if the game hasn't started, then sets the ball free.
        void play() {
            ready();
            release_set_play();
        }

        // Ball free: ends a set play called by hand, as the referee's whistle would.
        void release_set_play() {
            if (!manual_set_play_) {
                return;
            }
            manual_set_play_ = false;
            apply(RoboCup::gc_action_wait_for_set_play());
            apply(RoboCup::gc_action_free_set_play());
        }

        // What the panel's main button does: starts the game, or pauses and resumes it.
        void primary_action() {
            if (!last_ || last_->state == gc::State::INITIAL) {
                start_kick_off();
            }
            else if (last_->state != gc::State::FINISHED) {
                apply(RoboCup::gc_action_stop_play(last_->stopped != 0));
            }
        }

        std::string primary_label() const {
            if (!last_ || last_->state == gc::State::INITIAL) {
                return "Start";
            }
            if (last_->state == gc::State::FINISHED) {
                return "";
            }
            return last_->stopped != 0 ? "Resume" : "Pause";
        }

        // Applies a GameController action as if a person had pressed its button.
        bool apply(RoboCup::VAction* action) {
            return RoboCup::gc_apply(game_.get(), action, RoboCup::ActionSource::user);
        }

        // A team's score. Control messages list the team defending the left goal first, so with
        // home in the -x half teams[0] is away: always look teams up by number.
        static int score(const gc::GameControllerPacket& packet, std::uint8_t team) {
            return find_team(packet, team).score;
        }

        // The next control message for robots. Each call uses up a packet number, so call this
        // only to send one.
        // The game as it stands this instant; control messages are up to half a second old. Doesn't use
        // up a packet number.
        std::optional<gc::GameControllerPacket> peek() const {
            std::array<std::uint8_t, sizeof(gc::GameControllerPacket)> bytes{};
            gc::GameControllerPacket packet{};
            if (!RoboCup::gc_read(game_.get(), packet_number_, false, bytes.data(), bytes.size())
                || !gc::try_parse(bytes.data(), bytes.size(), packet)) {
                return std::nullopt;
            }
            return packet;
        }

        gc::GameControllerPacket control_message() {
            std::array<std::uint8_t, sizeof(gc::GameControllerPacket)> bytes{};
            gc::GameControllerPacket packet{};
            if (!RoboCup::gc_read(game_.get(), packet_number_++, false, bytes.data(), bytes.size())
                || !gc::try_parse(bytes.data(), bytes.size(), packet)) {
                throw std::runtime_error(
                    "MatchReferee: the GameController API's control message doesn't match "
                    "GameControllerPacket.hpp (protocol version mismatch?)");
            }
            return packet;
        }

        // What changed since the last control message passed here. Also works out who is held.
        std::vector<std::string> report_changes(const gc::GameControllerPacket& now) {
            std::vector<std::string> events;
            if (!last_ || last_->state != now.state) {
                events.push_back(std::string("state -> ") + gc::state_name(now.state));
            }
            const int home = score(now, home_team_);
            const int away = score(now, away_team_);
            if (last_ && (home != score(*last_, home_team_) || away != score(*last_, away_team_))) {
                events.push_back("GOAL: home " + std::to_string(home) + " - " + std::to_string(away) + " away");
            }
            if (last_ && last_->set_play != now.set_play) {
                events.push_back("set play -> " + std::to_string(static_cast<int>(now.set_play)));
            }
            for (const auto& player : players_) {
                const gc::Robot& robot = robot_in(now, player);
                if (last_ && robot_in(*last_, player).penalty_state != robot.penalty_state) {
                    const bool free = robot.penalty_state == gc::PenaltyState::UNPENALISED;
                    events.push_back("robot " + std::to_string(player.identity.robot_id)
                                     + (free ? " back in play"
                                             : std::string(" penalised: ") + gc::penalty_state_name(robot.penalty_state)
                                                   + " for " + std::to_string(robot.penalised_time_left) + " s"));
                    add_message(now, name(player) + (free ? " back in play" : penalty_message(robot.penalty_state)));
                }
            }
            report_game(now);
            last_ = now;

            const auto was_held = held_;
            for (std::size_t i = 0; i < players_.size(); ++i) {
                held_[i] = cfg_.hold_robots
                           && (now.state != gc::State::PLAYING || now.stopped != 0
                               || robot_in(now, players_[i]).penalty_state != gc::PenaltyState::UNPENALISED);
            }
            if (held_ != was_held) {
                const auto held = std::count(held_.begin(), held_.end(), true);
                events.push_back("holding " + std::to_string(held) + " of " + std::to_string(held_.size()) + " robots");
            }
            return events;
        }

        // Where every robot is, for headless runs. Caller holds the sim mutex.
        std::string positions(const mjData* d) const {
            std::string text;
            for (const auto& player : players_) {
                char position[48];
                std::snprintf(position,
                              sizeof(position),
                              "%d: (%.1f, %.1f)  ",
                              player.identity.robot_id,
                              d->xpos[3 * player.root_body],
                              d->xpos[3 * player.root_body + 1]);
                text += position;
            }
            return text;
        }

        // What to show above each of players().
        std::vector<std::string> robot_labels() const {
            std::vector<std::string> labels;
            for (const auto& player : players_) {
                labels.push_back(name(player));
            }
            return labels;
        }

        // The game's recent updates, oldest first.
        std::string messages() const {
            std::string text;
            for (const auto& message : messages_) {
                text += message + "\n";
            }
            return text;
        }

        // The summary's facts as values. False until the first control message.
        bool scoreboard(int& home, int& away, int& seconds, std::string& state, int& players) const {
            if (!last_) {
                return false;
            }
            home    = score(*last_, home_team_);
            away    = score(*last_, away_team_);
            seconds = last_->secs_remaining;
            state   = last_->stopped != 0 ? "PAUSED" : gc::state_name(last_->state);
            players = static_cast<int>(players_.size());
            return true;
        }

        std::string summary(int connected) const {
            if (!last_) {
                return "";
            }
            const int seconds = last_->secs_remaining;
            char text[128];
            std::snprintf(text,
                          sizeof(text),
                          "Home %d - %d Away\n%s\n%d:%02d\nRobots: %d/%zu",
                          score(*last_, home_team_),
                          score(*last_, away_team_),
                          last_->stopped != 0 ? "PAUSED" : gc::state_name(last_->state),
                          seconds / 60,
                          std::abs(seconds % 60),
                          connected,
                          players_.size());
            return text;
        }

        std::set<MatchRoster::Player> expected_players() const {
            std::set<MatchRoster::Player> players;
            for (const auto& player : players_) {
                players.insert({static_cast<std::uint8_t>(player.identity.team_id),
                                static_cast<std::uint8_t>(player.identity.player_id)});
            }
            return players;
        }

        const std::vector<MatchPlayer>& players() const {
            return players_;
        }

        // Which of players() touched the ball since the last update().
        const std::vector<bool>& touching() const {
            return touching_;
        }

        // Which of players() should be held in the ready pose.
        const std::vector<bool>& held() const {
            return held_;
        }

    private:
        template <typename T, void (*Destroy)(T*)>
        struct Deleter {
            void operator()(T* p) const {
                Destroy(p);
            }
        };
        template <typename T, void (*Destroy)(T*)>
        using Handle = std::unique_ptr<T, Deleter<T, Destroy>>;

        template <typename T>
        static T* check(T* handle) {
            if (handle == nullptr) {
                throw std::runtime_error(std::string("MatchReferee: ") + RoboCup::gc_last_error());
            }
            return handle;
        }

        std::string name(const MatchPlayer& player) const {
            return (player.side == RoboCup::Side::home ? "Home " : "Away ") + std::to_string(player.identity.player_id);
        }

        static const char* penalty_message(gc::PenaltyState penalty) {
            switch (penalty) {
                // Why, and what happens: every penalised robot is stood on the touchline and walks back
                // on when its penalty ends.
                case gc::PenaltyState::LEAVING_THE_FIELD: return " left the field: back to the touchline";
                case gc::PenaltyState::LOCAL_GAME_STUCK:
                    return " moved to the touchline: the ball was stuck between robots (nobody's fault)";
                case gc::PenaltyState::INCAPABLE_ROBOT: return " lay on the ground too long: back to the touchline";
                case gc::PenaltyState::ILLEGAL_POSITIONING: return " penalised: stood where the rules don't allow";
                case gc::PenaltyState::MOTION_IN_SET: return " penalised: moved during Set";
                case gc::PenaltyState::MOTION_IN_STOP: return " penalised: moved while play was stopped";
                case gc::PenaltyState::PICK_UP: return " picked up: back to the touchline";
                case gc::PenaltyState::BALL_HOLDING: return " penalised: held the ball";
                case gc::PenaltyState::PLAYING_WITH_ARMS_HANDS: return " penalised: played the ball with its arms";
                case gc::PenaltyState::PLAYER_PUSHING: return " penalised: pushed another robot";
                case gc::PenaltyState::CAUTIONED: return " cautioned";
                case gc::PenaltyState::SENT_OFF: return " sent off";
                case gc::PenaltyState::SUBSTITUTE: return " is a substitute";
                default: return " penalised";
            }
        }

        static const char* set_play_message(gc::SetPlay set_play) {
            switch (set_play) {
                case gc::SetPlay::DIRECT_FREE_KICK:
                case gc::SetPlay::INDIRECT_FREE_KICK: return "Free kick";
                case gc::SetPlay::PENALTY_KICK: return "Penalty kick";
                case gc::SetPlay::THROW_IN: return "Throw-in";
                case gc::SetPlay::GOAL_KICK: return "Goal kick";
                case gc::SetPlay::CORNER_KICK: return "Corner kick";
                default: return nullptr;
            }
        }

        const gc::Robot& robot_in(const gc::GameControllerPacket& packet, const MatchPlayer& player) const {
            return find_team(packet, static_cast<std::uint8_t>(player.identity.team_id))
                .players[player.identity.player_id - 1];
        }

        // Adds to the message box, under the game clock.
        void add_message(const gc::GameControllerPacket& now, const std::string& message) {
            char clock[16];
            std::snprintf(clock,
                          sizeof(clock),
                          "%dm %02ds  ",
                          now.secs_remaining / 60,
                          std::abs(now.secs_remaining % 60));
            messages_.push_back(clock + message);
            if (messages_.size() > MAX_MESSAGES) {
                messages_.pop_front();
            }
        }

        // Puts what happened to the game since the last control message in the message box.
        void report_game(const gc::GameControllerPacket& now) {
            if (!last_) {
                return;
            }
            const int home = score(now, home_team_);
            const int away = score(now, away_team_);
            if (home != score(*last_, home_team_) || away != score(*last_, away_team_)) {
                add_message(now, "Goal! Home " + std::to_string(home) + " - " + std::to_string(away) + " Away");
            }
            if (last_->state != now.state && now.state == gc::State::PLAYING) {
                add_message(now, "Kick-off");
            }
            if (last_->state != now.state && now.state == gc::State::FINISHED) {
                add_message(now, "Full time");
            }
            if (last_->stopped != now.stopped) {
                add_message(now, now.stopped != 0 ? "Paused" : "Resumed");
            }
            const char* set_play = set_play_message(now.set_play);
            if (last_->set_play != now.set_play && set_play != nullptr) {
                add_message(now, std::string(set_play) + (now.kicking_team == home_team_ ? " to Home" : " to Away"));
            }
        }

        static const gc::Team& find_team(const gc::GameControllerPacket& packet, std::uint8_t team) {
            return packet.teams[0].team_id == team ? packet.teams[0] : packet.teams[1];
        }

        // Puts a robot upright in the ready pose, at rest.
        void stand(mjData* d, const MatchPlayer& player, double x, double y, double yaw) const {
            supervisor::place_free_body(m_, d, player.root_body, x, y, cfg_.stand_height, yaw);
            for (std::size_t i = 0; i < JOINT_COUNT; ++i) {
                d->qpos[player.map.qpos_adr[i]] = ready_[i];
                d->qvel[player.map.dof_adr[i]]  = 0.0;
            }
        }

        RoboCup::AutorefereeWorld observe(const mjData* d) const {
            RoboCup::AutorefereeWorld world{};
            for (int i = 0; i < 3; ++i) {
                world.ball[i] = static_cast<float>(d->geom_xpos[3 * ball_geom_ + i]);
            }
            for (const auto& player : players_) {
                const mjtNum* pos  = d->xpos + 3 * player.root_body;
                const mjtNum* quat = d->xquat + 4 * player.root_body;
                const double yaw   = std::atan2(2.0 * (quat[0] * quat[3] + quat[1] * quat[2]),
                                              1.0 - 2.0 * (quat[2] * quat[2] + quat[3] * quat[3]));
                world.players[player.side == RoboCup::Side::home ? 0 : 1][player.identity.player_id - 1] =
                    RoboCup::AutorefereePlayer{true,
                                               static_cast<float>(pos[0]),
                                               static_cast<float>(pos[1]),
                                               static_cast<float>(yaw),
                                               cfg_.player_radius};
            }
            return world;
        }

        // Runs a kick-off started with start_kick_off(). Ready lasts until every robot has stood still
        // for a moment (or READY_LIMIT passes); then it is Set and the ball goes to the centre mark;
        // SET_SECONDS later it is Playing, which is when the GameController's clock starts.
        void advance_kick_off(mjData* d, double sim_time) {
            static constexpr double SAMPLE_SECONDS = 0.25;  // how often movement is measured
            static constexpr double STILL_METRES   = 0.02;  // less than this per sample is standing still
            static constexpr double STILL_RADIANS  = 0.05;
            static constexpr double STILL_SECONDS  = 1.0;   // how long everyone must stay still
            static constexpr double READY_MINIMUM  = 3.0;   // players need a moment to hear Ready
            static constexpr double READY_LIMIT    = 45.0;  // the rule book's Ready period
            static constexpr double SET_SECONDS    = 2.0;

            if (kick_off_ == KickOff::NONE) {
                return;
            }
            if (phase_started_ < 0.0 || sim_time < phase_started_) {
                phase_started_ = sim_time;  // first update of this phase, or the sim was reset
                still_since_   = sim_time;
                sampled_at_    = -1.0;
            }

            if (kick_off_ == KickOff::READY) {
                if (sampled_at_ < 0.0 || sim_time - sampled_at_ >= SAMPLE_SECONDS) {
                    const RoboCup::AutorefereeWorld world = observe(d);
                    bool moving = false;
                    std::vector<std::array<float, 3>> now;
                    for (const auto& player : players_) {
                        const auto& seen = world.players[player.side == RoboCup::Side::home ? 0 : 1]
                                                        [player.identity.player_id - 1];
                        now.push_back({seen.x, seen.y, seen.theta});
                    }
                    for (std::size_t i = 0; sampled_at_ >= 0.0 && i < now.size() && i < sampled_.size(); ++i) {
                        const double turn = std::remainder(now[i][2] - sampled_[i][2], 2.0 * 3.14159265358979);
                        moving = moving || std::hypot(now[i][0] - sampled_[i][0], now[i][1] - sampled_[i][1]) > STILL_METRES
                                 || std::abs(turn) > STILL_RADIANS;
                    }
                    sampled_    = std::move(now);
                    sampled_at_ = sim_time;
                    if (moving) {
                        still_since_ = sim_time;
                    }
                }
                const double in_phase = sim_time - phase_started_;
                const bool settled    = in_phase >= READY_MINIMUM && sim_time - still_since_ >= STILL_SECONDS;
                if ((settled || in_phase >= READY_LIMIT) && apply(RoboCup::gc_action_wait_for_set_play())) {
                    supervisor::place_free_body_by_geom_center(m_, d, ball_body_, ball_geom_, 0.0, 0.0,
                                                               m_->geom_size[3 * ball_geom_]);
                    events_.push_back(settled ? "robots in position: Set, ball on the centre mark"
                                              : "Ready time is up: Set, ball on the centre mark");
                    kick_off_      = KickOff::SET;
                    phase_started_ = sim_time;
                }
            }
            else if (sim_time - phase_started_ >= SET_SECONDS) {
                kick_off_ = KickOff::NONE;
                release_set_play();
            }
        }

        // A robot that has been lying on the ground for INCAPABLE_SECONDS while the game is Playing is
        // not going to get up by itself: call it incapable, which sends it to the touchline, stood
        // up, to come back on when the penalty ends.
        void remove_incapable(mjData* d, double sim_time) {
            static constexpr double INCAPABLE_SECONDS = 10.0;
            static constexpr double DOWN_HEIGHT       = 0.35;  // trunk height (m); it stands at about 0.55
            static constexpr double DOWN_UPRIGHT      = 0.5;   // or trunk z axis more than 60 degrees from vertical
            down_since_.resize(players_.size(), -1.0);
            const bool playing = last_ && last_->state == gc::State::PLAYING;
            for (std::size_t i = 0; i < players_.size(); ++i) {
                const auto& player = players_[i];
                const double height  = d->xpos[3 * player.root_body + 2];
                const double upright = d->xmat[9 * player.root_body + 8];
                const bool penalised =
                    last_ && robot_in(*last_, player).penalty_state != gc::PenaltyState::UNPENALISED;
                if (!playing || penalised || (height >= DOWN_HEIGHT && upright >= DOWN_UPRIGHT)
                    || sim_time < down_since_[i]) {
                    down_since_[i] = -1.0;
                    continue;
                }
                if (down_since_[i] < 0.0) {
                    down_since_[i] = sim_time;
                }
                else if (sim_time - down_since_[i] >= INCAPABLE_SECONDS) {
                    down_since_[i] = -1.0;
                    if (apply(RoboCup::gc_action_penalize(player.side,
                                                          static_cast<std::uint8_t>(player.identity.player_id),
                                                          RoboCup::PenaltyCall::incapableRobot))) {
                        events_.push_back(name(player) + " has been down for 10 s: incapable, back to the touchline");
                    }
                }
            }
        }

        static void on_place_ball(void* self_ptr, float x, float y, float z) {
            auto* self = static_cast<MatchRefereeLogic*>(self_ptr);
            supervisor::place_free_body_by_geom_center(self->m_, self->d_, self->ball_body_, self->ball_geom_, x, y, z);
            self->events_.push_back("ball placed at (" + std::to_string(x) + ", " + std::to_string(y) + ")");
        }

        static void on_place_player(void* self_ptr,
                                    RoboCup::Side side,
                                    std::uint8_t number,
                                    float x,
                                    float y,
                                    float theta) {
            auto* self = static_cast<MatchRefereeLogic*>(self_ptr);
            for (const auto& player : self->players_) {
                if (player.side == side && player.identity.player_id == number) {
                    self->stand(self->d_, player, x, y, theta);
                    // Now, not at the next control message: its player doesn't know it has moved.
                    self->held_[&player - self->players_.data()] = self->cfg_.hold_robots;
                    self->events_.push_back("player " + std::to_string(number) + " placed");
                }
            }
        }

        static void on_whistle(void* self_ptr) {
            static_cast<MatchRefereeLogic*>(self_ptr)->events_.push_back("whistle");
        }

        MatchConfig cfg_;
        const mjModel* m_;
        mjData* d_ = nullptr;  // only set during update(), for the placement callbacks
        std::array<double, JOINT_COUNT> ready_;
        std::uint8_t home_team_ = 0;
        std::uint8_t away_team_ = 0;
        std::vector<MatchPlayer> players_;
        std::vector<int> player_of_root_;  // body id -> index into players_, or -1
        std::vector<bool> touching_;       // since the last update()
        std::vector<bool> held_;
        int ball_body_ = -1;
        int ball_geom_ = -1;

        Handle<RoboCup::CompetitionParams, RoboCup::gc_competition_params_destroy> competition_;
        Handle<RoboCup::Params, RoboCup::gc_params_destroy> params_;
        Handle<RoboCup::GameController, RoboCup::gc_destroy> game_;
        Handle<RoboCup::Autoreferee, RoboCup::gc_autoreferee_destroy> autoreferee_;

        bool started_               = false;
        // The automatic kick-off sequence (advance_kick_off()), in simulation time.
        enum class KickOff { NONE, READY, SET } kick_off_ = KickOff::NONE;
        double phase_started_ = -1.0;
        double still_since_   = 0.0;
        double sampled_at_    = -1.0;
        std::vector<std::array<float, 3>> sampled_;  // each player's x, y, yaw at the last sample
        std::vector<double> down_since_;             // sim time each player went down, or -1 (remove_incapable())
        bool staged_kick_offs_      = false;  // kick-offs go Ready, Set, Playing as the robots settle
        bool manual_set_play_       = false;  // a set play called from the viewer is waiting for its release
        std::uint8_t packet_number_ = 0;
        std::optional<gc::GameControllerPacket> last_;
        std::vector<std::string> events_;
        std::deque<std::string> messages_;
    };

}  // namespace k1sim::module::matchreferee

#endif  // K1SIM_MODULE_MATCHREFEREE_MATCHREFEREELOGIC_HPP
