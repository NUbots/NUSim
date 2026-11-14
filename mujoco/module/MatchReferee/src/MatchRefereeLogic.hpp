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
            if (cfg_.skip_kick_off_phases
                && !RoboCup::gc_apply(game_.get(),
                                      RoboCup::gc_action_wait_for_set_play(),
                                      RoboCup::ActionSource::autoreferee)) {
                RoboCup::gc_apply(game_.get(), RoboCup::gc_action_free_set_play(), RoboCup::ActionSource::autoreferee);
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

        // What the panel's main button does: starts the game, or pauses and resumes it.
        void primary_action() {
            if (!last_ || last_->state == gc::State::INITIAL) {
                start();
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
                case gc::PenaltyState::LEAVING_THE_FIELD: return " left the field";
                case gc::PenaltyState::LOCAL_GAME_STUCK: return " penalised: ball stuck";
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
        std::uint8_t packet_number_ = 0;
        std::optional<gc::GameControllerPacket> last_;
        std::vector<std::string> events_;
        std::deque<std::string> messages_;
    };

}  // namespace k1sim::module::matchreferee

#endif  // K1SIM_MODULE_MATCHREFEREE_MATCHREFEREELOGIC_HPP
