#include "module/MatchReferee/src/MatchReferee.hpp"

#include <algorithm>
#include <arpa/inet.h>
#include <atomic>
#include <fstream>
#include <netinet/in.h>
#include <sstream>
#include <stdexcept>
#include <sys/socket.h>
#include <unistd.h>

#include "module/MatchReferee/src/HoldController.hpp"
#include "module/MatchReferee/src/MatchRefereeConfig.hpp"
#include "module/MatchReferee/src/MatchRefereeLogic.hpp"
#include "module/MatchReferee/src/MatchRoster.hpp"
#include "shared/CliOptions.hpp"
#include "shared/k1/BoosterApi.hpp"
#include "shared/message/MatchMessages.hpp"
#include "shared/message/SimMessages.hpp"
#include "shared/util/Config.hpp"
#include "shared/util/RobotRoster.hpp"

namespace k1sim::module {

    using matchreferee::HoldController;
    using matchreferee::MatchConfig;
    using matchreferee::MatchRefereeLogic;
    using matchreferee::MatchRoster;

    namespace {

        // mjcb_control is a plain global function pointer, so it reaches the logic through this.
        // MuJoCo calls it inside mj_step after collision detection, which gives every physics step's
        // contacts without changing SimCore, under the mutex the physics thread already holds.
        std::atomic<MatchRefereeLogic*> g_logic{nullptr};
        const mjData* g_data = nullptr;

        void on_physics_step(const mjModel* /*m*/, mjData* d) {
            if (MatchRefereeLogic* logic = g_logic.load(std::memory_order_acquire); logic != nullptr && d == g_data) {
                logic->record_contacts(d);
            }
        }

        std::string read_competition(const std::string& name) {
            const std::string path = std::string(K1SIM_GC_PARAMS_DIR) + "/" + name + ".yaml";
            std::ifstream file(path);
            if (!file) {
                throw std::runtime_error("MatchReferee: can't read " + path
                                         + " (see tools/install_game_controller_api.sh)");
            }
            std::stringstream content;
            content << file.rdbuf();
            return content.str();
        }

        std::array<double, JOINT_COUNT> ready_pose() {
            const auto gains = config::load("gains.yaml");
            std::array<double, JOINT_COUNT> ready{};
            for (std::size_t i = 0; i < JOINT_COUNT; ++i) {
                ready[i] = gains["ready_pose"][i].as<double>();
            }
            return ready;
        }

        double seconds_since_start() {
            static const auto start = std::chrono::steady_clock::now();
            return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        }

    }  // namespace

    MatchReferee::~MatchReferee() {
        g_logic.store(nullptr, std::memory_order_release);
        if (socket_ >= 0) {
            ::close(socket_);
        }
    }

    MatchReferee::MatchReferee(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        cfg_ = std::make_unique<MatchConfig>(matchreferee::load_config(config::load("match.yaml")));

        if (cli().game == 0) {
            throw std::runtime_error("sim/match needs --game <n> (see --help)");
        }
        // Nothing can press Start without a window.
        if (cli().headless && cfg_->auto_start_after < 0.0) {
            cfg_->auto_start_after = 30.0;
        }

        socket_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        int yes = 1;
        if (socket_ < 0 || ::setsockopt(socket_, SOL_SOCKET, SO_BROADCAST, &yes, sizeof(yes)) != 0) {
            throw std::runtime_error("MatchReferee: can't create the broadcast socket");
        }
        if (!cfg_->interface.empty()
            && ::setsockopt(socket_,
                            SOL_SOCKET,
                            SO_BINDTODEVICE,
                            cfg_->interface.c_str(),
                            static_cast<socklen_t>(cfg_->interface.size() + 1))
                   != 0) {
            throw std::runtime_error("MatchReferee: can't send on interface " + cfg_->interface);
        }

        on<Trigger<message::SimHandles>, Sync<MatchReferee>>().then([this](const message::SimHandles& handles) {
            model_       = handles.model;
            data_        = handles.data;
            sim_mutex_   = handles.mutex;
            reset_world_ = handles.reset_world;
            g_data       = handles.data;
            new_game();
            roster_      = std::make_unique<MatchRoster>(logic_->expected_players(), cfg_->status_timeout);
            mjcb_control = on_physics_step;

            log<NUClear::LogLevel::INFO>("MatchReferee ready:",
                                         cli().game,
                                         "a side - broadcasting to",
                                         cfg_->broadcast_address,
                                         "on",
                                         cfg_->interface.empty() ? "the network" : cfg_->interface);
        });

        // Each controller goes back to Simulation wrapped, so the game can hold its robot. Sent
        // from the update reaction below: Simulation must get the original first, or it would
        // overwrite the wrapper.
        if (cfg_->hold_robots) {
            on<Trigger<message::ControllerHandle>, Sync<MatchReferee>>().then(
                [this](const message::ControllerHandle& handle) {
                    if (handle.owner == nullptr || dynamic_cast<HoldController*>(handle.owner.get()) != nullptr) {
                        return;
                    }
                    const auto& roster = config::robot_roster();
                    const auto robot =
                        std::find_if(roster.begin(), roster.end(), [&handle](const RobotIdentity& identity) {
                            return identity.robot_id == handle.robot_id;
                        });
                    if (robot == roster.end()) {
                        return;
                    }
                    auto hold                          = std::make_shared<HoldController>(handle.owner,
                                                                 robot_prefix(robot->model_index),
                                                                 ready_pose(),
                                                                 cfg_->stand_height);
                    hold_controllers_[handle.robot_id] = hold;
                    pending_controllers_.push_back({hold.get(), handle.robot_id, hold});
                });
        }

        on<Trigger<message::RobotStatesUpdate>, Single, Sync<MatchReferee>>().then(
            [this](const message::RobotStatesUpdate& states) {
                if (logic_ == nullptr || states.robots.empty()) {
                    return;
                }

                for (const auto& handle : pending_controllers_) {
                    emit(std::make_unique<message::ControllerHandle>(handle));
                }
                pending_controllers_.clear();

                connected_ = 0;
                for (const auto& robot : states.robots) {
                    if (robot.mode == booster::CUSTOM) {
                        ++connected_;
                        custom_robots_.insert(robot.identity.robot_id);
                    }
                }
                if (cfg_->auto_start_when_connected && connected_ == static_cast<int>(states.robots.size())) {
                    logic_->start();
                }

                // gc_seek takes whole milliseconds, so carry the remainder over. Sim time rewinds
                // when the world resets.
                const double sim_time = states.robots.front().sim_time;
                unspent_ms_ += std::max(0.0, sim_time - last_sim_time_) * 1000.0;
                last_sim_time_   = sim_time;
                const auto dt_ms = static_cast<std::uint64_t>(unspent_ms_);
                unspent_ms_ -= static_cast<double>(dt_ms);

                std::vector<std::string> events;
                {
                    std::lock_guard<std::mutex> lock(*sim_mutex_);
                    events = logic_->update(data_, sim_time, dt_ms);
                    apply_holds();
                }
                for (const auto& event : events) {
                    log<NUClear::LogLevel::INFO>("MatchReferee:", event);
                }

                const auto wall_time = std::chrono::steady_clock::now();
                if (wall_time >= next_report_) {
                    next_report_ = wall_time + std::chrono::seconds(10);
                    std::lock_guard<std::mutex> lock(*sim_mutex_);
                    log<NUClear::LogLevel::INFO>("MatchReferee: robots", logic_->positions(data_));
                }

                // Wall-clock time like the robots, so the rate holds however fast the sim runs.
                const auto now = std::chrono::steady_clock::now();
                if (now >= next_control_) {
                    send_control_message();
                    next_control_ = now
                                    + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                        std::chrono::duration<double>(cfg_->control_interval));
                }
            });

        on<Trigger<message::MatchCommand>, Sync<MatchReferee>>().then([this](const message::MatchCommand& command) {
            if (logic_ == nullptr) {
                return;
            }
            if (command.action == message::MatchCommand::Action::PRIMARY) {
                logic_->primary_action();
            }
            else if (command.action == message::MatchCommand::Action::SET_PLAY) {
                const bool called = logic_->call_set_play(command.side == 0 ? RoboCup::Side::home : RoboCup::Side::away,
                                                          static_cast<RoboCup::SetPlay>(command.set_play));
                log<NUClear::LogLevel::INFO>("MatchReferee: set play", command.set_play, "for",
                                             command.side == 0 ? "home" : "away",
                                             called ? "called" : "refused (not legal right now)");
            }
            else if (command.action == message::MatchCommand::Action::RELEASE) {
                logic_->release_set_play();
            }
            else {
                reset_world_();
                new_game();
                last_sim_time_ = 0.0;

                // The reset drops every robot out of its mode, and stock NUbots requests it only once.
                for (const int robot_id : custom_robots_) {
                    auto request      = std::make_unique<message::ModeChangeRequest>();
                    request->mode     = booster::CUSTOM;
                    request->robot_id = robot_id;
                    emit(request);
                }
                log<NUClear::LogLevel::INFO>("MatchReferee: new game");
            }
        });

        try {
            on<UDP, Sync<MatchRoster>>(cfg_->status_port).then([this](const UDP::Packet& packet) {
                const auto status = matchreferee::parse_status(packet.payload.data(), packet.payload.size());
                if (status && roster_ != nullptr) {
                    for (const auto& message : roster_->on_status(*status, seconds_since_start())) {
                        log<NUClear::LogLevel::INFO>("MatchReferee:", message);
                    }
                }
            });
        }
        catch (const std::exception& e) {
            log<NUClear::LogLevel::WARN>("MatchReferee: can't listen for status messages on port",
                                         cfg_->status_port,
                                         "(",
                                         e.what(),
                                         ") - is the GameController app running?");
        }

        on<Every<1, std::chrono::seconds>, Sync<MatchRoster>>().then([this] {
            if (roster_ != nullptr) {
                for (const auto& message : roster_->expire(seconds_since_start())) {
                    log<NUClear::LogLevel::WARN>("MatchReferee:", message);
                }
            }
        });

        on<Shutdown>().then([this] {
            g_logic.store(nullptr, std::memory_order_release);
            log<NUClear::LogLevel::INFO>("MatchReferee shutting down");
        });
    }

    void MatchReferee::new_game() {
        auto logic = std::make_unique<MatchRefereeLogic>(
            *cfg_,
            matchreferee::with_penalty_time(read_competition(cfg_->competition), cfg_->penalty_seconds),
            model_,
            config::robot_roster(),
            ready_pose());
        std::lock_guard<std::mutex> lock(*sim_mutex_);
        g_logic.store(logic.get(), std::memory_order_release);
        logic_ = std::move(logic);
    }

    void MatchReferee::apply_holds() {
        for (std::size_t i = 0; i < logic_->players().size(); ++i) {
            const auto hold = hold_controllers_.find(logic_->players()[i].identity.robot_id);
            if (hold != hold_controllers_.end()) {
                hold->second->set_held(logic_->held()[i]);
            }
        }
    }

    void MatchReferee::send_control_message() {
        const auto packet = logic_->control_message();

        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_port   = htons(static_cast<std::uint16_t>(cfg_->control_port));
        ::inet_pton(AF_INET, cfg_->broadcast_address.c_str(), &address.sin_addr);
        if (::sendto(socket_, &packet, sizeof(packet), 0, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0) {
            log<NUClear::LogLevel::WARN>("MatchReferee: couldn't send a control message");
        }

        for (const auto& event : logic_->report_changes(packet)) {
            log<NUClear::LogLevel::INFO>("MatchReferee:", event);
        }
        auto status            = std::make_unique<message::MatchStatus>();
        status->summary        = logic_->summary(connected_);
        status->primary_button = logic_->primary_label();
        status->robot_labels   = logic_->robot_labels();
        status->messages       = logic_->messages();
        status->has_game       = logic_->scoreboard(status->home_score,
                                                    status->away_score,
                                                    status->secs_remaining,
                                                    status->state,
                                                    status->players);
        status->connected      = connected_;
        emit(status);
    }

}  // namespace k1sim::module
