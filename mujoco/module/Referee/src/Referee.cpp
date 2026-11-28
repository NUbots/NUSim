#include "module/Referee/src/Referee.hpp"

#include <array>
#include <cstring>
#include <fstream>
#include <sstream>

#include "module/Supervisor/src/GameControllerPacket.hpp"
#include "shared/CliOptions.hpp"
#include "shared/util/Config.hpp"
#include "shared/util/RobotRoster.hpp"

namespace k1sim::module {

    namespace gc = k1sim::module::supervisor::gc;
    using namespace k1sim::module::referee;

    Referee::Referee(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Startup>().then([this] {
            if (!cli().referee) {
                return;
            }
            std::string error;
            if (!start(error)) {
                log<NUClear::LogLevel::ERROR>("Referee disabled:", error);
                return;
            }
            running_.store(true, std::memory_order_release);
            log<NUClear::LogLevel::INFO>("Referee ready - the built-in GameController announces the game on UDP port",
                                         ntohs(targets_.empty() ? 0 : targets_.front().sin_port));
        });

        // Game time follows simulation time, so a slowed down or free-running sim still plays a full half.
        on<Trigger<message::SimStateUpdate>, Single>().then([this](const message::SimStateUpdate& state) {
            if (running_.load(std::memory_order_acquire)) {
                advance(state.sim_time);
            }
        });

        on<Trigger<message::RefereeCommand>>().then([this](const message::RefereeCommand& command) {
            if (running_.load(std::memory_order_acquire)) {
                apply(command);
            }
        });
    }

    Referee::~Referee() {
        if (game_ != nullptr && lib_) {
            lib_->destroy(game_);
        }
    }

    bool Referee::start(std::string& error) {
        const YAML::Node cfg = config::load("referee.yaml");
        publish_period_s_    = cfg["publish_period"].as<double>(publish_period_s_);

        if (std::string(K1SIM_GC_LIBRARY).empty()) {
            error = "built without the GameController library, which the docker image has (./b image)";
            return false;
        }
        lib_ = std::make_unique<GameControllerLibrary>();
        if (!lib_->load(K1SIM_GC_LIBRARY, error)) {
            return false;
        }

        const std::string competition = cfg["competition"].as<std::string>("middle_advanced");
        std::ifstream file(std::string(K1SIM_GC_CONFIG_DIR) + "/" + competition + "/params.yaml");
        if (!file.is_open()) {
            error = "no params.yaml for competition '" + competition + "' in " K1SIM_GC_CONFIG_DIR;
            return false;
        }
        std::ostringstream yaml;
        yaml << file.rdbuf();
        const std::string text = yaml.str();
        void* competition_params =
            lib_->competition_params_new(reinterpret_cast<const std::uint8_t*>(text.data()), text.size());
        if (competition_params == nullptr) {
            error = std::string("GameController: ") + lib_->last_error();
            return false;
        }

        TeamParams home = load_team(cfg["home"], 1, "blue", "white");
        TeamParams away = load_team(cfg["away"], 2, "red", "yellow");
        // A --match roster names the teams: first robot's team is home, last robot's away.
        if (!cli().match_config.empty()) {
            home.number = static_cast<std::uint8_t>(config::robot_roster().front().team_id);
            away.number = static_cast<std::uint8_t>(config::robot_roster().back().team_id);
        }
        const Side kick_off = cfg["kick_off"].as<std::string>("home") == "away" ? AWAY : HOME;
        const TestParams test{cfg["no_delay"].as<bool>(false), false};
        // The first --game team (home) stands in the -x half, i.e. it defends the left goal.
        void* params = lib_->params_new(competition_params, &home, &away, kick_off, HOME_DEFENDS_LEFT_GOAL, test);
        lib_->competition_params_destroy(competition_params);
        game_ = params != nullptr ? lib_->create(params) : nullptr;
        if (params != nullptr) {
            lib_->params_destroy(params);
        }
        if (game_ == nullptr) {
            error = std::string("GameController: ") + lib_->last_error();
            return false;
        }
        // The same goalkeeper as module::MatchReferee: the second robot of each team. One a side, that names
        // nobody, so a lone player isn't told to stay in its goal.
        for (const Side side : {HOME, AWAY}) {
            if (void* action = lib_->select_goalkeeper(side, 2)) {
                lib_->apply(game_, action, SOURCE_USER);
            }
        }

        std::string socket_error;
        sender_ = util::udp::open_sender(socket_error);
        if (!sender_.valid()) {
            log<NUClear::LogLevel::WARN>("Referee: no UDP socket (", socket_error, "), so the robots won't hear it");
        }
        // Packets go only to the match network's broadcast address, which ./b match passes in.
        std::vector<std::string> invalid;
        targets_ = util::udp::targets({cli().broadcast}, gc::CONTROL_PORT, invalid);
        if (targets_.empty()) {
            error = "no --broadcast address. Run it with ./b match --mode testing";
            return false;
        }
        return true;
    }

    void Referee::advance(double sim_time) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (last_sim_time_ < 0.0 || sim_time < last_sim_time_) {
            last_sim_time_ = sim_time;  // first update, or the sim was reset
            publish();
            return;
        }
        const double elapsed = sim_time - last_sim_time_;
        last_sim_time_       = sim_time;

        // The GameController counts whole milliseconds; carry the fractions.
        pending_ms_ += elapsed * 1000.0;
        const auto whole = static_cast<std::uint64_t>(pending_ms_);
        if (whole > 0) {
            lib_->seek(game_, whole);
            pending_ms_ -= static_cast<double>(whole);
        }

        since_publish_s_ += elapsed;
        if (since_publish_s_ >= publish_period_s_) {
            publish();
        }
    }

    void Referee::apply(const message::RefereeCommand& command) {
        std::lock_guard<std::mutex> lock(mutex_);
        void* action = lib_->action(command);
        ++result_seq_;
        if (action == nullptr) {
            result_ok_   = false;
            result_text_ = "unknown referee command: " + command.action;
        }
        else {
            result_ok_   = lib_->apply(game_, action, SOURCE_USER);  // takes the action, applied or not
            result_text_ = command.action + (result_ok_ ? "" : " is not allowed right now");
        }
        log<NUClear::LogLevel::INFO>("referee:",
                                     command.action,
                                     command.side,
                                     command.player > 0 ? std::to_string(command.player) : "",
                                     command.arg,
                                     result_ok_ ? "" : "(not allowed)");
        publish();
    }

    void Referee::publish() {
        since_publish_s_ = 0.0;

        // What the robots are told (possibly delayed after kick-off, like the real GameController) goes out on
        // the network; the true state goes to the Supervisor, which places ball and robots.
        std::array<std::uint8_t, sizeof(gc::GameControllerPacket)> buffer{};
        if (lib_->read(game_, packet_number_++, false, buffer.data(), buffer.size())) {
            util::udp::send_to_all(sender_, targets_, buffer.data(), buffer.size());
        }
        if (lib_->read(game_, packet_number_, true, buffer.data(), buffer.size())) {
            // True data carries the monitor magic ("RGTD"); the Supervisor expects the players' one.
            std::memcpy(buffer.data(), gc::RECEIVE_HEADER.data(), gc::RECEIVE_HEADER.size());
            auto packet = std::make_unique<gc::GameControllerPacket>();
            if (gc::try_parse(buffer.data(), buffer.size(), *packet)) {
                emit(packet);
            }
        }

        // Which buttons the referee panel may offer right now.
        auto status = std::make_unique<message::RefereeStatus>();
        auto offer  = [this, &status](const std::string& name, const message::RefereeCommand& command) {
            void* action = lib_->action(command);
            if (action != nullptr) {
                if (lib_->is_legal(game_, action)) {
                    status->legal.push_back(name);
                }
                lib_->action_destroy(action);
            }
        };
        for (const char* side : {"home", "away"}) {
            offer(std::string("kickoff:") + side, {"kickoff", side, 0, ""});
            offer(std::string("goal:") + side, {"goal", side, 0, ""});
            offer(std::string("timeout:") + side, {"timeout", side, 0, ""});
            offer(std::string("setplay:") + side, {"setplay", side, 0, "direct_free_kick"});
        }
        for (const char* name :
             {"set", "play", "finish_set_play", "stop", "resume", "finish_half", "switch_half", "stuck", "undo"}) {
            offer(name, {name, "", 0, ""});
        }
        offer("timeout:none", {"timeout", "", 0, ""});
        status->result_seq  = result_seq_;
        status->result_ok   = result_ok_;
        status->result_text = result_text_;

        std::string key = std::to_string(result_seq_);
        for (const auto& name : status->legal) {
            key += "|" + name;
        }
        if (key != last_status_) {
            last_status_ = key;
            emit(status);
        }
    }

}  // namespace k1sim::module
