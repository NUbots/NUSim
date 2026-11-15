#include "module/WebViewer/src/WebViewer.hpp"

#include <cstdlib>
#include <sstream>
#include <string>

#include "shared/CliOptions.hpp"
#include "shared/util/Config.hpp"
#include "shared/util/RobotRoster.hpp"

namespace k1sim::module {

    namespace {

        std::string game_json(const message::GameState& game) {
            using web::WebStream;
            std::ostringstream json;
            json << "{\"type\":\"game\",\"left\":" << WebStream::quote(game.left)
                 << ",\"right\":" << WebStream::quote(game.right) << ",\"scoreLeft\":" << game.score_left
                 << ",\"scoreRight\":" << game.score_right << ",\"time\":" << game.secs_remaining
                 << ",\"mode\":" << WebStream::quote(game.mode) << ",\"state\":" << WebStream::quote(game.state)
                 << ",\"firstHalf\":" << (game.first_half ? "true" : "false")
                 << ",\"secondaryTime\":" << game.secondary_time << ",\"colourLeft\":" << game.colour_left
                 << ",\"colourRight\":" << game.colour_right << ",\"kickingSide\":" << game.kicking_side
                 << ",\"robots\":[";
            for (std::size_t i = 0; i < game.robots.size(); ++i) {
                const auto& robot = game.robots[i];
                json << (i > 0 ? "," : "") << "{\"body\":" << WebStream::quote(robot.body) << ",\"side\":" << robot.side
                     << ",\"player\":" << robot.player << ",\"penalty\":" << WebStream::quote(robot.penalty)
                     << ",\"penaltySecs\":" << robot.penalty_secs << "}";
            }
            json << "]}";
            return json.str();
        }

        std::string referee_json(const message::RefereeStatus& status) {
            std::ostringstream json;
            json << "{\"type\":\"referee\",\"legal\":[";
            for (std::size_t i = 0; i < status.legal.size(); ++i) {
                json << (i > 0 ? "," : "") << web::WebStream::quote(status.legal[i]);
            }
            json << "],\"seq\":" << status.result_seq << ",\"ok\":" << (status.result_ok ? "true" : "false")
                 << ",\"text\":" << web::WebStream::quote(status.result_text) << "}";
            return json.str();
        }

    }  // namespace

    WebViewer::WebViewer(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        // cli() is set in main() before any module is installed.
        if (!cli().web && cli().record.empty()) {
            return;
        }

        web::WebStream::Options options;
        options.host        = cli().web_host;
        options.port        = cli().web ? cli().web_port : 0;
        options.web_root    = std::string(K1SIM_SOURCE_DIR) + "/module/WebViewer/web";
        options.record_path = cli().record;

        auto stream = std::make_unique<web::WebStream>(options);
        std::string error;
        if (!stream->start(error)) {
            log<NUClear::LogLevel::ERROR>("Web viewer disabled:", error);
            return;
        }
        stream_ = std::move(stream);

        if (cli().web) {
            log<NUClear::LogLevel::INFO>("Web viewer at http://" + options.host + ":" + std::to_string(options.port) + "/");
        }
        if (!options.record_path.empty()) {
            log<NUClear::LogLevel::INFO>("Recording to", options.record_path);
        }

        on<Trigger<message::SimHandles>>().then([this](const message::SimHandles& handles) {
            if (handles.model == nullptr || handles.data == nullptr || handles.mutex == nullptr) {
                return;
            }
            std::lock_guard<std::mutex> lock(mutex_);
            handles_ = handles;
            stream_->set_model(handles.model);  // the model itself is never modified while the sim runs
        });

        on<Trigger<message::SimStateUpdate>, Single>().then([this] {
            std::lock_guard<std::mutex> lock(mutex_);
            if (handles_.model == nullptr || (updates_++ % 2) != 0) {
                return;
            }
            // Same contract as module::Viewer: hold the sim mutex only for the (sub-millisecond) pose copy.
            std::lock_guard<std::mutex> sim_lock(*handles_.mutex);
            stream_->publish_frame(handles_.model, handles_.data);
        });

        // Only arrives while module::Supervisor is enabled (supervisor.yaml) and a GameController is on the network.
        on<Trigger<message::GameState>>().then([this](const message::GameState& game) {
            stream_->set_game(game_json(game));
        });

        // Built-in referee (--referee): its status goes to the viewers' referee panel, their button presses come
        // back as commands. Query format: action=<name>&side=<home|away>&player=<n>&arg=<text>.
        on<Trigger<message::RefereeStatus>>().then([this](const message::RefereeStatus& status) {
            stream_->set_referee(referee_json(status));
        });
        // sim/match: module::MatchReferee instead, through the same scoreboard and panel.
        on<Trigger<message::MatchStatus>>().then([this](const message::MatchStatus& status) {
            match_.store(true, std::memory_order_release);
            show_match(status);
        });

        if (cli().referee || cli().game > 0) {
            stream_->set_command_handler([this](const std::string& query) {
                auto command = std::make_unique<message::RefereeCommand>();
                std::istringstream pairs(query);
                std::string pair;
                while (std::getline(pairs, pair, '&')) {
                    const std::size_t equals = pair.find('=');
                    if (equals == std::string::npos) {
                        continue;
                    }
                    const std::string key   = pair.substr(0, equals);
                    const std::string value = pair.substr(equals + 1);
                    if (key == "action") command->action = value;
                    if (key == "side") command->side = value;
                    if (key == "arg") command->arg = value;
                    if (key == "player") command->player = std::atoi(value.c_str());
                }
                if (match_.load(std::memory_order_acquire)) {
                    send_match_command(command->action, command->side, command->arg);
                }
                else if (cli().referee) {
                    emit(command);
                }
            });
        }

        on<Shutdown>().then([this] {
            stream_->stop();
        });
    }

    void WebViewer::show_match(const message::MatchStatus& status) {
        const std::string state = status.has_game ? status.state : "INITIAL";
        {
            std::lock_guard<std::mutex> lock(mutex_);
            match_state_ = state;
        }

        if (status.has_game) {
            message::GameState game;
            game.left           = "Home";
            game.right          = "Away";
            game.score_left     = status.home_score;
            game.score_right    = status.away_score;
            game.secs_remaining = status.secs_remaining;
            game.state          = status.state;
            const auto& roster  = config::robot_roster();
            for (const auto& identity : roster) {
                message::GameState::Robot robot;
                robot.body   = identity.body();
                robot.side   = identity.team_id == roster.front().team_id ? 0 : 1;
                robot.player = identity.player_id;
                game.robots.push_back(robot);
            }
            stream_->set_game(game_json(game));
        }

        // The buttons MatchReferee answers in this state. Kick-off is called by hand one phase at a time, as in
        // the window's Testing Room: Ready, then Set, then Play.
        static const std::string kick_off_side = config::load("match.yaml")["kick_off_side"].as<std::string>("home");
        message::RefereeStatus referee;
        if (state == "INITIAL") {
            referee.legal = {"kickoff:" + kick_off_side, "play"};
        }
        else if (state == "READY") {
            referee.legal = {"set", "play"};
        }
        else if (state == "SET") {
            referee.legal = {"play"};
        }
        else if (state == "PLAYING") {
            referee.legal = {"stop", "setplay:home", "setplay:away", "finish_set_play"};
        }
        else if (state == "PAUSED") {
            referee.legal = {"resume"};
        }
        referee.legal.emplace_back("new_game");
        stream_->set_referee(referee_json(referee));
    }

    void WebViewer::send_match_command(const std::string& action, const std::string& side, const std::string& arg) {
        using Action = message::MatchCommand::Action;
        std::string state;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            state = match_state_;
        }

        message::MatchCommand command;
        if (action == "kickoff") {
            command.action = Action::READY;
        }
        else if (action == "set") {
            command.action = Action::SET;
        }
        else if (action == "play" || action == "finish_set_play") {
            command.action = Action::RELEASE;
        }
        // The window's main button pauses and resumes, so it may only be pressed for the one it does now.
        else if ((action == "stop" && state == "PLAYING") || (action == "resume" && state == "PAUSED")) {
            command.action = Action::PRIMARY;
        }
        else if (action == "setplay") {
            // GameController SetPlay values (game_controller_api's RoboCup::SetPlay).
            const int set_play = arg == "direct_free_kick"     ? 2
                                 : arg == "indirect_free_kick" ? 3
                                 : arg == "penalty_kick"       ? 4
                                 : arg == "throw_in"           ? 5
                                 : arg == "goal_kick"          ? 6
                                 : arg == "corner_kick"        ? 7
                                                               : 0;
            if (set_play == 0) {
                return;
            }
            command.action   = Action::SET_PLAY;
            command.side     = side == "away" ? 1 : 0;
            command.set_play = set_play;
        }
        else if (action == "new_game") {
            command.action = Action::RESET;
        }
        else {
            return;
        }
        emit(std::make_unique<message::MatchCommand>(command));
    }

}  // namespace k1sim::module
