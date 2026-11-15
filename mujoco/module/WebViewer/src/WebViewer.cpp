#include "module/WebViewer/src/WebViewer.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <sstream>
#include <string>

#include "shared/CliOptions.hpp"
#include "shared/k1/BoosterApi.hpp"
#include "shared/message/Commands.hpp"
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

        // `source` says who answers the panel: "referee" (module::Referee, the GameController operated by hand) or
        // "match" (module::MatchReferee, the automatic referee).
        std::string referee_json(const message::RefereeStatus& status, const char* source) {
            std::ostringstream json;
            json << "{\"type\":\"referee\",\"source\":\"" << source << "\",\"legal\":[";
            for (std::size_t i = 0; i < status.legal.size(); ++i) {
                json << (i > 0 ? "," : "") << web::WebStream::quote(status.legal[i]);
            }
            json << "],\"seq\":" << status.result_seq << ",\"ok\":" << (status.result_ok ? "true" : "false")
                 << ",\"text\":" << web::WebStream::quote(status.result_text) << "}";
            return json.str();
        }

        const char* mode_name(int mode) {
            switch (mode) {
                case booster::DAMPING: return "DAMPING";
                case booster::PREPARE: return "PREPARE";
                case booster::WALKING: return "WALKING";
                case booster::CUSTOM: return "CUSTOM";
                case booster::SOCCER: return "SOCCER";
                default: return "?";
            }
        }

        const char* fall_name(int fall_state) {
            switch (fall_state) {
                case booster::IS_FALLING: return "FALLING";
                case booster::HAS_FALLEN: return "FALLEN";
                case booster::IS_GETTING_UP: return "GETTING UP";
                default: return "";
            }
        }

        std::string url_decode(const std::string& text) {
            std::string out;
            for (std::size_t i = 0; i < text.size(); ++i) {
                if (text[i] == '%' && i + 2 < text.size() && std::isxdigit(static_cast<unsigned char>(text[i + 1]))
                    && std::isxdigit(static_cast<unsigned char>(text[i + 2]))) {
                    out.push_back(static_cast<char>(std::stoi(text.substr(i + 1, 2), nullptr, 16)));
                    i += 2;
                }
                else {
                    out.push_back(text[i] == '+' ? ' ' : text[i]);
                }
            }
            return out;
        }

        std::map<std::string, std::string> parse_query(const std::string& query) {
            std::map<std::string, std::string> values;
            std::istringstream pairs(query);
            std::string pair;
            while (std::getline(pairs, pair, '&')) {
                const std::size_t equals = pair.find('=');
                if (equals != std::string::npos) {
                    values[url_decode(pair.substr(0, equals))] = url_decode(pair.substr(equals + 1));
                }
            }
            return values;
        }

        std::string text_of(const std::map<std::string, std::string>& query, const char* key) {
            const auto found = query.find(key);
            return found == query.end() ? std::string() : found->second;
        }

        // A finite number from the query, or `fallback` if it is missing or not one.
        double number_of(const std::map<std::string, std::string>& query, const char* key, double fallback = 0.0) {
            const auto found = query.find(key);
            if (found == query.end()) {
                return fallback;
            }
            char* end          = nullptr;
            const double value = std::strtod(found->second.c_str(), &end);
            return end != found->second.c_str() && std::isfinite(value) ? value : fallback;
        }

        // The free-floating body (robot trunk, ball) that `body` belongs to, or -1.
        int free_root(const mjModel* m, int body) {
            if (body <= 0 || body >= m->nbody) {
                return -1;
            }
            const int root = m->body_rootid[body];
            if (root <= 0 || m->body_jntnum[root] != 1 || m->jnt_type[m->body_jntadr[root]] != mjJNT_FREE) {
                return -1;
            }
            return root;
        }

        // Rests the ball on the floor at (x, y). Caller holds the sim mutex.
        void place_ball(const mjModel* m, mjData* d, double x, double y) {
            const int body = mj_name2id(m, mjOBJ_BODY, "ball");
            const int geom = mj_name2id(m, mjOBJ_GEOM, "ball");
            if (body < 0 || geom < 0 || free_root(m, body) != body) {
                return;
            }
            const int jnt = m->body_jntadr[body];
            mjtNum* q     = d->qpos + m->jnt_qposadr[jnt];
            // Put the geom centre (not the body frame, should they differ) on the spot.
            q[0] += x - d->geom_xpos[3 * geom];
            q[1] += y - d->geom_xpos[3 * geom + 1];
            q[2] += m->geom_size[3 * geom] - d->geom_xpos[3 * geom + 2];
            mju_zero(d->qvel + m->jnt_dofadr[jnt], 6);
            mj_kinematics(m, d);
        }

        // Closes the sim from a reaction, once the page's restart request has been written down.
        struct Restart {};

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

        load_session();
        speed_ = cli().rtf >= 0.0 ? cli().rtf : config::load("simulation.yaml")["real_time_factor"].as<double>(1.0);

        on<Trigger<message::SimStateUpdate>, Single>().then([this] {
            std::lock_guard<std::mutex> lock(mutex_);
            if (handles_.model == nullptr) {
                return;
            }
            // Same contract as module::Viewer: hold the sim mutex only for the (sub-millisecond) touch of mjData.
            std::lock_guard<std::mutex> sim_lock(*handles_.mutex);
            apply_hands();
            if ((updates_++ % 2) == 0) {
                stream_->publish_frame(handles_.model, handles_.data);
            }
        });

        on<Trigger<message::RobotStatesUpdate>, Single>().then([this](const message::RobotStatesUpdate& states) {
            std::lock_guard<std::mutex> lock(mutex_);
            robots_.clear();
            for (const auto& robot : states.robots) {
                robots_.push_back({robot.identity, robot.mode, robot.fall_state});
            }
        });

        on<Every<200, std::chrono::milliseconds>, Single>().then([this] { publish_sim(); });

        // Only arrives while module::Supervisor is enabled (supervisor.yaml) and a GameController is on the network.
        on<Trigger<message::GameState>>().then([this](const message::GameState& game) {
            stream_->set_game(game_json(game));
        });

        // Built-in referee (--referee): its status goes to the viewers' referee panel, their button presses come
        // back as commands. Query format: action=<name>&side=<home|away>&player=<n>&arg=<text>.
        on<Trigger<message::RefereeStatus>>().then([this](const message::RefereeStatus& status) {
            stream_->set_referee(referee_json(status, "referee"));
        });
        // sim/match: module::MatchReferee instead, through the same scoreboard and panel.
        on<Trigger<message::MatchStatus>>().then([this](const message::MatchStatus& status) {
            match_.store(true, std::memory_order_release);
            show_match(status);
        });

        on<Trigger<Restart>>().then([this] { powerplant.shutdown(); });

        stream_->set_command_handler(
            [this](const std::string& path, const std::string& query) { on_command(path, parse_query(query)); });

        on<Shutdown>().then([this] {
            stream_->stop();
        });
    }

    void WebViewer::load_session() {
        nlohmann::json out;
        out["type"]     = "session";
        out["launcher"] = false;

        if (!cli().session.empty()) {
            std::ifstream file(std::filesystem::path(cli().session) / "session.json");
            const nlohmann::json json = nlohmann::json::parse(file, nullptr, false);
            if (json.is_object()) {
                launcher_ = true;
                mode_     = json.value("mode", std::string("game"));
                roles_    = json.value("roles", std::vector<std::string>{});
                fields_   = json.value("fields", std::vector<std::string>{});
                for (const auto& entry : json.value("codebases", nlohmann::json::array())) {
                    if (entry.value("available", true)) {
                        codebases_.push_back(entry.value("name", std::string("?")));
                    }
                }
                out["launcher"]     = true;
                out["mode"]         = mode_;
                out["game"]         = json.value("game", 1);
                out["groundTruth"]  = json.value("ground_truth", false);
                out["robots"]       = json.value("robots", 1);
                out["field"]        = json.value("field", std::string("middle"));
                out["homeCodebase"] = json.value("home_codebase", std::string("NUbots"));
                out["awayCodebase"] = json.value("away_codebase", std::string("NUbots"));
                out["homeRole"]     = json.value("home_role", std::string("nusim/behaviour"));
                out["awayRole"]     = json.value("away_role", std::string("nusim/behaviour"));
                out["roles"]        = roles_;
                out["fields"]       = fields_;
                out["codebases"]    = json.value("codebases", nlohmann::json::array());
            }
        }
        // Robots and the ball can be moved by hand everywhere except in a refereed game.
        out["handsOn"] = !(launcher_ && mode_ == "game");

        // Field dimensions (config/match.yaml), for placing the ball at a set play.
        double length = 14.0, width = 9.0, goal_area_length = 1.0, goal_area_width = 4.0, penalty_mark = 2.0;
        try {
            if (const YAML::Node field = config::load("match.yaml")["field"]) {
                length           = field["length"].as<double>(length);
                width            = field["width"].as<double>(width);
                goal_area_length = field["goal_area_length"].as<double>(goal_area_length);
                goal_area_width  = field["goal_area_width"].as<double>(goal_area_width);
                penalty_mark     = field["penalty_mark_distance"].as<double>(penalty_mark);
            }
        }
        catch (const std::exception&) {
            // no match.yaml in this config directory: the defaults above are the middle field
        }
        out["dims"] = {{"length", length},
                       {"width", width},
                       {"goalAreaLength", goal_area_length},
                       {"goalAreaWidth", goal_area_width},
                       {"penaltyMarkDistance", penalty_mark}};
        stream_->set_state("session", out.dump());
    }

    void WebViewer::publish_sim() {
        nlohmann::json out;
        out["type"] = "sim";
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (handles_.model == nullptr) {
                return;
            }
            out["paused"] = handles_.paused != nullptr && handles_.paused->load(std::memory_order_relaxed);
            // Two decimals: enough for the panel, and steady enough that repeats are dropped.
            const double rtf = handles_.measured_rtf != nullptr ? handles_.measured_rtf->load(std::memory_order_relaxed) : 0.0;
            out["rtf"]    = std::round(rtf * 100.0) / 100.0;
            out["speed"]  = speed_;
            out["robots"] = nlohmann::json::array();
            for (const auto& robot : robots_) {
                out["robots"].push_back({{"id", robot.identity.robot_id},
                                         {"side", robot.identity.team_id == robots_.front().identity.team_id ? 0 : 1},
                                         {"player", robot.identity.player_id},
                                         {"body", robot.identity.body()},
                                         {"mode", mode_name(robot.mode)},
                                         {"fall", fall_name(robot.fall_state)}});
            }
        }
        stream_->set_state("sim", out.dump());
    }

    void WebViewer::on_command(const std::string& path, const Query& query) {
        if (path == "/sim") {
            on_sim_command(query);
            return;
        }
        // /referee: a decision for whichever referee is running.
        if (match_.load(std::memory_order_acquire)) {
            send_match_command(text_of(query, "action"), text_of(query, "side"), text_of(query, "arg"));
        }
        else if (cli().referee) {
            auto command    = std::make_unique<message::RefereeCommand>();
            command->action = text_of(query, "action");
            command->side   = text_of(query, "side");
            command->arg    = text_of(query, "arg");
            command->player = static_cast<int>(number_of(query, "player"));
            emit(command);
        }
    }

    void WebViewer::on_sim_command(const Query& query) {
        const std::string action = text_of(query, "action");

        if (action == "pause") {
            emit(std::make_unique<message::SimPauseRequest>(message::SimPauseRequest{number_of(query, "paused") != 0.0}));
            return;
        }
        if (action == "speed") {
            const double rtf = std::clamp(number_of(query, "rtf", 1.0), 0.0, 16.0);
            {
                std::lock_guard<std::mutex> lock(mutex_);
                speed_ = rtf;
            }
            emit(std::make_unique<message::SimSpeedRequest>(message::SimSpeedRequest{rtf}));
            return;
        }
        if (action == "restart") {
            restart(query);
            return;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        if (handles_.model == nullptr) {
            return;
        }
        const mjModel* m = handles_.model;
        mjData* d        = handles_.data;

        if (action == "reset") {
            // A reset mid-drag must not leave a body pinned or a force applied.
            hold_.body = -1;
            if (push_.body > 0) {
                std::lock_guard<std::mutex> sim_lock(*handles_.mutex);
                mju_zero(d->xfrc_applied + 6 * push_.body, 6);
                push_.body = -1;
            }
            emit(std::make_unique<message::SimResetRequest>());
            emit(std::make_unique<message::SimPauseRequest>(message::SimPauseRequest{false}));
            return;
        }
        if (action == "release") {
            hold_.body = -1;
            if (push_.body > 0) {
                std::lock_guard<std::mutex> sim_lock(*handles_.mutex);
                mju_zero(d->xfrc_applied + 6 * push_.body, 6);
                push_.body = -1;
            }
            return;
        }

        // Everything below moves robots or the ball by hand, which a refereed game does not allow.
        if (launcher_ && mode_ == "game") {
            return;
        }
        if (action == "place_ball") {
            // A set play called by hand: the ball goes where the page clicked.
            std::lock_guard<std::mutex> sim_lock(*handles_.mutex);
            place_ball(m, d, number_of(query, "x"), number_of(query, "y"));
            return;
        }
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
        const int body      = static_cast<int>(number_of(query, "body", -1.0));

        if (action == "move") {
            const int root = free_root(m, body);
            if (root < 0) {
                return;
            }
            double quat[4] = {number_of(query, "qw", 1.0), number_of(query, "qx"), number_of(query, "qy"), number_of(query, "qz")};
            if (mju_normalize4(quat) < 1e-6) {
                return;
            }
            if (hold_.body != root) {
                std::lock_guard<std::mutex> sim_lock(*handles_.mutex);
                hold_.floor_z = d->qpos[m->jnt_qposadr[m->body_jntadr[root]] + 2];
            }
            hold_.body = root;
            // Stay on the modelled floor and above it.
            hold_.pos[0] = std::clamp(number_of(query, "x"), -8.0, 8.0);
            hold_.pos[1] = std::clamp(number_of(query, "y"), -6.0, 6.0);
            hold_.pos[2] = std::clamp(number_of(query, "z", hold_.floor_z), hold_.floor_z, hold_.floor_z + 2.5);
            mju_copy4(hold_.quat, quat);
            hold_.expires = deadline;
        }
        else if (action == "push") {
            if (free_root(m, body) < 0) {
                return;
            }
            if (push_.body > 0 && push_.body != body) {
                std::lock_guard<std::mutex> sim_lock(*handles_.mutex);
                mju_zero(d->xfrc_applied + 6 * push_.body, 6);
            }
            push_.body = body;
            const char* local[3]  = {"lx", "ly", "lz"};
            const char* target[3] = {"tx", "ty", "tz"};
            for (int i = 0; i < 3; ++i) {
                push_.local[i]  = std::clamp(number_of(query, local[i]), -2.0, 2.0);
                push_.target[i] = std::clamp(number_of(query, target[i]), -20.0, 20.0);
            }
            push_.expires = deadline;
        }
        else if (action == "shove") {
            // Shove the first robot over (a deterministic fall for testing fall recovery and get-up: dragging
            // is usually within what the push-randomised policy rides out).
            std::lock_guard<std::mutex> sim_lock(*handles_.mutex);
            if (m->njnt > 0 && m->jnt_type[0] == mjJNT_FREE) {
                const int dof = m->jnt_dofadr[0];
                d->qvel[dof + 0] += 1.5;  // linear kick, world x
                d->qvel[dof + 4] += 6.0;  // pitch rate: guarantees a topple
            }
        }
    }

    void WebViewer::apply_hands() {
        const mjModel* m = handles_.model;
        mjData* d        = handles_.data;
        const auto now   = std::chrono::steady_clock::now();

        if (hold_.body > 0) {
            if (now > hold_.expires) {
                hold_.body = -1;  // the page went away mid-drag: let go
            }
            else {
                const int jnt = m->body_jntadr[hold_.body];
                mjtNum* q     = d->qpos + m->jnt_qposadr[jnt];
                mju_copy3(q, hold_.pos);
                mju_copy4(q + 3, hold_.quat);
                mju_zero(d->qvel + m->jnt_dofadr[jnt], 6);
                mj_kinematics(m, d);  // so the move shows even while the sim is paused
            }
        }

        if (push_.body > 0) {
            mjtNum* force = d->xfrc_applied + 6 * push_.body;
            if (now > push_.expires) {
                mju_zero(force, 6);
                push_.body = -1;
                return;
            }
            // A critically damped spring on the grabbed point, scaled by the mass it has to move (what MuJoCo's
            // own mouse perturbation does).
            mjtNum grabbed[3];
            mju_mulMatVec3(grabbed, d->xmat + 9 * push_.body, push_.local);
            mju_addTo3(grabbed, d->xpos + 3 * push_.body);
            mjtNum velocity[6];  // rotational, then linear
            mj_objectVelocity(m, d, mjOBJ_BODY, push_.body, velocity, 0);
            const double mass      = m->body_subtreemass[m->body_rootid[push_.body]];
            const double stiffness = m->vis.map.stiffness;
            // The spring stretches a metre at most: a shove, however far the mouse went, not a launch.
            mjtNum pull[3];
            mju_sub3(pull, push_.target, grabbed);
            if (const double length = mju_norm3(pull); length > 1.0) {
                mju_scl3(pull, pull, 1.0 / length);
            }
            for (int i = 0; i < 3; ++i) {
                force[i] = mass * (stiffness * pull[i] - 2.0 * std::sqrt(stiffness) * velocity[3 + i]);
            }
            mjtNum arm[3];
            mju_sub3(arm, grabbed, d->xipos + 3 * push_.body);
            mju_cross(force + 3, arm, force);
        }
    }

    void WebViewer::restart(const Query& query) {
        if (!launcher_) {
            return;  // nothing would start the sim again
        }
        // Only what the launcher offered: these end up on its command line.
        auto offered = [&query](const char* key, const std::vector<std::string>& list) {
            const std::string value = text_of(query, key);
            return std::find(list.begin(), list.end(), value) != list.end() ? value : std::string();
        };
        nlohmann::json json;
        const std::string mode = offered("mode", {"game", "testing", "freeplay"});
        if (mode.empty()) {
            return;
        }
        json["mode"]         = mode;
        json["game"]         = std::clamp(static_cast<int>(number_of(query, "game", 1.0)), 1, 3);
        json["robots"]       = std::clamp(static_cast<int>(number_of(query, "robots", 1.0)), 1, 6);
        json["ground_truth"] = text_of(query, "ground_truth") == "1";
        const std::pair<const char*, const std::vector<std::string>*> choices[] = {{"home_codebase", &codebases_},
                                                                                   {"away_codebase", &codebases_},
                                                                                   {"field", &fields_},
                                                                                   {"home_role", &roles_},
                                                                                   {"away_role", &roles_}};
        for (const auto& [key, list] : choices) {
            if (const std::string value = offered(key, *list); !value.empty()) {
                json[key] = value;
            }
        }
        std::ofstream(std::filesystem::path(cli().session) / "request.json") << json.dump(2);
        log<NUClear::LogLevel::INFO>("Web viewer: restarting as", mode);
        emit(std::make_unique<Restart>());
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
        stream_->set_referee(referee_json(referee, "match"));

        // What the window's Match panel shows: the main button, the robots online and the referee's messages.
        nlohmann::json match;
        match["type"]      = "match";
        match["hasGame"]   = status.has_game;
        match["state"]     = state;
        match["primary"]   = status.primary_button;
        match["connected"] = status.connected;
        match["players"]   = status.players;
        match["messages"]  = status.messages;
        stream_->set_state("match", match.dump());
    }

    void WebViewer::send_match_command(const std::string& action, const std::string& side, const std::string& arg) {
        using Action = message::MatchCommand::Action;
        std::string state;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            state = match_state_;
        }

        message::MatchCommand command;
        if (action == "primary") {
            command.action = Action::PRIMARY;  // the window's main button: Start, Pause or Resume
        }
        else if (action == "kickoff") {
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
