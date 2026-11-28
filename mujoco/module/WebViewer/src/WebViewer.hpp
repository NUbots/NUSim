#ifndef K1SIM_MODULE_WEBVIEWER_HPP
#define K1SIM_MODULE_WEBVIEWER_HPP

#include <atomic>
#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <nuclear>
#include <string>
#include <vector>

#include "module/WebViewer/src/WebStream.hpp"
#include "shared/message/MatchMessages.hpp"
#include "shared/message/SimMessages.hpp"
#include "shared/sim/RobotIdentity.hpp"

namespace k1sim::module {

    // Browser based 3D viewer and match recorder (--web / --record).
    //
    // Serves the viewer page over HTTP and streams the body poses to it at half the state publish rate (25 Hz by
    // default; the viewer interpolates in between). Unlike module::Viewer it needs no GL context, so it also works
    // under --headless and from another machine. The same stream can be written to a recording file and replayed in
    // the viewer later. Inactive unless --web or --record is given.
    //
    // The page offers what module::Viewer's window does: the launcher's setup panel, the match and referee
    // controls, pause / reset / speed, the robots' modes, set plays with ball placement, and moving or shoving
    // robots and the ball by hand. Its requests arrive as POST /referee and POST /sim (see on_command).
    class WebViewer : public NUClear::Reactor {
    public:
        explicit WebViewer(std::unique_ptr<NUClear::Environment> environment);

    private:
        using Query = std::map<std::string, std::string>;

        // tools/match.py's session.json (what it launched and what it can launch instead), for the setup panel.
        void load_session();
        // A button pressed or a body dragged on the page: `path` is /referee or /sim, `query` its parameters.
        void on_command(const std::string& path, const Query& query);
        void on_sim_command(const Query& query);
        // Leaves the setup the page asked for where tools/match.py picks it up, then closes the sim.
        void restart(const Query& query);
        // Pins the body being dragged and pulls on the one being shoved. Caller holds mutex_ and the sim mutex.
        void apply_hands();
        // Pause state, speed and every robot's mode, for the Simulation and Robots panels.
        void publish_sim();

        // sim/match: module::MatchReferee's scoreboard and the panel buttons it answers, in the shape of the
        // built-in referee's GameState and RefereeStatus, so the page shows them unchanged.
        void show_match(const message::MatchStatus& status);
        // A referee panel button (see WebStream::set_command_handler) as a MatchCommand, if MatchReferee has one.
        void send_match_command(const std::string& action, const std::string& side, const std::string& arg);

        std::unique_ptr<web::WebStream> stream_;
        std::atomic<bool> match_{false};  // module::MatchReferee has reported in, so it takes the panel's buttons
        std::mutex mutex_;  // guards everything below (reactions and the page's requests run on different threads)
        message::SimHandles handles_{};
        uint64_t updates_ = 0;
        std::string match_state_;  // MatchReferee's game state, as show_match() last saw it

        // The launcher's session. Read once at startup; read-only afterwards.
        bool launcher_ = false;
        std::string mode_;                                     // game, testing or freeplay
        std::vector<std::string> roles_, codebases_, fields_;  // what a restart may ask for

        double speed_ = 1.0;  // the real-time factor asked for

        struct RobotMode {
            RobotIdentity identity;
            int mode       = -1;
            int fall_state = 0;
        };
        std::vector<RobotMode> robots_;  // every robot's latest state (RobotStatesUpdate)

        // A body the page is dragging: pinned to this pose until the page lets go, or stops saying so.
        struct Hold {
            int body = -1;  // free-floating root body, or -1 = nothing held
            double pos[3]{};
            double quat[4]{1, 0, 0, 0};
            double floor_z = 0.0;  // its height when picked up; it cannot be pushed below that
            std::chrono::steady_clock::time_point expires;
        } hold_;
        // A body the page is shoving: a spring pulls the grabbed point towards the target.
        struct Push {
            int body = -1;
            double local[3]{};   // the grabbed point, in the body's frame
            double target[3]{};  // where it is being pulled to, in the world
            std::chrono::steady_clock::time_point expires;
        } push_;
    };

}  // namespace k1sim::module

#endif  // K1SIM_MODULE_WEBVIEWER_HPP
