#ifndef K1SIM_MODULE_SUPERVISOR_HPP
#define K1SIM_MODULE_SUPERVISOR_HPP

#include <atomic>
#include <map>
#include <memory>
#include <mujoco/mujoco.h>
#include <mutex>
#include <nuclear>
#include <string>
#include <thread>

#include "module/Supervisor/src/GameControllerPacket.hpp"
#include "module/Supervisor/src/SupervisorLogic.hpp"
#include "shared/util/Udp.hpp"

namespace k1sim::module {

    // Listens to the RoboCup GameController (UDP broadcast, port 3838 by default
    // — see config/supervisor.yaml) and places physics bodies per game phase
    // (ball to centre on kickoff, penalised robots off to the side line, etc.) —
    // the sim-side supervisor role Webots used to provide. NUbots hears the
    // GameController directly and independently over its own socket; this module
    // never replies to it, it only watches the broadcast and moves bodies. A
    // no-op (idle, no errors) whenever no GameController is on the network.
    class Supervisor : public NUClear::Reactor {
    public:
        explicit Supervisor(std::unique_ptr<NUClear::Environment> environment);

    private:
        std::function<void(int)> placed_robot_;
        std::function<void()> reset_world_;
        int last_state_ = -1;
        // Decision logic (packet parsing + diffing + placement) — NUClear-free,
        // see SupervisorLogic.hpp. Owned via pointer only so it can be
        // constructed after config load, inside this constructor's body.
        std::unique_ptr<supervisor::SupervisorLogic> logic_;

        // Cached from message::SimHandles (Trigger fires once, after the model
        // loads — see module::Simulation). Set on whichever thread pool worker
        // runs that reaction and read from whichever worker runs the UDP
        // reaction; both can run concurrently (neither is MainThread — this
        // module never touches GL), so these are atomics rather than the plain
        // pointers Viewer.cpp uses (Viewer pins both sides to MainThread instead,
        // which isn't appropriate for a UDP listener). The mjData contents behind
        // them are separately guarded by *sim_mutex_ while in use, exactly like
        // every other module that touches mjData outside the physics thread.
        std::atomic<const mjModel*> model_{nullptr};
        std::atomic<mjData*> data_{nullptr};
        std::atomic<std::mutex*> sim_mutex_{nullptr};

        // GameController listener thread: receives on `fd` until listening_ goes false, and emits each valid
        // packet as a gc::GameControllerPacket. Its own socket rather than NUClear's on<UDP>, so the port can be
        // shared (SO_REUSEADDR + SO_REUSEPORT) with robot software on the same host that listens to the same
        // broadcast (see shared/util/Udp.hpp).
        void listen(int fd);

        // Acts on one GameController packet, from the network or from module::Referee: publishes the scoreboard
        // (message::GameState) for the viewers, resets the world when the game goes back to INITIAL, and moves the
        // ball and the robots the packet says need moving (SupervisorLogic::process decides which, by comparing it
        // to the last one). Runs in a Single reaction, so packets are handled one at a time and in order.
        void handle_packet(const supervisor::gc::GameControllerPacket& packet);

        util::udp::Socket socket_;
        std::atomic<bool> listening_{false};
        std::thread listener_;
        std::map<int, std::string> team_names_;  // GameController team number -> name
    };

}  // namespace k1sim::module

#endif  // K1SIM_MODULE_SUPERVISOR_HPP
