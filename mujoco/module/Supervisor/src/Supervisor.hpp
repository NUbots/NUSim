#ifndef K1SIM_MODULE_SUPERVISOR_HPP
#define K1SIM_MODULE_SUPERVISOR_HPP

#include <atomic>
#include <memory>
#include <mujoco/mujoco.h>
#include <mutex>
#include <nuclear>

#include "module/Supervisor/src/SupervisorLogic.hpp"

namespace k1sim::module {

    // Watches the RoboCup GameController broadcast (UDP port 3838 by default, see
    // config/supervisor.yaml) and places bodies per game phase: ball to centre on kickoff, penalised
    // robots to the side line. Never replies; NUbots talks to the GameController itself. Idle when no
    // GameController is on the network.
    class Supervisor : public NUClear::Reactor {
    public:
        explicit Supervisor(std::unique_ptr<NUClear::Environment> environment);

    private:
        // Decision logic (NUClear-free, see SupervisorLogic.hpp). Held by pointer so it can be
        // constructed after the config loads.
        std::unique_ptr<supervisor::SupervisorLogic> logic_;

        // From message::SimHandles. Atomic because the SimHandles and UDP reactions can run
        // concurrently on different pool threads. The mjData behind them is guarded by *sim_mutex_.
        std::atomic<const mjModel*> model_{nullptr};
        std::atomic<mjData*> data_{nullptr};
        std::atomic<std::mutex*> sim_mutex_{nullptr};
    };

}  // namespace k1sim::module

#endif  // K1SIM_MODULE_SUPERVISOR_HPP
