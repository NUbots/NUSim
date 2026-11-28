#ifndef K1SIM_MODULE_REFEREE_HPP
#define K1SIM_MODULE_REFEREE_HPP

// clang-format off
// <nuclear> must come before any socket header: on macOS it selects the IPv6 socket API they declare.
#include <nuclear>
// clang-format on

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "module/Referee/src/GameControllerLibrary.hpp"
#include "shared/message/SimMessages.hpp"
#include "shared/util/Udp.hpp"

namespace k1sim::module {

    // Built-in referee (--referee): the official GameController running inside the sim.
    //
    // The game state, the clocks and the rules come from the GameController's own library (libgame_controller_api,
    // which the docker image has - see GameControllerLibrary.hpp), so a game here behaves like one run with the
    // GameController application. Referee decisions arrive as message::RefereeCommand (from the referee panel of the
    // window or the browser viewer). Time advances with simulation time.
    //
    // The game is announced the way the GameController does it: RoboCupGameControlData packets on UDP port 3838,
    // twice a second and whenever something happens, so robot software cannot tell the difference. The same
    // state goes to module::Supervisor in-process (as its GameControllerPacket), which places ball and robots.
    class Referee : public NUClear::Reactor {
    public:
        explicit Referee(std::unique_ptr<NUClear::Environment> environment);

        // Frees the game with lib_->destroy (gc_destroy: the game was allocated by the library, so only the library
        // can free it, and it must happen before lib_ dlclose's the library). The UDP socket closes with sender_.
        ~Referee() override;

    private:
        // Loads the library and config/referee.yaml, creates the game and opens the socket the packets go out on.
        // False, with the reason in `error`, if any of that fails; the referee then stays off.
        bool start(std::string& error);
        // Applies a referee panel command to the game, then announces the result.
        void apply(const message::RefereeCommand& command);
        // Moves the game's clocks on to `sim_time`, announcing the game every publish_period_s_.
        void advance(double sim_time);
        // Sends the game to the robots and the Supervisor, and the legal commands to the panels. Caller holds mutex_.
        void publish();

        std::unique_ptr<referee::GameControllerLibrary> lib_;
        void* game_ = nullptr;  // RoboCup::GameController, owned by the library

        // Set once start() has succeeded; until then (and for good if it failed) commands and sim time are ignored.
        std::atomic<bool> running_{false};

        // Guards game_ and every member below it. The library's game is not thread safe, and two reactions use it
        // at once: advance() on each SimStateUpdate (from the physics loop) and apply() on each RefereeCommand
        // (from the viewer's or web viewer's thread). publish() runs inside both.
        std::mutex mutex_;
        double publish_period_s_    = 0.5;  // config/referee.yaml publish_period
        double last_sim_time_       = -1.0;
        double pending_ms_          = 0.0;
        double since_publish_s_     = 0.0;
        std::uint8_t packet_number_ = 0;
        int result_seq_             = 0;
        std::string result_text_;
        bool result_ok_ = true;
        std::string last_status_;

        util::udp::Socket sender_;
        std::vector<sockaddr_in> targets_;
    };

}  // namespace k1sim::module

#endif  // K1SIM_MODULE_REFEREE_HPP
