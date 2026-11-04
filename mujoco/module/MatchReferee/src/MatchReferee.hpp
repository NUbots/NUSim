#ifndef K1SIM_MODULE_MATCHREFEREE_MATCHREFEREE_HPP
#define K1SIM_MODULE_MATCHREFEREE_MATCHREFEREE_HPP

#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <mujoco/mujoco.h>
#include <mutex>
#include <nuclear>
#include <set>
#include <vector>

#include "shared/message/Commands.hpp"

namespace k1sim::module {

    namespace matchreferee {
        class HoldController;
        class MatchRefereeLogic;
        class MatchRoster;
        struct MatchConfig;
    }  // namespace matchreferee

    // Runs a game inside the sim: the GameController and automatic referee advance with sim time,
    // see the ball and robots directly, and control messages go out over UDP exactly as from the
    // GameController app. Listens for status messages to report who has joined the game. Replaces
    // Supervisor, which needs an external GameController.
    class MatchReferee : public NUClear::Reactor {
    public:
        explicit MatchReferee(std::unique_ptr<NUClear::Environment> environment);
        ~MatchReferee() override;

    private:
        void new_game();
        void apply_holds();
        void send_control_message();

        std::unique_ptr<matchreferee::MatchConfig> cfg_;
        std::unique_ptr<matchreferee::MatchRefereeLogic> logic_;
        std::unique_ptr<matchreferee::MatchRoster> roster_;
        const mjModel* model_  = nullptr;
        mjData* data_          = nullptr;
        std::mutex* sim_mutex_ = nullptr;
        std::function<void()> reset_world_;

        // Wrappers still to hand to Simulation, then all of them by robot id.
        std::vector<message::ControllerHandle> pending_controllers_;
        std::map<int, std::shared_ptr<matchreferee::HoldController>> hold_controllers_;
        std::set<int> custom_robots_;  // robots whose player has taken control

        int socket_           = -1;
        double last_sim_time_ = 0.0;
        double unspent_ms_    = 0.0;
        int connected_        = 0;
        std::chrono::steady_clock::time_point next_control_{};
        std::chrono::steady_clock::time_point next_report_{};
    };

}  // namespace k1sim::module

#endif  // K1SIM_MODULE_MATCHREFEREE_MATCHREFEREE_HPP
