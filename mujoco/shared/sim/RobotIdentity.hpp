#ifndef K1SIM_SHARED_SIM_ROBOTIDENTITY_HPP
#define K1SIM_SHARED_SIM_ROBOTIDENTITY_HPP

#include <cstdio>
#include <stdexcept>
#include <string>

namespace k1sim {

    // Model slot 0 is the original K1; subsequent slots are attached copies.
    inline std::string robot_prefix(int model_index) {
        if (model_index < 0 || model_index >= 22) {
            throw std::invalid_argument("robot model index must be in [0, 21]");
        }
        if (model_index == 0) {
            return "";
        }
        char prefix[16];
        std::snprintf(prefix, sizeof(prefix), "sub%02d_", model_index);
        return prefix;
    }

    struct RobotIdentity {
        int robot_id    = 0;  // match-wide, 1-based; never a team-local player ID
        int model_index = 0;  // 0-based slot in the compiled scene
        int team_id     = 0;  // GameController team number
        int player_id   = 0;  // 1..11 within the team
        int dds_domain  = 0;  // dedicated Booster + ground-truth transport domain
        // LowState and ground truth go out with every Nth state update only, e.g. 50 Hz for a robot whose
        // software expects that while the sim publishes 500 Hz
        int state_every = 1;

        std::string body() const {
            return robot_prefix(model_index) + "Trunk";
        }
    };

}  // namespace k1sim

#endif  // K1SIM_SHARED_SIM_ROBOTIDENTITY_HPP
