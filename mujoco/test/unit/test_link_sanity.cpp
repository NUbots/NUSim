// M0 sanity: proves the MuJoCo and NUClear imports link and the frozen shared
// contracts compile. Replaced by real model/PD tests in M1/M2.
#include <cstdio>
#include <cstring>
#include <mujoco/mujoco.h>
#include <nuclear>

#include "shared/k1/BoosterApi.hpp"
#include "shared/k1/JointIndex.hpp"
#include "shared/message/Commands.hpp"
#include "shared/message/SimMessages.hpp"

int main() {
    // MuJoCo links and reports the pinned version
    if (std::strncmp(mj_versionString(), "3.3", 3) != 0) {
        std::fprintf(stderr, "unexpected MuJoCo version: %s\n", mj_versionString());
        return 1;
    }

    // NUClear links
    NUClear::Configuration config;
    (void) config;

    // Contract invariants
    static_assert(k1sim::JOINT_COUNT == 22, "K1 has 22 joints");
    static_assert(k1sim::JOINT_NAMES.size() == k1sim::JOINT_COUNT, "name table size");
    static_assert(k1sim::booster::CHANGE_MODE == 2000 && k1sim::booster::VISUAL_KICK == 2038, "api ids");

    std::printf("link sanity OK (MuJoCo %s)\n", mj_versionString());
    return 0;
}
