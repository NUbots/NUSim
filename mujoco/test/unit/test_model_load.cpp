// M2 acceptance: the scene MJCF resolves against the frozen JointIndexK1 contract.
//
// Model path resolution (highest priority first):
//   1. $K1SIM_TEST_MODEL   — interim override while models/k1/k1_scene_robocup.xml
//                            (workstream A, M1) is still landing; points at a scratch scene
//                            during development.
//   2. config/simulation.yaml's `model` key — the real acceptance path once A lands.
//
// Asserts: ModelMap::build succeeds (all 22 joints/actuators found), a free root joint is
// present, and the physics timestep is 0.001s.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <mujoco/mujoco.h>
#include <stdexcept>
#include <string>

#include "shared/sim/ModelMap.hpp"
#include "shared/util/Config.hpp"

namespace {

std::string resolve_test_model_path() {
    if (const char* override_path = std::getenv("K1SIM_TEST_MODEL")) {
        return override_path;
    }
    auto cfg = k1sim::config::load("simulation.yaml");
    return k1sim::config::resolve_path(cfg["model"].as<std::string>()).string();
}

}  // namespace

int main() {
    const std::string model_path = resolve_test_model_path();

    char error[1024] = {0};
    mjModel* m = mj_loadXML(model_path.c_str(), nullptr, error, sizeof(error));
    if (m == nullptr) {
        std::fprintf(stderr, "mj_loadXML failed for '%s': %s\n", model_path.c_str(), error);
        return 1;
    }

    bool ok = true;

    try {
        const k1sim::ModelMap map = k1sim::ModelMap::build(m);
        if (map.root_qpos_adr < 0 || map.root_dof_adr < 0 || map.root_body_id < 0) {
            std::fprintf(stderr, "ModelMap: no free root joint found\n");
            ok = false;
        }
    }
    catch (const std::exception& e) {
        std::fprintf(stderr, "ModelMap::build failed (missing joint/actuator?): %s\n", e.what());
        ok = false;
    }

    if (std::fabs(m->opt.timestep - 0.001) > 1e-9) {
        std::fprintf(stderr, "unexpected timestep: %.9f (expected 0.001)\n", m->opt.timestep);
        ok = false;
    }

    if (ok) {
        std::printf("test_model_load OK (model: %s, nq=%d, nu=%d, timestep=%.6f)\n",
                     model_path.c_str(),
                     m->nq,
                     m->nu,
                     m->opt.timestep);
    }

    mj_deleteModel(m);
    return ok ? 0 : 1;
}
