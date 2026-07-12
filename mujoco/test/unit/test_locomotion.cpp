// Headless, free-run unit tests for module::Locomotion's mode state machine
// (LocomotionController) and the default KinematicBackend. No NUClear/
// PowerPlant involved: LocomotionController is deliberately framework-free
// (see its header), so this test loads the model directly and drives
// controller.step() + mj_step() itself, exactly as module::Simulation's
// physics thread will in the real binary (StepController contract: step()
// runs every physics tick, immediately before mj_step()).
//
// The model under test is the real vendored models/k1/K1_22dof.xml (workstream
// A), NOT a copy -- a floor plane is added programmatically via MuJoCo's
// mjSpec model-editing API (mj_parseXML -> mjs_addGeom -> mj_compile) rather
// than an XML <include>, because MuJoCo resolves an included file's own
// <compiler meshdir="..."/> against the *outermost* file's directory, not
// the included file's (verified empirically while building this test) --
// mjSpec editing sidesteps that entirely and needs no companion XML fixture.
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>
#include <yaml-cpp/yaml.h>

#include "module/Locomotion/src/LocomotionController.hpp"
#include "module/Locomotion/src/backends/KinematicMath.hpp"
#include "shared/k1/BoosterApi.hpp"
#include "shared/k1/JointIndex.hpp"
#include "shared/message/Commands.hpp"
#include "shared/sim/ModelMap.hpp"
#include "shared/util/Config.hpp"

using k1sim::JointIndexK1;
using k1sim::JOINT_COUNT;
using k1sim::ModelMap;
using k1sim::module::LocomotionController;
namespace booster = k1sim::booster;

namespace {

int g_checks   = 0;
int g_failures = 0;

void check(bool cond, const std::string& msg) {
    ++g_checks;
    if (cond) {
        std::printf("  PASS: %s\n", msg.c_str());
    }
    else {
        std::printf("  FAIL: %s\n", msg.c_str());
        ++g_failures;
    }
}

double deg2rad(double deg) {
    return deg * 3.14159265358979323846 / 180.0;
}
double rad2deg(double rad) {
    return rad * 180.0 / 3.14159265358979323846;
}

// Loads the real vendored K1 model and adds a floor plane via the mjSpec
// model-editing API (see the file header comment for why not an XML scene).
mjModel* load_test_model() {
    const auto path = k1sim::config::resolve_path("models/k1/K1_22dof.xml");
    char error[1024] = {0};
    mjSpec* spec     = mj_parseXML(path.string().c_str(), nullptr, error, sizeof(error));
    if (!spec) {
        std::fprintf(stderr, "test_locomotion: failed to parse %s: %s\n", path.string().c_str(), error);
        std::exit(1);
    }

    mjsBody* world = mjs_findBody(spec, "world");
    mjsGeom* floor = mjs_addGeom(world, nullptr);
    floor->type    = mjGEOM_PLANE;
    floor->size[0] = 0.0;
    floor->size[1] = 0.0;
    floor->size[2] = 0.05;
    floor->friction[0] = 0.8;
    floor->friction[1] = 0.005;
    floor->friction[2] = 0.0001;

    mjModel* m = mj_compile(spec, nullptr);
    if (!m) {
        std::fprintf(stderr, "test_locomotion: failed to compile model: %s\n", mjs_getError(spec));
        mj_deleteSpec(spec);
        std::exit(1);
    }
    mj_deleteSpec(spec);
    return m;
}

void reset_to_keyframe(const mjModel* m, mjData* d, const char* keyframe) {
    const int key = mj_name2id(m, mjOBJ_KEY, keyframe);
    if (key < 0) {
        std::fprintf(stderr, "test_locomotion: model has no keyframe '%s'\n", keyframe);
        std::exit(1);
    }
    mj_resetDataKeyframe(m, d, key);
    mj_forward(m, d);
}

YAML::Node locomotion_cfg() {
    YAML::Node cfg = k1sim::config::load("locomotion.yaml");
    // The suite tests the kinematic backend regardless of the shipped default
    // (test_policy_backend_smoke overrides back to policy itself).
    cfg["backend"] = "kinematic";
    return cfg;
}
YAML::Node gains_cfg() {
    return k1sim::config::load("gains.yaml");
}

struct BaseState {
    double x, y, z;
    double tilt_rad;
    double yaw_rad;
};

BaseState read_base_state(const mjModel* m, mjData* d, const ModelMap& map) {
    BaseState s;
    s.x        = d->qpos[map.root_qpos_adr + 0];
    s.y        = d->qpos[map.root_qpos_adr + 1];
    s.z        = d->qpos[map.root_qpos_adr + 2];
    s.tilt_rad = k1sim::module::backends::base_tilt(m, d, map);
    s.yaw_rad  = k1sim::module::backends::base_yaw(m, d, map);
    return s;
}

constexpr double kStandHeightMin = 0.45;
constexpr double kStandHeightMax = 0.62;

// ---------------------------------------------------------------------
// Test 1: PREPARE from the ready keyframe -- 10s, height/tilt held.
// ---------------------------------------------------------------------
void test_prepare() {
    std::printf("test_prepare:\n");
    mjModel* m = load_test_model();
    mjData* d  = mj_makeData(m);
    reset_to_keyframe(m, d, "ready");

    LocomotionController controller(locomotion_cfg(), gains_cfg());
    controller.request_mode_change(booster::PREPARE);

    const ModelMap map = ModelMap::build(m);
    for (int i = 0; i < 10000; ++i) {  // 10 s @ 1 kHz
        controller.step(m, d);
        mj_step(m, d);
    }

    const BaseState s = read_base_state(m, d, map);
    check(s.z > kStandHeightMin && s.z < kStandHeightMax,
          "base height in [0.45,0.62] (z=" + std::to_string(s.z) + ")");
    check(rad2deg(s.tilt_rad) < 10.0, "tilt < 10 deg (tilt=" + std::to_string(rad2deg(s.tilt_rad)) + " deg)");
    check(controller.mode() == booster::PREPARE, "mode() reports PREPARE");

    mj_deleteData(d);
    mj_deleteModel(m);
}

// ---------------------------------------------------------------------
// Test 2: WALKING (kinematic backend) -- forward vx, then a separate vyaw run.
// ---------------------------------------------------------------------
void test_walking_forward() {
    std::printf("test_walking_forward:\n");
    mjModel* m = load_test_model();
    mjData* d  = mj_makeData(m);
    reset_to_keyframe(m, d, "ready");

    LocomotionController controller(locomotion_cfg(), gains_cfg());
    controller.request_mode_change(booster::SOCCER);
    controller.set_walk_command(0.2, 0.0, 0.0);

    const ModelMap map  = ModelMap::build(m);
    const BaseState s0  = read_base_state(m, d, map);
    for (int i = 0; i < 5000; ++i) {  // 5 s @ 1 kHz
        controller.step(m, d);
        mj_step(m, d);
    }
    const BaseState s1 = read_base_state(m, d, map);
    const double dx    = s1.x - s0.x;

    check(dx > 0.75 && dx < 1.25, "vx=0.2: base x advanced ~1.0 m in 5s (dx=" + std::to_string(dx) + ")");
    check(s1.z > kStandHeightMin && s1.z < kStandHeightMax, "height held (z=" + std::to_string(s1.z) + ")");
    check(rad2deg(s1.tilt_rad) < 15.0, "tilt held (tilt=" + std::to_string(rad2deg(s1.tilt_rad)) + " deg)");

    mj_deleteData(d);
    mj_deleteModel(m);
}

void test_walking_yaw() {
    std::printf("test_walking_yaw:\n");
    mjModel* m = load_test_model();
    mjData* d  = mj_makeData(m);
    reset_to_keyframe(m, d, "ready");

    LocomotionController controller(locomotion_cfg(), gains_cfg());
    controller.request_mode_change(booster::SOCCER);
    controller.set_walk_command(0.0, 0.0, 0.5);

    const ModelMap map = ModelMap::build(m);
    for (int i = 0; i < 5000; ++i) {  // 5 s @ 1 kHz
        controller.step(m, d);
        mj_step(m, d);
    }
    const BaseState s1 = read_base_state(m, d, map);

    check(s1.yaw_rad > 2.5 - 0.6 && s1.yaw_rad < 2.5 + 0.6,
          "vyaw=0.5: yaw integrates ~2.5 rad in 5s (yaw=" + std::to_string(s1.yaw_rad) + ")");
    check(s1.z > kStandHeightMin && s1.z < kStandHeightMax, "height held (z=" + std::to_string(s1.z) + ")");

    mj_deleteData(d);
    mj_deleteModel(m);
}

// ---------------------------------------------------------------------
// Test 3: Head tracking (RotateHead) -- HeadCommand.pitch -> Head_pitch,
// HeadCommand.yaw -> AAHead_yaw.
// ---------------------------------------------------------------------
void test_head() {
    std::printf("test_head:\n");
    mjModel* m = load_test_model();
    mjData* d  = mj_makeData(m);
    reset_to_keyframe(m, d, "ready");

    LocomotionController controller(locomotion_cfg(), gains_cfg());
    controller.request_mode_change(booster::SOCCER);
    controller.set_head_command(/*pitch=*/0.3, /*yaw=*/0.4);

    const ModelMap map = ModelMap::build(m);
    for (int i = 0; i < 1000; ++i) {  // 1 s @ 1 kHz
        controller.step(m, d);
        mj_step(m, d);
    }

    const double head_pitch = d->qpos[map.qpos_adr[JointIndexK1::HeadPitch]];
    const double head_yaw   = d->qpos[map.qpos_adr[JointIndexK1::HeadYaw]];
    check(std::abs(head_pitch - 0.3) < 0.05, "Head_pitch reaches 0.3 within 1s (q=" + std::to_string(head_pitch) + ")");
    check(std::abs(head_yaw - 0.4) < 0.05, "AAHead_yaw reaches 0.4 within 1s (q=" + std::to_string(head_yaw) + ")");

    mj_deleteData(d);
    mj_deleteModel(m);
}

// ---------------------------------------------------------------------
// Test 4: Fall detection + GetUp recovery, from the lying_front keyframe.
// ---------------------------------------------------------------------
void test_fall_and_getup() {
    std::printf("test_fall_and_getup:\n");
    mjModel* m = load_test_model();
    mjData* d  = mj_makeData(m);
    reset_to_keyframe(m, d, "lying_front");

    LocomotionController controller(locomotion_cfg(), gains_cfg());
    // No mode requested yet (default DAMPING) -- fall detection runs
    // unconditionally every step, regardless of mode.
    controller.step(m, d);
    check(controller.fall_state() == booster::HAS_FALLEN, "lying_front => fall_state()==HAS_FALLEN");

    controller.request_get_up(booster::SOCCER);

    const double duration = locomotion_cfg()["getup"]["duration"].as<double>(2.0);
    const int steps       = static_cast<int>((duration + 2.0) / m->opt.timestep);
    for (int i = 0; i < steps; ++i) {
        controller.step(m, d);
        mj_step(m, d);
    }

    const ModelMap map = ModelMap::build(m);
    const BaseState s  = read_base_state(m, d, map);
    check(s.z > kStandHeightMin && s.z < kStandHeightMax, "standing after GetUp (z=" + std::to_string(s.z) + ")");
    check(rad2deg(s.tilt_rad) < 10.0, "upright after GetUp (tilt=" + std::to_string(rad2deg(s.tilt_rad)) + " deg)");
    check(controller.fall_state() == booster::IS_READY, "fall_state()==IS_READY after GetUp");
    check(controller.mode() == booster::SOCCER, "mode()==SOCCER after GetUp");
    check(!controller.getting_up(), "getting_up()==false after GetUp completes");

    mj_deleteData(d);
    mj_deleteModel(m);
}

// ---------------------------------------------------------------------
// Test 5: DAMPING -- ctrl all ~0 after ChangeMode(DAMPING).
// ---------------------------------------------------------------------
void test_damping() {
    std::printf("test_damping:\n");
    mjModel* m = load_test_model();
    mjData* d  = mj_makeData(m);
    reset_to_keyframe(m, d, "ready");

    LocomotionController controller(locomotion_cfg(), gains_cfg());
    controller.request_mode_change(booster::PREPARE);
    for (int i = 0; i < 500; ++i) {  // stand for a bit first
        controller.step(m, d);
        mj_step(m, d);
    }

    controller.request_mode_change(booster::DAMPING);
    controller.step(m, d);  // the very next controller.step() should zero ctrl

    double max_abs_ctrl = 0.0;
    for (int i = 0; i < m->nu; ++i) {
        max_abs_ctrl = std::max(max_abs_ctrl, std::abs(d->ctrl[i]));
    }
    check(max_abs_ctrl < 1e-9, "ctrl all ~0 after ChangeMode(DAMPING) (max|ctrl|=" + std::to_string(max_abs_ctrl) + ")");
    check(controller.mode() == booster::DAMPING, "mode()==DAMPING");

    mj_deleteData(d);
    mj_deleteModel(m);
}

// ---------------------------------------------------------------------
// Bonus smoke tests (not in the required list, but cheap and worth having):
// LieDown and VisualKick should run without producing non-finite state and
// should return control to a sane mode afterward.
// ---------------------------------------------------------------------
bool all_finite(const mjData* d, int n) {
    for (int i = 0; i < n; ++i) {
        if (!std::isfinite(d->qpos[i])) {
            return false;
        }
    }
    return true;
}

void test_liedown_smoke() {
    std::printf("test_liedown_smoke:\n");
    mjModel* m = load_test_model();
    mjData* d  = mj_makeData(m);
    reset_to_keyframe(m, d, "ready");

    LocomotionController controller(locomotion_cfg(), gains_cfg());
    controller.request_mode_change(booster::PREPARE);
    for (int i = 0; i < 500; ++i) {
        controller.step(m, d);
        mj_step(m, d);
    }

    controller.request_lie_down();
    const double duration = locomotion_cfg()["liedown"]["duration"].as<double>(1.5);
    const int steps       = static_cast<int>((duration + 1.0) / m->opt.timestep);
    for (int i = 0; i < steps; ++i) {
        controller.step(m, d);
        mj_step(m, d);
    }

    check(all_finite(d, m->nq), "LieDown: qpos stays finite throughout");
    check(controller.mode() == booster::DAMPING, "LieDown ends in DAMPING");

    mj_deleteData(d);
    mj_deleteModel(m);
}

void test_kick_smoke() {
    std::printf("test_kick_smoke:\n");
    mjModel* m = load_test_model();
    mjData* d  = mj_makeData(m);
    reset_to_keyframe(m, d, "ready");

    LocomotionController controller(locomotion_cfg(), gains_cfg());
    controller.request_mode_change(booster::SOCCER);
    for (int i = 0; i < 500; ++i) {
        controller.step(m, d);
        mj_step(m, d);
    }

    controller.request_kick(1);
    const double duration = locomotion_cfg()["kick"]["duration"].as<double>(0.5);
    const int steps       = static_cast<int>((duration + 1.0) / m->opt.timestep);
    for (int i = 0; i < steps; ++i) {
        controller.step(m, d);
        mj_step(m, d);
    }

    const ModelMap map = ModelMap::build(m);
    const BaseState s  = read_base_state(m, d, map);
    check(all_finite(d, m->nq), "Kick: qpos stays finite throughout");
    check(controller.mode() == booster::SOCCER, "Kick returns to SOCCER");
    check(rad2deg(s.tilt_rad) < 20.0, "Kick: robot doesn't fall over (tilt=" + std::to_string(rad2deg(s.tilt_rad)) + " deg)");

    mj_deleteData(d);
    mj_deleteModel(m);
}

#ifdef K1_WITH_ONNX
// M7 accept criterion: "random-weights onnx loads & runs; obs construction
// unit-tested." Only exercised through LocomotionController's public
// interface (backend=policy in locomotion.yaml) -- PolicyBackend's obs
// construction is private, so this is a behavioural test: it must run for a
// few seconds without crashing/NaN-ing and drive finite actuator torques.
void test_policy_backend_smoke() {
    std::printf("test_policy_backend_smoke:\n");
    mjModel* m = load_test_model();
    mjData* d  = mj_makeData(m);
    reset_to_keyframe(m, d, "ready");

    YAML::Node cfg = locomotion_cfg();
    cfg["backend"] = "policy";
    cfg["policy"]["path"] = "test/unit/assets/random_policy.onnx";

    LocomotionController controller(cfg, gains_cfg());
    controller.request_mode_change(booster::SOCCER);
    controller.set_walk_command(0.1, 0.0, 0.05);

    for (int i = 0; i < 2000; ++i) {  // 2 s @ 1 kHz
        controller.step(m, d);
        mj_step(m, d);
    }

    bool ctrl_finite = true;
    for (int i = 0; i < m->nu; ++i) {
        ctrl_finite = ctrl_finite && std::isfinite(d->ctrl[i]);
    }
    check(ctrl_finite, "PolicyBackend: ctrl stays finite over 2s");
    check(all_finite(d, m->nq), "PolicyBackend: qpos stays finite over 2s");

    mj_deleteData(d);
    mj_deleteModel(m);
}
#endif

}  // namespace

int main() {
    test_prepare();
    test_walking_forward();
    test_walking_yaw();
    test_head();
    test_fall_and_getup();
    test_damping();
    test_liedown_smoke();
    test_kick_smoke();
#ifdef K1_WITH_ONNX
    test_policy_backend_smoke();
#endif

    std::printf("\n%d/%d checks passed\n", g_checks - g_failures, g_checks);
    return g_failures == 0 ? 0 : 1;
}
