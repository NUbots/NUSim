// BallForecast: the ball rolled ahead in a robot-free copy of the scene (rt/nusim/gt/ball_crossing/*).
//
// Model path resolution mirrors test_supervisor.cpp: $K1SIM_TEST_MODEL overrides the scene of
// config/simulation.yaml's default `field`.
//
//   A. The copy has the ball and the goals, and nothing of the robot.
//   B. Away from the robot, the forecast goal-line crossing is where the live scene's ball crosses.
//   C. The robot-plane crossing is in the robot's yaw frame, for a robot turned away from the axes.
//   D. A resting ball, and a ball behind the robot, cross nothing.
//   E. A full-horizon forecast is cheap enough to run at the state rate.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <mujoco/mujoco.h>
#include <string>

#include "shared/sim/BallForecast.hpp"
#include "shared/sim/FreeBody.hpp"
#include "shared/util/Config.hpp"

namespace fb = k1sim::freebody;
using k1sim::BallForecast;
using k1sim::message::BallState;
using k1sim::message::BaseState;

namespace {

    bool g_ok = true;

    void fail(const std::string& what) {
        std::fprintf(stderr, "FAIL: %s\n", what.c_str());
        g_ok = false;
    }

    std::string resolve_test_model_path() {
        if (const char* override_path = std::getenv("K1SIM_TEST_MODEL")) {
            return override_path;
        }
        auto cfg = k1sim::config::load("simulation.yaml");
        return k1sim::config::resolve_path(k1sim::config::field_scene(cfg)).string();
    }

    BaseState base_at(double x, double y, double yaw) {
        BaseState base{};
        base.x    = x;
        base.y    = y;
        base.quat = {std::cos(yaw / 2.0), 0.0, 0.0, std::sin(yaw / 2.0)};
        return base;
    }

    BallState rolling_ball(const BallForecast& forecast, double x, double y, double vx, double vy) {
        const mjModel* m    = forecast.model();
        const double radius = m->geom_size[3 * mj_name2id(m, mjOBJ_GEOM, "ball")];
        BallState ball{};
        ball.valid    = true;
        ball.position = {x, y, radius};
        ball.lin_vel  = {vx, vy, 0.0};
        ball.ang_vel  = fb::rolling_spin(ball.lin_vel, radius);
        return ball;
    }

    void test_robot_free(const BallForecast& forecast) {
        const mjModel* m = forecast.model();
        // The world, the ball and the two goals
        if (mj_name2id(m, mjOBJ_BODY, "Trunk") >= 0 || m->nbody != 4 || m->nu != 0 || m->njnt != 1) {
            fail("the forecast scene still has robot bodies, actuators or joints");
        }
        if (forecast.goal_lines().size() != 2 || forecast.goal_lines()[0] <= 0.0
            || std::fabs(forecast.goal_lines()[0] + forecast.goal_lines()[1]) > 1e-9) {
            fail("expected two goal lines at x = +-half the field length");
        }
    }

    // The live scene, loaded as SimCore does, with the robot at the "goalie" keyframe far from the ball
    void test_matches_live_scene(BallForecast& forecast, const std::string& model_path) {
        char error[1024] = {0};
        mjSpec* spec     = mj_parseXML(model_path.c_str(), nullptr, error, sizeof(error));
        if (spec == nullptr) {
            fail(std::string("mj_parseXML failed: ") + error);
            return;
        }
        const auto offset = fb::reorigin_body_at_geom(spec, "ball", "ball");
        mjModel* m        = mj_compile(spec, nullptr);
        mj_deleteSpec(spec);
        if (m == nullptr) {
            fail("mj_compile failed for the live scene");
            return;
        }
        fb::shift_reoriginated_keyframes(m, mj_name2id(m, mjOBJ_BODY, "ball"), offset);
        mjData* d     = mj_makeData(m);
        const int key = mj_name2id(m, mjOBJ_KEY, "goalie");
        key >= 0 ? mj_resetDataKeyframe(m, d, key) : mj_resetData(m, d);

        const double goal    = *std::max_element(forecast.goal_lines().begin(), forecast.goal_lines().end());
        const BallState ball = rolling_ball(forecast, goal - 4.0, 0.5, 3.5, 0.2);
        const int ball_body  = mj_name2id(m, mjOBJ_BODY, "ball");
        const int ball_geom  = mj_name2id(m, mjOBJ_GEOM, "ball");
        const int qadr       = m->jnt_qposadr[m->body_jntadr[ball_body]];
        d->qpos[qadr + 0]    = ball.position[0];
        d->qpos[qadr + 1]    = ball.position[1];
        d->qpos[qadr + 2]    = ball.position[2];
        d->qpos[qadr + 3]    = 1.0;
        d->qpos[qadr + 4] = d->qpos[qadr + 5] = d->qpos[qadr + 6] = 0.0;
        fb::set_geom_centre_velocity(m, d, ball_body, ball_geom, ball.lin_vel, ball.ang_vel);

        const auto result = forecast.forecast(ball, base_at(-100.0, 0.0, 0.0), 5.0);
        if (!result.goal_line.crosses) {
            fail("a 3.5 m/s ball 4 m from the goal line was not forecast to cross it");
        }

        // Step the live scene until its ball crosses the same line
        auto before   = fb::geom_centre_state(m, d, ball_body, ball_geom);
        bool crossed  = false;
        double t_live = 0.0, y_live = 0.0;
        for (int i = 0; i < 5000 && !crossed; ++i) {
            mj_step(m, d);
            const auto after = fb::geom_centre_state(m, d, ball_body, ball_geom);
            if (after.position[0] >= goal) {
                const double f = (goal - before.position[0]) / (after.position[0] - before.position[0]);
                t_live         = (i + f) * m->opt.timestep;
                y_live         = before.position[1] + f * (after.position[1] - before.position[1]);
                crossed        = true;
            }
            before = after;
        }
        if (!crossed) {
            fail("the live ball never crossed the goal line");
        }
        else if (std::fabs(result.goal_line.time - t_live) > 2e-3
                 || std::fabs(result.goal_line.position[1] - y_live) > 1e-3
                 || std::fabs(result.goal_line.position[0] - goal) > 1e-9) {
            char buf[256];
            std::snprintf(buf,
                          sizeof(buf),
                          "forecast crossing (t %.4f, y %.4f) != live (t %.4f, y %.4f)",
                          result.goal_line.time,
                          result.goal_line.position[1],
                          t_live,
                          y_live);
            fail(buf);
        }
        else {
            std::printf("goal line: forecast t %.4f s y %.4f m, live t %.4f s y %.4f m\n",
                        result.goal_line.time,
                        result.goal_line.position[1],
                        t_live,
                        y_live);
        }
        mj_deleteData(d);
        mj_deleteModel(m);
    }

    void test_robot_frame(BallForecast& forecast) {
        // Robot at the centre facing +y; the ball comes straight down at it 0.3 m to its right
        const auto result =
            forecast.forecast(rolling_ball(forecast, 0.3, 2.0, 0.0, -3.0), base_at(0.0, 0.0, M_PI / 2), 5.0);
        const auto& c = result.robot_plane;
        if (!c.crosses) {
            fail("a ball rolled at the robot was not forecast to cross its plane");
            return;
        }
        if (std::fabs(c.position[0]) > 1e-9 || std::fabs(c.position[1] + 0.3) > 0.01 || c.velocity[0] >= -1.0
            || std::fabs(c.velocity[1]) > 0.05 || c.time < 2.0 / 3.0 || c.time > 1.0) {
            char buf[256];
            std::snprintf(buf,
                          sizeof(buf),
                          "robot-plane crossing at (%.3f, %.3f) v (%.3f, %.3f) t %.3f",
                          c.position[0],
                          c.position[1],
                          c.velocity[0],
                          c.velocity[1],
                          c.time);
            fail(std::string("expected x 0, y -0.3, closing along -x within (2/3, 1) s; got ") + buf);
        }
    }

    void test_no_crossing(BallForecast& forecast) {
        const auto resting = forecast.forecast(rolling_ball(forecast, 2.0, 0.0, 0.0, 0.0), base_at(0.0, 0.0, 0.0), 5.0);
        if (resting.robot_plane.crosses || resting.goal_line.crosses) {
            fail("a resting ball was forecast to cross a line");
        }
        const auto behind =
            forecast.forecast(rolling_ball(forecast, -1.0, 0.0, -3.0, 0.0), base_at(0.0, 0.0, 0.0), 5.0);
        if (behind.robot_plane.crosses) {
            fail("a ball behind the robot was forecast to cross its plane");
        }
    }

    void test_cost(BallForecast& forecast) {
        // Slow enough to roll for the whole horizon without reaching anything
        const auto ball  = rolling_ball(forecast, 0.0, 0.0, 0.0, 0.4);
        const auto start = std::chrono::steady_clock::now();
        forecast.forecast(ball, base_at(-100.0, 0.0, 0.0), 5.0);
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
        std::printf("full 5 s forecast: %.2f ms\n", ms);
        // The state rate is 50 Hz: a forecast has to fit comfortably in its 20 ms
        if (ms > 15.0) {
            fail("a full-horizon forecast took longer than 15 ms");
        }
    }

}  // namespace

int main() {
    const std::string model_path = resolve_test_model_path();
    try {
        BallForecast forecast(model_path);
        test_robot_free(forecast);
        test_matches_live_scene(forecast, model_path);
        test_robot_frame(forecast);
        test_no_crossing(forecast);
        test_cost(forecast);
    }
    catch (const std::exception& e) {
        fail(e.what());
    }
    if (g_ok) {
        std::printf("test_ball_forecast OK\n");
    }
    return g_ok ? 0 : 1;
}
