#ifndef K1SIM_SHARED_SIM_BALLFORECAST_HPP
#define K1SIM_SHARED_SIM_BALLFORECAST_HPP

#include <array>
#include <cmath>
#include <cstring>
#include <memory>
#include <mujoco/mujoco.h>
#include <stdexcept>
#include <string>
#include <vector>

#include "shared/message/SimMessages.hpp"
#include "shared/sim/FreeBody.hpp"

// Ground truth for where the ball is going: rolls the scene ball ahead in its own copy of the scene
// with no robot in it, from a physics snapshot, and reports where it crosses the robot's frontal
// plane and the goal lines. The copy keeps the floor, the goals and the ball/floor contact pair, so
// the ball rolls, spins, bounces and hits the posts exactly as it does in the live sim, until the
// robot touches it. Leaving the robot out is the point: the crossing is where the shot would have
// gone, which is what a planner has to predict before it moves.
//
// Not thread-safe: one forecast at a time per instance (it owns its mjData).
namespace k1sim {

    class BallForecast {
    public:
        struct Crossing {
            bool crosses = false;
            double time  = 0.0;                // s after the snapshot
            std::array<double, 3> position{};  // ball centre where it crosses (m)
            std::array<double, 3> velocity{};  // ball centre velocity there (m/s)
        };

        struct Result {
            // The robot's frontal plane: x = 0 of the yaw-only frame at the Trunk's ground
            // projection (x forward, z up), crossed from in front. Position and velocity in that
            // frame, fixed at the snapshot's robot pose.
            Crossing robot_plane{};
            // The first goal line the ball leaves the field over, in the world frame.
            Crossing goal_line{};
        };

        // Horizontal speed (m/s) below which the ball has stopped and the rollout ends
        static constexpr double STOPPED_SPEED = 0.01;

        // Builds the robot-free copy of the scene at scene_path (resolved), re-originating the
        // ball as SimCore does. Throws std::runtime_error if the scene does not compile without
        // the robot or has no free "ball" body with a "ball" geom.
        explicit BallForecast(const std::string& scene_path) {
            char error[1024] = {0};
            mjSpec* spec     = mj_parseXML(scene_path.c_str(), nullptr, error, sizeof(error));
            if (spec == nullptr) {
                throw std::runtime_error("BallForecast: mj_parseXML failed for '" + scene_path + "': " + error);
            }
            try {
                remove_robot(spec);
            }
            catch (...) {
                mj_deleteSpec(spec);
                throw;
            }
            const auto ball_offset = freebody::reorigin_body_at_geom(spec, "ball", "ball");
            (void) ball_offset;  // no keyframes left to shift
            m_ = mj_compile(spec, nullptr);
            if (m_ == nullptr) {
                const std::string what =
                    std::string("BallForecast: mj_compile failed without the robot: ") + mjs_getError(spec);
                mj_deleteSpec(spec);
                throw std::runtime_error(what);
            }
            mj_deleteSpec(spec);

            ball_body_ = mj_name2id(m_, mjOBJ_BODY, "ball");
            ball_geom_ = mj_name2id(m_, mjOBJ_GEOM, "ball");
            if (!freebody::valid_free_body_geom(m_, ball_body_, ball_geom_)) {
                mj_deleteModel(m_);
                throw std::runtime_error("BallForecast: the scene has no free \"ball\" body with a \"ball\" geom");
            }
            // The goal lines, from the goal bodies at x = +-half the field length
            for (const char* goal : {"goal_pos_x", "goal_neg_x"}) {
                const int id = mj_name2id(m_, mjOBJ_BODY, goal);
                if (id >= 0) {
                    goal_lines_.push_back(m_->body_pos[3 * id]);
                }
            }
            d_ = mj_makeData(m_);
        }

        ~BallForecast() {
            mj_deleteData(d_);
            mj_deleteModel(m_);
        }

        BallForecast(const BallForecast&)            = delete;
        BallForecast& operator=(const BallForecast&) = delete;

        const mjModel* model() const noexcept {
            return m_;
        }
        const std::vector<double>& goal_lines() const noexcept {
            return goal_lines_;
        }

        // Rolls the ball ahead from its snapshot state for at most horizon seconds, stopping once
        // it has crossed both lines or stopped.
        Result forecast(const message::BallState& ball, const message::BaseState& base, double horizon) {
            Result result{};
            if (!ball.valid) {
                return result;
            }
            mj_resetData(m_, d_);
            const int qadr = m_->jnt_qposadr[m_->body_jntadr[ball_body_]];
            // A uniform sphere's orientation doesn't matter: identity, so the free joint's body-frame
            // spin is the world-frame spin
            d_->qpos[qadr + 3] = 1.0;
            const auto offset  = freebody::geom_offset_world(m_, d_, ball_body_, ball_geom_);
            for (int k = 0; k < 3; ++k) {
                d_->qpos[qadr + k] = ball.position[k] - offset[k];
            }
            freebody::set_geom_centre_velocity(m_, d_, ball_body_, ball_geom_, ball.lin_vel, ball.ang_vel);

            // The robot's yaw frame at the snapshot
            const double yaw = std::atan2(2.0 * (base.quat[0] * base.quat[3] + base.quat[1] * base.quat[2]),
                                          1.0 - 2.0 * (base.quat[2] * base.quat[2] + base.quat[3] * base.quat[3]));
            const double c   = std::cos(yaw);
            const double s   = std::sin(yaw);
            auto to_robot    = [&](const std::array<double, 3>& p) {
                const double dx = p[0] - base.x;
                const double dy = p[1] - base.y;
                return std::array<double, 3>{c * dx + s * dy, -s * dx + c * dy, p[2]};
            };
            auto rotate_to_robot = [&](const std::array<double, 3>& v) {
                return std::array<double, 3>{c * v[0] + s * v[1], -s * v[0] + c * v[1], v[2]};
            };

            // Only a ball starting in front of the robot can cross its plane from in front, and only a
            // ball on the field can leave it over a goal line
            bool robot_pending = to_robot(ball.position)[0] > 0.0;
            bool goal_pending  = !goal_lines_.empty();
            for (const double line : goal_lines_) {
                goal_pending = goal_pending && (ball.position[0] - line) * line < 0.0;
            }

            auto state      = freebody::geom_centre_state(m_, d_, ball_body_, ball_geom_);
            const double dt = m_->opt.timestep;
            const int steps = static_cast<int>(std::ceil(horizon / dt));
            for (int i = 0; i < steps && (robot_pending || goal_pending); ++i) {
                mj_step(m_, d_);
                const auto next = freebody::geom_centre_state(m_, d_, ball_body_, ball_geom_);

                if (robot_pending) {
                    const double before = to_robot(state.position)[0];
                    const double after  = to_robot(next.position)[0];
                    if (after <= 0.0) {
                        const double f              = before / (before - after);
                        result.robot_plane.crosses  = true;
                        result.robot_plane.time     = (i + f) * dt;
                        result.robot_plane.position = to_robot(lerp(state.position, next.position, f));
                        result.robot_plane.velocity = rotate_to_robot(lerp(state.lin_vel, next.lin_vel, f));
                        robot_pending               = false;
                    }
                }
                if (goal_pending) {
                    for (const double line : goal_lines_) {
                        const double before = state.position[0] - line;
                        const double after  = next.position[0] - line;
                        if (before * line < 0.0 && after * line >= 0.0) {
                            const double f            = before / (before - after);
                            result.goal_line.crosses  = true;
                            result.goal_line.time     = (i + f) * dt;
                            result.goal_line.position = lerp(state.position, next.position, f);
                            result.goal_line.velocity = lerp(state.lin_vel, next.lin_vel, f);
                            goal_pending              = false;
                            break;
                        }
                    }
                }

                state = next;
                if (std::hypot(state.lin_vel[0], state.lin_vel[1]) < STOPPED_SPEED) {
                    break;
                }
            }
            return result;
        }

    private:
        static std::array<double, 3> lerp(const std::array<double, 3>& a, const std::array<double, 3>& b, double f) {
            return {a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2])};
        }

        static void remove(mjSpec* spec, mjsElement* element) {
            if (mjs_delete(spec, element) != 0) {
                throw std::runtime_error(std::string("BallForecast: cannot take the robot out of the scene: ")
                                         + mjs_getError(spec));
            }
        }

        static void delete_all(mjSpec* spec, mjtObj type) {
            for (mjsElement* el = mjs_firstElement(spec, type); el != nullptr; el = mjs_firstElement(spec, type)) {
                remove(spec, el);
            }
        }

        // Takes the robot out of the scene: its body tree, and everything that names its joints,
        // bodies or geoms. Contact pairs that don't involve the robot (the tuned ball/floor pair)
        // stay, so the ball behaves as in the live sim. The keyframes go first: MuJoCo refuses to
        // delete a body tree while a keyframe still sizes its joints.
        static void remove_robot(mjSpec* spec) {
            for (const mjtObj type :
                 {mjOBJ_KEY, mjOBJ_ACTUATOR, mjOBJ_SENSOR, mjOBJ_TENDON, mjOBJ_EQUALITY, mjOBJ_EXCLUDE}) {
                delete_all(spec, type);
            }
            if (mjsBody* trunk = mjs_findBody(spec, "Trunk")) {
                remove(spec, trunk->element);
            }
            std::vector<mjsElement*> orphans{};
            for (mjsElement* el = mjs_firstElement(spec, mjOBJ_PAIR); el != nullptr; el = mjs_nextElement(spec, el)) {
                const mjsPair* pair = mjs_asPair(el);
                if (mjs_findElement(spec, mjOBJ_GEOM, mjs_getString(pair->geomname1)) == nullptr
                    || mjs_findElement(spec, mjOBJ_GEOM, mjs_getString(pair->geomname2)) == nullptr) {
                    orphans.push_back(el);
                }
            }
            for (mjsElement* el : orphans) {
                remove(spec, el);
            }
        }

        mjModel* m_    = nullptr;
        mjData* d_     = nullptr;
        int ball_body_ = -1;
        int ball_geom_ = -1;
        std::vector<double> goal_lines_{};
    };

}  // namespace k1sim

#endif  // K1SIM_SHARED_SIM_BALLFORECAST_HPP
