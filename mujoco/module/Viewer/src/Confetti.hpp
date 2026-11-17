#ifndef K1SIM_MODULE_VIEWER_CONFETTI_HPP
#define K1SIM_MODULE_VIEWER_CONFETTI_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <mujoco/mujoco.h>
#include <random>
#include <vector>

namespace k1sim::module::viewer {

    // Goal celebration: confetti blasted out of the goal mouth, plus a few firework bursts above the
    // goal. Purely visual: the pieces are extra geoms added to the scene each frame, with their own
    // simple flight (gravity and air drag, so paper flutters down), and never touch the physics.
    class Confetti {
    public:
        // A goal at (x, y), its line at x and the ball crossing towards +x (dir = +1) or -x (-1).
        // width and height are the goal mouth's (m).
        void celebrate(double x, double y, double width, double height, int dir) {
            std::uniform_real_distribution<double> u(0.0, 1.0);
            // Two cannons, one at each post, firing back out over the field and up.
            for (int side : {-1, 1}) {
                for (int i = 0; i < 160; ++i) {
                    const double speed = 8.0 + 6.0 * u(rng_);
                    const double up    = 0.9 + 0.5 * u(rng_);  // rad above the ground
                    const double fan   = (u(rng_) - 0.5) * 1.2 - side * 0.25;
                    const double vx    = -dir * speed * std::cos(up) * std::cos(fan);
                    const double vy    = speed * std::cos(up) * std::sin(fan);
                    const double vz    = speed * std::sin(up);
                    add_piece({x, y + side * width / 2.0, 0.1}, {vx, vy, vz}, 4.5 + 2.0 * u(rng_), false);
                }
            }
            // Fireworks: rockets that climb from behind the goal and burst.
            for (int i = 0; i < 4; ++i) {
                Rocket r;
                r.pos   = {x + dir * 0.6, y + (u(rng_) - 0.5) * width * 1.5, height};
                r.vel   = {(u(rng_) - 0.5) * 1.0, (u(rng_) - 0.5) * 1.0, 7.0 + 2.0 * u(rng_)};
                r.fuse  = 0.6 + 0.6 * u(rng_) + 0.35 * i;
                r.color = PALETTE[static_cast<std::size_t>(u(rng_) * PALETTE.size()) % PALETTE.size()];
                rockets_.push_back(r);
            }
        }

        bool active() const {
            return !pieces_.empty() || !rockets_.empty();
        }

        // Advance everything by dt seconds of wall time.
        void step(double dt) {
            dt = std::clamp(dt, 0.0, 0.05);
            for (auto& p : pieces_) {
                // Paper falls slowly: strong drag towards a terminal speed. Sparks fall faster.
                const double drag = p.spark ? 1.2 : 1.8;
                for (int k = 0; k < 3; ++k) {
                    p.vel[k] -= drag * p.vel[k] * dt;
                }
                p.vel[2] -= 9.81 * dt;
                // A little sideways wobble as the paper flutters.
                p.vel[0] += std::sin(p.phase * 7.0) * 0.6 * dt * (p.spark ? 0.0 : 1.0);
                for (int k = 0; k < 3; ++k) {
                    p.pos[k] += p.vel[k] * dt;
                }
                p.pos[2] = std::max(p.pos[2], 0.005);  // lands on the grass
                p.phase += dt;
                p.life -= dt;
            }
            std::erase_if(pieces_, [](const Piece& p) { return p.life <= 0.0; });

            std::uniform_real_distribution<double> u(0.0, 1.0);
            for (auto& r : rockets_) {
                r.vel[2] -= 9.81 * dt;
                for (int k = 0; k < 3; ++k) {
                    r.pos[k] += r.vel[k] * dt;
                }
                r.fuse -= dt;
                if (r.fuse <= 0.0) {
                    for (int i = 0; i < 90; ++i) {  // burst: sparks in every direction
                        const double a = 2.0 * M_PI * u(rng_), b = std::acos(2.0 * u(rng_) - 1.0);
                        const double s = 3.0 + 1.5 * u(rng_);
                        add_piece(r.pos,
                                  {s * std::sin(b) * std::cos(a), s * std::sin(b) * std::sin(a), s * std::cos(b)},
                                  1.2 + 0.8 * u(rng_),
                                  true,
                                  &r.color);
                    }
                }
            }
            std::erase_if(rockets_, [](const Rocket& r) { return r.fuse <= 0.0; });
        }

        // Append the pieces to the scene, as many as fit.
        void draw(mjvScene* scn) const {
            for (const auto& r : rockets_) {
                if (scn->ngeom >= scn->maxgeom) {
                    return;
                }
                const mjtNum size[3] = {0.04, 0.04, 0.04};
                add_geom(scn, mjGEOM_SPHERE, size, r.pos, 0.0, r.color, 1.0f);
            }
            for (const auto& p : pieces_) {
                if (scn->ngeom >= scn->maxgeom) {
                    return;
                }
                const float fade = static_cast<float>(std::clamp(p.life / 0.6, 0.0, 1.0));
                if (p.spark) {
                    const mjtNum size[3] = {0.03, 0.03, 0.03};
                    add_geom(scn, mjGEOM_SPHERE, size, p.pos, 0.0, p.color, fade);
                }
                else {
                    const mjtNum size[3] = {0.035, 0.022, 0.002};  // a flat scrap of paper
                    add_geom(scn, mjGEOM_BOX, size, p.pos, p.phase * p.spin, p.color, fade);
                }
            }
        }

    private:
        using Color = std::array<float, 3>;
        static constexpr std::array<Color, 7> PALETTE{{{1.0f, 0.25f, 0.3f},
                                                       {1.0f, 0.8f, 0.1f},
                                                       {0.2f, 0.75f, 1.0f},
                                                       {0.4f, 1.0f, 0.4f},
                                                       {1.0f, 0.5f, 0.1f},
                                                       {0.85f, 0.4f, 1.0f},
                                                       {1.0f, 1.0f, 1.0f}}};

        struct Piece {
            std::array<double, 3> pos{}, vel{};
            double life = 0.0, phase = 0.0, spin = 0.0;
            bool spark = false;
            Color color{};
        };
        struct Rocket {
            std::array<double, 3> pos{}, vel{};
            double fuse = 0.0;
            Color color{};
        };

        void add_piece(const std::array<double, 3>& pos,
                       const std::array<double, 3>& vel,
                       double life,
                       bool spark,
                       const Color* color = nullptr) {
            if (pieces_.size() > 1500) {
                return;
            }
            std::uniform_real_distribution<double> u(0.0, 1.0);
            Piece p;
            p.pos   = pos;
            p.vel   = vel;
            p.life  = life;
            p.spark = spark;
            p.phase = u(rng_) * 10.0;
            p.spin  = 4.0 + 10.0 * u(rng_);
            p.color = color ? *color : PALETTE[static_cast<std::size_t>(u(rng_) * PALETTE.size()) % PALETTE.size()];
            pieces_.push_back(p);
        }

        static void add_geom(mjvScene* scn,
                             int type,
                             const mjtNum size[3],
                             const std::array<double, 3>& pos,
                             double angle,
                             const Color& color,
                             float alpha) {
            // Tumbling: rotate about a tilted axis by `angle`.
            mjtNum quat[4], mat[9];
            const mjtNum axis[3] = {0.6, 0.48, 0.64};
            mju_axisAngle2Quat(quat, axis, angle);
            mju_quat2Mat(mat, quat);
            const mjtNum p[3]   = {pos[0], pos[1], pos[2]};
            const float rgba[4] = {color[0], color[1], color[2], alpha};
            mjvGeom& g          = scn->geoms[scn->ngeom++];
            mjv_initGeom(&g, type, size, p, mat, rgba);
            g.emission = 0.4f;
        }

        std::vector<Piece> pieces_;
        std::vector<Rocket> rockets_;
        std::mt19937 rng_{std::random_device{}()};
    };

}  // namespace k1sim::module::viewer

#endif  // K1SIM_MODULE_VIEWER_CONFETTI_HPP
