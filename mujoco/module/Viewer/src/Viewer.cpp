#include "module/Viewer/src/Viewer.hpp"

#include <algorithm>
#include <cmath>
#include <imgui.h>
#include <filesystem>
#include <fstream>
#include <imgui_impl_opengl3.h>
#include <nlohmann/json.hpp>

#include "module/Viewer/src/ViewerUI.hpp"
#include "shared/util/Config.hpp"

#include <GLFW/glfw3.h>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <functional>
#include <memory>
#include <mujoco/mujoco.h>
#include <mutex>

#include "MatchPanel.hpp"
#include "VideoRecorder.hpp"
#include "shared/CliOptions.hpp"
#include "shared/k1/BoosterApi.hpp"
#include "shared/message/Commands.hpp"
#include "shared/message/SimMessages.hpp"
#include "shared/util/RobotRoster.hpp"

namespace k1sim::module {

    namespace {

        // GLFW callbacks must be plain C function pointers, and this Reactor is only
        // ever installed once (PowerPlant::install<Viewer>() is called exactly once
        // from main.cpp), so the GL/mjv/mjr state lives here as translation-unit
        // globals rather than as Viewer members — that keeps GLFW/MuJoCo headers out
        // of Viewer.hpp (main.cpp only needs the Reactor type, matching every other
        // module in this tree).

        GLFWwindow* window = nullptr;

        mjvCamera cam;
        mjvOption opt;
        mjvPerturb pert;
        mjvScene scn;
        mjrContext con;
        bool scene_ready = false;
        std::unique_ptr<VideoRecorder> video;
        std::vector<unsigned char> video_rgb;
        NUClear::clock::time_point video_start;
        int video_frames = 0, video_limit = 0;
        std::vector<RobotIdentity> video_roster;

        // Cached from SimHandles (Trigger<SimHandles, MainThread>, set once after the
        // model loads). model/data/mutex are owned by module::Simulation.
        const mjModel* g_model              = nullptr;
        mjData* g_data                      = nullptr;
        std::mutex* g_mutex                 = nullptr;
        std::atomic<double>* g_measured_rtf = nullptr;

        // Cached from SimStateUpdate (50 Hz) — overlay text only, never used for
        // anything physics-critical, so plain atomics without the sim mutex are fine.
        std::atomic<double> g_sim_time{0.0};
        std::atomic<int> g_mode{-1};

        // Set by the Viewer constructor; lets the plain-C GLFW key callback emit a
        // NUClear message (SimResetRequest) without holding a Reactor pointer here.
        std::function<void()> g_request_reset;

        MatchPanel match_panel;
        std::function<void(const message::MatchCommand&)> g_send_match_command;

        // Control panels (Dear ImGui). g_ui is null until the model has loaded, because the ImGui
        // context is created alongside the MuJoCo render context, so callbacks must check it.
        std::unique_ptr<viewer::ViewerUI> g_ui;
        viewer::UIState g_ui_state{};
        std::atomic<bool>* g_paused = nullptr;
        std::mutex g_match_mutex;
        message::MatchStatus g_match_status;
        std::vector<std::string> g_legal;  // module::Referee: decisions the rules allow now
        std::vector<viewer::UIState::Robot> g_robots;
        // Set by the mouse callback when a robot is clicked with "Penalise" armed: (side, player, penalised).
        std::optional<viewer::UIState::Robot> g_penalise_pick;

        // Reads tools/match.py's session.json (what it launched and which roles exist) into the UI.
        void load_session(viewer::UIState& ui) {
            if (cli().session.empty()) {
                return;
            }
            std::ifstream file(std::filesystem::path(cli().session) / "session.json");
            const nlohmann::json json = nlohmann::json::parse(file, nullptr, false);
            if (!json.is_object()) {
                return;
            }
            ui.roles  = json.value("roles", std::vector<std::string>{});
            ui.fields = json.value("fields", std::vector<std::string>{});
            for (const auto& entry : json.value("codebases", nlohmann::json::array())) {
                ui.codebases.push_back({entry.value("name", std::string("?")),
                                        entry.value("available", true),
                                        entry.value("note", std::string())});
            }
            auto index_of = [](std::vector<std::string>& list, const std::string& value) {
                const auto found = std::find(list.begin(), list.end(), value);
                if (found == list.end()) {
                    list.push_back(value);
                    return static_cast<int>(list.size()) - 1;
                }
                return static_cast<int>(found - list.begin());
            };
            auto codebase_index = [&ui](const std::string& name) {
                for (std::size_t i = 0; i < ui.codebases.size(); ++i) {
                    if (ui.codebases[i].name == name) {
                        return static_cast<int>(i);
                    }
                }
                ui.codebases.push_back({name, true, ""});
                return static_cast<int>(ui.codebases.size()) - 1;
            };
            const std::string mode   = json.value("mode", std::string("game"));
            ui.running.mode          = mode == "testing"    ? viewer::MODE_TESTING
                                       : mode == "freeplay" ? viewer::MODE_FREEPLAY
                                                            : viewer::MODE_GAME;
            ui.running.game          = json.value("game", 1);
            ui.running.home_codebase = codebase_index(json.value("home_codebase", std::string("NUbots")));
            ui.running.away_codebase = codebase_index(json.value("away_codebase", std::string("NUbots")));
            ui.running.home_role     = index_of(ui.roles, json.value("home_role", std::string("nusim/behaviour")));
            ui.running.away_role     = index_of(ui.roles, json.value("away_role", std::string("nusim/behaviour")));
            ui.running.ground_truth  = json.value("ground_truth", false);
            ui.running.field         = index_of(ui.fields, json.value("field", std::string("middle")));
            ui.running.robots        = json.value("robots", 1);
            ui.wanted                = ui.running;
            ui.launcher              = true;
            if (cli().web) {
                ui.web_url = "http://" + cli().web_host + ":" + std::to_string(cli().web_port);
            }

            const YAML::Node field = config::load("match.yaml")["field"];
            if (field) {
                ui.field_length          = field["length"].as<double>(ui.field_length);
                ui.field_width           = field["width"].as<double>(ui.field_width);
                ui.goal_area_length      = field["goal_area_length"].as<double>(ui.goal_area_length);
                ui.goal_area_width       = field["goal_area_width"].as<double>(ui.goal_area_width);
                ui.penalty_mark_distance = field["penalty_mark_distance"].as<double>(ui.penalty_mark_distance);
            }
        }

        // Leaves the setup the user asked for where tools/match.py picks it up after the sim exits.
        void write_restart_request(const viewer::UIState& ui) {
            static constexpr const char* modes[3] = {"game", "testing", "freeplay"};
            nlohmann::json json;
            json["mode"]         = modes[ui.wanted.mode];
            json["game"]         = ui.wanted.game;
            json["ground_truth"] = ui.wanted.ground_truth;
            json["robots"]       = ui.wanted.robots;
            if (!ui.codebases.empty()) {
                json["home_codebase"] = ui.codebases[static_cast<std::size_t>(ui.wanted.home_codebase)].name;
                json["away_codebase"] = ui.codebases[static_cast<std::size_t>(ui.wanted.away_codebase)].name;
            }
            if (!ui.fields.empty()) {
                json["field"] = ui.fields[static_cast<std::size_t>(ui.wanted.field)];
            }
            if (!ui.roles.empty()) {
                json["home_role"] = ui.roles[static_cast<std::size_t>(ui.wanted.home_role)];
                json["away_role"] = ui.roles[static_cast<std::size_t>(ui.wanted.away_role)];
            }
            std::ofstream(std::filesystem::path(cli().session) / "request.json") << json.dump(2);
        }

        // Rests the ball on the floor at (x, y). Caller holds the sim mutex.
        void place_ball(double x, double y) {
            const int body = mj_name2id(g_model, mjOBJ_BODY, "ball");
            const int geom = mj_name2id(g_model, mjOBJ_GEOM, "ball");
            if (body < 0 || geom < 0 || g_model->body_jntnum[body] != 1
                || g_model->jnt_type[g_model->body_jntadr[body]] != mjJNT_FREE) {
                return;
            }
            const int jnt = g_model->body_jntadr[body];
            mjtNum* q     = g_data->qpos + g_model->jnt_qposadr[jnt];
            // Put the geom centre (not the body frame, should they differ) on the spot.
            q[0] += x - g_data->geom_xpos[3 * geom];
            q[1] += y - g_data->geom_xpos[3 * geom + 1];
            q[2] += g_model->geom_size[3 * geom] - g_data->geom_xpos[3 * geom + 2];
            mju_zero(g_data->qvel + g_model->jnt_dofadr[jnt], 6);
            mj_kinematics(g_model, g_data);
        }

        // Robots and the ball can be moved by hand everywhere except in a refereed game.
        bool hands_on() {
            return !(g_ui_state.launcher && g_ui_state.running.mode == viewer::MODE_GAME);
        }

        bool ui_wants_mouse() {
            return g_ui != nullptr && ImGui::GetIO().WantCaptureMouse;
        }

        // Mouse state for the simulate-style camera + perturb controls.
        bool button_left   = false;
        bool button_middle = false;
        bool button_right  = false;
        double last_x      = 0.0;
        double last_y      = 0.0;

        const char* mode_name(int mode) {
            switch (mode) {
                case booster::DAMPING: return "DAMPING";
                case booster::PREPARE: return "PREPARE";
                case booster::WALKING: return "WALKING";
                case booster::CUSTOM: return "CUSTOM";
                case booster::SOCCER: return "SOCCER";
                default: return "?";
            }
        }

        // The free camera mjv_updateScene last built, as a pinhole in world coordinates.
        struct CameraFrame {
            double pos[3], fwd[3], up[3], right[3];
            double near, height, half_w, centre, bottom;
        };

        int g_win_w = 0;  // window size in screen coordinates (what GLFW cursor positions use)
        int g_win_h = 0;

        bool camera_frame(CameraFrame& c) {
            if (!scene_ready || g_win_w <= 0 || g_win_h <= 0) {
                return false;
            }
            const mjvGLCamera& a = scn.camera[0];
            const mjvGLCamera& b = scn.camera[1];
            for (int i = 0; i < 3; ++i) {
                c.pos[i] = 0.5 * (a.pos[i] + b.pos[i]);
                c.fwd[i] = 0.5 * (a.forward[i] + b.forward[i]);
                c.up[i]  = 0.5 * (a.up[i] + b.up[i]);
            }
            mju_normalize3(c.fwd);
            mju_normalize3(c.up);
            mju_cross(c.right, c.fwd, c.up);
            c.near   = a.frustum_near;
            c.height = a.frustum_top - a.frustum_bottom;
            c.bottom = a.frustum_bottom;
            c.half_w = a.frustum_width > 0.0f ? a.frustum_width : 0.5 * c.height * g_win_w / g_win_h;
            c.centre = 0.5 * (a.frustum_center + b.frustum_center);
            return c.height > 0.0 && c.half_w > 0.0;
        }

        // World point -> window coordinates. False when the point is behind the camera or
        // (with `clip`) outside the view.
        bool project(const double world[3], ImVec2& out, bool clip = true) {
            CameraFrame c;
            if (!camera_frame(c)) {
                return false;
            }
            double rel[3];
            mju_sub3(rel, world, c.pos);
            const double depth = mju_dot3(rel, c.fwd);
            if (depth <= c.near) {
                return false;
            }
            const double nx = (mju_dot3(rel, c.right) * c.near / depth - c.centre) / c.half_w;
            const double ny = (mju_dot3(rel, c.up) * c.near / depth - c.bottom) / c.height * 2.0 - 1.0;
            if (clip && (std::abs(nx) > 1.0 || std::abs(ny) > 1.0)) {
                return false;
            }
            out = {static_cast<float>((nx * 0.5 + 0.5) * g_win_w), static_cast<float>((0.5 - ny * 0.5) * g_win_h)};
            return true;
        }

        // Window coordinates -> the point under the cursor on the horizontal plane z = plane_z.
        bool ground_point(double x, double y, double plane_z, double out[2]) {
            CameraFrame c;
            if (!camera_frame(c)) {
                return false;
            }
            const double nx = x / g_win_w * 2.0 - 1.0;
            const double ny = 1.0 - y / g_win_h * 2.0;
            const double r  = nx * c.half_w + c.centre;
            const double u  = (ny + 1.0) * 0.5 * c.height + c.bottom;
            double dir[3];
            for (int i = 0; i < 3; ++i) {
                dir[i] = c.fwd[i] * c.near + c.right[i] * r + c.up[i] * u;
            }
            if (std::abs(dir[2]) < 1e-9) {
                return false;
            }
            const double t = (plane_z - c.pos[2]) / dir[2];
            if (t <= 0.0) {
                return false;  // the plane is behind the camera along this ray
            }
            out[0] = c.pos[0] + t * dir[0];
            out[1] = c.pos[1] + t * dir[1];
            return true;
        }

        // Move/rotate handles for the selected robot or ball (CAD-style triad): three axis
        // arrows and a yaw ring, drawn over the scene and dragged with the left button. While a
        // handle (or a Ctrl + drag) is held, the body is pinned to `pos`/`quat` every frame.
        struct Gizmo {
            enum Handle { NONE = -1, AXIS_X = 0, AXIS_Y = 1, AXIS_Z = 2, RING = 3, CARRY = 4 };
            int body    = -1;    // free-floating root body, or -1 = nothing selected
            Handle drag = NONE;
            double pos[3]{};     // drag target (root body frame origin)
            double quat[4]{1, 0, 0, 0};
            double grab[2]{};    // CARRY: body position minus the grabbed ground point
            double plane_z = 0.0;  // CARRY: height of the grabbed point
            double floor_z = 0.0;  // height at the start of the drag; the body cannot be pushed below it
            double pivot[3]{};     // RING: point the body turns about (its centre of mass)
            double last_angle = 0.0;  // RING: bearing of the cursor at the previous mouse event

            // Geometry of the handles as last drawn (window coordinates), for hit-testing.
            bool drawn = false;
            double centre_world[3]{};
            double length = 0.6;  // arrow length, metres
            ImVec2 centre{};
            ImVec2 tip[3]{};
            bool tip_ok[3]{};
            std::vector<ImVec2> ring;
        };
        Gizmo g_gizmo;

        // The free-floating body (robot trunk, ball) that `body` belongs to, or -1.
        int free_root(int body) {
            if (g_model == nullptr || body <= 0) {
                return -1;
            }
            const int root = g_model->body_rootid[body];
            if (root <= 0 || g_model->body_jntnum[root] != 1 || g_model->jnt_type[g_model->body_jntadr[root]] != mjJNT_FREE) {
                return -1;
            }
            return root;
        }

        float segment_distance(ImVec2 p, ImVec2 a, ImVec2 b) {
            const float vx = b.x - a.x;
            const float vy = b.y - a.y;
            const float l2 = vx * vx + vy * vy;
            const float t  = l2 > 0.0f ? std::clamp(((p.x - a.x) * vx + (p.y - a.y) * vy) / l2, 0.0f, 1.0f) : 0.0f;
            return std::hypot(p.x - (a.x + t * vx), p.y - (a.y + t * vy));
        }

        // Which handle is under the cursor. Arrows win over the ring where they overlap.
        Gizmo::Handle gizmo_hit(double x, double y) {
            if (g_gizmo.body < 0 || !g_gizmo.drawn) {
                return Gizmo::NONE;
            }
            const ImVec2 p{static_cast<float>(x), static_cast<float>(y)};
            const float reach = std::max(9.0f, g_win_h * 0.011f);
            for (int axis = 0; axis < 3; ++axis) {
                if (!g_gizmo.tip_ok[axis]) {
                    continue;
                }
                // Skip the stretch near the origin, where all three arrows meet.
                const ImVec2 from{g_gizmo.centre.x + 0.3f * (g_gizmo.tip[axis].x - g_gizmo.centre.x),
                                  g_gizmo.centre.y + 0.3f * (g_gizmo.tip[axis].y - g_gizmo.centre.y)};
                if (segment_distance(p, from, g_gizmo.tip[axis]) < reach) {
                    return static_cast<Gizmo::Handle>(axis);
                }
            }
            for (std::size_t i = 0; i + 1 < g_gizmo.ring.size(); ++i) {
                if (segment_distance(p, g_gizmo.ring[i], g_gizmo.ring[i + 1]) < reach) {
                    return Gizmo::RING;
                }
            }
            return Gizmo::NONE;
        }

        // Caller holds the sim mutex. Seeds the drag target from the body's current pose.
        void gizmo_begin(Gizmo::Handle handle) {
            const mjtNum* q = g_data->qpos + g_model->jnt_qposadr[g_model->body_jntadr[g_gizmo.body]];
            mju_copy3(g_gizmo.pos, q);
            mju_copy4(g_gizmo.quat, q + 3);
            g_gizmo.floor_z = q[2];
            mju_copy3(g_gizmo.pivot, g_data->xipos + 3 * g_gizmo.body);
            g_gizmo.drag = handle;
        }

        // Caller holds the sim mutex. Pins the dragged body to the drag target.
        void gizmo_hold() {
            const int jnt = g_model->body_jntadr[g_gizmo.body];
            mjtNum* q     = g_data->qpos + g_model->jnt_qposadr[jnt];
            mju_copy3(q, g_gizmo.pos);
            mju_copy4(q + 3, g_gizmo.quat);
            mju_zero(g_data->qvel + g_model->jnt_dofadr[jnt], 6);
            mj_kinematics(g_model, g_data);  // so the move shows even while the sim is paused
        }

        void gizmo_draw(ImDrawList* dl) {
            g_gizmo.drawn = false;
            g_gizmo.ring.clear();
            if (g_gizmo.body < 0) {
                return;
            }
            const double* c = g_gizmo.centre_world;
            if (!project(c, g_gizmo.centre, false)) {
                return;
            }
            // Keep the handles a constant size on screen (about 80 px), however far the camera is.
            double len = std::clamp(0.035 * cam.distance, 0.25, 1.6);
            CameraFrame frame;
            if (camera_frame(frame)) {
                const double beside[3] = {c[0] + frame.right[0], c[1] + frame.right[1], c[2] + frame.right[2]};
                ImVec2 at;
                if (project(beside, at, false)) {
                    const double pixels_per_metre = std::hypot(at.x - g_gizmo.centre.x, at.y - g_gizmo.centre.y);
                    if (pixels_per_metre > 1.0) {
                        len = std::clamp(std::max(80.0, g_win_h * 0.09) / pixels_per_metre, 0.15, 6.0);
                    }
                }
            }
            g_gizmo.length   = len;
            for (int axis = 0; axis < 3; ++axis) {
                double tip[3] = {c[0], c[1], c[2]};
                tip[axis] += len;
                g_gizmo.tip_ok[axis] = project(tip, g_gizmo.tip[axis], false);
            }
            constexpr int SEGMENTS = 48;
            for (int i = 0; i <= SEGMENTS; ++i) {
                const double a   = 2.0 * mjPI * i / SEGMENTS;
                const double w[3] = {c[0] + 0.72 * len * std::cos(a), c[1] + 0.72 * len * std::sin(a), c[2]};
                ImVec2 at;
                if (!project(w, at, false)) {
                    g_gizmo.ring.clear();
                    break;
                }
                g_gizmo.ring.push_back(at);
            }
            g_gizmo.drawn = true;

            const Gizmo::Handle hot = g_gizmo.drag != Gizmo::NONE ? g_gizmo.drag
                                      : ui_wants_mouse()          ? Gizmo::NONE
                                                                  : gizmo_hit(last_x, last_y);
            const float thick  = std::max(3.0f, g_win_h * 0.0042f);
            const ImU32 shadow = IM_COL32(0, 0, 0, 150);
            auto colour        = [&](Gizmo::Handle h, int r, int g, int b) {
                return hot == h ? IM_COL32(255, 255, 255, 255) : IM_COL32(r, g, b, 255);
            };

            if (g_gizmo.ring.size() > 1) {
                const float t = hot == Gizmo::RING ? thick * 1.5f : thick;
                dl->AddPolyline(g_gizmo.ring.data(), static_cast<int>(g_gizmo.ring.size()), shadow, 0, t + 3.0f);
                dl->AddPolyline(g_gizmo.ring.data(), static_cast<int>(g_gizmo.ring.size()),
                                colour(Gizmo::RING, 255, 200, 60), 0, t);
            }
            static constexpr int rgb[3][3] = {{240, 76, 76}, {80, 214, 100}, {84, 150, 255}};
            for (int axis = 0; axis < 3; ++axis) {
                if (!g_gizmo.tip_ok[axis]) {
                    continue;
                }
                const ImVec2 a = g_gizmo.centre;
                const ImVec2 b = g_gizmo.tip[axis];
                const float l  = std::hypot(b.x - a.x, b.y - a.y);
                if (l < 4.0f) {
                    continue;  // looking straight down this axis
                }
                const Gizmo::Handle h = static_cast<Gizmo::Handle>(axis);
                const float t         = hot == h ? thick * 1.5f : thick;
                const float head      = std::min(t * 4.0f, l * 0.6f);
                const ImVec2 d{(b.x - a.x) / l, (b.y - a.y) / l};
                const ImVec2 n{-d.y, d.x};
                const ImVec2 neck{b.x - d.x * head, b.y - d.y * head};
                const ImU32 ink = colour(h, rgb[axis][0], rgb[axis][1], rgb[axis][2]);
                dl->AddLine(a, neck, shadow, t + 3.0f);
                dl->AddLine(a, neck, ink, t);
                const float half = head * 0.55f;
                const ImVec2 p1{neck.x + n.x * half, neck.y + n.y * half};
                const ImVec2 p2{neck.x - n.x * half, neck.y - n.y * half};
                dl->AddTriangleFilled(b, p1, p2, ink);
                dl->AddTriangle(b, p1, p2, shadow, 1.5f);
            }
            dl->AddCircleFilled(g_gizmo.centre, thick * 1.3f, IM_COL32(255, 255, 255, 255));
            dl->AddCircle(g_gizmo.centre, thick * 1.3f, shadow, 0, 1.5f);
        }

        // Release any perturbation force/selection so a stray drag never leaves the
        // robot pinned once the mouse button comes back up.
        void end_perturb() {
            pert.active  = 0;
            g_gizmo.drag = Gizmo::NONE;
        }

        // The body under the cursor (mjv_select), or -1. Caller holds the sim mutex.
        int select_at(GLFWwindow* w, mjtNum selpnt[3], int* skinid) {
            int width  = 0;
            int height = 0;
            glfwGetWindowSize(w, &width, &height);
            if (width <= 0 || height <= 0) {
                return -1;
            }
            int geomid = -1;
            int flexid = -1;
            return mjv_select(g_model, g_data, &opt, static_cast<double>(width) / height, last_x / width,
                              1.0 - last_y / height, &scn, selpnt, &geomid, &flexid, skinid);
        }

        void keyboard_cb(GLFWwindow* w, int key, int /*scancode*/, int act, int /*mods*/) {
            if (act != GLFW_PRESS) {
                return;
            }
            if (key == GLFW_KEY_ESCAPE) {
                glfwSetWindowShouldClose(w, GLFW_TRUE);
            }
            // BACKSPACE (MuJoCo simulate convention): reset the sim to its startup state.
            if (key == GLFW_KEY_BACKSPACE && g_request_reset) {
                end_perturb();  // a reset mid-drag must not leave a perturb force pinned
                g_request_reset();
            }
            // F: shove the robot over (deterministic fall for testing FallRecovery/GetUp —
            // mouse-drag perturbs are usually within what the push-randomised policy rides out).
            if (key == GLFW_KEY_F && hands_on() && g_model != nullptr && g_data != nullptr && g_mutex != nullptr) {
                std::lock_guard<std::mutex> lock(*g_mutex);
                if (g_model->njnt > 0 && g_model->jnt_type[0] == mjJNT_FREE) {
                    const int dof = g_model->jnt_dofadr[0];
                    g_data->qvel[dof + 0] += 1.5;  // linear kick, world x
                    g_data->qvel[dof + 4] += 6.0;  // pitch rate — guarantees a topple
                }
            }
            if (key == GLFW_KEY_SPACE && g_ui != nullptr) {
                g_ui->toggle_pause(g_ui_state);
            }
        }

        void mouse_button_cb(GLFWwindow* w, int button, int act, int mods) {
            button_left   = glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
            button_middle = glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
            button_right  = glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
            glfwGetCursorPos(w, &last_x, &last_y);

            if (act == GLFW_RELEASE) {
                end_perturb();  // wherever the cursor ended up, a release always drops the body
            }

            // A click on a control panel must not also orbit the camera or grab a robot.
            if (ui_wants_mouse()) {
                button_left = button_middle = button_right = false;
                return;
            }

            if (g_model == nullptr || g_data == nullptr || g_mutex == nullptr || act == GLFW_RELEASE) {
                return;
            }

            // Testing Room, "Penalise" armed: the robot under this click is penalised or let back on.
            if (g_ui_state.penalise_armed && button == GLFW_MOUSE_BUTTON_LEFT) {
                button_left = false;
                std::lock_guard<std::mutex> lock(*g_mutex);
                mjtNum selpnt[3];
                int skinid     = -1;
                const int root = free_root(select_at(w, selpnt, &skinid));
                const char* name = root >= 0 ? mj_id2name(g_model, mjOBJ_BODY, root) : nullptr;
                for (const auto& robot : g_ui_state.robots) {
                    if (name != nullptr && robot.body == name) {
                        g_penalise_pick           = robot;
                        g_ui_state.penalise_armed = false;
                    }
                }
                return;
            }

            // Testing Room ball placement owns left clicks on the field (ViewerUI::render_placement).
            if (g_ui_state.placing && button == GLFW_MOUSE_BUTTON_LEFT) {
                button_left = false;
                return;
            }

            // A refereed game: the mouse only moves the camera.
            if (!hands_on()) {
                return;
            }

            const bool mod_ctrl = (mods & GLFW_MOD_CONTROL) != 0;

            std::lock_guard<std::mutex> lock(*g_mutex);

            static double last_click_time = 0.0;
            static int last_click_button  = -1;

            if (button == GLFW_MOUSE_BUTTON_LEFT) {
                if (mod_ctrl) {
                    // Ctrl + drag: pick up whatever is under the cursor and carry it over the field.
                    mjtNum selpnt[3];
                    int skinid     = -1;
                    const int root = free_root(select_at(w, selpnt, &skinid));
                    if (root >= 0) {
                        g_gizmo.body    = root;
                        pert.select     = root;
                        pert.skinselect = -1;
                        mju_zero3(pert.localpos);
                        gizmo_begin(Gizmo::CARRY);
                        g_gizmo.plane_z = selpnt[2];
                        g_gizmo.grab[0] = g_gizmo.pos[0] - selpnt[0];
                        g_gizmo.grab[1] = g_gizmo.pos[1] - selpnt[1];
                    }
                    last_click_button = -1;
                    return;
                }
                const Gizmo::Handle handle = gizmo_hit(last_x, last_y);
                if (handle != Gizmo::NONE) {
                    gizmo_begin(handle);
                    double ground[2];
                    if (handle == Gizmo::RING) {
                        if (ground_point(last_x, last_y, g_gizmo.pivot[2], ground)) {
                            g_gizmo.last_angle = std::atan2(ground[1] - g_gizmo.pivot[1], ground[0] - g_gizmo.pivot[0]);
                        }
                        else {
                            g_gizmo.drag = Gizmo::NONE;
                        }
                    }
                    last_click_button = -1;
                    return;
                }
            }

            // Ctrl + right-drag on the selected body: pull it with a force (push/fall testing).
            if (mod_ctrl && pert.select > 0 && button_right) {
                mjv_initPerturb(g_model, g_data, &scn, &pert);
                pert.active = mjPERT_TRANSLATE;
            }

            // Double-click (any button, <300 ms apart): select the body under the cursor.
            const double now        = glfwGetTime();
            const bool double_click = (button == last_click_button) && (now - last_click_time < 0.3);
            last_click_time         = now;
            last_click_button       = button;

            if (double_click) {
                mjtNum selpnt[3];
                int skinid     = -1;
                const int body = select_at(w, selpnt, &skinid);
                if (body > 0) {
                    pert.select     = body;
                    pert.skinselect = skinid;
                    mjtNum tmp[3];
                    mju_sub3(tmp, selpnt, g_data->xpos + 3 * body);
                    mju_mulMatTVec3(pert.localpos, g_data->xmat + 9 * body, tmp);
                    g_gizmo.body = free_root(body);
                }
                else {
                    pert.select     = 0;
                    pert.skinselect = -1;
                    g_gizmo.body    = -1;
                    end_perturb();
                }
            }
        }

        void mouse_move_cb(GLFWwindow* w, double xpos, double ypos) {
            const double dx = xpos - last_x;
            const double dy = ypos - last_y;
            last_x           = xpos;
            last_y           = ypos;

            if (!button_left && !button_middle && !button_right) {
                return;
            }

            // Dragging a move handle: update the pose the viewer tick pins the body to.
            if (g_gizmo.drag != Gizmo::NONE && g_gizmo.body >= 0) {
                double ground[2];
                switch (g_gizmo.drag) {
                    case Gizmo::AXIS_X:
                    case Gizmo::AXIS_Y:
                    case Gizmo::AXIS_Z: {
                        // Cursor travel along the arrow as drawn -> metres along that axis.
                        const int axis   = g_gizmo.drag;
                        const double sx  = g_gizmo.tip[axis].x - g_gizmo.centre.x;
                        const double sy  = g_gizmo.tip[axis].y - g_gizmo.centre.y;
                        const double len2 = sx * sx + sy * sy;
                        if (g_gizmo.tip_ok[axis] && len2 > 16.0) {
                            g_gizmo.pos[axis] += (dx * sx + dy * sy) / len2 * g_gizmo.length;
                        }
                        break;
                    }
                    case Gizmo::RING:
                        if (ground_point(xpos, ypos, g_gizmo.pivot[2], ground)) {
                            const double angle = std::atan2(ground[1] - g_gizmo.pivot[1], ground[0] - g_gizmo.pivot[0]);
                            const double delta = std::remainder(angle - g_gizmo.last_angle, 2.0 * mjPI);
                            g_gizmo.last_angle = angle;
                            const double turn[4] = {std::cos(delta / 2.0), 0.0, 0.0, std::sin(delta / 2.0)};
                            double turned[4];
                            mju_mulQuat(turned, turn, g_gizmo.quat);
                            mju_normalize4(turned);
                            mju_copy4(g_gizmo.quat, turned);
                            // Swing the body frame about the pivot so the body turns on the spot.
                            const double rx = g_gizmo.pos[0] - g_gizmo.pivot[0];
                            const double ry = g_gizmo.pos[1] - g_gizmo.pivot[1];
                            g_gizmo.pos[0]  = g_gizmo.pivot[0] + std::cos(delta) * rx - std::sin(delta) * ry;
                            g_gizmo.pos[1]  = g_gizmo.pivot[1] + std::sin(delta) * rx + std::cos(delta) * ry;
                        }
                        break;
                    case Gizmo::CARRY:
                        if (ground_point(xpos, ypos, g_gizmo.plane_z, ground)) {
                            g_gizmo.pos[0] = ground[0] + g_gizmo.grab[0];
                            g_gizmo.pos[1] = ground[1] + g_gizmo.grab[1];
                        }
                        break;
                    case Gizmo::NONE: break;
                }
                // Stay on the modelled floor and above it.
                g_gizmo.pos[0] = std::clamp(g_gizmo.pos[0], -8.0, 8.0);
                g_gizmo.pos[1] = std::clamp(g_gizmo.pos[1], -6.0, 6.0);
                g_gizmo.pos[2] = std::clamp(g_gizmo.pos[2], g_gizmo.floor_z, g_gizmo.floor_z + 2.5);
                return;
            }

            if (g_model == nullptr || ui_wants_mouse()) {
                return;
            }

            int width  = 0;
            int height = 0;
            glfwGetWindowSize(w, &width, &height);
            if (height == 0) {
                return;
            }

            const bool mod_shift = glfwGetKey(w, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS
                                    || glfwGetKey(w, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;
            const bool mod_ctrl = glfwGetKey(w, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS
                                   || glfwGetKey(w, GLFW_KEY_RIGHT_CONTROL) == GLFW_PRESS;

            if (pert.active != 0 && g_data != nullptr && g_mutex != nullptr) {
                std::lock_guard<std::mutex> lock(*g_mutex);
                mjv_movePerturb(g_model, g_data, mjMOUSE_MOVE_V, dx / height, dy / height, &scn, &pert);
                return;
            }
            if (mod_ctrl) {
                return;  // Ctrl is for moving things, never the camera
            }

            mjtMouse action = mjMOUSE_ZOOM;
            if (button_right) {
                action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
            }
            else if (button_left) {
                action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
            }
            mjv_moveCamera(g_model, action, dx / height, dy / height, &scn, &cam);
        }

        void scroll_cb(GLFWwindow* /*w*/, double /*xoffset*/, double yoffset) {
            if (g_model == nullptr || ui_wants_mouse()) {
                return;
            }
            mjv_moveCamera(g_model, mjMOUSE_ZOOM, 0.0, -0.05 * yoffset, &scn, &cam);
        }

    }  // namespace

    Viewer::Viewer(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        // Bridge for the plain-C GLFW key callback (Backspace = sim reset). emit() is
        // thread-safe, and the callback only ever runs on the MainThread glfwPollEvents pump.
        g_request_reset = [this] { emit(std::make_unique<message::SimResetRequest>()); };

        g_send_match_command = [this](const message::MatchCommand& command) {
            emit(std::make_unique<message::MatchCommand>(command));
        };

        on<Startup, MainThread>().then([this] {
            if (cli().headless) {
                log<NUClear::LogLevel::INFO>("Viewer disabled (--headless)");
                return;
            }

            if (glfwInit() == GLFW_FALSE) {
                log<NUClear::LogLevel::ERROR>("glfwInit() failed — continuing without a viewer window");
                return;
            }

            window = glfwCreateWindow(1200, 900, "K1 MuJoCo Sim", nullptr, nullptr);
            if (window == nullptr) {
                log<NUClear::LogLevel::ERROR>("glfwCreateWindow() failed — continuing without a viewer window");
                glfwTerminate();
                return;
            }

            glfwMakeContextCurrent(window);
            // No vsync: frame pacing is already owned by the Every<16ms> reaction
            // below, and blocking glfwSwapBuffers() on a vblank the driver never
            // delivers (headless/virtual displays, some remote X servers) would
            // stall the NUClear MainThread pool — the same thread glfwPollEvents
            // needs, so the whole viewer (and its window-close handling) hangs.
            glfwSwapInterval(0);

            mjv_defaultCamera(&cam);
            mjv_defaultOption(&opt);
            mjv_defaultPerturb(&pert);
            mjr_defaultContext(&con);
            // mjv_makeScene/mjr_makeContext need an mjModel, so they're deferred
            // to the Trigger<SimHandles> reaction below (also MainThread — GL
            // context creation is thread-bound to whichever thread called
            // glfwMakeContextCurrent, i.e. this one).

            glfwSetMouseButtonCallback(window, mouse_button_cb);
            glfwSetCursorPosCallback(window, mouse_move_cb);
            glfwSetScrollCallback(window, scroll_cb);
            glfwSetKeyCallback(window, keyboard_cb);

            log<NUClear::LogLevel::INFO>("Viewer window created (1200x900) — waiting for the simulation model");
        });

        on<Trigger<message::SimHandles>, MainThread>().then([this](const message::SimHandles& handles) {
            if (cli().headless) {
                return;
            }

            g_model        = handles.model;
            g_data         = handles.data;
            g_mutex        = handles.mutex;
            g_measured_rtf = handles.measured_rtf;
            g_paused       = handles.paused;

            if (window != nullptr && !scene_ready && g_model != nullptr) {
                mjv_makeScene(g_model, &scn, 2000);
                mjr_makeContext(g_model, &con, mjFONTSCALE_150);
                // mjv_defaultCamera (Startup) doesn't know the model's scale; now
                // that it's loaded, frame the free camera on it (lookat/distance
                // from the compiled model extent) so the scene isn't clipped or
                // too distant to see on first frame.
                mjv_defaultFreeCamera(g_model, &cam);
                if (g_ui == nullptr) {
                    g_ui = std::make_unique<viewer::ViewerUI>();
                    g_ui->init(window);
                    g_ui_state.draw_world = gizmo_draw;
                    g_ui_state.project    = [](const double world[3], ImVec2& out) { return project(world, out); };
                    g_ui_state.ground_point = [](ImVec2 screen, double out[2]) {
                        return ground_point(screen.x, screen.y, 0.0, out);
                    };
                    load_session(g_ui_state);
                    g_ui_state.speed      = static_cast<float>(
                        cli().rtf >= 0.0 ? cli().rtf : config::load("simulation.yaml")["real_time_factor"].as<double>(1.0));
                }
                scene_ready = true;
                log<NUClear::LogLevel::INFO>("Viewer scene ready (",
                                             g_model->nbody,
                                             "bodies,",
                                             g_model->ngeom,
                                             "geoms)");
            }
        });

        on<Trigger<message::VideoRecordRequest>, MainThread>().then([this](const message::VideoRecordRequest& request) {
            if (!scene_ready || video) {
                log<NUClear::LogLevel::ERROR>("Video requires a ready viewer and no existing recording");
                return;
            }
            try {
                video_roster = config::robot_roster();
                video        = std::make_unique<VideoRecorder>(request.path);
                video_rgb.resize(VideoRecorder::width * VideoRecorder::height * 3);
                mjr_resizeOffscreen(VideoRecorder::width, VideoRecorder::height, &con);
                cam.lookat[0] = cam.lookat[1] = cam.lookat[2] = 0.;
                cam.distance                                  = 12.5;
                cam.azimuth                                   = 90.;
                cam.elevation                                 = -80.;
                video_frames                                  = 0;
                video_limit                                   = int(std::ceil(request.duration * VideoRecorder::fps));
                video_start                                   = NUClear::clock::now();
                log<NUClear::LogLevel::INFO>("VIDEO started",
                                             request.path,
                                             "frames",
                                             video_limit,
                                             "encoder_pid",
                                             video->pid());
            }
            catch (const std::exception& error) {
                video.reset();
                log<NUClear::LogLevel::ERROR>("VIDEO failed", error.what());
            }
        });

        // Overlay-only telemetry — never touches mjData, so no MainThread/mutex needed.
        // Not MainThread, for the same reason as SimStateUpdate below: the render loop can starve it.
        on<Trigger<message::MatchStatus>>().then([](const message::MatchStatus& status) {
            match_panel.set(status);  // robot labels
            std::lock_guard<std::mutex> lock(g_match_mutex);
            g_match_status = status;
        });

        // The Testing Room's built-in GameController (module::Referee, or an external one through
        // module::Supervisor) reports the game here; show it on the same scoreboard.
        on<Trigger<message::GameState>>().then([](const message::GameState& game) {
            std::lock_guard<std::mutex> lock(g_match_mutex);
            g_match_status.has_game       = true;
            g_match_status.home_score     = game.score_left;  // the home team defends the left goal
            g_match_status.away_score     = game.score_right;
            g_match_status.secs_remaining = game.secs_remaining;
            g_match_status.state          = game.state;
            g_match_status.players        = static_cast<int>(game.robots.size());
            g_robots.clear();
            for (const auto& robot : game.robots) {
                g_robots.push_back({robot.side, robot.player, !robot.penalty.empty(), robot.body});
            }
        });
        on<Trigger<message::RefereeStatus>>().then([](const message::RefereeStatus& status) {
            std::lock_guard<std::mutex> lock(g_match_mutex);
            g_legal = status.legal;
        });

        on<Trigger<message::SimStateUpdate>>().then([](const message::SimStateUpdate& state) {
            g_sim_time.store(state.sim_time, std::memory_order_relaxed);
            g_mode.store(state.mode, std::memory_order_relaxed);
        });

        on<Every<16, std::chrono::milliseconds>, MainThread>().then([this] {
            if (cli().headless || window == nullptr) {
                return;
            }

            if (glfwWindowShouldClose(window) != 0) {
                powerplant.shutdown();
                return;
            }

            glfwPollEvents();
            glfwGetWindowSize(window, &g_win_w, &g_win_h);

            if (scene_ready && g_data != nullptr && g_mutex != nullptr) {
                // Lock only around the mjData touch points: applying the perturb
                // force/clearing it, and mjv_updateScene's read of mjData. Both
                // are sub-millisecond; the physics thread never blocks on us for
                // longer than that.
                std::lock_guard<std::mutex> lock(*g_mutex);
                if (g_gizmo.body >= 0) {
                    if (g_gizmo.drag != Gizmo::NONE) {
                        gizmo_hold();
                    }
                    // Centre of mass, not the body frame, so the handles sit on the body.
                    mju_copy3(g_gizmo.centre_world, g_data->xipos + 3 * g_gizmo.body);
                }
                if (pert.select > 0) {
                    if (pert.active != 0) {
                        mjv_applyPerturbForce(g_model, g_data, &pert);
                    }
                    else {
                        mju_zero(g_data->xfrc_applied + 6 * pert.select, 6);
                    }
                }
                mjv_updateScene(g_model, g_data, &opt, &pert, &cam, mjCAT_ALL, &scn);
                if (!video) {
                    match_panel.add_labels(g_model, g_data, &scn);
                }
                if (video) {
                    for (const auto& robot : video_roster) {
                        const int body = mj_name2id(g_model, mjOBJ_BODY, robot.body().c_str());
                        if (body < 0 || scn.ngeom >= scn.maxgeom)
                            continue;
                        const mjtNum size[3] = {0., 0., 0.};
                        const mjtNum pos[3]  = {g_data->xpos[3 * body],
                                                g_data->xpos[3 * body + 1],
                                                g_data->xpos[3 * body + 2] + .5};
                        const mjtNum mat[9]  = {1., 0., 0., 0., 1., 0., 0., 0., 1.};
                        const float blue[4] = {.2f, .55f, 1.f, 1.f}, red[4] = {1.f, .3f, .25f, 1.f};
                        auto& label = scn.geoms[scn.ngeom++];
                        mjv_initGeom(&label,
                                     mjGEOM_LABEL,
                                     size,
                                     pos,
                                     mat,
                                     robot.team_id == video_roster.front().team_id ? blue : red);
                        std::snprintf(label.label, sizeof(label.label), "%d/%d", robot.team_id, robot.player_id);
                    }
                }
            }

            int width  = 0;
            int height = 0;
            glfwGetFramebufferSize(window, &width, &height);
            const mjrRect viewport{0, 0, width, height};

            if (scene_ready) {
                mjr_render(viewport, &scn, &con);

                if (g_ui != nullptr) {
                    viewer::UIState& ui = g_ui_state;
                    ui.sim_time     = g_sim_time.load(std::memory_order_relaxed);
                    ui.measured_rtf = g_measured_rtf != nullptr ? g_measured_rtf->load(std::memory_order_relaxed) : 0.0;
                    ui.robot_mode   = mode_name(g_mode.load(std::memory_order_relaxed));
                    ui.paused       = g_paused != nullptr && g_paused->load(std::memory_order_relaxed);
                    if (const int ball = mj_name2id(g_model, mjOBJ_GEOM, "ball"); ball >= 0 && g_mutex != nullptr) {
                        std::lock_guard<std::mutex> lock(*g_mutex);
                        ui.ball_now = {g_data->geom_xpos[3 * ball], g_data->geom_xpos[3 * ball + 1]};
                    }
                    {
                        std::lock_guard<std::mutex> lock(g_match_mutex);
                        ui.match = g_match_status;
                        ui.legal  = g_legal;
                        ui.robots = g_robots;
                    }
                    const bool hand_referee = ui.launcher && ui.running.mode == viewer::MODE_TESTING;
                    const viewer::UIActions actions = g_ui->render(ui);
                    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

                    if (actions.restart) {
                        write_restart_request(ui);
                        powerplant.shutdown();
                        return;
                    }
                    auto referee = [this](const std::string& action,
                                          const std::string& side,
                                          const std::string& arg,
                                          int player = 0) {
                        auto command    = std::make_unique<message::RefereeCommand>();
                        command->action = action;
                        command->side   = side;
                        command->player = player;
                        command->arg    = arg;
                        emit(command);
                    };
                    if (g_penalise_pick) {
                        const auto robot = *g_penalise_pick;
                        g_penalise_pick.reset();
                        referee(robot.penalised ? "unpenalize" : "penalize", robot.side == 0 ? "home" : "away",
                                "request_for_pick_up", robot.player);
                    }
                    for (const auto& command : actions.referee) {
                        referee(command.action, command.side, command.arg, command.player);
                    }
                    if (actions.call_set_play) {
                        const int team = actions.call_set_play->first;
                        const int play = actions.call_set_play->second;
                        if (hand_referee) {
                            const char* name = play == viewer::SP_CORNER_KICK    ? "corner_kick"
                                               : play == viewer::SP_GOAL_KICK    ? "goal_kick"
                                               : play == viewer::SP_THROW_IN     ? "throw_in"
                                               : play == viewer::SP_PENALTY_KICK ? "penalty_kick"
                                                                                 : "direct_free_kick";
                            referee("setplay", team == 0 ? "home" : "away", name);
                        }
                        else {
                            message::MatchCommand command{message::MatchCommand::Action::SET_PLAY};
                            command.side     = team;
                            command.set_play = play;
                            emit(std::make_unique<message::MatchCommand>(command));
                        }
                    }
                    if (actions.release_set_play) {
                        if (hand_referee) {
                            referee("play", "", "");
                        }
                        else {
                            emit(std::make_unique<message::MatchCommand>(
                                message::MatchCommand{message::MatchCommand::Action::RELEASE}));
                        }
                    }
                    if (actions.match_ready) {
                        emit(std::make_unique<message::MatchCommand>(
                            message::MatchCommand{message::MatchCommand::Action::READY}));
                    }
                    if (actions.match_set) {
                        emit(std::make_unique<message::MatchCommand>(
                            message::MatchCommand{message::MatchCommand::Action::SET}));
                    }
                    if (actions.place_ball && g_mutex != nullptr) {
                        std::lock_guard<std::mutex> lock(*g_mutex);
                        place_ball((*actions.place_ball)[0], (*actions.place_ball)[1]);
                    }
                    if (actions.reset_sim) {
                        end_perturb();
                        pert.select  = 0;
                        g_gizmo.body = -1;
                        emit(std::make_unique<message::SimResetRequest>());
                    }
                    if (actions.pause) {
                        emit(std::make_unique<message::SimPauseRequest>(message::SimPauseRequest{*actions.pause}));
                    }
                    if (actions.speed) {
                        emit(std::make_unique<message::SimSpeedRequest>(message::SimSpeedRequest{*actions.speed}));
                    }
                    if (actions.match_primary) {
                        emit(std::make_unique<message::MatchCommand>(
                            message::MatchCommand{message::MatchCommand::Action::PRIMARY}));
                    }
                    if (actions.match_new_game) {
                        emit(std::make_unique<message::MatchCommand>(
                            message::MatchCommand{message::MatchCommand::Action::RESET}));
                    }
                }
            }

            if (scene_ready && video) {
                const int due = std::min(
                    video_limit,
                    int(std::chrono::duration<double>(NUClear::clock::now() - video_start).count() * VideoRecorder::fps)
                        + 1);
                if (due > video_frames) {
                    mjr_setBuffer(mjFB_OFFSCREEN, &con);
                    const mjrRect recording{0, 0, VideoRecorder::width, VideoRecorder::height};
                    mjr_render(recording, &scn, &con);
                    char caption[128];
                    std::snprintf(caption,
                                  sizeof(caption),
                                  "NUSim native 3v3 | six independent players\nsim time %.1f s | RTF %.2f",
                                  g_sim_time.load(),
                                  g_measured_rtf ? g_measured_rtf->load() : 0.);
                    mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, recording, caption, nullptr, &con);
                    mjr_readPixels(video_rgb.data(), nullptr, recording, &con);
                    mjr_setBuffer(mjFB_WINDOW, &con);
                    try {
                        while (video_frames < due) {
                            video->write(video_rgb);
                            ++video_frames;
                        }
                        if (video_frames == video_limit) {
                            const bool ok = video->finish();
                            video.reset();
                            log<NUClear::LogLevel::INFO>("VIDEO finished", video_frames, "frames, encoder success", ok);
                        }
                    }
                    catch (const std::exception& error) {
                        video.reset();
                        log<NUClear::LogLevel::ERROR>("VIDEO failed", error.what());
                    }
                }
            }

            glfwSwapBuffers(window);
        });

        on<Shutdown, MainThread>().then([this] {
            if (!cli().headless) {
                if (video) {
                    video->finish();
                    video.reset();
                }
                if (g_ui != nullptr) {
                    g_ui->shutdown();
                    g_ui = nullptr;
                }
                if (scene_ready) {
                    mjv_freeScene(&scn);
                    mjr_freeContext(&con);
                    scene_ready = false;
                }
                if (window != nullptr) {
                    glfwDestroyWindow(window);
                    window = nullptr;
                }
                glfwTerminate();
            }
            log<NUClear::LogLevel::INFO>("Viewer shutting down");
        });
    }

}  // namespace k1sim::module
