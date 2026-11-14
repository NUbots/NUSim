#ifndef K1SIM_MODULE_VIEWER_VIEWERUI_HPP
#define K1SIM_MODULE_VIEWER_VIEWERUI_HPP

#include <GLFW/glfw3.h>
#include <functional>
#include <imgui.h>
#include <array>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "shared/message/MatchMessages.hpp"

namespace k1sim::module::viewer {

// The viewer's control panels (Dear ImGui): the match scoreboard and buttons when
// module::MatchReferee is running, and simulation playback controls for every role.

// GameController SetPlay values (game_controller_api's RoboCup::SetPlay).
enum SetPlay : int { SP_NONE = 0, SP_FREE_KICK = 2, SP_PENALTY_KICK = 4, SP_THROW_IN = 5, SP_GOAL_KICK = 6, SP_CORNER_KICK = 7 };

enum Mode : int { MODE_GAME = 0, MODE_TESTING = 1, MODE_FREEPLAY = 2 };

// What tools/match.py launched, and what it can launch instead. Changing it needs a restart,
// because the robot count is baked into the model and each robot's code runs in its own container.
struct Setup {
    int mode          = MODE_GAME;
    int game          = 1;  // robots a side
    int home_role     = 0;  // indices into roles
    int away_role     = 0;
    bool ground_truth = false;  // robots take pose + ball from the sim instead of vision + localisation

    bool operator==(const Setup&) const = default;
};

struct UIState {
    // Telemetry, written by the Viewer every frame before render().
    double sim_time        = 0.0;
    double measured_rtf    = 0.0;
    const char* robot_mode = "?";
    bool paused            = false;
    float speed            = 1.0f;

    // Latest report from module::MatchReferee; has_game stays false in roles without it.
    message::MatchStatus match;

    // Launcher setup; `launcher` is false when the sim was started without tools/match.py.
    bool launcher = false;
    std::vector<std::string> roles;
    Setup running;
    Setup wanted;

    // Testing Room: a set play called by hand. `placing` = waiting for a click on the field.
    int set_play      = SP_NONE;
    int set_play_team = 0;
    bool placing      = false;

    // Field dimensions (config/match.yaml), metres.
    double field_length          = 14.0;
    double field_width           = 9.0;
    double goal_area_length      = 1.0;
    double goal_area_width       = 4.0;
    double penalty_mark_distance = 2.0;

    // Camera helpers supplied by the Viewer. project: world point -> window coordinates (false if
    // off screen). ground_point: window coordinates -> point on the floor.
    std::function<bool(const double world[3], ImVec2& out)> project;
    std::function<bool(ImVec2 screen, double out[2])> ground_point;

    // World-anchored overlays the Viewer owns (the move/rotate handles).
    std::function<void(ImDrawList*)> draw_world;
};

// What the user asked for this frame; the Viewer turns these into NUClear messages.
struct UIActions {
    std::optional<bool> pause;
    std::optional<double> speed;
    bool reset_sim      = false;
    bool match_primary  = false;  // the referee's main button: Start / Pause / Resume
    bool match_new_game = false;
    bool restart        = false;                       // relaunch with UIState::wanted
    std::optional<std::pair<int, int>> call_set_play;  // (team, SetPlay)
    bool release_set_play = false;                     // ball free
    std::optional<std::array<double, 2>> place_ball;   // x, y
};

class ViewerUI {
public:
    void init(GLFWwindow* window);
    UIActions render(UIState& state);
    void shutdown();

    void toggle_pause(const UIState& state) { pending_.pause = !state.paused; }

private:
    void render_scoreboard(const UIState& state);
    void render_side_panel(UIState& state);
    void render_setup_panel(UIState& state);
    void render_placement(UIState& state);
    void render_simulation_controls(UIState& state);
    void render_playbar(UIState& state);

    UIActions pending_;
    ImFont* font_small_ = nullptr;
    ImFont* font_body_  = nullptr;
    ImFont* font_title_ = nullptr;
    ImFont* font_clock_ = nullptr;
    ImFont* font_score_ = nullptr;
    float baked_scale_  = 2.0f;
    ImGuiStyle base_style_{};
};

}  // namespace k1sim::module::viewer

#endif  // K1SIM_MODULE_VIEWER_VIEWERUI_HPP
