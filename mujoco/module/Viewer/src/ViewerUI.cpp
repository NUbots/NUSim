#include "module/Viewer/src/ViewerUI.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

namespace k1sim::module::viewer {

namespace {

// Everything is laid out in "dp": pixels at a 1200 x 900 window. g_scale follows the
// window size every frame, so the UI keeps its proportions when the window is resized.
float g_scale = 1.0f;
float dp(float v) {
    return std::round(v * g_scale);
}

const ImVec4 C_TEXT{0.93f, 0.95f, 0.98f, 1.0f};
const ImVec4 C_MUTED{0.60f, 0.66f, 0.74f, 1.0f};
const ImVec4 C_ACCENT{0.20f, 0.56f, 0.98f, 1.0f};
const ImVec4 C_HOME{0.30f, 0.74f, 1.00f, 1.0f};
const ImVec4 C_AWAY{1.00f, 0.42f, 0.64f, 1.0f};
const ImVec4 C_GOOD{0.27f, 0.82f, 0.50f, 1.0f};
const ImVec4 C_WARN{1.00f, 0.74f, 0.28f, 1.0f};
const ImVec4 C_BAD{0.98f, 0.42f, 0.40f, 1.0f};
const ImVec4 C_PANEL{0.065f, 0.085f, 0.115f, 0.94f};

const char* const TEAM_TAG[2]  = {"HOME", "AWAY"};


const ImVec4& team_colour(int team) {
    return team == 0 ? C_HOME : C_AWAY;
}

ImU32 col(ImVec4 c, float alpha = 1.0f) {
    c.w *= alpha;
    return ImGui::GetColorU32(c);  // also applies style.Alpha (disabled items)
}

float font_px(const ImFont* font) {
    return font->FontSize * ImGui::GetIO().FontGlobalScale;
}

ImVec2 text_size(const ImFont* font, const char* text) {
    return font->CalcTextSizeA(font_px(font), FLT_MAX, 0.0f, text);
}

// anchor_x: 0 = left-aligned at pos, 0.5 = centred, 1 = right-aligned.
void text_at(ImDrawList* dl, const ImFont* font, ImVec2 pos, float anchor_x, ImU32 colour, const char* text) {
    const ImVec2 size = text_size(font, text);
    dl->AddText(font, font_px(font), {std::round(pos.x - size.x * anchor_x), pos.y}, colour, text);
}

std::string clock_text(double seconds, bool tenths) {
    seconds = std::max(0.0, seconds);
    char buf[32];
    const int whole = static_cast<int>(seconds);
    if (tenths) {
        std::snprintf(buf, sizeof(buf), "%02d:%02d.%d", whole / 60, whole % 60,
                      static_cast<int>((seconds - whole) * 10.0));
    }
    else {
        std::snprintf(buf, sizeof(buf), "%02d:%02d", whole / 60, whole % 60);
    }
    return buf;
}

constexpr ImGuiWindowFlags PANEL_FLAGS = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove
                                         | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoNav;

// A fixed panel anchored to the window: `pivot` picks which corner sits at `pos`.
// Height 0 = fit the content. Always pair with ImGui::End().
void begin_panel(const char* id, ImVec2 pos, ImVec2 pivot, ImVec2 size) {
    ImGui::SetNextWindowPos(pos, ImGuiCond_Always, pivot);
    ImGui::SetNextWindowSize(size);
    ImGui::Begin(id, nullptr, PANEL_FLAGS);
}

void heading(ImFont* font, const char* text) {
    ImGui::PushFont(font);
    ImGui::TextUnformatted(text);
    ImGui::PopFont();
    ImGui::Dummy({0.0f, dp(2)});
}

void section(ImFont* small, const char* text) {
    ImGui::Dummy({0.0f, dp(6)});
    ImGui::PushFont(small);
    ImGui::TextColored(C_MUTED, "%s", text);
    ImGui::PopFont();
}

void note(ImFont* small, const ImVec4& colour, const char* text) {
    ImGui::PushFont(small);
    ImGui::PushStyleColor(ImGuiCol_Text, colour);
    ImGui::TextWrapped("%s", text);
    ImGui::PopStyleColor();
    ImGui::PopFont();
}

enum class Kind { Secondary, Primary, Selected };

void push_button_style(Kind kind) {
    switch (kind) {
        case Kind::Primary:
            ImGui::PushStyleColor(ImGuiCol_Button, C_ACCENT);
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4{0.32f, 0.64f, 1.0f, 1.0f});
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4{0.14f, 0.46f, 0.86f, 1.0f});
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4{1.0f, 1.0f, 1.0f, 1.0f});
            ImGui::PushStyleColor(ImGuiCol_Border, ImVec4{0, 0, 0, 0});
            break;
        case Kind::Selected:
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4{C_ACCENT.x, C_ACCENT.y, C_ACCENT.z, 0.24f});
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4{C_ACCENT.x, C_ACCENT.y, C_ACCENT.z, 0.32f});
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4{C_ACCENT.x, C_ACCENT.y, C_ACCENT.z, 0.40f});
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4{0.72f, 0.86f, 1.0f, 1.0f});
            ImGui::PushStyleColor(ImGuiCol_Border, C_ACCENT);
            break;
        case Kind::Secondary:
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4{1, 1, 1, 0.07f});
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4{1, 1, 1, 0.13f});
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4{1, 1, 1, 0.20f});
            ImGui::PushStyleColor(ImGuiCol_Text, C_TEXT);
            ImGui::PushStyleColor(ImGuiCol_Border, ImVec4{1, 1, 1, 0.10f});
            break;
    }
    ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, std::max(1.0f, dp(1)));
}

void pop_button_style() {
    ImGui::PopStyleVar();
    ImGui::PopStyleColor(5);
}

// width 0 = fill the rest of the row.
bool button(const char* label, float width, float height, Kind kind = Kind::Secondary) {
    push_button_style(kind);
    const bool pressed = ImGui::Button(label, {width > 0.0f ? width : ImGui::GetContentRegionAvail().x, height});
    pop_button_style();
    return pressed;
}

enum class Icon { Play, Pause };

bool icon_button(const char* id, Icon icon, float size, Kind kind = Kind::Secondary) {
    const ImVec2 p = ImGui::GetCursorScreenPos();
    const bool pressed = ImGui::InvisibleButton(id, {size, size});
    const bool hovered = ImGui::IsItemHovered();
    const bool held    = ImGui::IsItemActive();

    ImDrawList* dl = ImGui::GetWindowDrawList();
    ImU32 bg;
    if (kind == Kind::Primary) {
        bg = col(held ? ImVec4{0.14f, 0.46f, 0.86f, 1.0f} : hovered ? ImVec4{0.32f, 0.64f, 1.0f, 1.0f} : C_ACCENT);
    }
    else {
        bg = col(ImVec4{1, 1, 1, held ? 0.20f : hovered ? 0.13f : 0.07f});
    }
    dl->AddRectFilled(p, {p.x + size, p.y + size}, bg, size * 0.28f);

    const ImVec2 c{p.x + size * 0.5f, p.y + size * 0.5f};
    const float r    = size * 0.20f;
    const ImU32 ink = col(kind == Kind::Primary ? ImVec4{1, 1, 1, 1} : C_TEXT);
    switch (icon) {
        case Icon::Play:
            dl->AddTriangleFilled({c.x - r * 0.7f, c.y - r}, {c.x - r * 0.7f, c.y + r}, {c.x + r, c.y}, ink);
            break;
        case Icon::Pause:
            dl->AddRectFilled({c.x - r * 0.85f, c.y - r}, {c.x - r * 0.25f, c.y + r}, ink, 1.0f);
            dl->AddRectFilled({c.x + r * 0.25f, c.y - r}, {c.x + r * 0.85f, c.y + r}, ink, 1.0f);
            break;
    }
    return pressed;
}

void stat_row(const char* label, const char* value) {
    const float right = ImGui::GetCursorPosX() + ImGui::GetContentRegionAvail().x;
    ImGui::TextColored(C_MUTED, "%s", label);
    ImGui::SameLine(right - ImGui::CalcTextSize(value).x);
    ImGui::TextUnformatted(value);
}

void chip(ImDrawList* dl, const ImFont* font, ImVec2 centre, const ImVec4& colour, const char* text) {
    const ImVec2 size = text_size(font, text);
    const float w     = size.x + dp(20);
    const float h     = size.y + dp(8);
    const ImVec2 a{std::round(centre.x - w * 0.5f), std::round(centre.y - h * 0.5f)};
    dl->AddRectFilled(a, {a.x + w, a.y + h}, col(colour, 0.18f), h * 0.5f);
    dl->AddText(font, font_px(font), {a.x + dp(10), a.y + dp(4)}, col(colour), text);
}

}  // namespace

void ViewerUI::init(GLFWwindow* window) {
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io    = ImGui::GetIO();
    io.IniFilename = nullptr;  // fixed layout: nothing worth persisting

    // Bake the fonts at the size the window opens at; g_scale / baked_scale_ covers resizes.
    int fb_w = 0;
    int fb_h = 0;
    glfwGetFramebufferSize(window, &fb_w, &fb_h);
    baked_scale_ = std::clamp(std::min(fb_w / 1200.0f, fb_h / 900.0f), 1.0f, 2.5f);

    auto load = [&](float size_dp) -> ImFont* {
        const float px = std::round(size_dp * baked_scale_);
#ifdef K1SIM_UI_FONT
        if (std::FILE* f = std::fopen(K1SIM_UI_FONT, "rb")) {
            std::fclose(f);
            ImFontConfig cfg;
            cfg.OversampleH = 2;
            cfg.OversampleV = 2;
            return io.Fonts->AddFontFromFileTTF(K1SIM_UI_FONT, px, &cfg);
        }
#endif
        ImFontConfig cfg;
        cfg.SizePixels = px;
        return io.Fonts->AddFontDefault(&cfg);
    };
    font_body_  = load(16);  // first font loaded = ImGui's default
    font_small_ = load(13);
    font_title_ = load(20);
    font_clock_ = load(28);
    font_score_ = load(46);

    ImGui::StyleColorsDark();
    ImGuiStyle& st       = ImGui::GetStyle();
    st.WindowPadding     = {16, 14};
    st.FramePadding      = {10, 7};
    st.ItemSpacing       = {8, 8};
    st.ItemInnerSpacing  = {6, 6};
    st.WindowRounding    = 12;
    st.FrameRounding     = 7;
    st.PopupRounding     = 8;
    st.GrabRounding      = 7;
    st.ScrollbarSize     = 10;
    st.WindowBorderSize  = 1;
    st.PopupBorderSize   = 1;
    st.DisabledAlpha     = 0.35f;
    st.Colors[ImGuiCol_Text]           = C_TEXT;
    st.Colors[ImGuiCol_TextDisabled]   = C_MUTED;
    st.Colors[ImGuiCol_WindowBg]       = C_PANEL;
    st.Colors[ImGuiCol_PopupBg]        = ImVec4{0.09f, 0.11f, 0.15f, 0.98f};
    st.Colors[ImGuiCol_Border]         = ImVec4{1, 1, 1, 0.09f};
    st.Colors[ImGuiCol_FrameBg]        = ImVec4{1, 1, 1, 0.07f};
    st.Colors[ImGuiCol_FrameBgHovered] = ImVec4{1, 1, 1, 0.12f};
    st.Colors[ImGuiCol_FrameBgActive]  = ImVec4{1, 1, 1, 0.16f};
    st.Colors[ImGuiCol_Header]         = ImVec4{C_ACCENT.x, C_ACCENT.y, C_ACCENT.z, 0.28f};
    st.Colors[ImGuiCol_HeaderHovered]  = ImVec4{C_ACCENT.x, C_ACCENT.y, C_ACCENT.z, 0.38f};
    st.Colors[ImGuiCol_HeaderActive]   = ImVec4{C_ACCENT.x, C_ACCENT.y, C_ACCENT.z, 0.48f};
    st.Colors[ImGuiCol_Button]         = ImVec4{1, 1, 1, 0.07f};
    st.Colors[ImGuiCol_ButtonHovered]  = ImVec4{1, 1, 1, 0.13f};
    st.Colors[ImGuiCol_ButtonActive]   = ImVec4{1, 1, 1, 0.20f};
    st.Colors[ImGuiCol_Separator]      = ImVec4{1, 1, 1, 0.08f};
    base_style_ = st;

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");
}

void ViewerUI::shutdown() {
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
}

UIActions ViewerUI::render(UIState& state) {
    ImGuiIO& io = ImGui::GetIO();
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();

    g_scale            = std::clamp(std::min(io.DisplaySize.x / 1200.0f, io.DisplaySize.y / 900.0f), 0.6f, 3.0f);
    io.FontGlobalScale = g_scale / baked_scale_;
    ImGui::GetStyle()  = base_style_;
    ImGui::GetStyle().ScaleAllSizes(g_scale);
    ImGui::NewFrame();

    if (state.draw_world) {
        state.draw_world(ImGui::GetBackgroundDrawList());
    }
    if (state.match.has_game) {
        render_scoreboard(state);
    }
    if (state.launcher) {
        render_placement(state);
        render_setup_panel(state);
    }
    render_side_panel(state);
    render_playbar(state);

    ImGui::Render();
    UIActions out = std::move(pending_);
    pending_      = {};
    return out;
}

void ViewerUI::render_scoreboard(const UIState& state) {
    const ImVec2 display = ImGui::GetIO().DisplaySize;
    const float w        = dp(520);
    const float h        = dp(100);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {0, 0});
    begin_panel("##scoreboard", {display.x * 0.5f, dp(14)}, {0.5f, 0.0f}, {w, h});
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImVec2 o = ImGui::GetWindowPos();
    const float cx = o.x + w * 0.5f;
    const int score[2] = {state.match.home_score, state.match.away_score};

    for (int team = 0; team < 2; ++team) {
        const float dir  = team == 0 ? -1.0f : 1.0f;  // which side of the clock
        const float edge = team == 0 ? o.x + dp(24) : o.x + w - dp(24);
        const float ax   = team == 0 ? 0.0f : 1.0f;
        const float bar  = team == 0 ? o.x + dp(8) : o.x + w - dp(12);
        dl->AddRectFilled({bar, o.y + dp(18)}, {bar + dp(4), o.y + h - dp(18)}, col(team_colour(team)), dp(2));
        text_at(dl, font_title_, {edge, o.y + dp(37)}, ax, col(team_colour(team)), TEAM_TAG[team]);

        char text[8];
        std::snprintf(text, sizeof(text), "%d", score[team]);
        text_at(dl, font_score_, {cx + dir * dp(118), o.y + dp(24)}, 0.5f, col(C_TEXT), text);
    }

    // Negative while the referee counts into extra time; show it like the GameController does.
    const int seconds       = state.match.secs_remaining;
    const std::string clock = (seconds < 0 ? "-" : "") + clock_text(std::abs(seconds), false);
    text_at(dl, font_clock_, {cx, o.y + dp(18)}, 0.5f, col(C_TEXT), clock.c_str());
    const std::string& phase   = state.match.state;
    const ImVec4& phase_colour = phase == "PLAYING"                          ? C_GOOD
                                 : phase == "FINISHED"                       ? C_BAD
                                 : phase == "INITIAL" || phase == "PAUSED"   ? C_MUTED
                                                                             : C_WARN;
    chip(dl, font_small_, {cx, o.y + dp(70)}, phase_colour, phase.c_str());
    ImGui::End();
    ImGui::PopStyleVar();
}

void ViewerUI::render_simulation_controls(UIState& state) {
    if (button(state.paused ? "Resume Simulation" : "Pause Simulation", 0.0f, dp(40), state.paused ? Kind::Primary : Kind::Secondary)) {
        pending_.pause = !state.paused;
    }
    if (button("Reset Simulation", 0.0f, dp(34))) {
        pending_.reset_sim = true;
        pending_.pause     = false;
    }

    section(font_small_, "SPEED");
    static constexpr float speeds[4]       = {0.25f, 0.5f, 1.0f, 2.0f};
    static constexpr const char* labels[4] = {"0.25x", "0.5x", "1x", "2x"};
    const float gap = dp(6);
    const float w   = std::floor((ImGui::GetContentRegionAvail().x - 3 * gap) / 4.0f);
    for (int i = 0; i < 4; ++i) {
        if (i > 0) {
            ImGui::SameLine(0.0f, gap);
        }
        if (button(labels[i], w, dp(30), state.speed == speeds[i] ? Kind::Selected : Kind::Secondary)) {
            state.speed    = speeds[i];
            pending_.speed = speeds[i];
        }
    }

    ImGui::Dummy({0.0f, dp(4)});
    char buf[32];
    stat_row("Sim time", clock_text(state.sim_time, true).c_str());
    std::snprintf(buf, sizeof(buf), "%.2fx", state.measured_rtf);
    stat_row("Measured speed", state.paused ? "paused" : buf);
    stat_row("Robot mode", state.robot_mode);
}

// Right-hand panel: the match (when module::MatchReferee is running) and the simulation.
void ViewerUI::render_side_panel(UIState& state) {
    const ImVec2 display = ImGui::GetIO().DisplaySize;
    begin_panel("##side", {display.x - dp(14), dp(14)}, {1.0f, 0.0f}, {dp(290), 0.0f});

    if (state.match.has_game || !state.match.primary_button.empty()) {
        heading(font_title_, "Match");
        if (!state.match.primary_button.empty()) {
            const bool go = state.match.primary_button != "Pause";
            const std::string label = state.match.primary_button + " Match";
            if (button(label.c_str(), 0.0f, dp(40), go ? Kind::Primary : Kind::Secondary)) {
                pending_.match_primary = true;
            }
        }
        if (button("New Game", 0.0f, dp(34))) {
            pending_.match_new_game = true;
        }
        char robots[48];
        std::snprintf(robots, sizeof(robots), "%d / %d", state.match.connected, state.match.players);
        stat_row("Robots online", robots);
        if (!state.match.messages.empty()) {
            section(font_small_, "REFEREE");
            note(font_small_, C_TEXT, state.match.messages.c_str());
        }
        section(font_small_, "SIMULATION");
    }
    else {
        heading(font_title_, "Simulation");
    }
    render_simulation_controls(state);

    section(font_small_, "MOVING THINGS");
    static constexpr const char* help[][2] = {
        {"Double-click", "Show move handles"},
        {"Drag an arrow", "Slide along that axis"},
        {"Drag the ring", "Turn on the spot"},
        {"Ctrl + drag", "Carry it across the field"},
        {"Ctrl + right-drag", "Shove it (applies force)"},
        {"Space", "Pause or resume"},
    };
    ImGui::PushFont(font_small_);
    for (const auto& row : help) {
        ImGui::TextUnformatted(row[0]);
        ImGui::SameLine(dp(128));
        ImGui::TextColored(C_MUTED, "%s", row[1]);
    }
    ImGui::PopFont();
    ImGui::End();
}

// Testing Room ball placement: highlight where the ball may go for the chosen set play
// and put it there on a click.
void ViewerUI::render_placement(UIState& state) {
    if (!state.placing || !state.project || !state.ground_point) {
        return;
    }
    const ImGuiIO& io   = ImGui::GetIO();
    ImDrawList* dl      = ImGui::GetBackgroundDrawList();
    const double attack = state.set_play_team == 0 ? 1.0 : -1.0;  // Home attacks the +x goal
    const ImVec4& tint  = team_colour(state.set_play_team);
    const bool can_click = !io.WantCaptureMouse;
    const float pulse   = 0.5f + 0.5f * std::sin(static_cast<float>(ImGui::GetTime()) * 5.0f);

    auto outline = [&](double x0, double y0, double x1, double y1) {
        const double corners[4][3] = {{x0, y0, 0.01}, {x1, y0, 0.01}, {x1, y1, 0.01}, {x0, y1, 0.01}};
        ImVec2 pts[4];
        for (int i = 0; i < 4; ++i) {
            if (!state.project(corners[i], pts[i])) {
                return;
            }
        }
        dl->AddConvexPolyFilled(pts, 4, col(tint, 0.16f));
        dl->AddPolyline(pts, 4, col(tint, 0.9f), ImDrawFlags_Closed, dp(2.5f));
    };
    auto marker = [&](ImVec2 at, bool hot) {
        const float r = dp(hot ? 17 : 13) + dp(3) * pulse;
        dl->AddCircleFilled(at, r, col(tint, hot ? 0.55f : 0.30f), 32);
        dl->AddCircle(at, r, col(ImVec4{1, 1, 1, 0.95f}), 32, dp(2.5f));
        dl->AddCircleFilled(at, dp(4), col(ImVec4{1, 1, 1, 1}));
    };
    auto place = [&](double x, double y) {
        pending_.place_ball = std::array<double, 2>{x, y};
        state.placing       = false;
    };

    const double L   = state.field_length * 0.5;
    const double W   = state.field_width * 0.5;
    const double box = L - state.goal_area_length;  // front line of a goal area
    const double gw  = state.goal_area_width * 0.5;
    std::vector<std::array<double, 2>> spots;
    switch (state.set_play) {
        case SP_CORNER_KICK: spots = {{attack * L, W}, {attack * L, -W}}; break;
        case SP_GOAL_KICK:  // anywhere along the front of its own goal area; offer three spots
            outline(-attack * L, -gw, -attack * box, gw);
            spots = {{-attack * box, gw}, {-attack * box, 0.0}, {-attack * box, -gw}};
            break;
        case SP_PENALTY_KICK: spots = {{attack * (L - state.penalty_mark_distance), 0.0}}; break;
        default: break;
    }

    if (!spots.empty()) {
        for (const auto& spot : spots) {
            const double world[3] = {spot[0], spot[1], 0.02};
            ImVec2 at;
            if (!state.project(world, at)) {
                continue;
            }
            const float dx = io.MousePos.x - at.x;
            const float dy = io.MousePos.y - at.y;
            const bool hot = can_click && dx * dx + dy * dy < dp(22) * dp(22);
            marker(at, hot);
            if (hot && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
                place(spot[0], spot[1]);
            }
        }
        return;
    }

    // Free kick: anywhere on the field. Throw-in: anywhere along the nearer touchline.
    double ground[2];
    if (!can_click || !state.ground_point(io.MousePos, ground)) {
        return;
    }
    ground[0] = std::clamp(ground[0], -L, L);
    ground[1] = state.set_play == SP_THROW_IN ? (ground[1] >= 0.0 ? W : -W) : std::clamp(ground[1], -W, W);
    const double world[3] = {ground[0], ground[1], 0.02};
    ImVec2 at;
    if (state.project(world, at)) {
        marker(at, true);
        if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
            place(ground[0], ground[1]);
        }
    }
}

// Left-hand panel: what tools/match.py launched (mode, teams, perception), and, in the Testing
// Room, the set plays.
void ViewerUI::render_setup_panel(UIState& state) {
    begin_panel("##setup", {dp(14), dp(14)}, {0.0f, 0.0f}, {dp(300), 0.0f});
    heading(font_title_, "Setup");
    const float gap = dp(6);

    section(font_small_, "MODE");
    static constexpr const char* modes[3] = {"Game", "Testing Room", "Free Play"};
    static constexpr const char* about[3] = {
        "A refereed match: the automatic referee starts play, counts goals and calls restarts.",
        "You are the referee: call a set play, place the ball, then set it free.",
        "No referee. Every robot just runs its code.",
    };
    const float third = std::floor((ImGui::GetContentRegionAvail().x - 2 * gap) / 3.0f);
    for (int m = 0; m < 3; ++m) {
        if (m > 0) {
            ImGui::SameLine(0.0f, gap);
        }
        ImGui::PushFont(font_small_);
        if (button(modes[m], third, dp(32), state.wanted.mode == m ? Kind::Selected : Kind::Secondary)) {
            state.wanted.mode = m;
        }
        ImGui::PopFont();
    }
    note(font_small_, C_MUTED, about[state.wanted.mode]);

    section(font_small_, "TEAMS");
    {
        const float h      = dp(30);
        const float y      = ImGui::GetCursorPosY();
        const float right  = ImGui::GetCursorPosX() + ImGui::GetContentRegionAvail().x;
        ImGui::SetCursorPosY(y + (h - ImGui::GetTextLineHeight()) * 0.5f);
        ImGui::TextUnformatted("Players a side");
        ImGui::SameLine(right - 3 * h - 2 * gap);
        ImGui::SetCursorPosY(y);
        ImGui::BeginDisabled(state.wanted.game <= 1);
        if (button("-##players", h, h)) {
            --state.wanted.game;
        }
        ImGui::EndDisabled();
        char count[8];
        std::snprintf(count, sizeof(count), "%d", state.wanted.game);
        ImGui::SameLine(0.0f, gap);
        ImGui::SetCursorPosY(y);
        ImGui::BeginDisabled();
        button(count, h, h);
        ImGui::EndDisabled();
        ImGui::SameLine(0.0f, gap);
        ImGui::SetCursorPosY(y);
        ImGui::BeginDisabled(state.wanted.game >= 3);
        if (button("+##players", h, h)) {
            ++state.wanted.game;
        }
        ImGui::EndDisabled();
    }
    auto role_row = [&](const char* label, const ImVec4& colour, int* role) {
        ImGui::PushID(label);
        ImGui::PushFont(font_small_);
        ImGui::TextColored(colour, "%s", label);
        ImGui::SameLine();
        ImGui::TextColored(C_MUTED, "NUbots role");
        ImGui::PopFont();
        ImGui::SetNextItemWidth(-FLT_MIN);
        const char* current = state.roles.empty() ? "(none found)" : state.roles[static_cast<std::size_t>(*role)].c_str();
        if (ImGui::BeginCombo("##role", current)) {
            for (int i = 0; i < static_cast<int>(state.roles.size()); ++i) {
                if (ImGui::Selectable(state.roles[static_cast<std::size_t>(i)].c_str(), i == *role)) {
                    *role = i;
                }
            }
            ImGui::EndCombo();
        }
        ImGui::PopID();
    };
    role_row("HOME", C_HOME, &state.wanted.home_role);
    role_row("AWAY", C_AWAY, &state.wanted.away_role);

    section(font_small_, "PERCEPTION");
    const float half = std::floor((ImGui::GetContentRegionAvail().x - gap) * 0.5f);
    for (int truth = 0; truth < 2; ++truth) {
        if (truth > 0) {
            ImGui::SameLine(0.0f, gap);
        }
        if (button(truth == 0 ? "Vision" : "Ground truth", half, dp(32),
                   state.wanted.ground_truth == (truth == 1) ? Kind::Selected : Kind::Secondary)) {
            state.wanted.ground_truth = truth == 1;
        }
    }
    note(font_small_, C_MUTED,
         state.wanted.ground_truth ? "Robots are given their true position and the ball's. Vision and localisation are bypassed."
                                   : "Robots find themselves and the ball with their own camera.");

    ImGui::Dummy({0.0f, dp(2)});
    const bool changed = !(state.wanted == state.running);
    ImGui::BeginDisabled(!changed);
    if (button("Apply and Restart", 0.0f, dp(40), Kind::Primary)) {
        pending_.restart = true;
    }
    ImGui::EndDisabled();
    if (changed) {
        note(font_small_, C_WARN, "Restarts the simulator and every robot's code with this setup.");
    }

    // Set plays: the call goes to the robots through the GameController, the ball goes where you click.
    if (state.running.mode == MODE_TESTING && state.match.has_game) {
        section(font_small_, "SET PLAY  \xC2\xB7  WHO TAKES IT");
        struct Play {
            const char* name;
            int value;
        };
        static constexpr Play plays[5] = {{"Corner kick", SP_CORNER_KICK},
                                          {"Goal kick", SP_GOAL_KICK},
                                          {"Free kick", SP_FREE_KICK},
                                          {"Throw-in", SP_THROW_IN},
                                          {"Penalty kick", SP_PENALTY_KICK}};
        const float row_h = dp(30);
        const float btn_w = dp(68);
        // Any time after kick-off; the GameController itself refuses a call the rules don't allow.
        const bool playing = state.match.state != "INITIAL" && state.match.state != "FINISHED";
        const char* active_name = "";
        for (const Play& play : plays) {
            const float right = ImGui::GetCursorPosX() + ImGui::GetContentRegionAvail().x;
            const float y     = ImGui::GetCursorPosY();
            ImGui::SetCursorPosY(y + (row_h - ImGui::GetTextLineHeight()) * 0.5f);
            ImGui::TextUnformatted(play.name);
            ImGui::PushID(play.value);
            for (int team = 0; team < 2; ++team) {
                ImGui::SameLine(right - (2 - team) * btn_w - (1 - team) * gap);
                ImGui::SetCursorPosY(y);
                ImGui::PushID(team);
                const bool active = state.set_play == play.value && state.set_play_team == team;
                if (active) {
                    active_name = play.name;
                }
                ImGui::BeginDisabled(!playing);
                if (button(TEAM_TAG[team], btn_w, row_h, active ? Kind::Selected : Kind::Secondary)) {
                    state.set_play         = play.value;
                    state.set_play_team    = team;
                    state.placing          = true;
                    pending_.call_set_play = std::pair<int, int>{team, play.value};
                }
                ImGui::EndDisabled();
                ImGui::PopID();
            }
            ImGui::PopID();
        }
        ImGui::Dummy({0.0f, dp(2)});
        char status[160];
        if (!playing) {
            note(font_small_, C_MUTED, "Start the match first: set plays can only be called while play is running.");
        }
        else if (state.placing) {
            const char* where = state.set_play == SP_CORNER_KICK    ? "Click a highlighted corner"
                                : state.set_play == SP_GOAL_KICK    ? "Click a highlighted spot on the goal area"
                                : state.set_play == SP_PENALTY_KICK ? "Click the highlighted penalty mark"
                                : state.set_play == SP_THROW_IN     ? "Click a point on a touchline"
                                                                    : "Click anywhere on the field";
            std::snprintf(status, sizeof(status), "%s to place the ball.", where);
            note(font_small_, C_WARN, status);
        }
        else if (state.set_play != SP_NONE) {
            std::snprintf(status, sizeof(status), "%s for %s called. Press Play to set the ball free.", active_name,
                          state.set_play_team == 0 ? "Home" : "Away");
            note(font_small_, team_colour(state.set_play_team), status);
        }
        ImGui::BeginDisabled(state.set_play == SP_NONE);
        if (button("Play  -  Ball Free", 0.0f, dp(40), Kind::Primary)) {
            state.set_play            = SP_NONE;
            state.placing             = false;
            pending_.release_set_play = true;
        }
        ImGui::EndDisabled();
    }
    ImGui::End();
}

// Bottom-centre bar: pause/resume and the simulation clock.
void ViewerUI::render_playbar(UIState& state) {
    const ImVec2 display = ImGui::GetIO().DisplaySize;
    const float w        = dp(250);
    const float h        = dp(56);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {0, 0});
    begin_panel("##playbar", {display.x * 0.5f, display.y - dp(14)}, {0.5f, 1.0f}, {w, h});
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImVec2 o = ImGui::GetWindowPos();
    ImGui::SetCursorPos({dp(10), dp(10)});
    if (icon_button("play", state.paused ? Icon::Play : Icon::Pause, dp(36), Kind::Primary)) {
        pending_.pause = !state.paused;
    }
    text_at(dl, font_body_, {o.x + dp(58), o.y + (h - font_px(font_body_)) * 0.5f}, 0.0f, col(C_TEXT),
            clock_text(state.sim_time, true).c_str());
    chip(dl, font_small_, {o.x + w - dp(48), o.y + h * 0.5f}, state.paused ? C_WARN : C_GOOD,
         state.paused ? "PAUSED" : "LIVE");
    ImGui::End();
    ImGui::PopStyleVar();
}

}  // namespace k1sim::module::viewer
