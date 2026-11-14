#ifndef K1SIM_MODULE_VIEWER_MATCHPANEL_HPP
#define K1SIM_MODULE_VIEWER_MATCHPANEL_HPP

#include <cstdio>
#include <mujoco/mujoco.h>
#include <mutex>
#include <optional>

#include "shared/message/MatchMessages.hpp"
#include "shared/util/RobotRoster.hpp"

namespace k1sim::module {

    // The score, game state and buttons for a match, top right of the window, and its updates, bottom
    // left. Hidden until
    // module::MatchReferee reports in, so other roles don't see it.
    class MatchPanel {
    public:
        void set(const message::MatchStatus& status) {
            std::lock_guard<std::mutex> lock(mutex_);
            status_ = status;
        }

        void render(const mjrRect& viewport, const mjrContext* con) const {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!status_) {
                return;
            }
            mjr_overlay(mjFONT_NORMAL, mjGRID_TOPRIGHT, viewport, status_->summary.c_str(), nullptr, con);
            if (!status_->messages.empty()) {
                mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, viewport, status_->messages.c_str(), nullptr, con);
            }
            if (!status_->primary_button.empty()) {
                draw(button(viewport, 0), status_->primary_button.c_str(), 0.1F, 0.45F, 0.2F, con);
            }
            draw(button(viewport, 1), "New game", 0.3F, 0.3F, 0.35F, con);
        }

        // Adds the label above each robot's head. Caller holds the sim mutex.
        void add_labels(const mjModel* m, const mjData* d, mjvScene* scn) const {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!status_) {
                return;
            }
            const auto& roster = config::robot_roster();
            for (std::size_t i = 0; i < roster.size() && i < status_->robot_labels.size(); ++i) {
                const int body = mj_name2id(m, mjOBJ_BODY, roster[i].body().c_str());
                if (body < 0 || scn->ngeom >= scn->maxgeom) {
                    continue;
                }
                const mjtNum size[3] = {0.0, 0.0, 0.0};
                const mjtNum pos[3]  = {d->xpos[3 * body], d->xpos[3 * body + 1], d->xpos[3 * body + 2] + 0.5};
                const mjtNum mat[9]  = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
                const float home[4] = {0.2F, 0.55F, 1.0F, 1.0F}, away[4] = {1.0F, 0.3F, 0.25F, 1.0F};
                mjvGeom& label = scn->geoms[scn->ngeom++];
                mjv_initGeom(&label,
                             mjGEOM_LABEL,
                             size,
                             pos,
                             mat,
                             roster[i].team_id == roster.front().team_id ? home : away);
                std::snprintf(label.label, sizeof(label.label), "%s", status_->robot_labels[i].c_str());
            }
        }

        // What a click at (x, y), in framebuffer pixels from the bottom left, asks for.
        std::optional<message::MatchCommand> click(const mjrRect& viewport, int x, int y) const {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!status_) {
                return std::nullopt;
            }
            if (!status_->primary_button.empty() && contains(button(viewport, 0), x, y)) {
                return message::MatchCommand{message::MatchCommand::Action::PRIMARY};
            }
            if (contains(button(viewport, 1), x, y)) {
                return message::MatchCommand{message::MatchCommand::Action::RESET};
            }
            return std::nullopt;
        }

    private:
        static constexpr int WIDTH     = 170;
        static constexpr int HEIGHT    = 44;
        static constexpr int MARGIN    = 12;
        static constexpr int TEXT_ROOM = 170;  // the summary's height above the buttons

        static mjrRect button(const mjrRect& viewport, int index) {
            return {viewport.width - WIDTH - MARGIN,
                    viewport.height - TEXT_ROOM - (index + 1) * (HEIGHT + MARGIN),
                    WIDTH,
                    HEIGHT};
        }

        static bool contains(const mjrRect& rect, int x, int y) {
            return x >= rect.left && x < rect.left + rect.width && y >= rect.bottom && y < rect.bottom + rect.height;
        }

        static void draw(const mjrRect& rect, const char* text, float r, float g, float b, const mjrContext* con) {
            mjr_label(rect, mjFONT_NORMAL, text, r, g, b, 1.0F, 1.0F, 1.0F, 1.0F, con);
        }

        mutable std::mutex mutex_;
        std::optional<message::MatchStatus> status_;
    };

}  // namespace k1sim::module

#endif  // K1SIM_MODULE_VIEWER_MATCHPANEL_HPP
