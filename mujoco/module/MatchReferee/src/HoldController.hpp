#ifndef K1SIM_MODULE_MATCHREFEREE_HOLDCONTROLLER_HPP
#define K1SIM_MODULE_MATCHREFEREE_HOLDCONTROLLER_HPP

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "shared/k1/JointIndex.hpp"
#include "shared/sim/ModelMap.hpp"
#include "shared/sim/StepController.hpp"

namespace k1sim::module::matchreferee {

    // Wraps a robot's controller. While held, an upright robot stops where it is and is moved smoothly
    // into the ready pose, whatever its player commands. It's positioned rather than balanced, so it
    // can't fall: driving it to the pose with its motors knocked over about a third of robots caught
    // mid-step. Ending in the ready pose matters too, as releasing a robot frozen mid-step knocked over
    // about a quarter. Being positioned, it can also end up inside another robot or the floor, which
    // physics would fire it out of on release, so it's nudged clear while held. A robot that's falling
    // or fallen is left alone until it's up again. The wrapped controller keeps running so its mode and
    // fall state stay current.
    class HoldController : public StepController {
    public:
        HoldController(std::shared_ptr<StepController> inner,
                       std::string prefix,
                       std::array<double, JOINT_COUNT> ready,
                       double stand_height)
            : inner_(std::move(inner)), prefix_(std::move(prefix)), ready_(ready), stand_height_(stand_height) {}

        void set_held(bool held) {
            held_.store(held, std::memory_order_relaxed);
        }

        // Runs when the world resets, so that a frozen robot goes back to its start with everything else.
        void reset() override {
            inner_->reset();
            frozen_.reset();
        }

        void step(const mjModel* m, mjData* d) override {
            inner_->step(m, d);
            if (!held_.load(std::memory_order_relaxed)) {
                frozen_.reset();
                return;
            }
            if (!map_) {
                map_ = ModelMap::build(m, prefix_);
            }
            if (!frozen_) {
                if (!upright(d)) {
                    return;
                }
                frozen_ = capture(d);
            }
            separate(m, d);
            place(d);
        }

        int mode() const override {
            return inner_->mode();
        }
        int fall_state() const override {
            return inner_->fall_state();
        }
        bool getting_up() const override {
            return inner_->getting_up();
        }

    private:
        static constexpr double UPRIGHT_HEIGHT = 0.45;  // metres; standing is about 0.55
        static constexpr double UPRIGHT_TILT   = 0.8;   // cosine of the trunk's lean from vertical
        static constexpr int ROOT_QPOS         = 7;
        static constexpr int ROOT_DOF          = 6;
        static constexpr double BLEND_SECONDS  = 1.0;    // to reach the ready pose
        static constexpr double NUDGE          = 0.002;  // metres per physics step, to get clear of a contact

        struct Pose {
            double time = 0.0;
            double lift = 0.0;  // above the blended height, to keep feet out of the floor
            std::array<double, ROOT_QPOS> root{};
            std::array<double, JOINT_COUNT> joints{};
        };

        bool upright(const mjData* d) const {
            const double* root = d->qpos + map_->root_qpos_adr;
            return root[2] > UPRIGHT_HEIGHT && 1.0 - 2.0 * (root[4] * root[4] + root[5] * root[5]) > UPRIGHT_TILT;
        }

        Pose capture(const mjData* d) const {
            Pose pose;
            pose.time = d->time;
            for (int i = 0; i < ROOT_QPOS; ++i) {
                pose.root[i] = d->qpos[map_->root_qpos_adr + i];
            }
            for (std::size_t i = 0; i < JOINT_COUNT; ++i) {
                pose.joints[i] = d->qpos[map_->qpos_adr[i]];
            }
            return pose;
        }

        // Moves the frozen pose out of the deepest thing the robot overlaps. Contacts are from the last
        // physics step. Other robots are left to move themselves, so two held robots share the work.
        void separate(const mjModel* m, const mjData* d) {
            frozen_->lift            = std::max(0.0, frozen_->lift - 0.1 * NUDGE);
            const mjContact* deepest = nullptr;
            bool mine_first          = false;
            for (int i = 0; i < d->ncon; ++i) {
                const mjContact& contact = d->contact[i];
                const int root1          = m->body_rootid[m->geom_bodyid[contact.geom[0]]];
                const int root2          = m->body_rootid[m->geom_bodyid[contact.geom[1]]];
                if (root1 != root2 && (root1 == map_->root_body_id || root2 == map_->root_body_id) && contact.dist < 0.0
                    && (deepest == nullptr || contact.dist < deepest->dist)) {
                    deepest    = &contact;
                    mine_first = root1 == map_->root_body_id;
                }
            }
            if (deepest == nullptr) {
                return;
            }
            // frame[0..2] is the contact normal, pointing from the first geom to the second.
            const double push = std::min(-deepest->dist, NUDGE) * (mine_first ? -1.0 : 1.0);
            frozen_->root[0] += push * deepest->frame[0];
            frozen_->root[1] += push * deepest->frame[1];
            frozen_->lift += std::max(0.0, push * deepest->frame[2]);
        }

        // Puts the robot part way from its frozen pose to the ready pose, level and facing the same way.
        void place(mjData* d) const {
            const Pose& from = *frozen_;
            const double t   = std::min(1.0, (d->time - from.time) / BLEND_SECONDS);

            const double* q  = from.root.data() + 3;
            const double yaw = std::atan2(2.0 * (q[0] * q[3] + q[1] * q[2]), 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]));
            std::array<double, 4> level{std::cos(0.5 * yaw), 0.0, 0.0, std::sin(0.5 * yaw)};
            const double dot  = q[0] * level[0] + q[3] * level[3];
            const double sign = dot < 0.0 ? -1.0 : 1.0;
            std::array<double, 4> quat{};
            double norm = 0.0;
            for (int i = 0; i < 4; ++i) {
                quat[i] = (1.0 - t) * q[i] + t * sign * level[i];
                norm += quat[i] * quat[i];
            }

            double* root = d->qpos + map_->root_qpos_adr;
            root[0]      = from.root[0];
            root[1]      = from.root[1];
            root[2]      = (1.0 - t) * from.root[2] + t * stand_height_ + from.lift;
            for (int i = 0; i < 4; ++i) {
                root[3 + i] = quat[i] / std::sqrt(norm);
            }
            for (int i = 0; i < ROOT_DOF; ++i) {
                d->qvel[map_->root_dof_adr + i] = 0.0;
            }
            for (std::size_t i = 0; i < JOINT_COUNT; ++i) {
                d->qpos[map_->qpos_adr[i]] = (1.0 - t) * from.joints[i] + t * ready_[i];
                d->qvel[map_->dof_adr[i]]  = 0.0;
                d->ctrl[map_->act_id[i]]   = 0.0;
            }
        }

        std::shared_ptr<StepController> inner_;
        std::string prefix_;
        std::array<double, JOINT_COUNT> ready_;
        double stand_height_;
        std::optional<ModelMap> map_;  // built on the physics thread, the only user
        std::optional<Pose> frozen_;   // likewise
        std::atomic<bool> held_{true};
    };

}  // namespace k1sim::module::matchreferee

#endif  // K1SIM_MODULE_MATCHREFEREE_HOLDCONTROLLER_HPP
