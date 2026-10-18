#include "module/Locomotion/src/WalkPolicy.hpp"

#include <cmath>
#include <fstream>
#include <nlohmann/json.hpp>
#include <stdexcept>

namespace k1sim::module {

    namespace {
        template <typename Arr>
        void load_fixed(const nlohmann::json& j, const char* key, Arr& out) {
            const auto& v = j.at(key);
            if (v.size() != out.size()) {
                throw std::runtime_error(std::string("walk policy: '") + key + "' has " + std::to_string(v.size())
                                         + " entries, expected " + std::to_string(out.size()));
            }
            for (std::size_t i = 0; i < out.size(); ++i) {
                out[i] = v[i].get<typename Arr::value_type>();
            }
        }
    }  // namespace

    std::unique_ptr<WalkPolicy> WalkPolicy::load(const std::string& path) {
        std::ifstream in(path);
        if (!in) {
            throw std::runtime_error("walk policy: cannot open " + path);
        }
        const nlohmann::json j = nlohmann::json::parse(in);

        auto p = std::unique_ptr<WalkPolicy>(new WalkPolicy());
        load_fixed(j, "obs_mean", p->obs_mean_);
        load_fixed(j, "obs_std", p->obs_std_);
        load_fixed(j, "default_joint_pos", p->default_pose_);
        load_fixed(j, "action_scale", p->action_scale_);
        load_fixed(j, "kp", p->kp_);
        load_fixed(j, "kd", p->kd_);

        std::size_t prev = OBS_DIM;
        for (const auto& lj : j.at("layers")) {
            Layer l;
            l.n_in  = lj.at("n_in").get<std::size_t>();
            l.n_out = lj.at("n_out").get<std::size_t>();
            l.w     = lj.at("w").get<std::vector<float>>();
            l.b     = lj.at("b").get<std::vector<float>>();
            if (l.n_in != prev || l.w.size() != l.n_in * l.n_out || l.b.size() != l.n_out) {
                throw std::runtime_error("walk policy: inconsistent layer shapes in " + path);
            }
            const std::string act = lj.at("act").get<std::string>();
            if (act == "elu") {
                l.act = Layer::Act::Elu;
            }
            else if (act == "relu") {
                l.act = Layer::Act::Relu;
            }
            else if (act == "tanh") {
                l.act = Layer::Act::Tanh;
            }
            else if (act != "none") {
                throw std::runtime_error("walk policy: unknown activation '" + act + "'");
            }
            prev = l.n_out;
            p->layers_.push_back(std::move(l));
        }
        if (p->layers_.empty() || prev != JOINT_COUNT) {
            throw std::runtime_error("walk policy: network must end in " + std::to_string(JOINT_COUNT) + " outputs");
        }
        return p;
    }

    void WalkPolicy::reset() {
        last_action_.fill(0.0);
    }

    WalkPolicy::JointArray WalkPolicy::infer(const std::array<double, 3>& gyro,
                                             const std::array<double, 3>& gravity,
                                             const JointArray& q,
                                             const JointArray& dq,
                                             const std::array<double, 3>& command) {
        std::vector<float> h(OBS_DIM);
        std::size_t k = 0;
        for (double v : gyro) {
            h[k++] = static_cast<float>(v);
        }
        for (double v : gravity) {
            h[k++] = static_cast<float>(v);
        }
        for (std::size_t i = 0; i < JOINT_COUNT; ++i) {
            h[k++] = static_cast<float>(q[i] - default_pose_[i]);
        }
        for (double v : dq) {
            h[k++] = static_cast<float>(v);
        }
        for (double v : last_action_) {
            h[k++] = static_cast<float>(v);
        }
        for (double v : command) {
            h[k++] = static_cast<float>(v);
        }
        for (std::size_t i = 0; i < OBS_DIM; ++i) {
            h[i] = (h[i] - obs_mean_[i]) / obs_std_[i];
        }

        std::vector<float> next;
        for (const auto& l : layers_) {
            next.assign(l.n_out, 0.0f);
            for (std::size_t o = 0; o < l.n_out; ++o) {
                const float* w = &l.w[o * l.n_in];
                float acc      = l.b[o];
                for (std::size_t i = 0; i < l.n_in; ++i) {
                    acc += w[i] * h[i];
                }
                switch (l.act) {
                    case Layer::Act::Elu: acc = acc > 0.0f ? acc : std::expm1(acc); break;
                    case Layer::Act::Relu: acc = acc > 0.0f ? acc : 0.0f; break;
                    case Layer::Act::Tanh: acc = std::tanh(acc); break;
                    case Layer::Act::Linear: break;
                }
                next[o] = acc;
            }
            h.swap(next);
        }

        JointArray q_ref{};
        for (std::size_t i = 0; i < JOINT_COUNT; ++i) {
            last_action_[i] = h[i];  // raw network output feeds back, as in training
            q_ref[i]        = default_pose_[i] + action_scale_[i] * h[i];
        }
        return q_ref;
    }

}  // namespace k1sim::module
