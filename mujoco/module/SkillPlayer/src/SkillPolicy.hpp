#ifndef K1SIM_MODULE_SKILLPLAYER_SKILLPOLICY_HPP
#define K1SIM_MODULE_SKILLPLAYER_SKILLPOLICY_HPP

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdint>
#include <onnxruntime_cxx_api.h>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "shared/k1/JointIndex.hpp"

namespace k1sim::skill {

    // The ball skills' observation: seven terms, each as its last three control steps, oldest first.
    inline constexpr std::size_t HISTORY  = 3;
    inline constexpr std::size_t OBS_SIZE = HISTORY * (3 + 3 + 3 * JOINT_COUNT + 2 + 4);  // 234
    // The walk policy's: six terms, current step only.
    inline constexpr std::size_t WALK_OBS_SIZE = 3 + 3 + 3 * JOINT_COUNT + 3;  // 75

    struct Gains {
        std::array<double, JOINT_COUNT> kp{};
        std::array<double, JOINT_COUNT> kd{};
    };

    // One exported mjlab policy: `obs` [1, N] -> `actions` [1, 22]. Observation normalisation is
    // inside the graph. The joint order, default pose, action scale and PD gains travel with the
    // file as metadata and are read from there, so a retrained policy needs no config change.
    //
    // Gains are the exception: files exported from runs with the torque-speed motor model carry
    // placeholder gains (kp 1, kd 0) in their metadata. If the file's gains look like that and
    // `fallback` is given, the fallback is used instead and gains_from_file() is false.
    class SkillPolicy {
    public:
        std::array<double, JOINT_COUNT> default_pose{};
        std::array<double, JOINT_COUNT> action_scale{};
        Gains gains;

        SkillPolicy(Ort::Env& env,
                    const std::string& path,
                    std::size_t obs_size,
                    const std::optional<Gains>& fallback = std::nullopt)
            : path_(path), session_(env, path.c_str(), options()) {
            Ort::AllocatorWithDefaultOptions allocator;
            if (session_.GetInputCount() != 1 || session_.GetOutputCount() != 1) {
                fail("expected one input and one output");
            }
            input_name_  = session_.GetInputNameAllocated(0, allocator).get();
            output_name_ = session_.GetOutputNameAllocated(0, allocator).get();
            expect_width(session_.GetInputTypeInfo(0), obs_size, "input");
            expect_width(session_.GetOutputTypeInfo(0), JOINT_COUNT, "output");
            obs_size_ = obs_size;

            const Ort::ModelMetadata metadata = session_.GetModelMetadata();
            auto lookup = [&](const char* key) -> std::string {
                const auto value = metadata.LookupCustomMetadataMapAllocated(key, allocator);
                if (!value) {
                    fail(std::string("no '") + key + "' in its metadata");
                }
                return value.get();
            };
            const auto names = split(lookup("joint_names"));
            if (names.size() != JOINT_COUNT) {
                fail("joint_names lists " + std::to_string(names.size()) + " joints, not 22");
            }
            for (std::size_t i = 0; i < JOINT_COUNT; ++i) {
                if (canonical(names[i]) != canonical(JOINT_NAMES[i])) {
                    fail("joint " + std::to_string(i) + " is '" + names[i] + "', the sim's is '" + JOINT_NAMES[i] + "'");
                }
            }
            default_pose = numbers(lookup("default_joint_pos"), "default_joint_pos");
            action_scale = numbers(lookup("action_scale"), "action_scale");
            gains.kp     = numbers(lookup("joint_stiffness"), "joint_stiffness");
            gains.kd     = numbers(lookup("joint_damping"), "joint_damping");
            if (!usable(gains)) {
                if (!fallback) {
                    fail("its metadata gains are placeholders (kp 1, kd 0) and no fallback gains were given");
                }
                gains           = *fallback;
                gains_from_file_ = false;
            }
        }

        const std::string& path() const {
            return path_;
        }
        std::size_t obs_size() const {
            return obs_size_;
        }
        bool gains_from_file() const {
            return gains_from_file_;
        }

        // Real PD gains: every kp above 1 and every kd above 0.
        static bool usable(const Gains& g) {
            return std::all_of(g.kp.begin(), g.kp.end(), [](double v) { return v > 1.0; })
                   && std::all_of(g.kd.begin(), g.kd.end(), [](double v) { return v > 0.0; });
        }

        // `obs` must hold obs_size() values; the graph reads from it in place.
        std::array<float, JOINT_COUNT> infer(std::vector<float>& obs) {
            if (obs.size() != obs_size_) {
                fail("given " + std::to_string(obs.size()) + " inputs, wants " + std::to_string(obs_size_));
            }
            const std::array<int64_t, 2> shape{1, static_cast<int64_t>(obs_size_)};
            const auto memory  = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
            Ort::Value input   = Ort::Value::CreateTensor<float>(memory, obs.data(), obs.size(), shape.data(), 2);
            const char* in[]   = {input_name_.c_str()};
            const char* out[]  = {output_name_.c_str()};
            const auto outputs = session_.Run(Ort::RunOptions{nullptr}, in, &input, 1, out, 1);
            const float* data  = outputs.front().GetTensorData<float>();
            std::array<float, JOINT_COUNT> actions{};
            for (std::size_t i = 0; i < JOINT_COUNT; ++i) {
                actions[i] = data[i];
            }
            return actions;
        }

    private:
        static Ort::SessionOptions options() {
            Ort::SessionOptions opts;
            opts.SetIntraOpNumThreads(1);  // a small MLP; threads would only add jitter
            opts.SetInterOpNumThreads(1);
            return opts;
        }

        // The model prefixes some joints with A/AA to fix MuJoCo's ordering ("AAHead_yaw",
        // "ALeft_Shoulder_Pitch"); the walk policy calls them "Head_Yaw", "Left_Shoulder_Pitch".
        static std::string canonical(std::string name) {
            std::transform(name.begin(), name.end(), name.begin(), [](unsigned char c) { return std::tolower(c); });
            const auto start = name.find_first_not_of('a');
            return start == std::string::npos ? name : name.substr(start);
        }

        [[noreturn]] void fail(const std::string& what) const {
            throw std::runtime_error("skill policy " + path_ + ": " + what);
        }

        void expect_width(const Ort::TypeInfo& info, std::size_t width, const char* which) const {
            const auto shape = info.GetTensorTypeAndShapeInfo().GetShape();
            if (shape.size() != 2 || shape[1] != static_cast<int64_t>(width)) {
                fail(std::string(which) + " is not [1, " + std::to_string(width) + "]");
            }
        }

        static std::vector<std::string> split(const std::string& text) {
            std::vector<std::string> parts;
            std::stringstream stream(text);
            for (std::string part; std::getline(stream, part, ',');) {
                parts.push_back(part);
            }
            return parts;
        }

        std::array<double, JOINT_COUNT> numbers(const std::string& text, const char* key) const {
            const auto parts = split(text);
            if (parts.size() != JOINT_COUNT) {
                fail(std::string(key) + " has " + std::to_string(parts.size()) + " values, not 22");
            }
            std::array<double, JOINT_COUNT> values{};
            for (std::size_t i = 0; i < JOINT_COUNT; ++i) {
                values[i] = std::stod(parts[i]);
            }
            return values;
        }

        std::string path_;
        Ort::Session session_;
        std::string input_name_, output_name_;
        std::size_t obs_size_  = 0;
        bool gains_from_file_ = true;
    };

}  // namespace k1sim::skill

#endif  // K1SIM_MODULE_SKILLPLAYER_SKILLPOLICY_HPP
