#ifndef K1SIM_MODULE_MATCHCAMERAS_MATCHCAMERAS_HPP
#define K1SIM_MODULE_MATCHCAMERAS_MATCHCAMERAS_HPP

#include <atomic>
#include <nuclear>
#include <string>
#include <thread>

#include "module/Camera/src/CameraConfig.hpp"
#include "shared/message/SimMessages.hpp"

namespace k1sim::module {

    // Renders every roster robot's head camera, each into its own shared-memory segment, for players
    // that use vision. module::Camera does this for the main robot alone. The scene is captured once
    // per frame, so the physics thread waits once however many robots there are. Like Camera, all GL
    // work stays on one dedicated thread.
    class MatchCameras : public NUClear::Reactor {
    public:
        explicit MatchCameras(std::unique_ptr<NUClear::Environment> environment);

        // The segment for a robot: camera.yaml's name for robot 1, so that a default NUbots config
        // works, then that name with the robot id. tools/match.py builds the same names.
        static std::string segment(const std::string& base, int robot_id) {
            return robot_id == 1 ? base : base + "_robot" + std::to_string(robot_id);
        }

    private:
        void render_loop(camera::CameraConfig cfg);

        std::atomic<bool> running_{false};
        std::thread render_thread_;
        std::atomic<bool> handles_ready_{false};
        message::SimHandles handles_{};
    };

}  // namespace k1sim::module

#endif  // K1SIM_MODULE_MATCHCAMERAS_MATCHCAMERAS_HPP
