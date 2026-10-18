#ifndef K1SIM_MODULE_CAMERA_HPP
#define K1SIM_MODULE_CAMERA_HPP

#include <atomic>
#include <nuclear>
#include <thread>

#include "module/Camera/src/CameraConfig.hpp"
#include "shared/message/SimMessages.hpp"

namespace k1sim::module {

    // Renders the K1 head camera offscreen and publishes rgb8 frames into a shared-memory segment
    // laid out like NUbots' K1Camera SharedImageHeader, so the unchanged input::K1Camera reads them.
    //
    // Threading: MuJoCo GL calls must stay on the thread that holds the context. Viewer owns
    // MainThread and is off under --headless, but Camera must render headless too, so render_loop()
    // runs on its own std::thread with its own CGL (macOS) or EGL context, paced by sleep_until.
    // on<Every<>> won't do: pool reactions aren't guaranteed to run on the same OS thread.
    class Camera : public NUClear::Reactor {
    public:
        explicit Camera(std::unique_ptr<NUClear::Environment> environment);

    private:
        void render_loop(camera::CameraConfig cfg);

        std::atomic<bool> running_{false};
        std::thread render_thread_;

        // Written once by the Trigger<SimHandles> reaction; render_thread_ reads it only after
        // seeing handles_ready_ (release/acquire). SimHandles is emitted only once.
        std::atomic<bool> handles_ready_{false};
        message::SimHandles handles_{};
    };

}  // namespace k1sim::module

#endif  // K1SIM_MODULE_CAMERA_HPP
