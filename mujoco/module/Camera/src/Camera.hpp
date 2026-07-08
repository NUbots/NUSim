#ifndef K1SIM_MODULE_CAMERA_HPP
#define K1SIM_MODULE_CAMERA_HPP

#include <atomic>
#include <nuclear>
#include <thread>

#include "module/Camera/src/CameraConfig.hpp"
#include "shared/message/SimMessages.hpp"

namespace k1sim::module {

// Renders the K1's head camera offscreen (MuJoCo mjv/mjr) and publishes rgb8
// frames into a Boost.Interprocess shared-memory segment matching NUbots'
// K1Camera SharedImageHeader byte-for-byte, so the unchanged input::K1Camera
// reads them -> ImageCompressor -> NetworkForwarder -> NUsight. No NUbots-side
// changes.
//
// Threading: all MuJoCo GL calls must happen on the one thread that holds the
// GL context current. module::Viewer already owns NUClear's MainThread for its
// own (window) GLFW context, and is disabled under --headless -- but Camera
// must keep rendering under --headless too (it feeds NUsight, not a local
// window), so it cannot piggyback on Viewer's MainThread reactions, and it must
// not gate on cli().headless the way Viewer does. Instead render_loop() runs on
// its own dedicated std::thread with its own hidden GLFW window/context, paced
// by its own sleep_until loop rather than on<Every<>> (a NUClear thread-pool
// reaction is not guaranteed to run on the same OS thread twice, which would
// violate the one-thread-owns-the-context rule). See Camera.cpp for the full
// rationale, including why it deliberately never calls glfwTerminate().
class Camera : public NUClear::Reactor {
public:
    explicit Camera(std::unique_ptr<NUClear::Environment> environment);

private:
    void render_loop(camera::CameraConfig cfg);

    std::atomic<bool> running_{false};
    std::thread render_thread_;

    // Written once by the Trigger<SimHandles> reaction (any NUClear pool
    // thread); read by render_thread_ only after it observes handles_ready_ ==
    // true (release/acquire), which happens-before publishes the plain
    // pointer/mutex writes in handles_ across threads -- module::Simulation
    // only ever emits SimHandles once and never touches it again afterwards.
    std::atomic<bool> handles_ready_{false};
    message::SimHandles handles_{};
};

}  // namespace k1sim::module

#endif  // K1SIM_MODULE_CAMERA_HPP
