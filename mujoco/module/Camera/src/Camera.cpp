#include "module/Camera/src/Camera.hpp"

#include <mujoco/mujoco.h>

#include <chrono>
#include <cmath>
#include <cstring>
#include <exception>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "module/Camera/src/CameraConfig.hpp"
#include "module/Camera/src/EglContext.hpp"
#include "module/Camera/src/SharedImageWriter.hpp"
#include "shared/util/Config.hpp"

namespace k1sim::module {

namespace {

// Mirrors K1Camera.cpp's MAX_IMAGE_BYTES exactly -- the reader drops (logs WARN
// and skips) any frame whose data_size exceeds this, so it's not enough to just
// fit in the segment; we must fit in what the reader will accept.
constexpr std::size_t kMaxImageBytes = 2 * 1024 * 1024;

// Avoid M_PI: not reliably available under this project's -std=c++17 build (see
// test/unit/test_supervisor.cpp's identical workaround/comment).
constexpr double kPi = 3.14159265358979323846;

}  // namespace

Camera::Camera(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

    on<Startup>().then([this] {
        camera::CameraConfig cfg = camera::load_config(config::load("camera.yaml"));
        log<NUClear::LogLevel::INFO>("Camera: starting offscreen render thread (segment",
                                      cfg.segment.c_str(),
                                      ",",
                                      cfg.width,
                                      "x",
                                      cfg.height,
                                      "@",
                                      cfg.fps,
                                      "fps, mjcf camera",
                                      cfg.mjcf_camera.c_str(),
                                      ")");
        running_.store(true, std::memory_order_release);
        render_thread_ = std::thread(&Camera::render_loop, this, cfg);
    });

    // Deliberately NOT MainThread: this reaction only stores plain pointers (no
    // GL work), so it can run on any NUClear pool thread. render_thread_ is the
    // only thread that ever touches GL/mjv/mjr state or handles_ after startup.
    on<Trigger<message::SimHandles>>().then([this](const message::SimHandles& h) {
        handles_ = h;
        handles_ready_.store(true, std::memory_order_release);
    });

    on<Shutdown>().then([this] {
        running_.store(false, std::memory_order_release);
        if (render_thread_.joinable()) {
            render_thread_.join();
        }
        log<NUClear::LogLevel::INFO>("Camera shutting down");
    });
}

void Camera::render_loop(camera::CameraConfig cfg) {
    // Everything below (GLFW window, mjvScene/mjrContext, the shared-memory
    // writer) is local to this function/thread on purpose -- see the
    // class-level comment in Camera.hpp for the threading rationale.

    const auto frame_bytes = static_cast<std::size_t>(cfg.width) * static_cast<std::size_t>(cfg.height) * 3;
    if (frame_bytes > kMaxImageBytes) {
        log<NUClear::LogLevel::ERROR>("Camera: configured",
                                       cfg.width,
                                       "x",
                                       cfg.height,
                                       "rgb8 exceeds K1Camera's MAX_IMAGE_BYTES (2 MiB) -- the reader would drop "
                                       "every frame. Lower width/height in config/camera.yaml.");
        return;
    }

    // Offscreen OpenGL via EGL (not GLFW): no shared GLFW/Xlib global state with
    // module::Viewer's on-screen window, so the two render threads don't corrupt
    // each other's heap; also works with no display (headless). RAII — tears the
    // context down on any return below.
    camera::EglContext egl(cfg.width, cfg.height);
    if (!egl.valid()) {
        log<NUClear::LogLevel::ERROR>("Camera: EGL offscreen context creation failed -- no camera frames "
                                       "will be published (need libEGL + a GPU/mesa device)");
        return;
    }

    // Created from config alone (doesn't need the model), so the segment exists
    // whether the sim or NUbots' K1Camera process starts first -- K1Camera
    // retries opening it every 500 ms until it appears.
    std::unique_ptr<camera::SharedImageWriter> writer;
    try {
        writer = std::make_unique<camera::SharedImageWriter>(cfg.segment, cfg.width, cfg.height);
    }
    catch (const std::exception& e) {
        log<NUClear::LogLevel::ERROR>("Camera: failed to create shared-memory segment",
                                       cfg.segment.c_str(),
                                       ":",
                                       e.what());
        return;
    }

    while (running_.load(std::memory_order_acquire) && !handles_ready_.load(std::memory_order_acquire)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    if (!running_.load(std::memory_order_acquire)) {
        return;
    }

    const mjModel* model    = handles_.model;
    mjData* data            = handles_.data;
    std::mutex* sim_mutex   = handles_.mutex;

    const int cam_id = mj_name2id(model, mjOBJ_CAMERA, cfg.mjcf_camera.c_str());
    if (cam_id < 0) {
        log<NUClear::LogLevel::ERROR>("Camera: mjcf camera",
                                       cfg.mjcf_camera.c_str(),
                                       "not found in the model -- check config/camera.yaml's mjcf_camera "
                                       "against models/k1/K1_22dof.xml");
        return;
    }

    if (cfg.width > model->vis.global.offwidth || cfg.height > model->vis.global.offheight) {
        log<NUClear::LogLevel::ERROR>("Camera: configured",
                                       cfg.width,
                                       "x",
                                       cfg.height,
                                       "exceeds the model's compiled offscreen buffer",
                                       model->vis.global.offwidth,
                                       "x",
                                       model->vis.global.offheight,
                                       "-- fix config/camera.yaml or the model's "
                                       "<visual><global offwidth=.../offheight=.../></visual>");
        return;
    }

    mjvCamera cam;
    mjv_defaultCamera(&cam);
    cam.type       = mjCAMERA_FIXED;
    cam.fixedcamid = cam_id;

    mjvOption opt;
    mjv_defaultOption(&opt);

    mjvScene scn;
    mjv_defaultScene(&scn);
    mjv_makeScene(model, &scn, 2000);

    mjrContext con;
    mjr_defaultContext(&con);
    mjr_makeContext(model, &con, mjFONTSCALE_150);
    mjr_setBuffer(mjFB_OFFSCREEN, &con);

    const mjrRect viewport{0, 0, cfg.width, cfg.height};
    std::vector<unsigned char> raw(frame_bytes);      // mjr_readPixels output: bottom-up (OpenGL convention)
    std::vector<unsigned char> flipped(frame_bytes);  // top-down, matches what K1Camera/NUsight expect

    // Pinhole intrinsics derived from the MJCF camera's fovy + configured
    // resolution (MuJoCo has no lens-distortion model, so k1/k2 published via
    // SharedImageWriter::publish are always 0). `fov` follows the same
    // "diagonal angle from the optical axis to the farthest image corner"
    // convention module/platform/Webots.cpp used for its own simulated camera on
    // the NUbots side (see utility::vision::projection's RECTILINEAR model and
    // Webots.cpp's "auto fov" branch): unproject the far image corner and take
    // twice that half-angle.
    const double fovy_rad          = model->cam_fovy[cam_id] * kPi / 180.0;
    const double focal_px          = (static_cast<double>(cfg.height) * 0.5) / std::tan(fovy_rad * 0.5);
    const double focal_length_norm = focal_px / static_cast<double>(cfg.width);
    const double aspect            = static_cast<double>(cfg.height) / static_cast<double>(cfg.width);
    const double half_diag_norm    = 0.5 * std::sqrt(1.0 + aspect * aspect);
    const double fov_rad           = 2.0 * std::atan(half_diag_norm / focal_length_norm);

    log<NUClear::LogLevel::INFO>("Camera: rendering",
                                  cfg.width,
                                  "x",
                                  cfg.height,
                                  "fovy",
                                  model->cam_fovy[cam_id],
                                  "deg, focal_length_norm",
                                  focal_length_norm,
                                  "fov_diag_rad",
                                  fov_rad);

    const auto frame_period = std::chrono::duration<double>(1.0 / std::max(1.0, cfg.fps));
    auto next_frame         = std::chrono::steady_clock::now();

    while (running_.load(std::memory_order_acquire)) {
        {
            std::lock_guard<std::mutex> lock(*sim_mutex);
            mjv_updateScene(model, data, &opt, nullptr, &cam, mjCAT_ALL, &scn);
        }
        mjr_render(viewport, &scn, &con);
        mjr_readPixels(raw.data(), nullptr, viewport, &con);

        // mjr_readPixels is bottom-up (OpenGL convention); K1Camera/NUsight
        // expect top-down rgb8 like any normal image buffer, so flip row order
        // once here rather than downstream.
        for (int row = 0; row < cfg.height; ++row) {
            const unsigned char* src =
                raw.data() + static_cast<std::size_t>(cfg.height - 1 - row) * static_cast<std::size_t>(cfg.width) * 3;
            unsigned char* dst = flipped.data() + static_cast<std::size_t>(row) * static_cast<std::size_t>(cfg.width) * 3;
            std::memcpy(dst, src, static_cast<std::size_t>(cfg.width) * 3);
        }

        writer->publish(flipped.data(),
                         flipped.size(),
                         static_cast<float>(focal_length_norm),
                         static_cast<float>(fov_rad),
                         0.0f,
                         0.0f);

        next_frame += std::chrono::duration_cast<std::chrono::steady_clock::duration>(frame_period);
        const auto now = std::chrono::steady_clock::now();
        if (next_frame < now) {
            next_frame = now;  // fell behind -- resync instead of spinning to catch up
        }
        std::this_thread::sleep_until(next_frame);
    }

    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    writer.reset();  // destructor removes the shm segment
    // EGL context torn down by egl's destructor (RAII), independent of Viewer's GLFW.
}

}  // namespace k1sim::module
