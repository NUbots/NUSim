#include "module/MatchCameras/src/MatchCameras.hpp"

#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <mujoco/mujoco.h>
#include <vector>

#include "module/Camera/src/OffscreenContext.hpp"
#include "module/Camera/src/SharedImageWriter.hpp"
#include "module/MatchCameras/src/GpuEglContext.hpp"
#include "shared/sim/RobotIdentity.hpp"
#include "shared/util/Config.hpp"
#include "shared/util/RobotRoster.hpp"

namespace k1sim::module {

    namespace {

        struct RobotCamera {
            int id = -1;
            std::unique_ptr<camera::SharedImageWriter> writer;
            mjvGLCamera view[2]{};
            float focal_length_norm = 0.0F;
            float fov               = 0.0F;
        };

    }  // namespace

    MatchCameras::MatchCameras(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Startup>().then([this] {
            camera::CameraConfig cfg = camera::load_config(config::load("camera.yaml"));
            cfg.fps                  = config::load("match.yaml")["cameras_fps"].as<double>();
            running_.store(true, std::memory_order_release);
            render_thread_ = std::thread(&MatchCameras::render_loop, this, cfg);
        });

        on<Trigger<message::SimHandles>>().then([this](const message::SimHandles& handles) {
            handles_ = handles;
            handles_ready_.store(true, std::memory_order_release);
        });

        on<Shutdown>().then([this] {
            running_.store(false, std::memory_order_release);
            if (render_thread_.joinable()) {
                render_thread_.join();
            }
        });
    }

    void MatchCameras::render_loop(camera::CameraConfig cfg) {
        matchcameras::GpuEglContext gpu(cfg.width, cfg.height);
        std::unique_ptr<camera::OffscreenContext> fallback;
        if (!gpu.valid()) {
            log<NUClear::LogLevel::WARN>("MatchCameras: no GPU found through EGL, so cameras may be slow");
            fallback = std::make_unique<camera::OffscreenContext>(cfg.width, cfg.height);
            if (!fallback->valid()) {
                log<NUClear::LogLevel::ERROR>("MatchCameras: no offscreen OpenGL context, so no camera frames");
                return;
            }
        }

        while (running_.load(std::memory_order_acquire) && !handles_ready_.load(std::memory_order_acquire)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        if (!running_.load(std::memory_order_acquire)) {
            return;
        }

        const mjModel* model = handles_.model;
        mjData* data         = handles_.data;

        std::vector<RobotCamera> cameras;
        for (const auto& robot : config::robot_roster()) {
            RobotCamera camera;
            const std::string name = robot_prefix(robot.model_index) + cfg.mjcf_camera;
            camera.id              = mj_name2id(model, mjOBJ_CAMERA, name.c_str());
            if (camera.id < 0) {
                log<NUClear::LogLevel::ERROR>("MatchCameras: no camera", name.c_str(), "in the model");
                return;
            }
            camera.writer = std::make_unique<camera::SharedImageWriter>(segment(cfg.segment, robot.robot_id),
                                                                        cfg.width,
                                                                        cfg.height);

            // See Camera.cpp: pinhole intrinsics from the MJCF camera and the render size.
            const double focal_px    = (cfg.height * 0.5) / std::tan(model->cam_fovy[camera.id] * M_PI / 360.0);
            const double focal       = focal_px / cfg.width;
            const double aspect      = static_cast<double>(cfg.height) / cfg.width;
            camera.focal_length_norm = static_cast<float>(focal);
            camera.fov = static_cast<float>(2.0 * std::atan(0.5 * std::sqrt(1.0 + aspect * aspect) / focal));
            cameras.push_back(std::move(camera));
        }

        mjvCamera cam;
        mjv_defaultCamera(&cam);
        cam.type       = mjCAMERA_FIXED;
        cam.fixedcamid = cameras.front().id;

        mjvOption opt;
        mjv_defaultOption(&opt);

        mjvScene scn;
        mjv_defaultScene(&scn);
        mjv_makeScene(model, &scn, 4000);

        mjrContext con;
        mjr_defaultContext(&con);
        mjr_makeContext(model, &con, mjFONTSCALE_150);
        mjr_setBuffer(mjFB_OFFSCREEN, &con);

        const mjrRect viewport{0, 0, cfg.width, cfg.height};
        const std::size_t row_bytes = static_cast<std::size_t>(cfg.width) * 3;
        std::vector<unsigned char> raw(row_bytes * cfg.height);
        std::vector<unsigned char> flipped(raw.size());

        log<NUClear::LogLevel::INFO>("MatchCameras: rendering", cameras.size(), "cameras at", cfg.fps, "fps");

        const auto frame_period = std::chrono::duration<double>(1.0 / std::max(1.0, cfg.fps));
        auto next_frame         = std::chrono::steady_clock::now();
        auto rate_start         = next_frame;
        int rate_frames         = 0;

        while (running_.load(std::memory_order_acquire)) {
            {
                std::lock_guard<std::mutex> lock(*handles_.mutex);
                mjv_updateScene(model, data, &opt, nullptr, &cam, mjCAT_ALL, &scn);
                for (auto& camera : cameras) {
                    cam.fixedcamid = camera.id;
                    mjv_updateCamera(model, data, &cam, &scn);
                    std::memcpy(camera.view, scn.camera, sizeof(camera.view));
                }
            }

            for (auto& camera : cameras) {
                std::memcpy(scn.camera, camera.view, sizeof(camera.view));
                mjr_render(viewport, &scn, &con);
                mjr_readPixels(raw.data(), nullptr, viewport, &con);

                // mjr_readPixels is bottom-up, and the reader expects top-down.
                for (int row = 0; row < cfg.height; ++row) {
                    std::memcpy(flipped.data() + row * row_bytes,
                                raw.data() + (cfg.height - 1 - row) * row_bytes,
                                row_bytes);
                }
                camera.writer
                    ->publish(flipped.data(), flipped.size(), camera.focal_length_norm, camera.fov, 0.0F, 0.0F);
            }

            next_frame += std::chrono::duration_cast<std::chrono::steady_clock::duration>(frame_period);
            const auto now = std::chrono::steady_clock::now();

            // Robots act on what they see, so a slow camera shows up only as strange behaviour.
            ++rate_frames;
            const double rate_seconds = std::chrono::duration<double>(now - rate_start).count();
            if (rate_seconds >= 5.0) {
                const double fps = rate_frames / rate_seconds;
                if (fps < 0.5 * cfg.fps) {
                    log<NUClear::LogLevel::WARN>("MatchCameras: only", fps, "fps of the", cfg.fps, "asked for");
                }
                rate_start  = now;
                rate_frames = 0;
            }
            if (next_frame < now) {
                next_frame = now;
            }
            std::this_thread::sleep_until(next_frame);
        }

        mjv_freeScene(&scn);
        mjr_freeContext(&con);
    }

}  // namespace k1sim::module
