// The macOS camera must render on its worker thread, including under --headless,
// without initialising GLFW/Cocoa. Exercise native rendering and shared-memory publication.
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <exception>
#include <memory>
#include <mujoco/mujoco.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include "module/Camera/src/OffscreenContext.hpp"
#include "module/Camera/src/SharedImageWriter.hpp"

int main() {
    using namespace k1sim::module::camera;
    int result = 1;
    std::thread worker([&] {
        try {
            // Recreate the context to catch lifetime/cleanup problems as well as initialisation.
            for (int attempt = 0; attempt < 2; ++attempt) {
                OffscreenContext context(320, 240);
                if (!context.valid()) {
                    std::fprintf(stderr, "Could not create an offscreen context on a worker thread\n");
                    return;
                }
                char error[1024]{};
                std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
                    mj_loadXML(K1SIM_SOURCE_DIR "/models/k1/k1_scene_robocup_middle.xml",
                               nullptr,
                               error,
                               sizeof(error)),
                    mj_deleteModel);
                if (!model) {
                    std::fprintf(stderr, "Could not load model: %s\n", error);
                    return;
                }
                std::unique_ptr<mjData, decltype(&mj_deleteData)> data(mj_makeData(model.get()), mj_deleteData);
                mj_resetDataKeyframe(model.get(), data.get(), mj_name2id(model.get(), mjOBJ_KEY, "ready"));
                mj_forward(model.get(), data.get());

                mjvCamera camera;
                mjv_defaultCamera(&camera);
                mjv_defaultFreeCamera(model.get(), &camera);
                mjvOption options;
                mjv_defaultOption(&options);
                mjvScene scene;
                mjv_defaultScene(&scene);
                mjv_makeScene(model.get(), &scene, 2000);
                mjrContext renderer;
                mjr_defaultContext(&renderer);
                mjr_makeContext(model.get(), &renderer, mjFONTSCALE_100);
                mjr_setBuffer(mjFB_OFFSCREEN, &renderer);
                mjv_updateScene(model.get(), data.get(), &options, nullptr, &camera, mjCAT_ALL, &scene);
                const mjrRect viewport{0, 0, 320, 240};
                std::vector<uint8_t> pixels(320 * 240 * 3);
                mjr_render(viewport, &scene, &renderer);
                mjr_readPixels(pixels.data(), nullptr, viewport, &renderer);
                mjr_freeContext(&renderer);
                mjv_freeScene(&scene);

                if (std::none_of(pixels.begin(), pixels.end(), [](uint8_t value) { return value != 0; })) {
                    std::fprintf(stderr, "Offscreen rendering produced a black frame\n");
                    return;
                }
                const std::string segment = "_k1sim_test_" + std::to_string(getpid());
                {
                    SharedImageWriter writer(segment, 320, 240);
                    writer.publish(pixels.data(), pixels.size(), 1.0f, 1.0f, 0.0f, 0.0f);
                    bip::shared_memory_object shm(bip::open_only, segment.c_str(), bip::read_only);
                    bip::mapped_region region(shm, bip::read_only);
                    const auto* header = static_cast<const SharedImageHeader*>(region.get_address());
                    if (header->magic != SharedImageHeader::MAGIC || header->version != SharedImageHeader::VERSION
                        || header->seqlock.load() != 2 || header->width != 320 || header->height != 240
                        || header->data_size != pixels.size()
                        || std::memcmp(header + 1, pixels.data(), pixels.size()) != 0) {
                        std::fprintf(stderr, "Shared camera frame does not match the rendered pixels\n");
                        return;
                    }
                }
            }
            result = 0;
        }
        catch (const std::exception& error) {
            std::fprintf(stderr, "Camera context test failed: %s\n", error.what());
        }
    });
    worker.join();
    return result;
}
