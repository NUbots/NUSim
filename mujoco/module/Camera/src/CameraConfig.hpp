#ifndef K1SIM_MODULE_CAMERA_CAMERACONFIG_HPP
#define K1SIM_MODULE_CAMERA_CAMERACONFIG_HPP

#include <string>
#include <yaml-cpp/yaml.h>

// config/camera.yaml schema, parsed independently of NUClear -- mirrors
// module::Supervisor's SupervisorConfig.hpp / module::Simulation's
// build_sim_config() (this port doesn't use NUClear's on<Configuration>
// hot-reload extension anywhere; config is read once at Startup).
namespace k1sim::module::camera {

    struct CameraConfig {
        // DDS topic module::SdkBridge publishes each frame on as sensor_msgs Image, with
        // CameraInfo on `<topic>/camera_info`. Must match the entry's `topic:` in NUbots_K1's
        // module/input/K1Camera/data/config/K1Camera.yaml (default below matches that file's
        // "Left Camera" entry). Empty disables.
        std::string topic = "rt/boostercamera/head/rgb";

        // Legacy POSIX shared-memory segment read by NUbots_K1's input::NUSimCamera (the
        // roles/nusim/* roles on jmontano/nusim-local and its descendants). Empty disables.
        std::string segment = "_boostercamera_head_rgb";

        // Head-pose segment K1Sensors reads (K1Sensors.yaml `segment:`). Empty disables.
        std::string pose_segment = "_head_pose";

        // Render resolution. rgb8, so each frame is width*height*3 bytes -- with the
        // shared-memory segment enabled keep this under NUSimCamera's
        // MAX_IMAGE_BYTES (2 MiB); the 640x480 default (921600 bytes) leaves plenty
        // of margin. Must also not exceed the compiled model's <visual><global offwidth/offheight/> (MuJoCo default
        // 640x480, unmodified by any scene in this repo) -- Camera checks this at
        // Startup and refuses to render rather than silently truncating.
        int width  = 640;
        int height = 480;

        double fps = 30.0;

        // Name of the <camera> element the model attaches to the head (see
        // models/k1/K1_22dof.xml's Head_2 body).
        std::string mjcf_camera = "head";
    };

    inline CameraConfig load_config(const YAML::Node& root) {
        CameraConfig cfg;
        cfg.topic        = root["topic"].as<std::string>(cfg.topic);
        cfg.segment      = root["segment"].as<std::string>(cfg.segment);
        cfg.pose_segment = root["pose_segment"].as<std::string>(cfg.pose_segment);
        cfg.width        = root["width"].as<int>(cfg.width);
        cfg.height       = root["height"].as<int>(cfg.height);
        cfg.fps          = root["fps"].as<double>(cfg.fps);
        cfg.mjcf_camera  = root["mjcf_camera"].as<std::string>(cfg.mjcf_camera);
        return cfg;
    }

}  // namespace k1sim::module::camera

#endif  // K1SIM_MODULE_CAMERA_CAMERACONFIG_HPP
