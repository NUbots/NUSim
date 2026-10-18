#ifndef K1SIM_MODULE_CAMERA_CAMERACONFIG_HPP
#define K1SIM_MODULE_CAMERA_CAMERACONFIG_HPP

#include <string>
#include <yaml-cpp/yaml.h>

// config/camera.yaml schema, parsed without NUClear.
namespace k1sim::module::camera {

    struct CameraConfig {
        // POSIX shared-memory segment name. Must match the corresponding entry's
        // `segment:` in NUbots_K1's module/input/K1Camera/data/config/K1Camera.yaml
        // (default below matches that file's "Left Camera" entry).
        std::string segment = "_boostercamera_head_rgb";

        // Head-pose segment K1Sensors reads (K1Sensors.yaml `segment:`). Empty disables.
        std::string pose_segment = "_head_pose";

        // Render resolution. A frame is width*height*3 bytes (rgb8) and must fit K1Camera's
        // MAX_IMAGE_BYTES (2 MiB). It must also fit the model's <visual><global offwidth/offheight/>
        // (MuJoCo default 640x480); Camera checks this at Startup and refuses to render otherwise.
        int width  = 640;
        int height = 480;

        double fps = 30.0;

        // Name of the <camera> element the model attaches to the head (see
        // models/k1/K1_22dof.xml's Head_2 body).
        std::string mjcf_camera = "head";
    };

    inline CameraConfig load_config(const YAML::Node& root) {
        CameraConfig cfg;
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
