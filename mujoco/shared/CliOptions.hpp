#ifndef K1SIM_SHARED_CLIOPTIONS_HPP
#define K1SIM_SHARED_CLIOPTIONS_HPP

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

namespace k1sim {

struct CliOptions {
    bool headless = false;
    std::string model;       // override for simulation.yaml model path
    std::string config_dir;  // override for the config directory
    std::string keyframe;    // override for the startup keyframe (default "ready")
    double rtf = -1.0;       // override real-time factor; <0 = use config (0 = free-run)
    int robots = 1;          // total K1s on the field; extras are PD-held at "ready"
    bool viser     = false;  // serve the browser viewer (module::ViserViewer) instead of the window
    int viser_port = 8080;   // the browser viewer's HTTP port
};

inline constexpr int MAX_ROBOTS = 20;

// Set once in main() before the PowerPlant starts; read-only afterwards.
inline CliOptions& cli() {
    static CliOptions options;
    return options;
}

inline CliOptions parse_cli(int argc, char** argv) {
    CliOptions opts;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto value            = [&](const char* flag) -> std::string {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "%s requires a value\n", flag);
                std::exit(1);
            }
            return argv[++i];
        };
        if (arg == "--headless") {
            opts.headless = true;
        }
        else if (arg == "--model") {
            opts.model = value("--model");
        }
        else if (arg == "--config-dir") {
            opts.config_dir = value("--config-dir");
        }
        else if (arg == "--keyframe") {
            opts.keyframe = value("--keyframe");
        }
        else if (arg == "--rtf") {
            opts.rtf = std::stod(value("--rtf"));
        }
        else if (arg == "--robots") {
            opts.robots = std::stoi(value("--robots"));
            if (opts.robots < 1 || opts.robots > MAX_ROBOTS) {
                std::fprintf(stderr, "--robots must be between 1 and %d\n", MAX_ROBOTS);
                std::exit(1);
            }
        }
        else if (arg == "--viser") {
            opts.viser = true;
        }
        else if (arg == "--viser-port") {
            opts.viser      = true;
            opts.viser_port = std::stoi(value("--viser-port"));
            if (opts.viser_port < 1 || opts.viser_port > 65535) {
                std::fprintf(stderr, "--viser-port must be between 1 and 65535\n");
                std::exit(1);
            }
        }
        else if (arg == "--help" || arg == "-h") {
            std::printf("k1_mujoco_sim — MuJoCo simulator for the Booster K1 (Booster SDK DDS surface)\n"
                        "  --headless          run without the viewer window\n"
                        "  --model <path>      MJCF scene to load (default: config simulation.yaml)\n"
                        "  --config-dir <dir>  config directory (default: mujoco/config)\n"
                        "  --keyframe <name>   startup keyframe (default: ready; e.g. lying_front)\n"
                        "  --rtf <factor>      real-time factor; 0 = free-run\n"
                        "  --robots <n>        total K1s on the field, 1-20 (default 1); extra robots\n"
                        "                      are uncontrolled and PD-held at the ready pose\n"
                        "  --viser             view the sim in a browser (viser) instead of the window\n"
                        "  --viser-port <port> the browser viewer's port (default 8080; implies --viser)\n");
            std::exit(0);
        }
        else {
            std::fprintf(stderr, "unknown argument '%s' (see --help)\n", arg.c_str());
            std::exit(1);
        }
    }
    return opts;
}

}  // namespace k1sim

#endif  // K1SIM_SHARED_CLIOPTIONS_HPP
