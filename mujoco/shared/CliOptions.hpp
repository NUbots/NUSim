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
    double rtf = -1.0;       // override real-time factor; <0 = use config (0 = free-run)
};

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
        else if (arg == "--rtf") {
            opts.rtf = std::stod(value("--rtf"));
        }
        else if (arg == "--help" || arg == "-h") {
            std::printf("k1_mujoco_sim — MuJoCo simulator for the Booster K1 (Booster SDK DDS surface)\n"
                        "  --headless          run without the viewer window\n"
                        "  --model <path>      MJCF scene to load (default: config simulation.yaml)\n"
                        "  --config-dir <dir>  config directory (default: mujoco/config)\n"
                        "  --rtf <factor>      real-time factor; 0 = free-run\n");
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
