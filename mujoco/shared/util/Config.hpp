#ifndef K1SIM_SHARED_UTIL_CONFIG_HPP
#define K1SIM_SHARED_UTIL_CONFIG_HPP

#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <yaml-cpp/yaml.h>

#include "shared/CliOptions.hpp"

namespace k1sim::config {

// Config directory resolution order: --config-dir, $K1SIM_CONFIG_DIR, <source>/config.
inline std::filesystem::path config_dir() {
    if (!cli().config_dir.empty()) {
        return cli().config_dir;
    }
    if (const char* env = std::getenv("K1SIM_CONFIG_DIR")) {
        return env;
    }
    return std::filesystem::path(K1SIM_SOURCE_DIR) / "config";
}

inline YAML::Node load(const std::string& filename) {
    return YAML::LoadFile((config_dir() / filename).string());
}

// Model/asset paths in configs are relative to the mujoco/ source root.
inline std::filesystem::path resolve_path(const std::string& path) {
    std::filesystem::path p(path);
    return p.is_absolute() ? p : std::filesystem::path(K1SIM_SOURCE_DIR) / p;
}

// The scene simulation.yaml lists under `fields` for the named field, or for its default `field`
// when none is named. Exits listing the known fields if there is no such field.
inline std::string field_scene(const YAML::Node& sim_cfg, std::string field = "") {
    if (field.empty()) {
        field = sim_cfg["field"].as<std::string>();
    }
    const YAML::Node fields = sim_cfg["fields"];
    if (!fields[field]) {
        std::string known;
        for (const auto& entry : fields) {
            known += " " + entry.first.as<std::string>();
        }
        std::fprintf(stderr, "unknown field '%s' (known:%s)\n", field.c_str(), known.c_str());
        std::exit(1);
    }
    return fields[field].as<std::string>();
}

}  // namespace k1sim::config

#endif  // K1SIM_SHARED_UTIL_CONFIG_HPP
