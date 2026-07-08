#ifndef K1SIM_SHARED_UTIL_CONFIG_HPP
#define K1SIM_SHARED_UTIL_CONFIG_HPP

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

}  // namespace k1sim::config

#endif  // K1SIM_SHARED_UTIL_CONFIG_HPP
