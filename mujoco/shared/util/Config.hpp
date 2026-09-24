#ifndef K1SIM_SHARED_UTIL_CONFIG_HPP
#define K1SIM_SHARED_UTIL_CONFIG_HPP

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <vector>
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

    // A --game from simulation.yaml's `games`: the field it is played on, and a spawn (x, y, yaw)
    // per robot, main robot first. The yaml lists one team's (x, y) kickoff positions, facing +x;
    // the other team is the point-mirror through the centre spot, facing -x.
    struct Game {
        std::string field;
        std::vector<std::array<double, 3>> spawns;
    };

    // Exits listing the known games if there is no such game.
    inline Game game(const YAML::Node& sim_cfg, const std::string& name) {
        const YAML::Node games = sim_cfg["games"];
        if (!games[name]) {
            std::string known;
            for (const auto& entry : games) {
                known += " " + entry.first.as<std::string>();
            }
            std::fprintf(stderr, "unknown game '%s' (known:%s)\n", name.c_str(), known.c_str());
            std::exit(1);
        }
        Game g;
        g.field = games[name]["field"].as<std::string>();
        std::vector<std::array<double, 2>> team;
        for (const auto& p : games[name]["team"]) {
            team.push_back({p[0].as<double>(), p[1].as<double>()});
        }
        for (const auto& p : team) {
            g.spawns.push_back({p[0], p[1], 0.0});
        }
        for (const auto& p : team) {
            g.spawns.push_back({-p[0], -p[1], M_PI});
        }
        return g;
    }

}  // namespace k1sim::config

#endif  // K1SIM_SHARED_UTIL_CONFIG_HPP
