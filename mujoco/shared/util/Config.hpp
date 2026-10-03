#ifndef K1SIM_SHARED_UTIL_CONFIG_HPP
#define K1SIM_SHARED_UTIL_CONFIG_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
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

    // Spreads `free` robots evenly through one half of the field (x in [-length/2, 0]), around the
    // `fixed` robots and outside the centre circle, by Lloyd relaxation: the half is sampled on a
    // grid, each sample belongs to its nearest robot, and each free robot moves to the centroid of
    // its samples. Seeded by farthest-point picks, so the result is deterministic.
    inline std::vector<std::array<double, 2>> spread_in_half(const std::vector<std::array<double, 2>>& fixed,
                                                             int free,
                                                             double length,
                                                             double width,
                                                             double circle_radius) {
        constexpr double STEP = 0.1;
        std::vector<std::array<double, 2>> samples;
        for (double x = -length / 2 + STEP / 2; x < 0.0; x += STEP) {
            for (double y = -width / 2 + STEP / 2; y < width / 2; y += STEP) {
                if (std::hypot(x, y) > circle_radius) {
                    samples.push_back({x, y});
                }
            }
        }

        std::vector<std::array<double, 2>> robots = fixed;
        auto nearest                              = [&](const std::array<double, 2>& p, std::size_t& index) {
            double best = INFINITY;
            for (std::size_t r = 0; r < robots.size(); ++r) {
                const double d = std::hypot(p[0] - robots[r][0], p[1] - robots[r][1]);
                if (d < best) {
                    best  = d;
                    index = r;
                }
            }
            return best;
        };

        std::size_t index = 0;
        for (int i = 0; i < free; ++i) {
            // The sample farthest from every robot so far (the first sample if there are none)
            std::array<double, 2> pick = samples.front();
            double farthest            = -1.0;
            for (const auto& p : samples) {
                const double d = nearest(p, index);
                if (d > farthest) {
                    farthest = d;
                    pick     = p;
                }
            }
            robots.push_back(pick);
        }

        for (int iter = 0; iter < 100; ++iter) {
            std::vector<std::array<double, 3>> sums(robots.size(), {0.0, 0.0, 0.0});
            for (const auto& p : samples) {
                nearest(p, index);
                sums[index][0] += p[0];
                sums[index][1] += p[1];
                sums[index][2] += 1.0;
            }
            for (std::size_t r = fixed.size(); r < robots.size(); ++r) {
                if (sums[r][2] > 0.0) {
                    robots[r] = {sums[r][0] / sums[r][2], sums[r][1] / sums[r][2]};
                }
            }
        }
        return {robots.begin() + static_cast<std::ptrdiff_t>(fixed.size()), robots.end()};
    }

    // A --game <n> from simulation.yaml's `game`: the field it is played on, and a spawn (x, y, yaw)
    // per robot, main robot first. One team's n positions are built in its own -x half, either in
    // kickoff positions facing +x (on_field) or lined up off the touchlines facing into the field;
    // the other team is the point-mirror through the centre spot, facing the other way.
    struct Game {
        std::string field;
        std::vector<std::array<double, 3>> spawns;
    };

    inline Game game(const YAML::Node& sim_cfg, int n, bool on_field) {
        const YAML::Node cfg = sim_cfg["game"];
        auto xy = [](const YAML::Node& p) { return std::array<double, 2>{p[0].as<double>(), p[1].as<double>()}; };
        const double length = cfg["length"].as<double>();
        const double width  = cfg["width"].as<double>();

        std::vector<std::array<double, 3>> team;
        if (on_field) {
            std::vector<std::array<double, 2>> fixed{xy(cfg["attacker"]), xy(cfg["goalkeeper"])};
            for (const auto& w : cfg["wings"]) {
                fixed.push_back(xy(w));
            }
            fixed.resize(std::min<std::size_t>(fixed.size(), n));
            for (const auto& p : fixed) {
                team.push_back({p[0], p[1], 0.0});
            }
            const int rest = n - static_cast<int>(fixed.size());
            for (const auto& p : spread_in_half(fixed, rest, length, width, cfg["centre_circle_radius"].as<double>())) {
                team.push_back({p[0], p[1], 0.0});
            }
        }
        else {
            // Robots 0-1 on the -y side, 2-3 on the +y side, 4-5 on the -y side, ...
            const double first_x = -length / 2 + cfg["penalty_mark_distance"].as<double>();
            const double side_y  = width / 2 + cfg["sideline_offset"].as<double>();
            const double spacing = cfg["sideline_spacing"].as<double>();
            for (int i = 0; i < n; ++i) {
                const double side = (i / 2) % 2 == 0 ? -1.0 : 1.0;
                const int slot    = (i / 4) * 2 + i % 2;  // place along this side, from the penalty mark
                team.push_back({first_x + slot * spacing, side * side_y, -side * M_PI / 2.0});
            }
        }

        Game g;
        g.field  = cfg["field"].as<std::string>();
        g.spawns = team;
        for (const auto& p : team) {
            g.spawns.push_back({-p[0], -p[1], p[2] + M_PI});
        }
        return g;
    }

}  // namespace k1sim::config

#endif  // K1SIM_SHARED_UTIL_CONFIG_HPP
