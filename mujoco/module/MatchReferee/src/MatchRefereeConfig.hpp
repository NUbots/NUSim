#ifndef K1SIM_MODULE_MATCHREFEREE_MATCHREFEREECONFIG_HPP
#define K1SIM_MODULE_MATCHREFEREE_MATCHREFEREECONFIG_HPP

#include <GameController.h>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include <yaml-cpp/yaml.h>

// config/match.yaml, parsed. Header-only and free of NUClear, like SupervisorConfig.hpp.
namespace k1sim::module::matchreferee {

    struct MatchConfig {
        std::string competition;
        RoboCup::Side kick_off_side    = RoboCup::Side::home;
        double auto_start_after        = -1.0;
        bool auto_start_when_connected = true;
        bool skip_kick_off_phases      = true;
        bool hold_robots               = true;
        int penalty_seconds            = 10;

        std::string broadcast_address;
        std::string interface;
        int control_port        = 3838;
        int status_port         = 3939;
        double control_interval = 0.5;
        double status_timeout   = 4.0;

        RoboCup::AutorefereeFeatures features{0};
        double time_until_switch_to_set     = 5.0;
        double time_until_switch_to_playing = 5.0;
        float penalty_placement_distance    = 0.6F;
        float player_radius                 = 0.2F;
        double stand_height                 = 0.555;

        RoboCup::AutorefereeFieldDimensions field{};
    };

    inline RoboCup::AutorefereeFeatures feature_from_name(const std::string& name) {
        using namespace RoboCup;
        const std::vector<std::pair<std::string, AutorefereeFeatures>> features = {
            {"switch_to_set", AutorefereeFeatures_SWITCH_TO_SET},
            {"switch_to_playing", AutorefereeFeatures_SWITCH_TO_PLAYING},
            {"switch_to_finished", AutorefereeFeatures_SWITCH_TO_FINISHED},
            {"ball_out", AutorefereeFeatures_BALL_OUT},
            {"invalid_goal", AutorefereeFeatures_INVALID_GOAL},
            {"ball_free", AutorefereeFeatures_BALL_FREE},
            {"double_touch", AutorefereeFeatures_DOUBLE_TOUCH},
            {"game_stuck", AutorefereeFeatures_GAME_STUCK},
            {"clear_ball", AutorefereeFeatures_CLEAR_BALL},
            {"penalize_leaving_the_field", AutorefereeFeatures_PENALIZE_LEAVING_THE_FIELD},
            {"penalize_illegal_positioning", AutorefereeFeatures_PENALIZE_ILLEGAL_POSITIONING},
            {"penalize_motion_in_set", AutorefereeFeatures_PENALIZE_MOTION_IN_SET},
            {"penalize_motion_in_stop", AutorefereeFeatures_PENALIZE_MOTION_IN_STOP},
            {"unpenalize", AutorefereeFeatures_UNPENALIZE},
            {"place_for_penalty", AutorefereeFeatures_PLACE_FOR_PENALTY},
            {"place_for_penalty_shoot_out", AutorefereeFeatures_PLACE_FOR_PENALTY_SHOOT_OUT},
            {"place_ball", AutorefereeFeatures_PLACE_BALL},
        };
        for (const auto& [known, feature] : features) {
            if (known == name) {
                return feature;
            }
        }
        throw std::runtime_error("match.yaml: unknown autoreferee feature '" + name + "'");
    }

    // A competition's params.yaml with every penalty lasting `seconds`, without the increase for
    // repeat offences. Negative leaves the competition's times alone.
    inline std::string with_penalty_time(const std::string& competition_yaml, int seconds) {
        if (seconds < 0) {
            return competition_yaml;
        }
        YAML::Node competition = YAML::Load(competition_yaml);
        for (auto penalty : competition["penalties"]) {
            if (penalty.second["duration"]["secs"].as<int>() > 0) {
                penalty.second["duration"]["secs"] = seconds;
                penalty.second["incremental"]      = false;
            }
        }
        competition["penaltyDurationIncrement"]["secs"] = 0;
        return YAML::Dump(competition);
    }

    inline MatchConfig load_config(const YAML::Node& yaml) {
        MatchConfig cfg;
        cfg.competition = yaml["competition"].as<std::string>();

        const auto kick_off_side = yaml["kick_off_side"].as<std::string>();
        if (kick_off_side != "home" && kick_off_side != "away") {
            throw std::runtime_error("match.yaml: kick_off_side must be home or away");
        }
        cfg.kick_off_side             = kick_off_side == "home" ? RoboCup::Side::home : RoboCup::Side::away;
        cfg.auto_start_after          = yaml["auto_start_after"].as<double>();
        cfg.auto_start_when_connected = yaml["auto_start_when_connected"].as<bool>();
        cfg.skip_kick_off_phases      = yaml["skip_kick_off_phases"].as<bool>();
        cfg.hold_robots               = yaml["hold_robots"].as<bool>();
        cfg.penalty_seconds           = yaml["penalty_seconds"].as<int>();

        const auto network    = yaml["network"];
        cfg.broadcast_address = network["broadcast_address"].as<std::string>();
        cfg.interface         = network["interface"].as<std::string>("");
        cfg.control_port      = network["control_port"].as<int>();
        cfg.status_port       = network["status_port"].as<int>();
        cfg.control_interval  = network["control_interval"].as<double>();
        cfg.status_timeout    = network["status_timeout"].as<double>();

        const auto referee = yaml["autoreferee"];
        for (const auto& feature : referee["features"]) {
            cfg.features |= feature_from_name(feature.as<std::string>());
        }
        cfg.time_until_switch_to_set     = referee["time_until_switch_to_set"].as<double>();
        cfg.time_until_switch_to_playing = referee["time_until_switch_to_playing"].as<double>();
        cfg.penalty_placement_distance   = referee["penalty_placement_distance"].as<float>();
        cfg.player_radius                = referee["player_radius"].as<float>();
        cfg.stand_height                 = referee["stand_height"].as<double>();

        const auto field              = yaml["field"];
        cfg.field.length              = field["length"].as<float>();
        cfg.field.width               = field["width"].as<float>();
        cfg.field.goalWidth           = field["goal_width"].as<float>();
        cfg.field.goalHeight          = field["goal_height"].as<float>();
        cfg.field.goalAreaLength      = field["goal_area_length"].as<float>();
        cfg.field.goalAreaWidth       = field["goal_area_width"].as<float>();
        cfg.field.penaltyAreaLength   = field["penalty_area_length"].as<float>();
        cfg.field.penaltyAreaWidth    = field["penalty_area_width"].as<float>();
        cfg.field.penaltyMarkDistance = field["penalty_mark_distance"].as<float>();
        cfg.field.centerCircleRadius  = field["center_circle_radius"].as<float>();
        cfg.field.borderStripWidth    = field["border_strip_width"].as<float>();
        cfg.field.markRadius          = field["mark_radius"].as<float>();
        cfg.field.lineWidth           = field["line_width"].as<float>();
        cfg.field.ballRadius          = field["ball_radius"].as<float>();
        return cfg;
    }

}  // namespace k1sim::module::matchreferee

#endif  // K1SIM_MODULE_MATCHREFEREE_MATCHREFEREECONFIG_HPP
