#ifndef K1SIM_MODULE_REFEREE_GAMECONTROLLERLIBRARY_HPP
#define K1SIM_MODULE_REFEREE_GAMECONTROLLERLIBRARY_HPP

#include <array>
#include <cstdint>
#include <dlfcn.h>
#include <map>
#include <stdexcept>
#include <string>
#include <yaml-cpp/yaml.h>

#include "shared/message/SimMessages.hpp"

// The RoboCup GameController's C API (libgame_controller_api, built from its source into the docker image), as
// module::Referee uses it.
//
// Why it's loaded at run time (dlopen/dlsym) instead of linked: module::Referee is part of every role, sim/soccer
// included, and has to build and start on machines without the library (a native macOS build, or an image from
// before the GameController was added). module::MatchReferee, which only sim/match uses, links the library and
// includes GameController.h instead, and is skipped where they don't exist.
//
// What dlsym gives us: dlopen maps the shared library into the process, and dlsym looks a function up in it by its
// exported (C, unmangled) name and returns its address, which is cast to a function pointer of the right signature.
// Nothing checks that signature: the declarations below must match GameController.h of the commit the docker image
// builds (GC_COMMIT in docker/Dockerfile), or the calls are undefined behaviour. A missing function is caught, as
// load() fails naming it.
namespace k1sim::module::referee {

    // Mirrors of the plain C enums and structs in GameController.h, for the same reason the functions are looked up:
    // the header may not exist where this builds.
    enum Side : int { HOME = 0, AWAY = 1 };
    enum SideMapping : int { HOME_DEFENDS_LEFT_GOAL = 0, HOME_DEFENDS_RIGHT_GOAL = 1 };
    enum ActionSource : int { SOURCE_NETWORK = 0, SOURCE_TIMER = 1, SOURCE_USER = 2 };
    enum SetPlay : int {
        NO_SET_PLAY,
        KICK_OFF,
        DIRECT_FREE_KICK,
        INDIRECT_FREE_KICK,
        PENALTY_KICK,
        THROW_IN,
        GOAL_KICK,
        CORNER_KICK
    };
    struct TeamParams {
        std::uint8_t number;
        int field_player_colour;
        int goalkeeper_colour;
    };
    struct TestParams {
        bool no_delay;
        bool penalty_shootout;
    };

    // Names the referee panel and config/referee.yaml use, in the order of the GameController's enums.
    inline const std::array<const char*, 10> COLOURS =
        {"blue", "red", "yellow", "black", "white", "green", "orange", "purple", "brown", "gray"};
    inline const std::array<const char*, 12> PENALTY_CALLS = {"illegal_position",
                                                              "motion_in_set",
                                                              "motion_in_stop",
                                                              "local_game_stuck",
                                                              "incapable_robot",
                                                              "request_for_pick_up",
                                                              "ball_holding",
                                                              "leaving_the_field",
                                                              "playing_with_arms_hands",
                                                              "pushing",
                                                              "caution",
                                                              "send_off"};
    inline const std::map<std::string, SetPlay> SET_PLAYS  = {{"direct_free_kick", DIRECT_FREE_KICK},
                                                              {"indirect_free_kick", INDIRECT_FREE_KICK},
                                                              {"penalty_kick", PENALTY_KICK},
                                                              {"throw_in", THROW_IN},
                                                              {"goal_kick", GOAL_KICK},
                                                              {"corner_kick", CORNER_KICK}};

    // Position of `name` in `names`, or -1.
    template <typename Names>
    int index_of(const Names& names, const std::string& name) {
        for (std::size_t i = 0; i < names.size(); ++i) {
            if (name == names[i]) {
                return static_cast<int>(i);
            }
        }
        return -1;
    }

    // A team from config/referee.yaml (number and jersey colours), with defaults for what it leaves out.
    inline TeamParams load_team(const YAML::Node& node, std::uint8_t number, const char* colour, const char* keeper) {
        TeamParams team{};
        team.number              = static_cast<std::uint8_t>(node["number"].as<int>(number));
        team.field_player_colour = index_of(COLOURS, node["field_player_colour"].as<std::string>(colour));
        team.goalkeeper_colour   = index_of(COLOURS, node["goalkeeper_colour"].as<std::string>(keeper));
        if (team.field_player_colour < 0 || team.goalkeeper_colour < 0) {
            throw std::runtime_error("config/referee.yaml: unknown jersey colour");
        }
        return team;
    }

    // The library, loaded, with a function pointer per C function used. Every pointer is null until load() succeeds.
    //
    // Ownership follows the C API: the game (gc_new), params (gc_*params_new) and actions (gc_action_*) are allocated
    // by the library and must go back to it (gc_destroy, gc_*params_destroy, gc_action_destroy), never to free/delete.
    // gc_apply takes ownership of the action it is given, whether it applies it or not.
    struct GameControllerLibrary {
        void* handle = nullptr;  // from dlopen; dlclose'd on destruction, after which every pointer below dangles

        void* (*competition_params_new)(const std::uint8_t*, std::uintptr_t)                                  = nullptr;
        void (*competition_params_destroy)(void*)                                                             = nullptr;
        void* (*params_new)(const void*, const TeamParams*, const TeamParams*, Side, SideMapping, TestParams) = nullptr;
        void (*params_destroy)(void*)                                                                         = nullptr;
        void* (*create)(const void*)                                                                          = nullptr;
        void (*destroy)(void*)                                                                                = nullptr;
        void (*seek)(void*, std::uint64_t)                                                                    = nullptr;
        bool (*apply)(void*, void*, ActionSource)                                                             = nullptr;
        bool (*is_legal)(void*, const void*)                                                                  = nullptr;
        void (*action_destroy)(void*)                                                                         = nullptr;
        bool (*read)(void*, std::uint8_t, bool, std::uint8_t*, std::uintptr_t)                                = nullptr;
        const char* (*last_error)()                                                                           = nullptr;

        void* (*finish_half)()                         = nullptr;
        void* (*finish_set_play)()                     = nullptr;
        void* (*free_set_play)()                       = nullptr;
        void* (*global_game_stuck)()                   = nullptr;
        void* (*goal)(Side)                            = nullptr;
        void* (*penalize)(Side, std::uint8_t, int)     = nullptr;
        void* (*start_set_play)(const Side*, SetPlay)  = nullptr;
        void* (*stop_play)(bool)                       = nullptr;
        void* (*switch_half)()                         = nullptr;
        void* (*timeout)(const Side*)                  = nullptr;
        void* (*undo)(std::uint32_t)                   = nullptr;
        void* (*unpenalize)(Side, std::uint8_t, bool)  = nullptr;
        void* (*wait_for_set_play)()                   = nullptr;
        void* (*select_goalkeeper)(Side, std::uint8_t) = nullptr;

        GameControllerLibrary()                                        = default;
        GameControllerLibrary(const GameControllerLibrary&)            = delete;
        GameControllerLibrary& operator=(const GameControllerLibrary&) = delete;
        ~GameControllerLibrary() {
            if (handle != nullptr) {
                ::dlclose(handle);
            }
        }

        // Opens the library at `path` and looks up every function. False, with the reason in `error`, if the file
        // can't be loaded or lacks one of them (a GameController commit other than the one these mirror).
        bool load(const std::string& path, std::string& error) {
            // RTLD_NOW resolves everything up front, so a broken library fails here rather than mid-game;
            // RTLD_LOCAL keeps its symbols out of the way of the rest of the process.
            handle = ::dlopen(path.c_str(), RTLD_NOW | RTLD_LOCAL);
            if (handle == nullptr) {
                error = std::string("cannot load ") + path + " (" + ::dlerror() + ")";
                return false;
            }
            return bind(competition_params_new, "gc_competition_params_new", error)
                   && bind(competition_params_destroy, "gc_competition_params_destroy", error)
                   && bind(params_new, "gc_params_new", error) && bind(params_destroy, "gc_params_destroy", error)
                   && bind(create, "gc_new", error) && bind(destroy, "gc_destroy", error)
                   && bind(seek, "gc_seek", error) && bind(apply, "gc_apply", error)
                   && bind(is_legal, "gc_is_legal", error) && bind(action_destroy, "gc_action_destroy", error)
                   && bind(read, "gc_read", error) && bind(last_error, "gc_last_error", error)
                   && bind(finish_half, "gc_action_finish_half", error)
                   && bind(finish_set_play, "gc_action_finish_set_play", error)
                   && bind(free_set_play, "gc_action_free_set_play", error)
                   && bind(global_game_stuck, "gc_action_global_game_stuck", error)
                   && bind(goal, "gc_action_goal", error) && bind(penalize, "gc_action_penalize", error)
                   && bind(start_set_play, "gc_action_start_set_play", error)
                   && bind(stop_play, "gc_action_stop_play", error) && bind(switch_half, "gc_action_switch_half", error)
                   && bind(timeout, "gc_action_timeout", error) && bind(undo, "gc_action_undo", error)
                   && bind(unpenalize, "gc_action_unpenalize", error)
                   && bind(wait_for_set_play, "gc_action_wait_for_set_play", error)
                   && bind(select_goalkeeper, "gc_action_select_goalkeeper", error);
        }

        // The action a referee panel command stands for (the caller owns it), or nullptr if the command is unknown
        // or incomplete.
        void* action(const message::RefereeCommand& c) const {
            const Side side     = c.side == "away" ? AWAY : HOME;
            const bool has_side = c.side == "home" || c.side == "away";
            const auto player   = static_cast<std::uint8_t>(c.player);
            if (c.action == "kickoff" && has_side)
                return start_set_play(&side, KICK_OFF);
            if (c.action == "setplay" && has_side && SET_PLAYS.count(c.arg) != 0)
                return start_set_play(&side, SET_PLAYS.at(c.arg));
            if (c.action == "set")
                return wait_for_set_play();
            if (c.action == "play")
                return free_set_play();
            if (c.action == "finish_set_play")
                return finish_set_play();
            if (c.action == "stop")
                return stop_play(false);
            if (c.action == "resume")
                return stop_play(true);
            if (c.action == "goal" && has_side)
                return goal(side);
            if (c.action == "finish_half")
                return finish_half();
            if (c.action == "switch_half")
                return switch_half();
            if (c.action == "timeout")
                return timeout(has_side ? &side : nullptr);
            if (c.action == "stuck")
                return global_game_stuck();
            if (c.action == "undo")
                return undo(1);
            if (c.action == "penalize" && has_side && index_of(PENALTY_CALLS, c.arg) >= 0) {
                return penalize(side, player, index_of(PENALTY_CALLS, c.arg));
            }
            if (c.action == "unpenalize" && has_side)
                return unpenalize(side, player, c.arg == "force");
            return nullptr;
        }

    private:
        // Looks `name` up with dlsym and stores it in `function`, cast to that pointer's type.
        template <typename Function>
        bool bind(Function& function, const char* name, std::string& error) {
            function = reinterpret_cast<Function>(::dlsym(handle, name));
            if (function == nullptr) {
                error = std::string("the GameController library has no function ") + name;
            }
            return function != nullptr;
        }
    };

}  // namespace k1sim::module::referee

#endif  // K1SIM_MODULE_REFEREE_GAMECONTROLLERLIBRARY_HPP
