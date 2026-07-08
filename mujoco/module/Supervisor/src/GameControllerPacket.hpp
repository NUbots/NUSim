#ifndef K1SIM_MODULE_SUPERVISOR_GAMECONTROLLERPACKET_HPP
#define K1SIM_MODULE_SUPERVISOR_GAMECONTROLLERPACKET_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

// RoboCup GameController UDP broadcast packet (receive side only — this sim
// never sends a reply; NUbots' own module::input::GameController does that
// independently over its own socket).
//
// Provenance: field layout, enum values and struct order are ported from
// NUbots_K1's module/input/GameController/src/GameControllerData.hpp
// (MIT License, Copyright (c) 2014 NUbots — https://github.com/NUbots/NUbots),
// which itself mirrors the official RoboCup GameController network protocol
// (RoboCupGameControlData.h, struct RoboCupGameControlData, version 20).
// Trimmed here to the receive-only subset (drops RETURN_HEADER/RETURN_VERSION
// and GameControllerReplyPacket, which only the reply side needs) and
// re-namespaced under k1sim so this module has no build/source dependency on
// the separate NUbots_K1 repo. #pragma pack(1) and every field's type/order
// below must stay byte-for-byte identical to the wire format for
// reinterpret_cast parsing to work — see try_parse() at the bottom.
namespace k1sim::module::supervisor::gc {

inline constexpr std::size_t MAX_NUM_PLAYERS = 20;
inline constexpr std::array<char, 4> RECEIVE_HEADER = {'R', 'G', 'm', 'e'};
inline constexpr std::uint8_t SUPPORTED_VERSION      = 20;

#pragma pack(push, 1)

enum class State : std::uint8_t { INITIAL = 0, READY = 1, SET = 2, PLAYING = 3, FINISHED = 4 };

enum class GamePhase : std::uint8_t {
    NORMAL           = 0,
    PENALTY_SHOOTOUT = 1,
    EXTRA_TIME       = 2,
    TIMEOUT          = 3,
};

enum class SetPlay : std::uint8_t {
    NONE               = 0,
    DIRECT_FREE_KICK   = 1,
    INDIRECT_FREE_KICK = 2,
    PENALTY_KICK       = 3,
    THROW_IN           = 4,
    GOAL_KICK          = 5,
    CORNER_KICK        = 6,
};

enum class TeamColour : std::uint8_t {
    BLUE   = 0,
    RED    = 1,
    YELLOW = 2,
    BLACK  = 3,
    WHITE  = 4,
    GREEN  = 5,
    ORANGE = 6,
    PURPLE = 7,
    BROWN  = 8,
    GRAY   = 9,
};

// Per-player penalty state. UNPENALISED is the only "in play" value; every
// other value means the player is currently sitting out for some reason.
enum class PenaltyState : std::uint8_t {
    UNPENALISED             = 0,
    ILLEGAL_POSITIONING     = 1,
    MOTION_IN_SET           = 2,
    MOTION_IN_STOP          = 3,
    LOCAL_GAME_STUCK        = 4,
    INCAPABLE_ROBOT         = 5,
    PICK_UP                 = 6,
    BALL_HOLDING            = 7,
    LEAVING_THE_FIELD       = 8,
    PLAYING_WITH_ARMS_HANDS = 9,
    PLAYER_PUSHING          = 10,
    CAUTIONED               = 11,
    SENT_OFF                = 12,
    SUBSTITUTE              = 13,
};

struct Robot {
    PenaltyState penalty_state;   // penalty state of the player
    std::uint8_t penalised_time_left;  // estimate of time till unpenalised (seconds)
    std::uint8_t cautions;             // number of cautions (yellow cards)
};

struct Team {
    std::uint8_t team_id;                // unique team number
    TeamColour field_player_colour;      // colour of the field players
    TeamColour goalkeeper_colour;        // colour of the goalkeeper
    std::uint8_t goalkeeper;             // player number of the goalkeeper (0-MAX_NUM_PLAYERS)
    std::uint8_t score;                  // team's score
    std::uint8_t penalty_shot;           // penalty shot counter
    std::uint16_t single_shots;          // bits represent penalty shot success
    std::uint16_t message_budget;        // remaining team message budget
    std::array<Robot, MAX_NUM_PLAYERS> players;
};

struct GameControllerPacket {
    std::array<char, 4> header;  // must equal RECEIVE_HEADER ('R','G','m','e')
    std::uint8_t version;        // must equal SUPPORTED_VERSION (20)
    std::uint8_t packet_number;
    std::uint8_t players_per_team;
    std::uint8_t competition_type;
    std::uint8_t stopped;      // 1 = play currently stopped
    GamePhase game_phase;
    State state;
    SetPlay set_play;
    bool first_half;           // true = first half
    std::uint8_t kicking_team;  // team_id of the team with the next kickoff/free kick
    std::int16_t secs_remaining;
    std::int16_t secondary_time;
    std::array<Team, 2> teams;
};

#pragma pack(pop)

inline const char* state_name(State s) {
    switch (s) {
        case State::INITIAL: return "INITIAL";
        case State::READY: return "READY";
        case State::SET: return "SET";
        case State::PLAYING: return "PLAYING";
        case State::FINISHED: return "FINISHED";
        default: return "UNKNOWN";
    }
}

inline const char* penalty_state_name(PenaltyState p) {
    switch (p) {
        case PenaltyState::UNPENALISED: return "UNPENALISED";
        case PenaltyState::ILLEGAL_POSITIONING: return "ILLEGAL_POSITIONING";
        case PenaltyState::MOTION_IN_SET: return "MOTION_IN_SET";
        case PenaltyState::MOTION_IN_STOP: return "MOTION_IN_STOP";
        case PenaltyState::LOCAL_GAME_STUCK: return "LOCAL_GAME_STUCK";
        case PenaltyState::INCAPABLE_ROBOT: return "INCAPABLE_ROBOT";
        case PenaltyState::PICK_UP: return "PICK_UP";
        case PenaltyState::BALL_HOLDING: return "BALL_HOLDING";
        case PenaltyState::LEAVING_THE_FIELD: return "LEAVING_THE_FIELD";
        case PenaltyState::PLAYING_WITH_ARMS_HANDS: return "PLAYING_WITH_ARMS_HANDS";
        case PenaltyState::PLAYER_PUSHING: return "PLAYER_PUSHING";
        case PenaltyState::CAUTIONED: return "CAUTIONED";
        case PenaltyState::SENT_OFF: return "SENT_OFF";
        case PenaltyState::SUBSTITUTE: return "SUBSTITUTE";
        default: return "UNKNOWN";
    }
}

// A State value that never equals any real State — used as the "no packet
// seen yet" sentinel so the very first real packet is always treated as a
// transition (mirrors NUbots' GameController::reset_state(), which seeds
// packet.state = static_cast<State>(-1) for the same reason).
inline constexpr State UNKNOWN_STATE = static_cast<State>(0xFF);

// Parses a raw UDP payload into a GameControllerPacket. Returns false (leaves
// out untouched) if the payload is too short, or its header/version don't
// match — callers should treat that as "not a GameController packet" and
// ignore it silently, not as an error (harmless traffic on the port, or a GC
// version we don't speak, is expected and not exceptional).
inline bool try_parse(const std::uint8_t* data, std::size_t len, GameControllerPacket& out) {
    if (data == nullptr || len < sizeof(GameControllerPacket)) {
        return false;
    }
    GameControllerPacket packet;  // NOLINT — POD, filled by memcpy below
    std::memcpy(&packet, data, sizeof(GameControllerPacket));
    if (packet.header != RECEIVE_HEADER || packet.version != SUPPORTED_VERSION) {
        return false;
    }
    out = packet;
    return true;
}

}  // namespace k1sim::module::supervisor::gc

#endif  // K1SIM_MODULE_SUPERVISOR_GAMECONTROLLERPACKET_HPP
