#include "module/SdkBridge/src/RpcDispatch.hpp"

#include <cctype>
#include <nlohmann/json.hpp>

#include "shared/k1/BoosterApi.hpp"

namespace k1sim::module::sdkbridge {

namespace {

using nlohmann::json;

// RpcReqMsg_.body is sometimes a genuinely empty string (LIE_DOWN, GET_UP, GET_MODE
// requests carry no parameters) — nlohmann::json::parse("") throws, so treat blank
// (or whitespace-only) bodies as an empty object rather than a parse error.
json parse_or_empty(const std::string& text) {
    auto trimmed_empty = [&] {
        for (char c : text) {
            if (!std::isspace(static_cast<unsigned char>(c))) {
                return false;
            }
        }
        return true;
    }();
    if (trimmed_empty) {
        return json::object();
    }
    try {
        return json::parse(text);
    }
    catch (const json::parse_error&) {
        return json::object();
    }
}

}  // namespace

RpcOutcome dispatch_rpc(const std::string& header_json,
                        const std::string& body_json,
                        int current_mode,
                        int64_t unknown_api_status) {
    RpcOutcome outcome;

    const json header = parse_or_empty(header_json);
    if (!header.contains("api_id")) {
        outcome.unknown_api_id = true;
        outcome.status         = unknown_api_status;
        return outcome;
    }
    const int64_t api_id = header["api_id"].get<int64_t>();
    outcome.api_id        = api_id;

    const json body = parse_or_empty(body_json);
    auto get_double  = [&](const char* key, double fallback = 0.0) {
        return body.contains(key) ? body[key].get<double>() : fallback;
    };
    auto get_int = [&](const char* key, int fallback = 0) {
        return body.contains(key) ? body[key].get<int>() : fallback;
    };
    auto get_bool = [&](const char* key, bool fallback = true) {
        return body.contains(key) ? body[key].get<bool>() : fallback;
    };

    switch (api_id) {
        case booster::CHANGE_MODE: {
            outcome.action.kind             = RpcActionKind::MODE_CHANGE;
            outcome.action.mode_change.mode = get_int("mode", booster::UNKNOWN);
            break;
        }
        case booster::MOVE: {
            outcome.action.kind        = RpcActionKind::WALK;
            outcome.action.walk.vx     = get_double("vx");
            outcome.action.walk.vy     = get_double("vy");
            outcome.action.walk.vyaw   = get_double("vyaw");
            break;
        }
        case booster::ROTATE_HEAD: {
            outcome.action.kind       = RpcActionKind::HEAD;
            outcome.action.head.pitch = get_double("pitch");
            outcome.action.head.yaw   = get_double("yaw");
            break;
        }
        case booster::LIE_DOWN: {
            outcome.action.kind = RpcActionKind::LIE_DOWN;
            break;
        }
        case booster::GET_UP: {
            outcome.action.kind = RpcActionKind::GET_UP;
            // GetUpRequest's default member initialiser (target_mode = SOCCER) is what
            // NUbots' plain GetUp() call expects — no body fields to read.
            break;
        }
        case booster::GET_UP_WITH_MODE: {
            outcome.action.kind                  = RpcActionKind::GET_UP;
            outcome.action.get_up.target_mode = get_int("mode", k1sim::message::GetUpRequest{}.target_mode);
            break;
        }
        case booster::VISUAL_KICK: {
            outcome.action.kind               = RpcActionKind::VISUAL_KICK;
            outcome.action.visual_kick.start   = get_bool("start", k1sim::message::VisualKickRequest{}.start);
            outcome.action.visual_kick.version = get_int("version", k1sim::message::VisualKickRequest{}.version);
            break;
        }
        case booster::GET_MODE: {
            outcome.action.kind = RpcActionKind::NONE;
            outcome.response_body = json{{"mode", current_mode}}.dump();
            break;
        }
        default: {
            outcome.unknown_api_id = true;
            outcome.status         = unknown_api_status;
            break;
        }
    }

    return outcome;
}

}  // namespace k1sim::module::sdkbridge
