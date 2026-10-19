#ifndef K1SIM_MODULE_SDKBRIDGE_RPCDISPATCH_HPP
#define K1SIM_MODULE_SDKBRIDGE_RPCDISPATCH_HPP

#include <cstdint>
#include <string>

#include "shared/message/Commands.hpp"

// Parse/dispatch core for the Booster LocoApi RPC surface. Free of DDS and NUClear so
// test_rpc_dispatch.cpp can test it directly; RpcServer.cpp handles the DDS side. See
// module/SdkBridge/PROTOCOL.md §3.

namespace k1sim::module::sdkbridge {

    // Discriminates which (if any) NUClear message dispatch_rpc() wants emitted. Exactly
    // one of the payload fields below is meaningful, selected by `kind`.
    enum class RpcActionKind {
        NONE,  // GET_MODE (handled entirely via the reply body) and unknown api_ids
        MODE_CHANGE,
        WALK,
        HEAD,
        GET_UP,
        LIE_DOWN,
        VISUAL_KICK,
    };

    struct RpcAction {
        RpcActionKind kind = RpcActionKind::NONE;
        k1sim::message::ModeChangeRequest mode_change;
        k1sim::message::WalkCommand walk;
        k1sim::message::HeadCommand head;
        k1sim::message::GetUpRequest get_up;
        k1sim::message::LieDownRequest lie_down;
        k1sim::message::VisualKickRequest visual_kick;
    };

    struct RpcOutcome {
        RpcAction action;
        int64_t status            = 0;      // goes in the reply header {"status": status}
        std::string response_body = "{}";   // reply body; only GET_MODE returns non-trivial content
        bool unknown_api_id       = false;  // true => caller should log a WARN
        int64_t api_id            = 0;      // parsed api_id, for logging
    };

    // Parses `header_json` (RpcReqMsg_.header, expected `{"api_id": N, ...}`) and
    // `body_json` (RpcReqMsg_.body, a per-call JSON object or empty string) and returns
    // what to do. Never throws: malformed JSON or a missing/unrecognised api_id is
    // reported via `unknown_api_id` with `status == unknown_api_status`.
    RpcOutcome dispatch_rpc(const std::string& header_json,
                            const std::string& body_json,
                            int current_mode,
                            int64_t unknown_api_status);

}  // namespace k1sim::module::sdkbridge

#endif  // K1SIM_MODULE_SDKBRIDGE_RPCDISPATCH_HPP
