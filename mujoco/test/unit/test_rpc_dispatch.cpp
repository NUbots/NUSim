// Unit test for the pure RPC parse/dispatch core (module::sdkbridge::dispatch_rpc) —
// no DDS participant, no network. Feeds synthetic RpcReqMsg_ header/body JSON for
// every LocoApi id NUbots_K1 uses (plus an unknown id) and asserts the resulting
// NUClear message field values + reply status, per module/SdkBridge/PROTOCOL.md §3.
#include <cstdio>
#include <cstdlib>
#include <string>

#include "module/SdkBridge/src/RpcDispatch.hpp"
#include "shared/k1/BoosterApi.hpp"

namespace {

int failures = 0;

void expect(bool cond, const std::string& what) {
    if (!cond) {
        std::fprintf(stderr, "FAIL: %s\n", what.c_str());
        ++failures;
    }
}

std::string header_for(int api_id) {
    return "{\"api_id\":" + std::to_string(api_id) + "}";
}

}  // namespace

using k1sim::module::sdkbridge::dispatch_rpc;
using k1sim::module::sdkbridge::RpcActionKind;

int main() {
    constexpr int64_t kUnknownStatus = 0;

    // --- CHANGE_MODE ---
    {
        auto out = dispatch_rpc(header_for(k1sim::booster::CHANGE_MODE), R"({"mode":4})", 1, kUnknownStatus);
        expect(out.action.kind == RpcActionKind::MODE_CHANGE, "CHANGE_MODE: action kind");
        expect(out.action.mode_change.mode == 4, "CHANGE_MODE: mode == kSoccer(4)");
        expect(out.status == 0, "CHANGE_MODE: status 0");
        expect(!out.unknown_api_id, "CHANGE_MODE: not unknown");
    }

    // --- MOVE ---
    {
        auto out = dispatch_rpc(header_for(k1sim::booster::MOVE), R"({"vx":0.1,"vy":-0.2,"vyaw":0.3})", 2,
                                 kUnknownStatus);
        expect(out.action.kind == RpcActionKind::WALK, "MOVE: action kind");
        expect(out.action.walk.vx == 0.1, "MOVE: vx");
        expect(out.action.walk.vy == -0.2, "MOVE: vy");
        expect(out.action.walk.vyaw == 0.3, "MOVE: vyaw");
        expect(out.status == 0, "MOVE: status 0");
    }

    // --- ROTATE_HEAD ---
    {
        auto out =
            dispatch_rpc(header_for(k1sim::booster::ROTATE_HEAD), R"({"pitch":0.2,"yaw":0.3})", 2, kUnknownStatus);
        expect(out.action.kind == RpcActionKind::HEAD, "ROTATE_HEAD: action kind");
        expect(out.action.head.pitch == 0.2, "ROTATE_HEAD: pitch");
        expect(out.action.head.yaw == 0.3, "ROTATE_HEAD: yaw");
    }

    // --- LIE_DOWN (empty body) ---
    {
        auto out = dispatch_rpc(header_for(k1sim::booster::LIE_DOWN), "", 2, kUnknownStatus);
        expect(out.action.kind == RpcActionKind::LIE_DOWN, "LIE_DOWN: action kind");
        expect(out.status == 0, "LIE_DOWN: status 0");
    }

    // --- GET_UP (empty body -> default target_mode == SOCCER) ---
    {
        auto out = dispatch_rpc(header_for(k1sim::booster::GET_UP), "", 2, kUnknownStatus);
        expect(out.action.kind == RpcActionKind::GET_UP, "GET_UP: action kind");
        expect(out.action.get_up.target_mode == k1sim::message::GetUpRequest{}.target_mode,
               "GET_UP: default target_mode == SOCCER");
    }

    // --- GET_UP_WITH_MODE ---
    {
        auto out = dispatch_rpc(header_for(k1sim::booster::GET_UP_WITH_MODE), R"({"mode":2})", 0, kUnknownStatus);
        expect(out.action.kind == RpcActionKind::GET_UP, "GET_UP_WITH_MODE: action kind");
        expect(out.action.get_up.target_mode == 2, "GET_UP_WITH_MODE: target_mode == kWalking(2)");
    }

    // --- GET_MODE (no message emitted; reply body carries the mode) ---
    {
        auto out = dispatch_rpc(header_for(k1sim::booster::GET_MODE), "", 4, kUnknownStatus);
        expect(out.action.kind == RpcActionKind::NONE, "GET_MODE: no action emitted");
        expect(out.response_body == R"({"mode":4})", "GET_MODE: response body echoes current mode (got '"
                                                           + out.response_body + "')");
        expect(out.status == 0, "GET_MODE: status 0");
    }

    // --- VISUAL_KICK: explicit fields ---
    {
        auto out =
            dispatch_rpc(header_for(k1sim::booster::VISUAL_KICK), R"({"start":false,"version":0})", 2, kUnknownStatus);
        expect(out.action.kind == RpcActionKind::VISUAL_KICK, "VISUAL_KICK: action kind");
        expect(out.action.visual_kick.start == false, "VISUAL_KICK: start == false");
        expect(out.action.visual_kick.version == 0, "VISUAL_KICK: version == 0");
    }

    // --- VISUAL_KICK: empty body -> frozen struct defaults (start=true, version=1) ---
    {
        auto out = dispatch_rpc(header_for(k1sim::booster::VISUAL_KICK), "", 2, kUnknownStatus);
        expect(out.action.kind == RpcActionKind::VISUAL_KICK, "VISUAL_KICK (defaults): action kind");
        expect(out.action.visual_kick.start == k1sim::message::VisualKickRequest{}.start,
               "VISUAL_KICK (defaults): start default");
        expect(out.action.visual_kick.version == k1sim::message::VisualKickRequest{}.version,
               "VISUAL_KICK (defaults): version default");
    }

    // --- Unknown api_id: warn (caller's job) + configured unknown_api_status ---
    {
        constexpr int64_t kConfiguredUnknownStatus = 0;
        auto out = dispatch_rpc(header_for(999999), R"({})", 2, kConfiguredUnknownStatus);
        expect(out.unknown_api_id, "unknown api_id: flagged unknown");
        expect(out.action.kind == RpcActionKind::NONE, "unknown api_id: no action emitted");
        expect(out.status == kConfiguredUnknownStatus, "unknown api_id: status == configured unknown_api_status");
        expect(out.api_id == 999999, "unknown api_id: api_id echoed for logging");
    }

    // --- Missing api_id entirely (malformed header) ---
    {
        auto out = dispatch_rpc("{}", "{}", 2, 7);
        expect(out.unknown_api_id, "missing api_id: flagged unknown");
        expect(out.status == 7, "missing api_id: status == configured unknown_api_status (7)");
    }

    // --- Malformed JSON header doesn't throw, falls back to unknown ---
    {
        auto out = dispatch_rpc("{not json", "{}", 2, 7);
        expect(out.unknown_api_id, "malformed header: flagged unknown, no exception");
        expect(out.status == 7, "malformed header: status == configured unknown_api_status");
    }

    if (failures == 0) {
        std::printf("test_rpc_dispatch: all checks passed\n");
        return 0;
    }
    std::fprintf(stderr, "test_rpc_dispatch: %d check(s) failed\n", failures);
    return 1;
}
