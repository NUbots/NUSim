#ifndef K1SIM_MODULE_SDKBRIDGE_HPP
#define K1SIM_MODULE_SDKBRIDGE_HPP

#include <memory>
#include <nuclear>

#include "module/SdkBridge/src/CameraPublisher.hpp"
#include "module/SdkBridge/src/DdsParticipant.hpp"
#include "module/SdkBridge/src/RpcServer.hpp"
#include "module/SdkBridge/src/StatePublisher.hpp"

namespace k1sim::module {

    // The Booster SDK compatibility surface: FastDDS publishers for
    // rt/low_state, rt/odometer_state, rt/fall_down, rt/battery_state,
    // rt/button_event, the head camera's sensor_msgs Image + CameraInfo (topic from
    // config/camera.yaml) and the LocoApi RPC server (rt/LocoApiTopicReq/Resp).
    // See module/SdkBridge/PROTOCOL.md for the full wire contract this implements.
    class SdkBridge : public NUClear::Reactor {
    public:
        explicit SdkBridge(std::unique_ptr<NUClear::Environment> environment);

    private:
        // Constructed in on<Startup> (after config/dds.yaml is read) — see DdsParticipant
        // for why a live participant can't be created before that.
        std::unique_ptr<sdkbridge::DdsParticipant> dds_;
        std::unique_ptr<sdkbridge::StatePublisher> state_publisher_;
        std::unique_ptr<sdkbridge::RpcServer> rpc_server_;
        // Null when config/camera.yaml's topic is empty (camera not published over DDS)
        std::unique_ptr<sdkbridge::CameraPublisher> camera_publisher_;
    };

}  // namespace k1sim::module

#endif  // K1SIM_MODULE_SDKBRIDGE_HPP
