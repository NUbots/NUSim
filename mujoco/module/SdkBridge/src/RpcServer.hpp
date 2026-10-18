#ifndef K1SIM_MODULE_SDKBRIDGE_RPCSERVER_HPP
#define K1SIM_MODULE_SDKBRIDGE_RPCSERVER_HPP

#include <atomic>
#include <cstdint>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>
#include <nuclear>

#include "module/SdkBridge/src/DdsParticipant.hpp"

// DDS side of the LocoApi RPC surface: reads rt/LocoApiTopicReq, passes each request to
// dispatch_rpc(), emits the resulting message, and replies on rt/LocoApiTopicResp synchronously in
// the DDS callback, since B1LocoClient blocks up to 1000 ms per call (PROTOCOL.md §3).
//
// Also owns the optional rt/joint_ctrl (LowCmd_) reader, which emits LowCmdMessage.

namespace k1sim::module::sdkbridge {

    class RpcServer : public eprosima::fastdds::dds::DataReaderListener {
    public:
        RpcServer(DdsParticipant& dds, NUClear::Reactor& reactor, int64_t unknown_api_status);

        void on_data_available(eprosima::fastdds::dds::DataReader* reader) override;

        // SdkBridge's on<Trigger<SimStateUpdate>> handler calls this every tick so
        // GET_MODE can answer from an atomic without touching the physics thread.
        void set_current_mode(int mode) {
            current_mode_.store(mode, std::memory_order_relaxed);
        }

    private:
        void handle_rpc_request();
        void handle_joint_ctrl();

        NUClear::Reactor& reactor_;
        int64_t unknown_api_status_;
        std::atomic<int> current_mode_{0};

        eprosima::fastdds::dds::DataReader* rpc_req_reader_    = nullptr;
        eprosima::fastdds::dds::DataReader* joint_ctrl_reader_ = nullptr;
        eprosima::fastdds::dds::DataWriter* rpc_resp_writer_   = nullptr;
    };

}  // namespace k1sim::module::sdkbridge

#endif  // K1SIM_MODULE_SDKBRIDGE_RPCSERVER_HPP
