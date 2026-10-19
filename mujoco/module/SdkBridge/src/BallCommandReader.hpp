#ifndef K1SIM_MODULE_SDKBRIDGE_BALLCOMMANDREADER_HPP
#define K1SIM_MODULE_SDKBRIDGE_BALLCOMMANDREADER_HPP

#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>
#include <nuclear>

#include "module/SdkBridge/src/DdsParticipant.hpp"

// Reads rt/nusim/ball_command (NUSim test control, see shared/k1/NUSimApi.hpp and PROTOCOL.md §6)
// and emits it as message::BallCommand for module::Supervisor to apply. Used by NUbots-side test
// tools to roll or shoot the ball at the robot without a GameController.

namespace k1sim::module::sdkbridge {

    class BallCommandReader : public eprosima::fastdds::dds::DataReaderListener {
    public:
        BallCommandReader(DdsParticipant& dds, NUClear::Reactor& reactor);
        ~BallCommandReader() override;

        BallCommandReader(const BallCommandReader&)            = delete;
        BallCommandReader& operator=(const BallCommandReader&) = delete;

        void on_data_available(eprosima::fastdds::dds::DataReader* reader) override;

    private:
        DdsParticipant& dds_;
        NUClear::Reactor& reactor_;
        eprosima::fastdds::dds::DataReader* reader_ = nullptr;
    };

}  // namespace k1sim::module::sdkbridge

#endif  // K1SIM_MODULE_SDKBRIDGE_BALLCOMMANDREADER_HPP
