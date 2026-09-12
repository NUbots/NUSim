#ifndef K1SIM_MODULE_VISERVIEWER_HPP
#define K1SIM_MODULE_VISERVIEWER_HPP

#include <atomic>
#include <cstdint>
#include <nuclear>
#include <string>
#include <vector>

#include "shared/message/SimMessages.hpp"

namespace k1sim::module {

// The browser viewer, enabled with --viser. Saves the compiled model to a file and launches
// python/viser_server.py, which rebuilds the scene from it and serves it with viser
// (https://viser.studio) on --viser-port. The server says hello over loopback UDP, and this
// module then streams it the joint state (qpos, mocap) at 30 Hz and takes its commands (reset,
// shove) in return. Inert without --viser.
class ViserViewer : public NUClear::Reactor {
public:
    explicit ViserViewer(std::unique_ptr<NUClear::Environment> environment);

private:
    // Save the model, open the socket and launch the server
    void start(const message::SimHandles& handles);
    // Handle the server's datagrams: hello, reset, shove
    void receive();
    // Send the server the current state
    void send_frame();
    // Log and forget the server if it has exited
    void check_server();
    void stop();

    // From SimHandles; owned by module::Simulation
    const mjModel* model_              = nullptr;
    mjData* data_                      = nullptr;
    std::mutex* mutex_                 = nullptr;
    std::atomic<double>* measured_rtf_ = nullptr;

    std::string model_file_;
    int socket_         = -1;
    uint16_t peer_port_ = 0;  // the server's port on 127.0.0.1, from its hello; 0 until then
    int server_pid_     = -1;
    uint32_t seq_       = 0;
    std::vector<char> frame_;

    // From SimStateUpdate, for the status text
    std::atomic<int> mode_{-1};
};

}  // namespace k1sim::module

#endif  // K1SIM_MODULE_VISERVIEWER_HPP
