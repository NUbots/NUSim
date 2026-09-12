#include "module/ViserViewer/src/ViserViewer.hpp"

#include <arpa/inet.h>
#include <mujoco/mujoco.h>
#include <netinet/in.h>
#include <spawn.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstring>
#include <filesystem>
#include <string_view>
#include <thread>

#include "shared/CliOptions.hpp"
#include "shared/k1/BoosterApi.hpp"
#include "shared/message/Commands.hpp"
#include "shared/sim/Shove.hpp"

extern char** environ;  // NOLINT: posix_spawnp passes the sim's environment on to the server

namespace k1sim::module {

namespace {

// One state datagram: this header, then qpos[nq], mocap_pos[3 * nmocap], mocap_quat[4 * nmocap]
// as doubles. Must match HEADER in python/viser_server.py.
struct FrameHeader {
    char magic[4]   = {'N', 'S', 'V', '1'};
    uint32_t seq    = 0;
    double sim_time = 0.0;
    double rtf      = 0.0;
    int32_t nq      = 0;
    int32_t nmocap  = 0;
    char mode[16]   = {};
};
static_assert(sizeof(FrameHeader) == 48, "FrameHeader must pack as python/viser_server.py expects");

sockaddr_in loopback(uint16_t port) {
    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port        = htons(port);
    return addr;
}

}  // namespace

ViserViewer::ViserViewer(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {
    if (!cli().viser) {
        return;
    }

    on<Trigger<message::SimHandles>, Sync<ViserViewer>>().then(
        [this](const message::SimHandles& handles) { start(handles); });

    on<Trigger<message::SimStateUpdate>>().then(
        [this](const message::SimStateUpdate& state) { mode_.store(state.mode, std::memory_order_relaxed); });

    on<Every<30, Per<std::chrono::seconds>>, Sync<ViserViewer>>().then([this] {
        if (socket_ < 0) {
            return;
        }
        receive();
        check_server();
        send_frame();
    });

    on<Shutdown, Sync<ViserViewer>>().then([this] { stop(); });
}

void ViserViewer::start(const message::SimHandles& handles) {
    if (socket_ >= 0 || handles.model == nullptr) {
        return;
    }
    model_        = handles.model;
    data_         = handles.data;
    mutex_        = handles.mutex;
    measured_rtf_ = handles.measured_rtf;

    // The server rebuilds the scene from the compiled model, so it matches whatever this run
    // loaded (--model, --robots). mjModel is immutable after load: no lock needed.
    model_file_ =
        (std::filesystem::temp_directory_path() / ("nusim_viser_" + std::to_string(::getpid()) + ".mjb")).string();
    mj_saveModel(model_, model_file_.c_str(), nullptr, 0);
    if (!std::filesystem::exists(model_file_)) {
        log<NUClear::LogLevel::ERROR>("ViserViewer: could not save the model to", model_file_, "— no browser viewer");
        return;
    }

    socket_          = ::socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    sockaddr_in addr = loopback(0);
    socklen_t len    = sizeof(addr);
    if (socket_ < 0 || ::bind(socket_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0
        || ::getsockname(socket_, reinterpret_cast<sockaddr*>(&addr), &len) != 0) {
        log<NUClear::LogLevel::ERROR>("ViserViewer: could not open a UDP socket (", std::strerror(errno),
                                      ") — no browser viewer");
        stop();
        return;
    }

    const std::string sim_port = std::to_string(ntohs(addr.sin_port));
    const std::string port     = std::to_string(cli().viser_port);
    // Run from the source tree: the container that runs the sim mounts the repo where it was built
    const std::string server = std::string(K1SIM_SOURCE_DIR) + "/module/ViserViewer/python/viser_server.py";
    std::vector<std::string> args{"python3", server, "--model", model_file_, "--sim-port", sim_port, "--port", port};
    std::vector<char*> argv;
    for (auto& arg : args) {
        argv.push_back(arg.data());
    }
    argv.push_back(nullptr);

    pid_t pid     = -1;
    const int err = ::posix_spawnp(&pid, argv[0], nullptr, nullptr, argv.data(), environ);
    if (err != 0) {
        log<NUClear::LogLevel::ERROR>("ViserViewer: could not launch python3 (", std::strerror(err),
                                      ") — no browser viewer");
        stop();
        return;
    }
    server_pid_ = pid;

    // One string each: the logger spaces its arguments apart
    char host[256] = "localhost";
    ::gethostname(host, sizeof(host) - 1);
    const std::string url   = "http://" + std::string(host) + ":" + port;
    const std::string local = "http://localhost:" + port;
    log<NUClear::LogLevel::INFO>("ViserViewer: starting the browser viewer — open", url, "(or", local,
                                 "on this machine)");
}

void ViserViewer::receive() {
    char buf[64];
    sockaddr_in from{};
    socklen_t len = sizeof(from);
    ssize_t n     = 0;
    while ((n = ::recvfrom(socket_, buf, sizeof(buf), 0, reinterpret_cast<sockaddr*>(&from), &len)) >= 0) {
        const std::string_view command(buf, static_cast<std::size_t>(n));
        if (command == "hello") {
            if (peer_port_ == 0) {
                log<NUClear::LogLevel::INFO>("ViserViewer: browser viewer ready, streaming the sim state");
            }
            peer_port_ = ntohs(from.sin_port);
        }
        else if (command == "reset") {
            emit(std::make_unique<message::SimResetRequest>());
        }
        else if (command == "shove") {
            std::lock_guard<std::mutex> lock(*mutex_);
            sim::shove_robot(model_, data_);
        }
        len = sizeof(from);
    }
}

void ViserViewer::send_frame() {
    if (peer_port_ == 0) {
        return;
    }

    const int nq     = model_->nq;
    const int nmocap = model_->nmocap;
    frame_.resize(sizeof(FrameHeader) + sizeof(double) * (nq + 7 * nmocap));

    FrameHeader header;
    header.seq    = seq_++;
    header.rtf    = measured_rtf_ != nullptr ? measured_rtf_->load(std::memory_order_relaxed) : 0.0;
    header.nq     = nq;
    header.nmocap = nmocap;
    std::strncpy(header.mode, booster::mode_name(mode_.load(std::memory_order_relaxed)), sizeof(header.mode) - 1);

    auto* body = reinterpret_cast<double*>(frame_.data() + sizeof(FrameHeader));
    {
        std::lock_guard<std::mutex> lock(*mutex_);
        header.sim_time = data_->time;
        std::memcpy(body, data_->qpos, sizeof(double) * nq);
        std::memcpy(body + nq, data_->mocap_pos, sizeof(double) * 3 * nmocap);
        std::memcpy(body + nq + 3 * nmocap, data_->mocap_quat, sizeof(double) * 4 * nmocap);
    }
    std::memcpy(frame_.data(), &header, sizeof(header));

    // Fire and forget: a frame the server misses is superseded by the next one
    const sockaddr_in to = loopback(peer_port_);
    ::sendto(socket_, frame_.data(), frame_.size(), 0, reinterpret_cast<const sockaddr*>(&to), sizeof(to));
}

void ViserViewer::check_server() {
    int status = 0;
    if (server_pid_ > 0 && ::waitpid(server_pid_, &status, WNOHANG) == server_pid_) {
        const std::string how = WIFEXITED(status) ? "(code " + std::to_string(WEXITSTATUS(status)) + ")"
                                                  : "(signal " + std::to_string(WTERMSIG(status)) + ")";
        log<NUClear::LogLevel::ERROR>("ViserViewer: the browser viewer exited", how,
                                      "— see its output above. The sim keeps running.");
        server_pid_ = -1;
        peer_port_  = 0;
    }
}

void ViserViewer::stop() {
    if (server_pid_ > 0) {
        ::kill(server_pid_, SIGTERM);
        // Give it a moment to go, then make sure
        int status = 0;
        for (int i = 0; i < 50 && ::waitpid(server_pid_, &status, WNOHANG) == 0; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        if (::waitpid(server_pid_, &status, WNOHANG) == 0) {
            ::kill(server_pid_, SIGKILL);
            ::waitpid(server_pid_, &status, 0);
        }
        server_pid_ = -1;
    }
    if (socket_ >= 0) {
        ::close(socket_);
        socket_ = -1;
    }
    if (!model_file_.empty()) {
        std::error_code ec;
        std::filesystem::remove(model_file_, ec);  // the server normally removes it once loaded
    }
}

}  // namespace k1sim::module
