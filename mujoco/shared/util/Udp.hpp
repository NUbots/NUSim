#ifndef K1SIM_SHARED_UTIL_UDP_HPP
#define K1SIM_SHARED_UTIL_UDP_HPP

// clang-format off
// <nuclear> must come before any socket header: on macOS it selects the IPv6 socket API they declare.
#include <nuclear>
// clang-format on

#include <arpa/inet.h>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <netinet/in.h>
#include <string>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
#include <utility>
#include <vector>

// The plain UDP sockets the GameController modules share: module::Supervisor listens on the GameController port and
// module::Referee sends on it. They are their own sockets rather than NUClear's on<UDP> because the port has to be
// shared with robot software on the same host (NUbots binds 3838 itself), which on<UDP> can't do.
namespace k1sim::util::udp {

    // A UDP socket file descriptor, closed when the Socket is destroyed (or close() is called), so a module that
    // stops or fails half way through starting never leaves the port held or the descriptor leaked.
    class Socket {
    public:
        Socket() = default;
        explicit Socket(int fd) : fd_(fd) {}
        ~Socket() {
            close();
        }
        Socket(const Socket&)            = delete;
        Socket& operator=(const Socket&) = delete;
        Socket(Socket&& other) noexcept : fd_(std::exchange(other.fd_, -1)) {}
        Socket& operator=(Socket&& other) noexcept {
            if (this != &other) {
                close();
                fd_ = std::exchange(other.fd_, -1);
            }
            return *this;
        }

        [[nodiscard]] int fd() const {
            return fd_;
        }
        [[nodiscard]] bool valid() const {
            return fd_ >= 0;
        }
        void close() {
            if (fd_ >= 0) {
                ::close(fd_);
                fd_ = -1;
            }
        }

    private:
        int fd_ = -1;
    };

    // A socket bound to `port` on every interface, which receives the GameController's broadcasts (and unicasts) to
    // it. SO_REUSEADDR and SO_REUSEPORT are two different options, both needed so other listeners on this host can
    // bind the same port beside it. recv() gives up after `recv_timeout`, so a listener thread blocked on it wakes up
    // regularly to notice that it should stop. Invalid, with the reason in `error`, if the port can't be bound.
    inline Socket open_listener(int port, std::chrono::milliseconds recv_timeout, std::string& error) {
        Socket socket(::socket(AF_INET, SOCK_DGRAM, 0));
        if (!socket.valid()) {
            error = std::strerror(errno);
            return socket;
        }
        const int yes = 1;
        ::setsockopt(socket.fd(), SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
#ifdef SO_REUSEPORT
        ::setsockopt(socket.fd(), SOL_SOCKET, SO_REUSEPORT, &yes, sizeof(yes));
#endif
        const auto micros = std::chrono::duration_cast<std::chrono::microseconds>(recv_timeout).count();
        const timeval timeout{static_cast<time_t>(micros / 1000000), static_cast<suseconds_t>(micros % 1000000)};
        ::setsockopt(socket.fd(), SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

        sockaddr_in address{};
        address.sin_family      = AF_INET;
        address.sin_port        = htons(static_cast<std::uint16_t>(port));
        address.sin_addr.s_addr = htonl(INADDR_ANY);
        if (::bind(socket.fd(), reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0) {
            error = std::strerror(errno);
            socket.close();
        }
        return socket;
    }

    // An unbound socket for sending. SO_BROADCAST lets it send to broadcast addresses such as 255.255.255.255, which
    // is how the GameController reaches every robot; the kernel refuses those sends without it.
    inline Socket open_sender(std::string& error) {
        Socket socket(::socket(AF_INET, SOCK_DGRAM, 0));
        if (!socket.valid()) {
            error = std::strerror(errno);
            return socket;
        }
        const int yes = 1;
        ::setsockopt(socket.fd(), SOL_SOCKET, SO_BROADCAST, &yes, sizeof(yes));
        return socket;
    }

    // `port` on each of the dotted IPv4 `addresses`. Addresses that aren't one are skipped and returned in `invalid`.
    inline std::vector<sockaddr_in> targets(const std::vector<std::string>& addresses,
                                            int port,
                                            std::vector<std::string>& invalid) {
        std::vector<sockaddr_in> result;
        for (const auto& text : addresses) {
            sockaddr_in address{};
            address.sin_family = AF_INET;
            address.sin_port   = htons(static_cast<std::uint16_t>(port));
            if (::inet_pton(AF_INET, text.c_str(), &address.sin_addr) == 1) {
                result.push_back(address);
            }
            else {
                invalid.push_back(text);
            }
        }
        return result;
    }

    // Sends one datagram to every target. Best effort, like the GameController: a target that can't be reached
    // misses this packet and gets the next one.
    inline void send_to_all(const Socket& socket,
                            const std::vector<sockaddr_in>& targets,
                            const void* data,
                            std::size_t size) {
        if (!socket.valid()) {
            return;
        }
        for (const auto& target : targets) {
            ::sendto(socket.fd(), data, size, 0, reinterpret_cast<const sockaddr*>(&target), sizeof(target));
        }
    }

}  // namespace k1sim::util::udp

#endif  // K1SIM_SHARED_UTIL_UDP_HPP
