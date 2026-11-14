#ifndef K1SIM_MODULE_MATCHREFEREE_MATCHROSTER_HPP
#define K1SIM_MODULE_MATCHREFEREE_MATCHROSTER_HPP

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

// Which players are actually running, from the status messages they send back. This is what shows
// whether robot IDs are right: the autoreferee only sees sim bodies, so a process with the wrong
// player number is invisible to it, but shows up here as an unexpected or a missing player.
namespace k1sim::module::matchreferee {

    // RoboCupGameControlReturnData, version 4.
    struct StatusMessage {
        std::uint8_t player = 0;
        std::uint8_t team   = 0;
        bool fallen         = false;
    };

    inline constexpr std::size_t STATUS_MESSAGE_SIZE     = 32;
    inline constexpr std::uint8_t STATUS_MESSAGE_VERSION = 4;
    inline constexpr char STATUS_MESSAGE_HEADER[4]       = {'R', 'G', 'r', 't'};

    inline std::optional<StatusMessage> parse_status(const std::uint8_t* data, std::size_t len) {
        if (data == nullptr || len != STATUS_MESSAGE_SIZE
            || std::memcmp(data, STATUS_MESSAGE_HEADER, sizeof(STATUS_MESSAGE_HEADER)) != 0
            || data[4] != STATUS_MESSAGE_VERSION) {
            return std::nullopt;
        }
        return StatusMessage{data[5], data[6], data[7] != 0};
    }

    class MatchRoster {
    public:
        using Player = std::pair<std::uint8_t, std::uint8_t>;  // team number, player number

        MatchRoster(std::set<Player> expected, double timeout) : expected_(std::move(expected)), timeout_(timeout) {}

        // Records a status message received at `now` (seconds), returning what to report.
        std::vector<std::string> on_status(const StatusMessage& status, double now) {
            std::vector<std::string> messages;
            const Player player{status.team, status.player};
            if (last_seen_.find(player) == last_seen_.end()) {
                messages.push_back((expected_.count(player) != 0 ? "online: " : "UNEXPECTED player online: ")
                                   + name(player));
            }
            last_seen_[player] = now;
            return messages;
        }

        // Drops players that stopped reporting, returning what to report.
        std::vector<std::string> expire(double now) {
            std::vector<std::string> messages;
            for (auto it = last_seen_.begin(); it != last_seen_.end();) {
                if (now - it->second > timeout_) {
                    messages.push_back("offline: " + name(it->first));
                    it = last_seen_.erase(it);
                }
                else {
                    ++it;
                }
            }
            return messages;
        }

        static std::string name(const Player& player) {
            return "team " + std::to_string(player.first) + " player " + std::to_string(player.second);
        }

    private:
        std::set<Player> expected_;
        double timeout_;
        std::map<Player, double> last_seen_;
    };

}  // namespace k1sim::module::matchreferee

#endif  // K1SIM_MODULE_MATCHREFEREE_MATCHROSTER_HPP
