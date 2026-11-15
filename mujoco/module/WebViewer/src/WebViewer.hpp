#ifndef K1SIM_MODULE_WEBVIEWER_HPP
#define K1SIM_MODULE_WEBVIEWER_HPP

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <nuclear>
#include <string>

#include "module/WebViewer/src/WebStream.hpp"
#include "shared/message/MatchMessages.hpp"
#include "shared/message/SimMessages.hpp"

namespace k1sim::module {

    // Browser based 3D viewer and match recorder (--web / --record).
    //
    // Serves the viewer page over HTTP and streams the body poses to it at half the state publish rate (25 Hz by
    // default; the viewer interpolates in between). Unlike module::Viewer it needs no GL context, so it also works
    // under --headless and from another machine. The same stream can be written to a recording file and replayed in
    // the viewer later. Inactive unless --web or --record is given.
    class WebViewer : public NUClear::Reactor {
    public:
        explicit WebViewer(std::unique_ptr<NUClear::Environment> environment);

    private:
        // sim/match: module::MatchReferee's scoreboard and the panel buttons it answers, in the shape of the
        // built-in referee's GameState and RefereeStatus, so the page shows them unchanged.
        void show_match(const message::MatchStatus& status);
        // A referee panel button (see WebStream::set_command_handler) as a MatchCommand, if MatchReferee has one.
        void send_match_command(const std::string& action, const std::string& side, const std::string& arg);

        std::unique_ptr<web::WebStream> stream_;
        std::atomic<bool> match_{false};  // module::MatchReferee has reported in, so it takes the panel's buttons
        std::mutex mutex_;  // guards handles_, updates_ and match_state_ (reactions may run on different pool threads)
        message::SimHandles handles_{};
        uint64_t updates_ = 0;
        std::string match_state_;  // MatchReferee's game state, as show_match() last saw it; guarded by mutex_
    };

}  // namespace k1sim::module

#endif  // K1SIM_MODULE_WEBVIEWER_HPP
