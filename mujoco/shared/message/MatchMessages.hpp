#ifndef K1SIM_SHARED_MESSAGE_MATCHMESSAGES_HPP
#define K1SIM_SHARED_MESSAGE_MATCHMESSAGES_HPP

#include <string>
#include <vector>

// Between module::MatchReferee and the viewer's match panel.
namespace k1sim::message {

    struct MatchStatus {
        std::string summary;                    // the panel's text
        std::string primary_button;             // what the main button does now; empty hides it
        std::vector<std::string> robot_labels;  // shown above each robot, in roster order
        std::string messages;                   // the message box's text

        // The summary's facts as values, for a graphical scoreboard. has_game is false until the first
        // control message.
        bool has_game      = false;
        int home_score     = 0;
        int away_score     = 0;
        int secs_remaining = 0;
        std::string state;  // INITIAL, READY, SET, PLAYING, FINISHED, or PAUSED while play is stopped
        int connected      = 0;  // robots whose player process is online
        int players        = 0;
    };

    struct MatchCommand {
        // SET_PLAY calls a set play by hand and holds it until RELEASE sets the ball free.
        enum class Action { PRIMARY, RESET, SET_PLAY, RELEASE } action = Action::PRIMARY;
        int side     = 0;  // SET_PLAY: 0 = home, 1 = away
        int set_play = 0;  // SET_PLAY: a GameController SetPlay value
    };

}  // namespace k1sim::message

#endif  // K1SIM_SHARED_MESSAGE_MATCHMESSAGES_HPP
