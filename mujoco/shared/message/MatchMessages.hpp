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
    };

    struct MatchCommand {
        enum class Action { PRIMARY, RESET } action = Action::PRIMARY;
    };

}  // namespace k1sim::message

#endif  // K1SIM_SHARED_MESSAGE_MATCHMESSAGES_HPP
