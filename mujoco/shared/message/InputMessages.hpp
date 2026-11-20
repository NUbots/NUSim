#ifndef K1SIM_SHARED_MESSAGE_INPUTMESSAGES_HPP
#define K1SIM_SHARED_MESSAGE_INPUTMESSAGES_HPP

#include <string>

// Operator input, from the viewer window to whichever module plays the robot by hand
// (module::SkillPlayer), and that module's status line back to the viewer.
namespace k1sim::message {

    // A key going down or up in the viewer window. Codes are GLFW's: letters are their upper-case
    // ASCII value, space is 32. Key repeats are not sent.
    struct KeyEvent {
        int key   = 0;
        bool down = false;
    };

    inline constexpr int KEY_SPACE       = 32;
    inline constexpr int KEY_LEFT_SHIFT  = 340;
    inline constexpr int KEY_RIGHT_SHIFT = 344;

    // Which way the viewer's camera looks along the ground (world yaw, rad), sent whenever it turns,
    // so direction keys can mean "away from the camera" and "to its left".
    struct ViewDirection {
        double yaw = 0.0;
    };

    // The ball has gone into a goal (keyboard play): the viewer celebrates.
    struct GoalScored {
        double x = 0.0, y = 0.0;   // the goal's centre on its line (m)
        double width  = 0.0;       // between the posts (m)
        double height = 0.0;       // to the crossbar (m)
        int dir       = 1;         // +1: the goal at +x, the ball went in towards +x; -1: the other one
    };

    // Shown in the viewer window while a module plays the robot from the keyboard. Its arrival also
    // tells the viewer that Space belongs to that module (P pauses instead).
    struct SkillStatus {
        std::string skill;   // e.g. "RECEIVE"
        std::string detail;  // what the robot is doing or waiting for
        std::string keys;    // the key bindings

        // A shot being wound up, drawn as an arrow from the ball: longer and redder as it charges.
        bool aiming      = false;
        double ball_x = 0.0, ball_y = 0.0;  // world frame (m)
        double aim_angle = 0.0;             // world frame (rad)
        double charge    = 0.0;             // 0 = a tap, 1 = full power

        // The training drill running ("free", "rondo" or "rondo_defend") and the robot the keys play: the camera's
        // Follow and Behind track its trunk, and in a drill a marker hangs over it.
        std::string drill      = "free";
        std::string robot_body = "Trunk";
        bool marker            = false;
    };

    // From the viewer's Training buttons to module::SkillPlayer: start a drill ("free", "rondo", "rondo_defend").
    // Asking for the one already running starts it again.
    struct SkillDrill {
        std::string name;
    };

    // From module::SkillPlayer to the viewer: switch the camera mode (0 Free, 1 Follow, 2 Behind,
    // as the Simulation panel's buttons) and, with `place`, point it at (x, y) from `distance` m.
    struct CameraRequest {
        int mode   = 0;
        bool place = false;
        double x = 0.0, y = 0.0;
        double distance = 8.0, azimuth = 90.0, elevation = -50.0;  // m, degrees
    };

}  // namespace k1sim::message

#endif  // K1SIM_SHARED_MESSAGE_INPUTMESSAGES_HPP
