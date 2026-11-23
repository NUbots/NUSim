#ifndef K1SIM_SHARED_MESSAGE_INPUTMESSAGES_HPP
#define K1SIM_SHARED_MESSAGE_INPUTMESSAGES_HPP

#include <string>
#include <vector>

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

        // The training drill running ("free", "rondo", "rondo_defend", "rondo_ekf" or "rondo_solo") and the robot the keys play: the camera's
        // Follow and Behind track its trunk, and in a drill a marker hangs over it.
        std::string drill      = "free";
        std::string robot_body = "Trunk";
        bool marker            = false;
    };

    // From module::SkillPlayer to the viewer, every control step of the tracking rondos (and once, inactive,
    // when they end): the teammates' EKF track(s) of the defender (one shared in rondo_ekf, one each in
    // rondo_solo), and what each one's head camera sees of it, for the viewer to draw the track(s) and show
    // the cameras beside the scene.
    struct DefenderTrack {
        bool active   = false;
        double aspect = 1.22;  // width / height of the camera views (the K1's 106 x 95 degree lens)
        // A track: position, velocity and the position's covariance (field frame); and, for show, how
        // far it is from where the defender really is. One, shared, or one per teammate (`shared` false).
        struct Track {
            bool valid = false;
            double x = 0.0, y = 0.0, vx = 0.0, vy = 0.0;
            double cxx = 0.0, cxy = 0.0, cyy = 0.0;
            double error = 0.0, sigma = 0.0;
        };
        bool shared = true;
        std::vector<Track> tracks;
        double truth_x = 0.0, truth_y = 0.0;
        struct View {
            int robot_id = 0;
            int camera   = -1;  // the model's head camera on that robot
            // It sees the defender now: its box in the image, in units of half the image height
            // about the centre (u right, v up; |u| < aspect, |v| < 1).
            bool seen = false;
            double u0 = 0.0, v0 = 0.0, u1 = 0.0, v1 = 0.0;
            // Its last detection, where it put the defender (field frame), and how long ago.
            bool detected = false;
            double det_x = 0.0, det_y = 0.0, cam_x = 0.0, cam_y = 0.0, age = 1e9;
            bool occluded = false;  // in view but hidden behind another robot
        };
        std::vector<View> views;
    };

    // From the viewer's Training buttons to module::SkillPlayer: start a drill ("free", "rondo", "rondo_defend", "rondo_ekf", "rondo_solo").
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
