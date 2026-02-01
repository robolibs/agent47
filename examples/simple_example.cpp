#include <agent47.hpp>
#include <echo/echo.hpp>

#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <thread>

namespace {
    struct TerminalRawMode {
        termios orig{};
        bool active = false;

        TerminalRawMode() {
            if (!isatty(STDIN_FILENO)) {
                return;
            }
            if (tcgetattr(STDIN_FILENO, &orig) != 0) {
                return;
            }
            termios raw = orig;
            raw.c_lflag &= static_cast<unsigned long>(~(ECHO | ICANON));
            raw.c_cc[VMIN] = 0;
            raw.c_cc[VTIME] = 0;
            if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0) {
                active = true;
            }
        }

        ~TerminalRawMode() {
            if (active) {
                tcsetattr(STDIN_FILENO, TCSANOW, &orig);
            }
        }
    };

    static int read_key_nonblocking() {
        unsigned char ch = 0;
        const auto n = ::read(STDIN_FILENO, &ch, 1);
        if (n == 1) {
            return static_cast<int>(ch);
        }
        return -1;
    }
} // namespace

int main() {
#ifdef AGENT47_HAS_ROS2

    dp::robot::Robot model;
    model.id.uuid = datapod::sugar::uuid::generate_v4();
    model.id.name = dp::String("turtle1");
    model.props[dp::String("drive_type")] = dp::String("diff_drive");
    model.props[dp::String("steering_type")] = dp::String("ACKERMANN");

    agent47::Ros2Bridge ros;
    ros.connect(model.id.name.c_str());
    agent47::Agent agent(model, &ros);

    TerminalRawMode raw;
    if (!raw.active) {
        echo("stdin is not a TTY; WASD disabled");
    }
    echo("WASD teleop: w/s linear, a/d angular, space stop, q quit");

    // Latched teleop command; published at a steady rate.
    dp::Twist teleop;

    while (true) {
        // Publish at a steady rate; feedback may come and go.
        constexpr dp::f64 PUBLISH_DT_S = 0.05;

        // Best-effort feedback: update dt from ROS time when available.
        static dp::i64 last_ts_ns = 0;
        static bool have_last = false;
        dp::f64 dt_s = PUBLISH_DT_S;

        dp::Stamp<agent47::types::Feedback> fb;
        if (ros.recv(fb, 0)) {
            if (have_last) {
                const auto dts = static_cast<dp::f64>(fb.timestamp - last_ts_ns) * 1e-9;
                if (dts > 0.0 && dts < 1.0) {
                    dt_s = dts;
                }
            }
            last_ts_ns = fb.timestamp;
            have_last = true;
            agent.update(fb.value);
        }

        bool key_handled = false;
        if (raw.active) {
            const int key = read_key_nonblocking();
            if (key >= 0) {
                constexpr dp::f64 STEP_LIN = 0.2;
                constexpr dp::f64 STEP_ANG = 0.2;
                constexpr dp::f64 MAX_LIN = 2.0;
                constexpr dp::f64 MAX_ANG = 2.0;
                if (key == 'q' || key == 'Q') {
                    break;
                }
                if (key == ' ') {
                    teleop = dp::Twist{};
                    key_handled = true;
                } else if (key == 'w' || key == 'W') {
                    teleop.linear.vx += STEP_LIN;
                    key_handled = true;
                } else if (key == 's' || key == 'S') {
                    teleop.linear.vx -= STEP_LIN;
                    key_handled = true;
                } else if (key == 'a' || key == 'A') {
                    teleop.angular.vz += STEP_ANG;
                    key_handled = true;
                } else if (key == 'd' || key == 'D') {
                    teleop.angular.vz -= STEP_ANG;
                    key_handled = true;
                }

                if (teleop.linear.vx > MAX_LIN) {
                    teleop.linear.vx = MAX_LIN;
                } else if (teleop.linear.vx < -MAX_LIN) {
                    teleop.linear.vx = -MAX_LIN;
                }

                if (teleop.angular.vz > MAX_ANG) {
                    teleop.angular.vz = MAX_ANG;
                } else if (teleop.angular.vz < -MAX_ANG) {
                    teleop.angular.vz = -MAX_ANG;
                }

                if (key_handled) {
                    echo("key=", static_cast<char>(key), " vx=", teleop.linear.vx, " wz=", teleop.angular.vz,
                         " pos=", agent.odom.pose.point.x, ",", agent.odom.pose.point.y);
                }
            }
        }

        agent.set_velocity(teleop);
        // Always publish the latest commanded twist (if any).
        (void)agent.tick(dt_s);

        std::this_thread::sleep_for(std::chrono::duration<double>(PUBLISH_DT_S));
    }

#else
    echo("AGENT47_HAS_ROS2 is not enabled");
#endif
    return 0;
}
