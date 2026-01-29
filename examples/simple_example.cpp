#include <agent47.hpp>
#include <echo/echo.hpp>

int main() {
#ifdef AGENT47_HAS_ROS2

    agent47::model::Robot model;
    model.identity.uuid = "robot_0";
    model.identity.name = "turtle1";
    model.identity.type = "diff_drive";

    model.body.steering_type = agent47::model::SteeringType::ACKERMANN;

    agent47::Ros2Bridge ros;
    ros.connect(model.identity);
    agent47::Agent agent(model, &ros);

    while (true) {
        dp::Stamp<agent47::types::Feedback> fb;
        if (!ros.recv(fb, 1000)) {
            continue;
        }

        static dp::i64 last_ts_ns = 0;
        static bool have_last = false;

        dp::f64 dt_s = 0.0;
        if (have_last) {
            dt_s = static_cast<dp::f64>(fb.timestamp - last_ts_ns) * 1e-9;
        }
        last_ts_ns = fb.timestamp;
        have_last = true;

        // Clamp dt to avoid exploding on clock jumps.
        if (dt_s <= 0.0 || dt_s > 1.0) {
            dt_s = 0.0;
        }

        (void)agent.tick(fb, dt_s);

        const auto &p = agent.pose;
        echo("t_ns=", fb.timestamp, " dt_s=", dt_s, " pos=", p.point.x, ",", p.point.y, ",", p.point.z,
             " q=", p.rotation.w, ",", p.rotation.x, ",", p.rotation.y, ",", p.rotation.z)
            .every(1000);
    }

#else
    echo("AGENT47_HAS_ROS2 is not enabled");
#endif
    return 0;
}
