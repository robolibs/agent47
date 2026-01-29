#include <agent47.hpp>
#include <echo/echo.hpp>

#include <iostream>

int main() {
    agent47::Agent agent;

    agent.attach_farmtrax();

    drivekit::RobotConstraints constraints;
    constraints.steering_type = drivekit::SteeringType::DIFFERENTIAL;
    constraints.max_linear_velocity = 1.0;
    constraints.max_angular_velocity = 1.0;

    // Populate the unified model (identity/body/runtime)
    agent.model().identity.uuid = "robot_0";
    agent.model().identity.name = "demo_robot";
    agent.model().identity.type = "diff_drive";
    agent.attach_drivekit(constraints);

    // -- PipeBridge usage (uncomment to connect to a running backend) --
    // auto bridge = std::make_shared<agent47::PipeBridge>();
    // if (bridge->connect("tcp://127.0.0.1:9000")) {
    //     agent.set_bridge(bridge);
    // }

    agent47::types::Feedback fb;
    fb.pose.point = datapod::Point{0.0, 0.0, 0.0};
    fb.pose.rotation = datapod::Quaternion::from_euler(0.0, 0.0, 0.0);

    const double dt_s = 0.1;
    for (int i = 0; i < 20; ++i) {
        fb.stamp_s = i * dt_s;
        fb.tick_seq = static_cast<dp::u64>(i);
        auto cmd_res = agent.tick(fb, dt_s);
        if (cmd_res.is_ok()) {
            const auto &cmd = cmd_res.value();
            echo("t=", fb.stamp_s, " valid=", cmd.valid, " v=", cmd.twist.linear.vx, " w=", cmd.twist.angular.vz);
        } else {
            echo("t=", fb.stamp_s, " error=", cmd_res.error().message);
        }
    }

    return 0;
}
