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

    agent47::types::Observation obs;
    obs.robot_id = agent.model().identity.uuid;
    obs.pose.point = datapod::Point{0.0, 0.0, 0.0};
    obs.pose.rotation = datapod::Quaternion::from_euler(0.0, 0.0, 0.0);
    obs.allow_move = true;
    obs.allow_reverse = true;

    const double dt_s = 0.1;
    for (int i = 0; i < 20; ++i) {
        obs.stamp_s = i * dt_s;
        obs.tick_seq = static_cast<uint64_t>(i);
        agent.model().runtime.stamp_s = obs.stamp_s;
        agent.model().runtime.tick_seq = obs.tick_seq;
        auto cmd_res = agent.tick(obs, dt_s);
        if (cmd_res.is_ok()) {
            const auto &cmd = cmd_res.value();
            echo("t=", obs.stamp_s, " valid=", cmd.valid, " v=", cmd.linear_mps, " w=", cmd.angular_rps, " status='",
                 cmd.status, "'");
        } else {
            echo("t=", obs.stamp_s, " error=", cmd_res.error().message);
        }
    }

    return 0;
}
