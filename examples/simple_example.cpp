#include <agent47.hpp>
#include <echo/echo.hpp>

#include <chrono>
#include <thread>

int main() {

    agent47::model::Robot model;
    model.identity.uuid = "robot_0";
    model.identity.name = "turtle1";
    model.identity.type = "diff_drive";

    model.body.steering_type = agent47::model::SteeringType::ACKERMANN;

#ifdef AGENT47_HAS_ROS2
    agent47::Ros2Bridge ros;
    ros.connect(model.identity);
    agent47::Agent agent(model, &ros);
#else
    agent47::Agent agent(model, nullptr);
#endif

    const double dt_s = 0.1;
    while (true) {
        (void)agent.tick(dt_s);

        const auto &p = agent.pose;
        echo("pos=", p.point.x, ",", p.point.y, ",", p.point.z, " q=", p.rotation.w, ",", p.rotation.x, ",",
             p.rotation.y, ",", p.rotation.z);

        std::this_thread::sleep_for(std::chrono::duration<double>(dt_s));
    }

    return 0;
}
