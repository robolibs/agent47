#include <agent47.hpp>

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <thread>

int main(int argc, char **argv) {
    dp::robot::Identity id_01;
    id_01.uuid = dp::sugar::uuid::generate_v4();
    id_01.name = dp::String("carter_01");
    id_01.ip = dp::sugar::ip::from_string(dp::String("0.0.0.0"));
    id_01.rci = dp::u8(1);

    dp::robot::Identity id_02;
    id_02.uuid = dp::sugar::uuid::generate_v4();
    id_02.name = dp::String("carter_02");
    id_02.ip = dp::sugar::ip::from_string(dp::String("0.0.0.0"));
    id_02.rci = dp::u8(2);

    agent47::AgentSpec spec;
    spec.drive_type = dp::String("diff_drive");
    spec.steering_type = dp::String("DIFFERENTIAL");
    spec.track_width_m = 0.55;
    spec.wheel_diameter_m = 0.31;
    spec.length_m = 1.00;
    spec.width_m = 0.78;
    spec.height_m = 0.90;

    agent47::Ros2Bridge ros_01;
    agent47::Ros2Bridge ros_02;

    // Namespaces map to the topics you listed:
    // /carter_01/cmd_vel, /carter_01/odom and /carter_02/cmd_vel, /carter_02/odom.
    ros_01.connect("/carter_01");
    ros_02.connect("/carter_02");

    agent47::Agent robot_01(id_01, spec, &ros_01);
    agent47::Agent robot_02(id_02, spec, &ros_02);

    dp::Twist cmd_01;
    cmd_01.linear.vx = 0.20;
    cmd_01.angular.vz = 0.10;

    dp::Twist cmd_02;
    cmd_02.linear.vx = 0.20;
    cmd_02.angular.vz = -0.10;

    constexpr dp::f64 DT_S = 0.05;
    constexpr int STEP_MS = 50;
    int duration_s = 0;
    if (argc >= 2) {
        duration_s = std::atoi(argv[1]);
        if (duration_s < 0) {
            duration_s = 0;
        }
    }

    std::cout << "Publishing to /carter_01/cmd_vel and /carter_02/cmd_vel ...\n";
    if (duration_s == 0) {
        std::cout << "Duration: infinite (Ctrl+C to stop)\n";
    } else {
        std::cout << "Duration: " << duration_s << "s\n";
    }

    const int max_steps = (duration_s == 0) ? -1 : (duration_s * 1000 / STEP_MS);
    int step = 0;
    while (max_steps < 0 || step < max_steps) {
        robot_01.set_velocity(cmd_01);
        robot_02.set_velocity(cmd_02);

        (void)robot_01.tick(DT_S);
        (void)robot_02.tick(DT_S);

        if ((step % 20) == 0) {
            std::cout << "carter_01 pose x=" << robot_01.odom.pose.point.x << " y=" << robot_01.odom.pose.point.y
                      << " | carter_02 pose x=" << robot_02.odom.pose.point.x << " y=" << robot_02.odom.pose.point.y
                      << "\n";
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(STEP_MS));
        ++step;
    }

    std::cout << "Done.\n";
    return 0;
}
