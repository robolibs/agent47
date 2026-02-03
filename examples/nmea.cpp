#include <agent47.hpp>
#include <argu/argu.hpp>
#include <echo/echo.hpp>

#include <nonsens/nonsens.hpp>
#include <termios.h>
#include <unistd.h>
#include <wirebit/wirebit.hpp>

#include <chrono>
#include <thread>

int main(int argc, char *argv[]) {

    std::string name;
    auto cmd = argu::Command("key_op")
                   .version("1.0.0")
                   .about("WASD teleop: w/s linear, a/d angular, space stop, q quit")
                   .auto_exit()
                   .arg(argu::Arg("name")
                            .positional()
                            .help("Robot name")
                            .value_of(name)
                            .value_name("NAME")
                            .default_value("carter_1"));

    auto result = cmd.parse(argc, argv);
    if (!result) {
        return result.exit();
    }

    echo("Name: ", name);
    const char *link_path = (argc >= 2) ? argv[1] : "/tmp/pty07";
    auto pty = wirebit::PtyLink::create({.raw_bytes = true}).value();
    auto slave_path = std::string(pty.slave_path().c_str());
    auto link = std::make_shared<wirebit::PtyLink>(std::move(pty));

    std::error_code ec;
    std::filesystem::remove(link_path, ec);
    std::filesystem::create_symlink(slave_path, link_path, ec);

    if (ec) {
        std::cerr << "Warning: failed to create symlink " << link_path << " -> " << slave_path << ": " << ec.message()
                  << "\n";
    }

    wirebit::SerialConfig serial_cfg{};
    wirebit::SerialEndpoint serial(link, serial_cfg, 1);

    std::cout << "GNSS PTY slave: " << slave_path << "\n";
    std::cout << "Symlink:       " << link_path << "\n";
    std::cout << "This example only outputs (manual pod updates).\n";

    // wageningen datum
    datapod::Geo datum{51.98954034749562, 5.6584737410504715, 53.801823};

    dp::robot::Robot model;
    model.id.uuid = datapod::sugar::uuid::generate_v4();
    model.id.name = dp::String(name.c_str());
    model.id.ip = dp::sugar::ip::from_string(dp::String("192.168.1.100"));
    model.id.rci = dp::u8(1);

    model.props[dp::String("drive_type")] = dp::String("diff_drive");
    model.props[dp::String("steering_type")] = dp::String("ACKERMANN");

    echo::trace("Model loaded");

    agent47::Ros2Bridge ros;
    ros.connect(model.id.name.c_str());
    echo::trace("ROS2 bridge connected");

    agent47::Agent agent(model, &ros, datum);
    echo::trace("Agent started");

    auto res = agent.add_sensor("gnss", nonsens::sensor::SensorType::GNSS);
    if (!res.is_ok()) {
        std::cerr << "failed to create sensor: " << res.error().message.c_str() << "\n";
        return 1;
    }

    auto *sensor = agent.get_sensor("gnss");

    auto w = sensor->add_output(nonsens::sensor::Endpoint{&serial});
    if (!w.is_ok()) {
        std::cerr << "failed to set output: " << w.error().message.c_str() << "\n";
        return 1;
    }

    auto podv = sensor->pod();
    auto *gnss = dp::get<nonsens::pod::Gnss *>(podv);

    gnss->status.status = nonsens::pod::Gnss::NavSatStatus::STATUS_FIX;
    gnss->status.service = nonsens::pod::Gnss::NavSatStatus::SERVICE_GPS;
    gnss->rtk_status = nonsens::pod::Gnss::RtkStatus::RTK_FIXED;
    gnss->num_satellites = 10;
    gnss->hdop = 0.8;
    gnss->horizontal_accuracy = 0.02;
    gnss->vertical_accuracy = 0.03;
    gnss->phtg = true;

    dp::Twist teleop;

    while (true) {
        // Publish at a steady rate; feedback may come and go.
        constexpr dp::f64 PUBLISH_DT_S = 0.05;

        // Best-effort feedback: update dt from ROS time when available.
        static dp::i64 last_ts_ns = 0;
        static bool have_last = false;
        dp::f64 dt_s = PUBLISH_DT_S;

        gnss->latitude = agent.geopos.latitude;
        gnss->longitude = agent.geopos.longitude;
        gnss->altitude = agent.geopos.altitude;

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
        teleop = dp::Twist{};
        teleop.linear.vx = 0.1;
        teleop.angular.vz = 0.1;

        agent.set_velocity(teleop);
        // Always publish the latest commanded twist (if any).
        (void)agent.tick(dt_s);

        auto r = agent.nonsens_.push_all();
        if (!r.is_ok()) {
            std::cerr << "push error: " << r.error().message.c_str() << "\n";
        }

        echo("Agent pose: ", agent.odom.pose.point.x, " ", agent.odom.pose.point.y, " ", agent.odom.pose.point.z);
        echo("Agent geopose: ", agent.geopos.latitude, " ", agent.geopos.longitude, " ", agent.geopos.altitude);

        std::this_thread::sleep_for(std::chrono::duration<double>(PUBLISH_DT_S));
    }
    return 0;
}
