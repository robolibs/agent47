#include <agent47.hpp>

#include <chrono>
#include <iostream>
#include <thread>

namespace {
    class LoopbackBridge : public agent47::Bridge {
      public:
        bool connect(const std::string &endpoint) override {
            endpoint_ = endpoint;
            connected_ = true;
            return true;
        }

        void disconnect() override { connected_ = false; }

        bool is_connected() const override { return connected_; }

        bool send(const dp::Stamp<agent47::types::Command> &cmd) override {
            last_cmd_ = cmd;
            return connected_;
        }

        bool recv(dp::Stamp<agent47::types::Feedback> &fb, dp::i32 timeout_ms = 100) override {
            (void)timeout_ms;
            fb.timestamp = dp::Stamp<agent47::types::Feedback>::now();
            fb.value = last_fb_;
            return connected_;
        }

        bool sensor(agent47::types::SensorPacket &pkt, dp::i32 timeout_ms = 100) override {
            (void)pkt;
            (void)timeout_ms;
            return false;
        }

        void set_feedback(const agent47::types::Feedback &fb) { last_fb_ = fb; }

      private:
        bool connected_ = false;
        std::string endpoint_;
        dp::Stamp<agent47::types::Command> last_cmd_{};
        agent47::types::Feedback last_fb_{};
    };
} // namespace

int main() {
    LoopbackBridge bridge_a;
    LoopbackBridge bridge_b;
    bridge_a.connect("robot_a");
    bridge_b.connect("robot_b");

    dp::robot::Identity id_a;
    id_a.uuid = datapod::sugar::uuid::generate_v4();
    id_a.name = dp::String("robot_a");
    id_a.ip = dp::sugar::ip::from_string(dp::String("0.0.0.0"));
    id_a.rci = dp::u8(1);

    dp::robot::Identity id_b;
    id_b.uuid = datapod::sugar::uuid::generate_v4();
    id_b.name = dp::String("robot_b");
    id_b.ip = dp::sugar::ip::from_string(dp::String("0.0.0.0"));
    id_b.rci = dp::u8(2);

    agent47::AgentSpec spec_a;
    spec_a.drive_type = dp::String("diff_drive");
    spec_a.steering_type = dp::String("DIFFERENTIAL");
    spec_a.track_width_m = 0.55;
    spec_a.wheel_diameter_m = 0.32;
    spec_a.length_m = 1.10;
    spec_a.width_m = 0.80;
    spec_a.height_m = 0.95;

    agent47::AgentSpec spec_b;
    spec_b.drive_type = dp::String("ackermann");
    spec_b.steering_type = dp::String("ACKERMANN");
    spec_b.wheel_base_m = 1.25;
    spec_b.track_width_m = 0.70;
    spec_b.wheel_diameter_m = 0.38;
    spec_b.length_m = 1.75;
    spec_b.width_m = 1.05;
    spec_b.height_m = 1.10;

    agent47::Agent robot_a(id_a, spec_a, &bridge_a);
    agent47::Agent robot_b(id_b, spec_b, &bridge_b);

    dp::Twist cmd_a;
    cmd_a.linear.vx = 0.40;
    cmd_a.angular.vz = 0.00;

    dp::Twist cmd_b;
    cmd_b.linear.vx = 0.25;
    cmd_b.angular.vz = 0.15;

    std::cout << "Running two model-less agents with AgentSpec...\n";
    std::cout << "A drive_type=" << robot_a.model_.props[dp::String("drive_type")].c_str() << "\n";
    std::cout << "B drive_type=" << robot_b.model_.props[dp::String("drive_type")].c_str() << "\n";

    constexpr dp::f64 DT_S = 0.05;
    for (int i = 0; i < 20; ++i) {
        robot_a.set_velocity(cmd_a);
        robot_b.set_velocity(cmd_b);

        (void)robot_a.tick(DT_S);
        (void)robot_b.tick(DT_S);

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    std::cout << "Done.\n";
    return 0;
}
