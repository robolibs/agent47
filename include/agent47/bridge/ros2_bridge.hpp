#pragma once

#ifdef AGENT47_HAS_ROS2

    #include <cmath>
    #include <memory>
    #include <mutex>
    #include <string>

    #include <rclcpp/rclcpp/rclcpp.hpp>

    #include <geometry_msgs/msg/twist.hpp>
    #include <nav_msgs/msg/odometry.hpp>
    #include <sensor_msgs/msg/imu.hpp>
    #include <sensor_msgs/msg/joint_state.hpp>
    #include <sensor_msgs/msg/laser_scan.hpp>
    #include <sensor_msgs/msg/nav_sat_fix.hpp>
    // turtlesim support
    #include <turtlesim/msg/pose.hpp>

    #include "agent47/bridge.hpp"
    #include "agent47/model/robot.hpp"
    #include "agent47/sensors/gnss.hpp"
    #include "agent47/sensors/imu.hpp"
    #include "agent47/sensors/lidar.hpp"

namespace agent47 {

    /// Bridge implementation backed by ROS 2 topics.
    ///
    /// connect(ns) creates a node in the given namespace and subscribes to
    /// standard topics. send() publishes geometry_msgs::msg::Twist on
    /// {ns}/cmd_vel, recv() returns the latest cached feedback after
    /// spinning the node once.
    class Ros2Bridge : public Bridge {
      public:
        Ros2Bridge() = default;
        ~Ros2Bridge() override { disconnect(); }

        Ros2Bridge(const Ros2Bridge &) = delete;
        Ros2Bridge &operator=(const Ros2Bridge &) = delete;

        /// Convenience overload: connect using a robot identity.
        ///
        /// Uses identity.name as the namespace root.
        /// - "turtle1" -> topics like "/turtle1/pose", "/turtle1/cmd_vel"
        /// - "/turtle1" -> same (leading slash kept)
        bool connect(const model::Identity &id) {
            if (id.name.empty()) {
                return connect("/");
            }
            if (id.name[0] == '/') {
                return connect(id.name);
            }
            return connect(std::string("/") + id.name);
        }

        bool connect(const std::string &ns) override {
            if (!rclcpp::ok()) {
                rclcpp::init(0, nullptr);
                owns_init_ = true;
            }

            rclcpp::NodeOptions opts;
            opts.use_intra_process_comms(true);
            node_ = std::make_shared<rclcpp::Node>("agent47", ns, opts);

            // -- Subscribers --

            odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
                "odom", 10, [this](nav_msgs::msg::Odometry::SharedPtr msg) {
                    std::lock_guard<std::mutex> lock(fb_mutex_);
                    const auto ns = static_cast<dp::i64>(msg->header.stamp.sec) * 1'000'000'000LL +
                                    static_cast<dp::i64>(msg->header.stamp.nanosec);
                    last_fb_.timestamp = ns;
                    last_fb_.value.tick_seq++;
                    last_fb_.value.pose.point.x = msg->pose.pose.position.x;
                    last_fb_.value.pose.point.y = msg->pose.pose.position.y;
                    last_fb_.value.pose.point.z = msg->pose.pose.position.z;
                    last_fb_.value.pose.rotation.w = msg->pose.pose.orientation.w;
                    last_fb_.value.pose.rotation.x = msg->pose.pose.orientation.x;
                    last_fb_.value.pose.rotation.y = msg->pose.pose.orientation.y;
                    last_fb_.value.pose.rotation.z = msg->pose.pose.orientation.z;
                    last_fb_.value.twist.linear.vx = msg->twist.twist.linear.x;
                    last_fb_.value.twist.linear.vy = msg->twist.twist.linear.y;
                    last_fb_.value.twist.linear.vz = msg->twist.twist.linear.z;
                    last_fb_.value.twist.angular.vx = msg->twist.twist.angular.x;
                    last_fb_.value.twist.angular.vy = msg->twist.twist.angular.y;
                    last_fb_.value.twist.angular.vz = msg->twist.twist.angular.z;
                    fb_ready_ = true;

                    last_pose_ = dp::Stamp<dp::Pose>{ns, last_fb_.value.pose};
                    last_twist_ = dp::Stamp<dp::Twist>{ns, last_fb_.value.twist};
                    pose_ready_ = true;
                    twist_ready_ = true;
                });

            pose_sub_ = node_->create_subscription<turtlesim::msg::Pose>(
                "pose", 10, [this](turtlesim::msg::Pose::SharedPtr msg) {
                    std::lock_guard<std::mutex> lock(fb_mutex_);

                    const auto ns = static_cast<dp::i64>(node_->now().nanoseconds());

                    // turtlesim provides planar pose + yaw + linear/angular velocity.
                    last_fb_.timestamp = ns;
                    last_fb_.value.tick_seq++;
                    last_fb_.value.pose.point.x = static_cast<dp::f64>(msg->x);
                    last_fb_.value.pose.point.y = static_cast<dp::f64>(msg->y);
                    last_fb_.value.pose.point.z = 0.0;
                    last_fb_.value.pose.rotation =
                        datapod::Quaternion::from_euler(0.0, 0.0, static_cast<dp::f64>(msg->theta));

                    last_fb_.value.twist.linear.vx = static_cast<dp::f64>(msg->linear_velocity);
                    last_fb_.value.twist.linear.vy = 0.0;
                    last_fb_.value.twist.linear.vz = 0.0;
                    last_fb_.value.twist.angular.vx = 0.0;
                    last_fb_.value.twist.angular.vy = 0.0;
                    last_fb_.value.twist.angular.vz = static_cast<dp::f64>(msg->angular_velocity);

                    fb_ready_ = true;

                    last_pose_ = dp::Stamp<dp::Pose>{ns, last_fb_.value.pose};
                    last_twist_ = dp::Stamp<dp::Twist>{ns, last_fb_.value.twist};
                    pose_ready_ = true;
                    twist_ready_ = true;
                });

            joint_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
                "joint_states", 10, [this](sensor_msgs::msg::JointState::SharedPtr msg) {
                    std::lock_guard<std::mutex> lock(fb_mutex_);
                    last_fb_.value.wheels.resize(msg->position.size());
                    for (dp::usize i = 0; i < msg->position.size(); ++i) {
                        last_fb_.value.wheels[i].angle_rad = msg->position[i];
                        if (i < msg->velocity.size()) {
                            last_fb_.value.wheels[i].speed_rps = msg->velocity[i];
                        }
                    }
                });

            scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
                "scan", 10, [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
                    std::lock_guard<std::mutex> lock(fb_mutex_);

                    LidarData out;
                    out.angle_min = static_cast<dp::f32>(msg->angle_min);
                    out.angle_max = static_cast<dp::f32>(msg->angle_max);
                    out.angle_increment = static_cast<dp::f32>(msg->angle_increment);
                    out.time_increment = static_cast<dp::f32>(msg->time_increment);
                    out.scan_time = static_cast<dp::f32>(msg->scan_time);
                    out.range_min = static_cast<dp::f32>(msg->range_min);
                    out.range_max = static_cast<dp::f32>(msg->range_max);
                    out.ranges_m.clear();
                    out.ranges_m.reserve(msg->ranges.size());
                    for (const auto r : msg->ranges) {
                        out.ranges_m.push_back(static_cast<dp::f32>(r));
                    }
                    out.intensities.clear();
                    out.intensities.reserve(msg->intensities.size());
                    for (const auto it : msg->intensities) {
                        out.intensities.push_back(static_cast<dp::f32>(it));
                    }

                    const auto ns = static_cast<dp::i64>(msg->header.stamp.sec) * 1'000'000'000LL +
                                    static_cast<dp::i64>(msg->header.stamp.nanosec);
                    last_lidar_ = dp::Stamp<LidarData>{ns, out};
                    lidar_ready_ = true;
                });

            imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
                "imu", 10, [this](sensor_msgs::msg::Imu::SharedPtr msg) {
                    std::lock_guard<std::mutex> lock(fb_mutex_);

                    ImuData out;
                    out.accel_x = static_cast<dp::f32>(msg->linear_acceleration.x);
                    out.accel_y = static_cast<dp::f32>(msg->linear_acceleration.y);
                    out.accel_z = static_cast<dp::f32>(msg->linear_acceleration.z);
                    out.gyro_x = static_cast<dp::f32>(msg->angular_velocity.x);
                    out.gyro_y = static_cast<dp::f32>(msg->angular_velocity.y);
                    out.gyro_z = static_cast<dp::f32>(msg->angular_velocity.z);

                    // yaw from orientation quaternion
                    dp::f64 siny =
                        2.0 * (msg->orientation.w * msg->orientation.z + msg->orientation.x * msg->orientation.y);
                    dp::f64 cosy =
                        1.0 - 2.0 * (msg->orientation.y * msg->orientation.y + msg->orientation.z * msg->orientation.z);
                    out.yaw_rad = static_cast<dp::f32>(std::atan2(siny, cosy));

                    const auto ns = static_cast<dp::i64>(msg->header.stamp.sec) * 1'000'000'000LL +
                                    static_cast<dp::i64>(msg->header.stamp.nanosec);
                    last_imu_ = dp::Stamp<ImuData>{ns, out};
                    imu_ready_ = true;
                });

            gnss_sub_ = node_->create_subscription<sensor_msgs::msg::NavSatFix>(
                "gnss", 10, [this](sensor_msgs::msg::NavSatFix::SharedPtr msg) {
                    std::lock_guard<std::mutex> lock(fb_mutex_);

                    GnssData out;
                    out.latitude_deg = msg->latitude;
                    out.longitude_deg = msg->longitude;
                    out.altitude_m = msg->altitude;

                    const auto ns = static_cast<dp::i64>(msg->header.stamp.sec) * 1'000'000'000LL +
                                    static_cast<dp::i64>(msg->header.stamp.nanosec);
                    last_gnss_ = dp::Stamp<GnssData>{ns, out};
                    gnss_ready_ = true;
                });

            // -- Publisher --

            cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

            return true;
        }

        void disconnect() override {
            odom_sub_.reset();
            pose_sub_.reset();
            joint_sub_.reset();
            scan_sub_.reset();
            imu_sub_.reset();
            gnss_sub_.reset();
            cmd_pub_.reset();
            node_.reset();

            if (owns_init_) {
                rclcpp::shutdown();
                owns_init_ = false;
            }
        }

        bool is_connected() const override { return node_ != nullptr && rclcpp::ok(); }

        bool send(const dp::Stamp<types::Command> &cmd) override {
            if (!cmd_pub_) {
                return false;
            }
            geometry_msgs::msg::Twist twist;
            twist.linear.x = cmd.value.twist.linear.vx;
            twist.linear.y = cmd.value.twist.linear.vy;
            twist.linear.z = cmd.value.twist.linear.vz;
            twist.angular.x = cmd.value.twist.angular.vx;
            twist.angular.y = cmd.value.twist.angular.vy;
            twist.angular.z = cmd.value.twist.angular.vz;
            cmd_pub_->publish(twist);
            return true;
        }

        bool recv(dp::Stamp<types::Feedback> &fb, dp::i32 /*timeout_ms*/ = 100) override {
            if (!node_) {
                return false;
            }
            rclcpp::spin_some(node_);

            std::lock_guard<std::mutex> lock(fb_mutex_);
            if (fb_ready_) {
                fb = last_fb_;
                fb_ready_ = false;
                return true;
            }
            return false;
        }

        /// Read the latest pose with a nanosecond timestamp.
        ///
        /// For stamped messages (e.g. nav_msgs/Odometry), the stamp is derived
        /// from the ROS message header. For turtlesim pose (no header), the
        /// stamp uses node_->now().
        bool read(dp::Stamp<dp::Pose> &out) {
            std::lock_guard<std::mutex> lock(fb_mutex_);
            if (!pose_ready_) {
                return false;
            }
            out = last_pose_;
            pose_ready_ = false;
            return true;
        }

        /// Read the latest twist with a nanosecond timestamp.
        ///
        /// For stamped messages (e.g. nav_msgs/Odometry), the stamp is derived
        /// from the ROS message header. For turtlesim pose (no header), the
        /// stamp uses node_->now().
        bool read(dp::Stamp<dp::Twist> &out) {
            std::lock_guard<std::mutex> lock(fb_mutex_);
            if (!twist_ready_) {
                return false;
            }
            out = last_twist_;
            twist_ready_ = false;
            return true;
        }

        bool read(dp::Stamp<ImuData> &out) {
            std::lock_guard<std::mutex> lock(fb_mutex_);
            if (!imu_ready_) {
                return false;
            }
            out = last_imu_;
            imu_ready_ = false;
            return true;
        }

        bool read(dp::Stamp<GnssData> &out) {
            std::lock_guard<std::mutex> lock(fb_mutex_);
            if (!gnss_ready_) {
                return false;
            }
            out = last_gnss_;
            gnss_ready_ = false;
            return true;
        }

        bool read(dp::Stamp<LidarData> &out) {
            std::lock_guard<std::mutex> lock(fb_mutex_);
            if (!lidar_ready_) {
                return false;
            }
            out = last_lidar_;
            lidar_ready_ = false;
            return true;
        }

      private:
        std::shared_ptr<rclcpp::Node> node_;
        bool owns_init_ = false;

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_sub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

        mutable std::mutex fb_mutex_;
        dp::Stamp<types::Feedback> last_fb_;
        bool fb_ready_ = false;

        dp::Stamp<dp::Pose> last_pose_;
        dp::Stamp<dp::Twist> last_twist_;
        bool pose_ready_ = false;
        bool twist_ready_ = false;

        dp::Stamp<ImuData> last_imu_;
        dp::Stamp<GnssData> last_gnss_;
        dp::Stamp<LidarData> last_lidar_;
        bool imu_ready_ = false;
        bool gnss_ready_ = false;
        bool lidar_ready_ = false;
    };

} // namespace agent47

#endif // AGENT47_HAS_ROS2
