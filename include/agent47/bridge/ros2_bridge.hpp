#pragma once

#ifdef AGENT47_HAS_ROS2

#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include "agent47/bridge.hpp"

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
                    last_fb_.stamp_s = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
                    last_fb_.tick_seq++;
                    last_fb_.pose.point.x = msg->pose.pose.position.x;
                    last_fb_.pose.point.y = msg->pose.pose.position.y;
                    last_fb_.pose.point.z = msg->pose.pose.position.z;
                    last_fb_.pose.rotation.w = msg->pose.pose.orientation.w;
                    last_fb_.pose.rotation.x = msg->pose.pose.orientation.x;
                    last_fb_.pose.rotation.y = msg->pose.pose.orientation.y;
                    last_fb_.pose.rotation.z = msg->pose.pose.orientation.z;
                    last_fb_.linear_mps = msg->twist.twist.linear.x;
                    last_fb_.angular_rps = msg->twist.twist.angular.z;
                    fb_ready_ = true;
                });

            joint_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
                "joint_states", 10, [this](sensor_msgs::msg::JointState::SharedPtr msg) {
                    std::lock_guard<std::mutex> lock(fb_mutex_);
                    last_fb_.wheels.resize(msg->position.size());
                    for (size_t i = 0; i < msg->position.size(); ++i) {
                        last_fb_.wheels[i].angle_rad = msg->position[i];
                        if (i < msg->velocity.size()) {
                            last_fb_.wheels[i].speed_rps = msg->velocity[i];
                        }
                    }
                });

            scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
                "scan", 10, [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
                    std::lock_guard<std::mutex> lock(fb_mutex_);
                    last_fb_.has_lidar = true;
                    last_fb_.lidar.ranges_m.assign(msg->ranges.begin(), msg->ranges.end());
                    last_fb_.lidar.min_range_m = msg->range_min;
                    last_fb_.lidar.max_range_m = msg->range_max;

                    last_fb_.lidar.angles_rad.resize(msg->ranges.size());
                    for (size_t i = 0; i < msg->ranges.size(); ++i) {
                        last_fb_.lidar.angles_rad[i] = msg->angle_min + i * msg->angle_increment;
                    }
                });

            imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
                "imu", 10, [this](sensor_msgs::msg::Imu::SharedPtr msg) {
                    std::lock_guard<std::mutex> lock(fb_mutex_);
                    last_fb_.has_imu = true;
                    last_fb_.imu.accel_x = static_cast<float>(msg->linear_acceleration.x);
                    last_fb_.imu.accel_y = static_cast<float>(msg->linear_acceleration.y);
                    last_fb_.imu.accel_z = static_cast<float>(msg->linear_acceleration.z);
                    last_fb_.imu.gyro_z = static_cast<float>(msg->angular_velocity.z);
                    // yaw from orientation quaternion (extract yaw from quaternion)
                    double siny =
                        2.0 * (msg->orientation.w * msg->orientation.z + msg->orientation.x * msg->orientation.y);
                    double cosy =
                        1.0 - 2.0 * (msg->orientation.y * msg->orientation.y + msg->orientation.z * msg->orientation.z);
                    last_fb_.imu.yaw_rad = static_cast<float>(std::atan2(siny, cosy));
                });

            gps_sub_ = node_->create_subscription<sensor_msgs::msg::NavSatFix>(
                "gps", 10, [this](sensor_msgs::msg::NavSatFix::SharedPtr msg) {
                    std::lock_guard<std::mutex> lock(fb_mutex_);
                    last_fb_.has_gps = true;
                    last_fb_.gps.latitude = msg->latitude;
                    last_fb_.gps.longitude = msg->longitude;
                    last_fb_.gps.altitude = msg->altitude;
                });

            // -- Publisher --

            cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

            return true;
        }

        void disconnect() override {
            odom_sub_.reset();
            joint_sub_.reset();
            scan_sub_.reset();
            imu_sub_.reset();
            gps_sub_.reset();
            cmd_pub_.reset();
            node_.reset();

            if (owns_init_) {
                rclcpp::shutdown();
                owns_init_ = false;
            }
        }

        bool is_connected() const override { return node_ != nullptr && rclcpp::ok(); }

        bool send(const types::Command &cmd) override {
            if (!cmd_pub_) {
                return false;
            }
            geometry_msgs::msg::Twist twist;
            twist.linear.x = cmd.linear_mps;
            twist.angular.z = cmd.angular_rps;
            cmd_pub_->publish(twist);
            return true;
        }

        bool recv(types::Feedback &fb, int /*timeout_ms*/ = 100) override {
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

      private:
        std::shared_ptr<rclcpp::Node> node_;
        bool owns_init_ = false;

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

        mutable std::mutex fb_mutex_;
        types::Feedback last_fb_;
        bool fb_ready_ = false;
    };

} // namespace agent47

#endif // AGENT47_HAS_ROS2
