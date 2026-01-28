#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include <datapod/adapters.hpp>
#include <datapod/spatial.hpp>

#include "agent47/model/entity.hpp"

#include <drivekit/types.hpp>

namespace agent47 {
    namespace model {

        enum class Role : uint8_t {
            MASTER = 0,
            FOLLOWER = 1,
            SLAVE = 2,
        };

        enum class OpMode : uint8_t {
            IDLE = 0,
            CHARGING = 1,
            STOP = 2,
            PAUSE = 3,
            EMERGENCY = 4,
            TRANSPORT = 5,
            WORK = 6,
        };

        struct Identity {
            std::string uuid;
            std::string name;
            std::string type;

            uint32_t group = 0;
            uint32_t rci = 0;

            Role role = Role::MASTER;
        };

        struct Frames {
            // GPS reference used for ENU/WGS conversions.
            dp::Optional<datapod::Geo> datum;

            // Frame ids are adapter-defined but should be consistent across systems.
            std::string frame_map = "map";
            std::string frame_odom = "odom";
            std::string frame_base = "base_link";
        };

        struct Runtime {
            uint64_t tick_seq = 0;
            double stamp_s = 0.0;

            bool online = true;
            OpMode mode = OpMode::IDLE;

            // Safety gates.
            bool allow_move = true;
            bool allow_reverse = true;
            bool turn_first = false;

            // Scale factor applied by the consumer of the command (0..1).
            float speed_scale = 1.0f;

            datapod::Pose pose{};

            // Optional velocities (robot-local convention).
            dp::Optional<double> linear_mps;
            dp::Optional<double> angular_rps;
        };

        struct LidarConfig {
            bool enabled = false;
            float min_range_m = 0.5f;
            float max_range_m = 15.0f;
            float fov_deg = 45.0f;
            float resolution_deg = 3.0f;
        };

        struct SensorConfig {
            // Presence + configuration. Keep this minimal and extend as needed.
            std::optional<LidarConfig> lidar;
            bool has_gps = false;
            bool has_imu = false;
        };

        struct LidarScan {
            std::vector<float> ranges_m;
            std::vector<float> angles_rad;
            std::vector<uint8_t> valid;
            float min_range_m = 0.1f;
            float max_range_m = 30.0f;
        };

        struct GpsFix {
            double latitude = 0.0;
            double longitude = 0.0;
            double altitude = 0.0;
            float heading_rad = 0.0f;
            float speed_mps = 0.0f;
        };

        struct ImuSample {
            float accel_x = 0.0f;
            float accel_y = 0.0f;
            float accel_z = 9.81f;
            float gyro_z = 0.0f;
            float yaw_rad = 0.0f;
        };

        struct SensorData {
            uint64_t tick_seq = 0;
            double stamp_s = 0.0;

            bool has_lidar = false;
            bool has_gps = false;
            bool has_imu = false;

            LidarScan lidar;
            GpsFix gps;
            ImuSample imu;
        };

        struct Sensors {
            SensorConfig config;
            SensorData data;
        };

        // Everything physical goes here.
        struct Body {
            // Global bounding box and outline (in local body frame).
            datapod::Box bound;
            datapod::Polygon outline;

            // Approximate minimum turning radius (meters).
            float turning_radius_m = 1.0f;

            // Vehicle kinematics/dynamics (optional until adapters provide them).
            dp::Optional<double> wheelbase_m;
            dp::Optional<double> track_width_m;

            // DriveKit constraints stored with the embodiment.
            dp::Optional<drivekit::RobotConstraints> constraints;
        };

        /// Unified robot model used by agent47.
        ///
        /// This is intended to be filled by adapters (simulators or real robots) and
        /// consumed by planning/control modules.
        struct Robot : public Entity {
            Identity identity;
            Frames frames;
            Runtime runtime;
            Sensors sensors;
            Body body;
        };

    } // namespace model
} // namespace agent47
