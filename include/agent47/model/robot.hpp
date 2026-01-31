#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include <datapod/adapters.hpp>
#include <datapod/spatial.hpp>

#include "agent47/model/entity.hpp"

#include <drivekit/types.hpp>

namespace agent47 {
    namespace model {

        enum class Role : dp::u8 {
            MASTER = 0,
            FOLLOWER = 1,
            SLAVE = 2,
        };

        enum class SteeringType {
            DIFFERENTIAL, // Differential drive (can turn in place)
            ACKERMANN,    // Ackermann steering (needs forward motion to turn)
            HOLONOMIC,    // Omnidirectional (can move in any direction)
            SKID_STEER    // Skid steering (like differential but with lateral motion)
        };

        enum class OpMode : dp::u8 {
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
            dp::u32 group = 0;
            dp::u32 rci = 0;
            Role role = Role::MASTER;
        };

        struct Runtime {
            bool online = true;
            OpMode mode = OpMode::IDLE;
            bool allow_move = true;
            bool allow_reverse = true;
            bool turn_first = false;
            dp::f32 speed_scale = 1.0f;
            datapod::Pose pose{};
            datapod::Twist twist{};
        };

        struct Constraints {
            double axis_distance = 1.0; // Distance between axles (m)
            double track_width = 0.5;   // Distance between wheels (m)
            double wheel_radius = 0.1;  // Wheel radius (m)

            // Dynamic limits
            double max_linear_velocity = 1.0;      // m/s
            double min_linear_velocity = -0.5;     // m/s (negative for reverse)
            double max_angular_velocity = 1.0;     // rad/s
            double max_linear_acceleration = 1.0;  // m/s²
            double max_angular_acceleration = 1.0; // rad/s²

            // Steering limits (for Ackermann)
            double max_steering_angle = 0.5; // radians
            double max_steering_rate = 1.0;  // rad/s
            double min_turning_radius = 1.0; // meters

            double robot_width = 0.0;  // Robot width
            double robot_length = 0.0; // Robot length
        };

        struct Body {
            dp::Optional<dp::Box> bound;
            dp::Optional<dp::Polygon> outline;
            SteeringType steering_type;
            Constraints constraints;
        };

        /// This is intended to be filled by adapters (simulators or real robots) and
        /// consumed by planning/control modules.
        struct Robot : public Entity {
            Identity identity;
            dp::Geo datum;
            Runtime runtime;
            Body body;
        };

    } // namespace model
} // namespace agent47
