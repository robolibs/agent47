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

        struct Body {
            datapod::Box bound;
            datapod::Polygon outline;
            dp::f32 turning_radius_m = 1.0f;
            dp::Optional<dp::f64> wheelbase_m;
            dp::Optional<dp::f64> track_width_m;
            dp::Optional<drivekit::RobotConstraints> constraints;
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
