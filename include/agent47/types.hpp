#pragma once

#include <string>
#include <vector>

#include <datapod/adapters.hpp>
#include <datapod/spatial.hpp>

#include <datapod/robot.hpp>
#include <datapod/serialization/buf.hpp>

namespace agent47 {
    namespace types {

        /// Discriminated wrapper for sensor samples transported over a Bridge.
        ///
        /// For PipeBridge, the payload is a datapod-serialized buffer encoded with
        /// Mode::WITH_VERSION so the receiver can dispatch by type hash.
        ///
        /// Intended usage:
        /// - call Bridge::sensor(packet)
        /// - inspect packet.kind
        /// - deserialize packet.payload into the matching type
        enum class SensorKind : dp::u8 {
            Unknown = 0,
            Lidar = 1,
            Imu = 2,
            Gnss = 3,
            Custom = 100,
        };

        struct SensorPacket {
            dp::i64 timestamp = 0;
            SensorKind kind = SensorKind::Unknown;
            dp::ByteBuf payload;
        };

        /// Per-wheel state snapshot.
        struct WheelState {
            dp::f64 angle_rad = 0.0; // Steering angle (0 for differential drive).
            dp::f64 speed_rps = 0.0; // Wheel speed in revolutions per second.
        };

        /// Canonical feedback from the robot / simulator.
        ///
        /// Replaces the former Observation struct. Adapters fill this from their
        /// internal state and pass it to Agent::tick().
        struct Feedback {
            dp::Pose pose{};
            dp::Twist twist{};
            dp::Vector<WheelState> wheels;
        };

        /// Canonical command returned by the agent.
        ///
        /// The base contract is cmd-vel. Lower-level or domain-specific commands can be
        /// added later without breaking existing adapters.
        struct Command {
            dp::Twist twist{};
            bool valid = false;
        };

    } // namespace types
} // namespace agent47
