#pragma once

#include <string>
#include <vector>

#include <datapod/adapters.hpp>
#include <datapod/spatial.hpp>

#include <datapod/robot.hpp>
#include <datapod/serialization/buf.hpp>
#include <datapod/serialization/serialize.hpp>

namespace agent47 {
    namespace types {

        // =========================================================================================
        // Sensor payload types (carried in SensorPacket.payload)
        // =========================================================================================

        struct ImuData {
            dp::f32 accel_x = 0.0F;
            dp::f32 accel_y = 0.0F;
            dp::f32 accel_z = 0.0F;
            dp::f32 gyro_x = 0.0F;
            dp::f32 gyro_y = 0.0F;
            dp::f32 gyro_z = 0.0F;
            dp::f32 yaw_rad = 0.0F;
        };

        struct GnssData {
            dp::f64 latitude_deg = 0.0;
            dp::f64 longitude_deg = 0.0;
            dp::f64 altitude_m = 0.0;
        };

        struct LidarData {
            dp::f32 angle_min = 0.0;
            dp::f32 angle_max = 0.0;
            dp::f32 angle_increment = 0.0;
            dp::f32 time_increment = 0.0;
            dp::f32 scan_time = 0.0;
            dp::f32 range_min = 0.0;
            dp::f32 range_max = 0.0;
            dp::Vector<dp::f32> ranges_m;
            dp::Vector<dp::f32> intensities;
        };

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

        inline bool decode_lidar(const SensorPacket &pkt, LidarData &out) {
            if (pkt.kind != SensorKind::Lidar) {
                return false;
            }
            out = dp::deserialize<dp::Mode::WITH_VERSION, LidarData>(pkt.payload);
            return true;
        }

        inline bool decode_imu(const SensorPacket &pkt, ImuData &out) {
            if (pkt.kind != SensorKind::Imu) {
                return false;
            }
            out = dp::deserialize<dp::Mode::WITH_VERSION, ImuData>(pkt.payload);
            return true;
        }

        inline bool decode_gnss(const SensorPacket &pkt, GnssData &out) {
            if (pkt.kind != SensorKind::Gnss) {
                return false;
            }
            out = dp::deserialize<dp::Mode::WITH_VERSION, GnssData>(pkt.payload);
            return true;
        }

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
