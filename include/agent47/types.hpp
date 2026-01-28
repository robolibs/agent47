#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <datapod/adapters.hpp>
#include <datapod/spatial.hpp>

#include "agent47/model/robot.hpp"

namespace agent47 {
    namespace types {

        /// Per-wheel state snapshot.
        struct WheelState {
            double angle_rad = 0.0; // Steering angle (0 for differential drive).
            double speed_rps = 0.0; // Wheel speed in revolutions per second.
        };

        /// Canonical feedback from the robot / simulator.
        ///
        /// Replaces the former Observation struct. Adapters fill this from their
        /// internal state and pass it to Agent::tick().
        struct Feedback {
            uint64_t tick_seq = 0;
            double stamp_s = 0.0;

            datapod::Pose pose{};
            double linear_mps = 0.0;
            double angular_rps = 0.0;

            std::vector<WheelState> wheels;

            // Sensor presence flags + data.
            bool has_lidar = false;
            bool has_gps = false;
            bool has_imu = false;

            model::LidarScan lidar;
            model::GpsFix gps;
            model::ImuSample imu;
        };

        /// Canonical command returned by the agent.
        ///
        /// The base contract is cmd-vel. Lower-level or domain-specific commands can be
        /// added later without breaking existing adapters.
        struct Command {
            double stamp_s = 0.0;
            double linear_mps = 0.0;
            double angular_rps = 0.0;
            bool valid = false;
        };

    } // namespace types
} // namespace agent47
