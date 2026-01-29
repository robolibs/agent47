#pragma once

#include <string>
#include <vector>

#include <datapod/adapters.hpp>
#include <datapod/spatial.hpp>

#include "agent47/model/robot.hpp"

namespace agent47 {
    namespace types {

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
            dp::u64 tick_seq = 0;
            dp::Pose pose{};
            dp::Twist twist{};
            dp::Vector<WheelState> wheels;
        };

        /// Canonical command returned by the agent.
        ///
        /// The base contract is cmd-vel. Lower-level or domain-specific commands can be
        /// added later without breaking existing adapters.
        struct Command {
            dp::i64 timestamp_ns = 0;
            dp::Twist twist{};
            bool valid = false;
        };

    } // namespace types
} // namespace agent47
