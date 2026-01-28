#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <datapod/adapters.hpp>
#include <datapod/spatial.hpp>

namespace agent47 {
    namespace types {

        /// Minimal robot observation.
        ///
        /// This is intentionally simulator/robot agnostic. Adapters are responsible for
        /// mapping their internal state into this structure.
        struct Observation {
            std::string robot_id;
            double stamp_s = 0.0;
            uint64_t tick_seq = 0;

            datapod::Pose pose{};
            dp::Optional<double> linear_mps;
            dp::Optional<double> angular_rps;

            bool allow_move = true;
            bool allow_reverse = true;
        };

        /// Canonical command returned by the agent.
        ///
        /// The base contract is cmd-vel. Lower-level or domain-specific commands can be
        /// added later without breaking existing adapters.
        struct Command {
            // Timestamp in seconds (for debugging / tracing)
            double stamp_s = 0.0;

            // cmd_vel output
            double linear_mps = 0.0;
            double angular_rps = 0.0;

            // If false, command should be ignored (e.g. no goal/path set).
            bool valid = false;

            std::string status;
        };
    } // namespace types
} // namespace agent47
