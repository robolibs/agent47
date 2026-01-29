#pragma once

#include <datapod/adapters.hpp>
#include <datapod/pods/sequential/bytes.hpp>
#include <datapod/pods/temporal/stamp.hpp>
#include <datapod/spatial.hpp>

namespace agent47 {
    namespace robot {

        struct Runtime {
            dp::Odom odom;

            // General runtime flags (ported from agent47::model runtime)
            bool allow_move = true;
            bool allow_reverse = true;
            bool navigation_enabled = true;
            dp::f64 speed_scale = 1.0;

            dp::Vector<dp::f64> joint_position;
            dp::Vector<dp::f64> joint_velocity;
            dp::Vector<dp::f64> joint_effort;

            dp::Vector<dp::Stamp<dp::Bytes>> sensor_samples;

            void resize(dp::usize joints, dp::usize sensors) {
                joint_position.assign(joints, 0.0);
                joint_velocity.assign(joints, 0.0);
                joint_effort.assign(joints, 0.0);
                sensor_samples.assign(sensors, dp::Stamp<dp::Bytes>{});
            }
        };

    } // namespace robot
} // namespace agent47
