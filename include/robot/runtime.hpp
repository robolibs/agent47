#pragma once

#include <datapod/adapters.hpp>
#include <datapod/pods/adapters/optional.hpp>
#include <datapod/pods/adapters/result.hpp>
#include <datapod/pods/sequential/bytes.hpp>
#include <datapod/pods/temporal/stamp.hpp>
#include <datapod/spatial.hpp>

namespace robot {

    struct Runtime {
        // World pose/twist of the robot root. Higher-level code can decide
        // whether this is map->base_link or odom->base_link.
        dp::Odom odom;

        // Joint states indexed by robot::Model::joints[index]
        dp::Vector<dp::f64> joint_position;
        dp::Vector<dp::f64> joint_velocity;
        dp::Vector<dp::f64> joint_effort;

        // Placeholder for stamped sensor samples indexed by Model::sensors.
        // Each sensor can define its own payload type; keep raw bytes for now.
        dp::Vector<dp::Stamp<dp::Bytes>> sensor_samples;

        void resize(dp::usize joints, dp::usize sensors) {
            joint_position.assign(joints, 0.0);
            joint_velocity.assign(joints, 0.0);
            joint_effort.assign(joints, 0.0);
            sensor_samples.assign(sensors, dp::Stamp<dp::Bytes>{});
        }
    };

} // namespace robot
