#pragma once

#include <tuple>

#include "datapod/pods/adapters/optional.hpp"
#include "datapod/pods/sequential/string.hpp"
#include "datapod/pods/sequential/vector.hpp"
#include "datapod/types/types.hpp"

namespace datapod {
    namespace robot {

        /**
         * @brief Actuator - Transmission actuator description (POD)
         *
         * URDF transmissions describe how joints map to actuators and their mechanical
         * reductions. This POD keeps the actuator name and optional reduction.
         */
        struct Actuator {
            String name;
            Optional<f64> mechanical_reduction;

            auto members() noexcept { return std::tie(name, mechanical_reduction); }
            auto members() const noexcept { return std::tie(name, mechanical_reduction); }

            inline bool operator==(const Actuator &other) const noexcept {
                return name == other.name && mechanical_reduction == other.mechanical_reduction;
            }
            inline bool operator!=(const Actuator &other) const noexcept { return !(*this == other); }
        };

        /**
         * @brief TransmissionJoint - Transmission joint mapping (POD)
         *
         * Represents a joint referenced by name inside a URDF <transmission>.
         */
        struct TransmissionJoint {
            String name;
            Optional<f64> mechanical_reduction;
            Optional<f64> offset;

            auto members() noexcept { return std::tie(name, mechanical_reduction, offset); }
            auto members() const noexcept { return std::tie(name, mechanical_reduction, offset); }

            inline bool operator==(const TransmissionJoint &other) const noexcept {
                return name == other.name && mechanical_reduction == other.mechanical_reduction &&
                       offset == other.offset;
            }
            inline bool operator!=(const TransmissionJoint &other) const noexcept { return !(*this == other); }
        };

        /**
         * @brief Transmission - URDF transmission block (POD)
         *
         * This captures common ROS-control style URDF transmissions.
         *
         * Notes:
         * - Joint/actuator names are stored as strings (URDF references).
         * - Hardware interfaces are intentionally not modeled here.
         */
        struct Transmission {
            String name;
            String type;
            Vector<TransmissionJoint> joints;
            Vector<Actuator> actuators;

            auto members() noexcept { return std::tie(name, type, joints, actuators); }
            auto members() const noexcept { return std::tie(name, type, joints, actuators); }

            inline bool operator==(const Transmission &other) const noexcept {
                return name == other.name && type == other.type && joints == other.joints &&
                       actuators == other.actuators;
            }
            inline bool operator!=(const Transmission &other) const noexcept { return !(*this == other); }
        };

    } // namespace robot
} // namespace datapod
