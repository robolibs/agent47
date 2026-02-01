#pragma once

#include <agent47/sensor.hpp>
#include <agent47/types.hpp>
#include <datapod/datapod.hpp>

namespace agent47 {
    namespace sensors {

        class ImuSensor : public agent47::Sensor<agent47::types::ImuData> {
          public:
            ImuSensor() = default;
            ~ImuSensor() override = default;

            ImuSensor(const ImuSensor &) = delete;
            ImuSensor &operator=(const ImuSensor &) = delete;

            bool connect(const dp::String &endpoint) override {
                (void)endpoint;
                return false;
            }

            void disconnect() override {}

            bool is_connected() const override { return false; }

            bool read(agent47::types::ImuData &out) override {
                (void)out;
                return false;
            }
        };

    } // namespace sensors
} // namespace agent47
