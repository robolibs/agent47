#pragma once

#include <agent47/sensor.hpp>
#include <agent47/types.hpp>
#include <datapod/datapod.hpp>

namespace agent47 {
    namespace sensors {

        class LidarSensor : public agent47::Sensor<agent47::types::LidarData> {
          public:
            LidarSensor() = default;
            ~LidarSensor() override = default;

            LidarSensor(const LidarSensor &) = delete;
            LidarSensor &operator=(const LidarSensor &) = delete;

            bool connect(const dp::String &endpoint) override {
                (void)endpoint;
                return false;
            }

            void disconnect() override {}

            bool is_connected() const override { return false; }

            bool read(agent47::types::LidarData &out) override {
                (void)out;
                return false;
            }
        };

    } // namespace sensors
} // namespace agent47
