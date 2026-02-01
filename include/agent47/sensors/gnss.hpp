#pragma once

#include <agent47/sensor.hpp>
#include <agent47/types.hpp>
#include <datapod/datapod.hpp>

namespace agent47 {
    namespace sensors {

        class GnssSensor : public agent47::Sensor<agent47::types::GnssData> {
          public:
            GnssSensor() = default;
            ~GnssSensor() override = default;

            GnssSensor(const GnssSensor &) = delete;
            GnssSensor &operator=(const GnssSensor &) = delete;

            bool connect(const dp::String &endpoint) override {
                (void)endpoint;
                return false;
            }

            void disconnect() override {}

            bool is_connected() const override { return false; }

            bool read(agent47::types::GnssData &out) override {
                (void)out;
                return false;
            }
        };

    } // namespace sensors
} // namespace agent47
