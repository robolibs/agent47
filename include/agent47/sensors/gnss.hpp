#pragma once

#include <agent47/sensor.hpp>
#include <datapod/datapod.hpp>

struct GnssData {
    dp::f64 latitude_deg = 0.0;
    dp::f64 longitude_deg = 0.0;
    dp::f64 altitude_m = 0.0;
};

class GnssSensor : public agent47::Sensor<GnssData> {
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

    bool read(GnssData &out) override {
        (void)out;
        return false;
    }
};
