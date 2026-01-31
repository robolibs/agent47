#pragma once

#include <agent47/sensor.hpp>
#include <datapod/datapod.hpp>

struct LidarData {
    dp::f32 angle_min = 0.0;
    dp::f32 angle_max = 0.0;
    dp::f32 angle_increment = 0.0;
    dp::f32 time_increment = 0.0;
    dp::f32 scan_time = 0.0;
    dp::f32 range_min = 0.0;
    dp::f32 range_max = 0.0;
    dp::Vector<dp::f32> ranges_m;
    dp::Vector<dp::f32> intensities;
};

class LidarSensor : public agent47::Sensor<LidarData> {
  public:
    LidarSensor() = default;
    ~LidarSensor() override = default;

    LidarSensor(const LidarSensor &) = delete;
    LidarSensor &operator=(const LidarSensor &) = delete;

    bool connect(const dp::String &endpoint) override { return false; }

    void disconnect() override {}

    bool is_connected() const override { return false; }

    bool read(LidarData &out) override {
        (void)out;
        return false;
    }
};
