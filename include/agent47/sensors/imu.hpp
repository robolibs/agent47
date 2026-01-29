#pragma once

#include <agent47/sensor.hpp>
#include <datapod/datapod.hpp>

struct ImuData {
    dp::f32 accel_x = 0.0F;
    dp::f32 accel_y = 0.0F;
    dp::f32 accel_z = 0.0F;
    dp::f32 gyro_x = 0.0F;
    dp::f32 gyro_y = 0.0F;
    dp::f32 gyro_z = 0.0F;
    dp::f32 yaw_rad = 0.0F;
};

class ImuSensor : public agent47::Sensor<ImuData> {
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

    bool read(ImuData &out) override {
        (void)out;
        return false;
    }
};
