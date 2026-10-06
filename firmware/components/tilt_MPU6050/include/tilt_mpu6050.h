#pragma once

#include <cstdint>

#include "tilt_config.h"

namespace tilt {

struct ImuRaw {
    std::int16_t ax;
    std::int16_t ay;
    std::int16_t az;
    std::int16_t gx;
    std::int16_t gy;
    std::int16_t gz;
};

struct Attitude {
    float roll_rad;
    float pitch_rad;
};

bool imu_init();
bool imu_read_raw(ImuRaw& out);

float accel_to_g(std::int16_t raw);
float gyro_to_rad_s(std::int16_t raw);

class ComplementaryFilter {
public:
    explicit ComplementaryFilter(float alpha = IMU_FILTER_ALPHA);

    void reset();
    Attitude update(const ImuRaw& raw, float dt_s);

    Attitude attitude() const { return attitude_; }
    bool initialized() const { return initialized_; }

private:
    float alpha_;
    Attitude attitude_{0.0f, 0.0f};
    bool initialized_ = false;
};

}  // namespace tilt
