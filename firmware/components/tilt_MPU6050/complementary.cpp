#include "tilt_mpu6050.h"

#include <cmath>
#include <cstdint>

namespace tilt {
namespace {

constexpr float kAccelLsbPerG = 8192.0f;
constexpr float kGyroLsbPerDegreePerSecond = 65.5f;
constexpr float kDegreesToRadians = 0.01745329251994329577f;
constexpr float kMaximumValidDtSeconds = 0.5f;

}  // namespace

float accel_to_g(std::int16_t raw) {
    return static_cast<float>(raw) / kAccelLsbPerG;
}

float gyro_to_rad_s(std::int16_t raw) {
    return static_cast<float>(raw) * kDegreesToRadians /
           kGyroLsbPerDegreePerSecond;
}

ComplementaryFilter::ComplementaryFilter(float alpha) : alpha_(alpha) {
    if (!std::isfinite(alpha_)) {
        alpha_ = 0.98f;
    } else if (alpha_ < 0.0f) {
        alpha_ = 0.0f;
    } else if (alpha_ > 1.0f) {
        alpha_ = 1.0f;
    }
}

void ComplementaryFilter::reset() {
    attitude_ = {0.0f, 0.0f};
    initialized_ = false;
}

Attitude ComplementaryFilter::update(const ImuRaw& raw, float dt_s) {
    const float ax = accel_to_g(raw.ax);
    const float ay = accel_to_g(raw.ay);
    const float az = accel_to_g(raw.az);
    const float roll_acc = std::atan2(ay, std::sqrt(ax * ax + az * az));
    const float pitch_acc = std::atan2(-ax, std::sqrt(ay * ay + az * az));

    const bool accelerometer_valid = std::isfinite(roll_acc) &&
                                     std::isfinite(pitch_acc);
    if (!accelerometer_valid) {
        return attitude_;
    }

    if (!initialized_) {
        attitude_ = {roll_acc, pitch_acc};
        initialized_ = true;
        return attitude_;
    }

    if (!std::isfinite(dt_s) || dt_s <= 0.0f ||
        dt_s > kMaximumValidDtSeconds) {
        attitude_ = {roll_acc, pitch_acc};
        return attitude_;
    }

    const float roll_gyro = attitude_.roll_rad + gyro_to_rad_s(raw.gx) * dt_s;
    const float pitch_gyro = attitude_.pitch_rad + gyro_to_rad_s(raw.gy) * dt_s;
    const float roll = alpha_ * roll_gyro + (1.0f - alpha_) * roll_acc;
    const float pitch = alpha_ * pitch_gyro + (1.0f - alpha_) * pitch_acc;

    attitude_ = {
        std::isfinite(roll) ? roll : roll_acc,
        std::isfinite(pitch) ? pitch : pitch_acc,
    };
    return attitude_;
}

}  // namespace tilt
