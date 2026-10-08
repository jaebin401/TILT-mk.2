#pragma once

// MPU6050 chip axes -> robot axes (+x forward, +y left, +z up).
// Applied to both accelerometer and gyro before the complementary filter.
// Determine these with the 'i' monitor (see README, "IMU check"), then set
// kImuVerified = true so DCM stepping is allowed right after boot.

namespace tilt_dcm_test {

inline constexpr bool kImuSwapXY = false;  // robot x <- chip y, robot y <- chip x
inline constexpr int kImuSignX = +1;
inline constexpr int kImuSignY = +1;
inline constexpr int kImuSignZ = +1;
inline constexpr bool kImuVerified = false;

}  // namespace tilt_dcm_test
