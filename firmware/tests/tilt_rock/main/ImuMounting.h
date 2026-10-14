#pragma once

// MPU6050 chip axes -> robot axes (+x forward, +y left, +z up).
// Applied to both accelerometer and gyro before the complementary filter.
// Determine these with the 'i' monitor (see README, "IMU check"), then set
// kImuVerified = true so DCM stepping is allowed right after boot.
// Then stand the robot level and press 'L' to measure the mounting tilt.

namespace tilt_dcm_test {

inline constexpr bool kImuSwapXY = false;  // robot x <- chip y, robot y <- chip x
inline constexpr int kImuSignX = -1;  // 2026-10-08 hardware check: chip rotated 180 deg about z
inline constexpr int kImuSignY = -1;
inline constexpr int kImuSignZ = +1;
inline constexpr bool kImuVerified = true;

// Mounting tilt of the IMU board relative to the robot (after the axis map
// above), measured with the 'L' level calibration while the robot stands
// level. Both accelerometer and gyro vectors are rotated by this before the
// complementary filter, so the filter sees robot-frame vectors.
inline constexpr float kImuMountRollDeg = 0.96f;   // 2026-10-08 'L', standing on the floor (stand pose)
inline constexpr float kImuMountPitchDeg = -0.45f;
// Gyro bias (axis-mapped raw counts, 65.5 counts = 1 deg/s), also from 'L'.
inline constexpr float kImuGyroBiasCounts[3] = {312.9f, -59.6f, -3.2f};  // +4.78 -0.91 -0.05 dps

}  // namespace tilt_dcm_test
