#pragma once

#include <cstdint>

namespace tilt {

inline constexpr float DEG2RAD = 0.01745329251994329577f;

inline constexpr float THIGH_LENGTH_MM = 50.0f;
inline constexpr float CALF_LENGTH_MM = 50.0f;
inline constexpr float FOOT_LENGTH_MM = 10.0f;
inline constexpr float Y_HIP_MM = 36.2f;

// The ankle bracket keeps the foot at this fixed angle relative to the calf.
inline constexpr float ANKLE_FIXED_RAD = -20.0f * DEG2RAD;

// The knee bracket contributes this fixed mechanical offset to the calf.
inline constexpr float KNEE_OFFSET_RAD = +20.0f * DEG2RAD;

inline constexpr float YAW_SINGULARITY_EPS_MM = 1.0f;

inline constexpr int IMU_I2C_PORT = 0;
inline constexpr int IMU_SDA_PIN = 8;
inline constexpr int IMU_SCL_PIN = 9;
inline constexpr std::uint8_t IMU_I2C_ADDR = 0x68;
inline constexpr float IMU_FILTER_ALPHA = 0.98f;
inline constexpr std::uint32_t IMU_SAMPLE_PERIOD_MS = 10;

// ── Six-leg-joint / STS3215 configuration ────────────────────────────────
// Keep these values free of ESP-IDF types: this header is also used by the
// host-side kinematics tests. ZERO_TICK and JOINT_SIGN are deliberate
// placeholders until measured with tools/servo_tool.
inline constexpr int NUM_JOINTS = 6;

enum JointIndex : int {
    L_HIP_YAW = 0,
    L_HIP_PITCH = 1,
    L_KNEE_PITCH = 2,
    R_HIP_YAW = 3,
    R_HIP_PITCH = 4,
    R_KNEE_PITCH = 5,
};

inline constexpr std::uint8_t SERVO_ID[NUM_JOINTS] = {11, 12, 13, 21, 22, 23};
inline constexpr const char* JOINT_NAME[NUM_JOINTS] = {
    "LHY", "LHP", "LKP", "RHY", "RHP", "RKP",
};

inline constexpr int SERVO_TICKS_PER_REV = 4096;
inline constexpr int SERVO_POS_MIN = 0;
inline constexpr int SERVO_POS_MAX = 4095;
inline constexpr float DEG_PER_TICK = 360.0f / SERVO_TICKS_PER_REV;
inline constexpr float RAD_PER_TICK = 6.28318530718f / SERVO_TICKS_PER_REV;

// TODO: Measure and replace all six calibration values with tools/servo_tool.
inline constexpr std::uint16_t ZERO_TICK[NUM_JOINTS] = {
    2117,  // L_HIP_YAW
    2027,  // L_HIP_PITCH
    2073,  // L_KNEE_PITCH
    2024,  // R_HIP_YAW
    2075,  // R_HIP_PITCH
    2020,  // R_KNEE_PITCH
};
inline constexpr std::int8_t JOINT_SIGN[NUM_JOINTS] = {
    +1, +1, +1, +1, +1, +1,
};

// ADR-008 zero pose.
inline constexpr float ZERO_POSE_RAD[NUM_JOINTS] = {
    0.0f * DEG2RAD, -20.0f * DEG2RAD, +20.0f * DEG2RAD,
    0.0f * DEG2RAD, -20.0f * DEG2RAD, +20.0f * DEG2RAD,
};
inline constexpr float ZERO_POSE_HEIGHT_MM = 103.97f;

// UART identifiers are integers here to avoid ESP headers in PC builds.
inline constexpr int SERVO_UART_PORT = 1;
inline constexpr int SERVO_UART_TX_PIN = 17;
inline constexpr int SERVO_UART_RX_PIN = 18;
inline constexpr std::uint32_t SERVO_UART_BAUD = 1'000'000;
inline constexpr std::uint32_t CONTROL_PERIOD_MS = 30;

struct JointLimit {
    float minimum_rad;
    float maximum_rad;
};

// Indexed as [Leg][Hip Yaw, Hip Pitch, Knee Pitch]. These are provisional
// software limits and remain separate from the raw-servo calibration table.
inline constexpr JointLimit JOINT_LIMIT[2][3] = {
    {
        {   -85.0f * DEG2RAD,   +95.0f * DEG2RAD },  // L_HIP_YAW
        {   -75.0f * DEG2RAD,   +76.10f * DEG2RAD },  // L_HIP_PITCH
        {   -85.0f * DEG2RAD,  +108.0f * DEG2RAD },  // L_KNEE_PITCH
    },
    {
        {   -85.0f * DEG2RAD,   +95.0f * DEG2RAD },  // R_HIP_YAW
        {   -75.0f * DEG2RAD,   +76.10f * DEG2RAD },  // R_HIP_PITCH
        {   -85.0f * DEG2RAD,  +108.0f * DEG2RAD },  // R_KNEE_PITCH
    },
};

}  // namespace tilt
