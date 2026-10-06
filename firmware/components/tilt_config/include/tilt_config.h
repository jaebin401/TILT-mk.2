#pragma once  
// 이건 뭐임?
// 헤더가 중복 include 되는것을 방지

#include <cstdint>

namespace tilt {

inline constexpr float DEG2RAD = 0.01745329251994329577f; // inline constexpr는 뭐지?

inline constexpr float THIGH_LENGTH_MM = 68.0f; 
inline constexpr float CALF_LENGTH_MM = 69.408f;
inline constexpr float FOOT_LENGTH_MM = 7.0f;
inline constexpr float Y_HIP_MM = 13.0f;             // 몸통 중심 → H (hip roll·pitch 축 교차점)
inline constexpr float HIP_LEG_OFFSET_Y_MM = 41.875f; // H → 다리 중심면 (왼다리 +, 오른다리 −)

// The ankle bracket keeps the foot at this fixed angle relative to the calf.
inline constexpr float ANKLE_FIXED_RAD = -20.0f * DEG2RAD;

// The knee bracket contributes this fixed mechanical offset to the calf.
inline constexpr float KNEE_OFFSET_RAD = +0.0f * DEG2RAD;

// mk.1에 있던 yaw 관절 전용 ik 파라미터. roll 에선 불필요
inline constexpr float YAW_SINGULARITY_EPS_MM = 1.0f; // 얜 뭐지?

inline constexpr int IMU_I2C_PORT = 0;
inline constexpr int IMU_SDA_PIN = 8;
inline constexpr int IMU_SCL_PIN = 9;
inline constexpr std::uint8_t IMU_I2C_ADDR = 0x68;
inline constexpr float IMU_FILTER_ALPHA = 0.98f;
inline constexpr std::uint32_t IMU_SAMPLE_PERIOD_MS = 10;

// ── Six-leg-joint / STS3215 configuration ────────────────────────────────
// Keep these values free of ESP-IDF types: this header is also used by the
// host-side kinematics tests. ZERO_TICK and JOINT_SIGN are deliberate
// placeholders until measured with tools/servo_tool_UART.
inline constexpr int NUM_JOINTS = 6;

// servo ID
// | ID |      joint       |
// | 11 | Left Hip Roll    |
// | 12 | Left Hip Pitch   |
// | 13 | Left Knee Pitch  |
// | 21 | Right Hip Roll   |
// | 22 | Right Hip Pitch  |
// | 23 | Right Knee Pitch |

enum JointIndex : int {
    L_HIP_ROLL = 0,
    L_HIP_PITCH = 1,
    L_KNEE_PITCH = 2,
    R_HIP_ROLL = 3,
    R_HIP_PITCH = 4,
    R_KNEE_PITCH = 5,
};

inline constexpr std::uint8_t SERVO_ID[NUM_JOINTS] = {11, 12, 13, 21, 22, 23};
inline constexpr const char* JOINT_NAME[NUM_JOINTS] = {
    "LHR", "LHP", "LKP", "RHR", "RHP", "RKP",
};

// 여기서 inline constexpr 
inline constexpr int SERVO_TICKS_PER_REV = 4096;
inline constexpr int SERVO_POS_MIN = 0;
inline constexpr int SERVO_POS_MAX = 4095;
inline constexpr float DEG_PER_TICK = 360.0f / SERVO_TICKS_PER_REV;
inline constexpr float RAD_PER_TICK = 6.28318530718f / SERVO_TICKS_PER_REV;

// TODO: Measure and replace all six calibration values with tools/servo_tool.
inline constexpr std::uint16_t ZERO_TICK[NUM_JOINTS] = {
    2048,  // L_HIP_ROLL
    2048,  // L_HIP_PITCH
    2048,  // L_KNEE_PITCH
    2048,  // R_HIP_ROLL
    2048,  // R_HIP_PITCH
    2048,  // R_KNEE_PITCH
};

// raw tick 증가 방향이 관절 + 방향과 같으면 +1.
// + 방향(로봇 시선, 오른손 규칙, 좌우 공통):
//   Hip Roll  = 발이 로봇의 왼쪽(+y)으로 (왼다리=벌림, 오른다리=모음)
//   Hip Pitch = 다리가 뒤로 스윙
//   Knee Pitch = 무릎이 굽음
// 아래 값은 예측값 — servo_tool `s` 부호 판별로 확정할 것.

inline constexpr std::int8_t JOINT_SIGN[NUM_JOINTS] = {
    +1, +1, +1,   // L: HR, HP, KP
    -1, -1, -1,   // R: HR, HP, KP
};

// zero pose.
inline constexpr float ZERO_POSE_RAD[NUM_JOINTS] = {
    0.0f * DEG2RAD, -20.0f * DEG2RAD, +40.0f * DEG2RAD,
    0.0f * DEG2RAD, -20.0f * DEG2RAD, +40.0f * DEG2RAD,
};
inline constexpr float ZERO_POSE_HEIGHT_MM = 136.608f; // hip pitch 축 → 발바닥, zero pose CAD 실측

// UART identifiers are integers here to avoid ESP headers in PC builds.
inline constexpr int SERVO_UART_PORT = 1;
inline constexpr int SERVO_UART_TX_PIN = 17;
inline constexpr int SERVO_UART_RX_PIN = 18;
inline constexpr std::uint32_t SERVO_UART_BAUD = 1'000'000;
inline constexpr std::uint32_t CONTROL_PERIOD_MS = 10; 

struct JointLimit {
    float minimum_rad;
    float maximum_rad;
};

// Indexed as [Leg][Hip Roll, Hip Pitch, Knee Pitch]. These are provisional
// software limits and remain separate from the raw-servo calibration table.

// mk.2: roll은 실측 전 임시 ±10°. pitch/knee는 mk.1 실측값 — servo_tool `l`로 재측정 후 교체.
inline constexpr JointLimit JOINT_LIMIT[2][3] = {
    {
        {   -10.0f * DEG2RAD,   +10.0f * DEG2RAD },  // L_HIP_ROLL (임시)
        {   -75.0f * DEG2RAD,   +76.10f * DEG2RAD },  // L_HIP_PITCH (mk.1)
        {   -85.0f * DEG2RAD,  +108.0f * DEG2RAD },  // L_KNEE_PITCH (mk.1)
    },
    {
        {   -10.0f * DEG2RAD,   +10.0f * DEG2RAD },  // R_HIP_ROLL (임시)
        {   -75.0f * DEG2RAD,   +76.10f * DEG2RAD },  // R_HIP_PITCH (mk.1)
        {   -85.0f * DEG2RAD,  +108.0f * DEG2RAD },  // R_KNEE_PITCH (mk.1)
    },
};

}  // namespace tilt
