#pragma once  
// 이건 뭐임?
// 헤더가 중복 include 되는것을 방지

#include <cstdint>

namespace tilt {

inline constexpr float DEG2RAD = 0.01745329251994329577f; // inline constexpr는 뭐지?

inline constexpr float THIGH_LENGTH_MM = 68.0f;
inline constexpr float CALF_LENGTH_MM = 70.431f;  // Knee pitch axis -> ankle pitch axis.
// Perpendicular distance from the ankle pitch axis to the foot-pad contact
// plane. The temporary FK/IK model places its sole reference directly below
// the ankle axis by this amount when the sole is flat.
inline constexpr float FOOT_LENGTH_MM = 22.9f;
inline constexpr float Y_HIP_MM = 13.0f;            // 몸통 중심 → H (hip roll·pitch 축 교차점)
inline constexpr float HIP_LEG_OFFSET_Y_MM = 41.875f; // H → 다리 중심면 (왼다리 +, 오른다리 −)

// Sole pitch relative to the torso when every leg pitch joint (hip, knee, ankle) is 0.
// mk.2 + ankle: 0 — the leg is straight and the sole is flat (servo_tool HOME, 2026-10-13).
// sole pitch = ANKLE_FIXED_RAD + hip_pitch + knee_pitch + ankle_pitch  (0 = flat, + = toes down)
// (mk.2 without the ankle had a fixed bracket here: -20 deg.)
inline constexpr float ANKLE_FIXED_RAD = 0.0f * DEG2RAD;

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

// ── Eight-leg-joint / STS3215 configuration ──────────────────────────────
// Keep these values free of ESP-IDF types: this header is also used by the
// host-side kinematics tests. Calibration lives here so every firmware and
// test target uses the same joint order, IDs, zero ticks, signs, and limits.
inline constexpr int NUM_LEG_JOINTS = 4;  // 다리당 관절 수 (roll, pitch, knee, ankle)
inline constexpr int NUM_JOINTS = 2 * NUM_LEG_JOINTS;  // 8

// servo ID
// | ID |      joint        |
// | 11 | Left Hip Roll     |
// | 12 | Left Hip Pitch    |
// | 13 | Left Knee Pitch   |
// | 14 | Left Ankle Pitch  |
// | 21 | Right Hip Roll    |
// | 22 | Right Hip Pitch   |
// | 23 | Right Knee Pitch  |
// | 24 | Right Ankle Pitch |

// 다리별 순서: index = leg * NUM_LEG_JOINTS + (0 roll, 1 pitch, 2 knee, 3 ankle)
enum JointIndex : int {
    L_HIP_ROLL = 0,
    L_HIP_PITCH = 1,
    L_KNEE_PITCH = 2,
    L_ANKLE_PITCH = 3,
    R_HIP_ROLL = 4,
    R_HIP_PITCH = 5,
    R_KNEE_PITCH = 6,
    R_ANKLE_PITCH = 7,
};

inline constexpr std::uint8_t SERVO_ID[NUM_JOINTS] = {11, 12, 13, 14, 21, 22, 23, 24};
inline constexpr const char* JOINT_NAME[NUM_JOINTS] = {
    "LHR", "LHP", "LKP", "LAP", "RHR", "RHP", "RKP", "RAP",
};

// 여기서 inline constexpr 
inline constexpr int SERVO_TICKS_PER_REV = 4096;
inline constexpr int SERVO_POS_MIN = 0;
inline constexpr int SERVO_POS_MAX = 4095;
inline constexpr float DEG_PER_TICK = 360.0f / SERVO_TICKS_PER_REV;
inline constexpr float RAD_PER_TICK = 6.28318530718f / SERVO_TICKS_PER_REV;

inline constexpr std::uint16_t ZERO_TICK[NUM_JOINTS] = {
    2076,  // L_HIP_ROLL
    2051,  // L_HIP_PITCH
    2119,  // L_KNEE_PITCH
    2050,  // L_ANKLE_PITCH
    2050,  // R_HIP_ROLL
    2051,  // R_HIP_PITCH
    2049,  // R_KNEE_PITCH
    2067,  // R_ANKLE_PITCH
};

// raw tick 증가 방향이 관절 + 방향과 같으면 +1.
// + 방향(로봇 시선, 오른손 규칙, 좌우 공통):
//   Hip Roll  = 발이 로봇의 왼쪽(+y)으로 (왼다리=벌림, 오른다리=모음)
//   Hip Pitch = 다리가 뒤로 스윙
//   Knee Pitch = 무릎이 굽음
//   Ankle Pitch = 발끝이 내려감 (toes down)
// 아래 값은 예측값 — servo_tool `s` 부호 판별로 확정할 것.
//
// Ankle Pitch 영점(theta = 0): 발바닥이 정강이와 직각 — 다리를 쭉 편 HOME 에서 발바닥이 지면과 평행.
//   발바닥 평행 조건: hip + knee + ankle = 0  →  zero pose [-20, +40, -20].

inline constexpr std::int8_t JOINT_SIGN[NUM_JOINTS] = {
    -1,  // L_HIP_ROLL
    -1,  // L_HIP_PITCH
    -1,  // L_KNEE_PITCH
    +1,  // L_ANKLE_PITCH (placeholder)
    -1,  // R_HIP_ROLL
    +1,  // R_HIP_PITCH
    +1,  // R_KNEE_PITCH
    -1,  // R_ANKLE_PITCH (placeholder)
};

// Zero pose: hip -20°, knee +40°, ankle -20° on each leg; sole stays flat.
inline constexpr float ZERO_POSE_RAD[NUM_JOINTS] = {
    0.0f * DEG2RAD, -20.0f * DEG2RAD, +40.0f * DEG2RAD, -20.0f * DEG2RAD,
    0.0f * DEG2RAD, -20.0f * DEG2RAD, +40.0f * DEG2RAD, -20.0f * DEG2RAD,
};
// Hip-pitch axis -> sole contact plane at ZERO_POSE_RAD, derived from the
// current 68 / 70.431 / 22.9 mm temporary kinematic model (L/R mean).
inline constexpr float ZERO_POSE_HEIGHT_MM = 152.982f;

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

// Indexed as [Leg][Hip Roll, Hip Pitch, Knee Pitch, Ankle Pitch]. These are provisional
// software limits and remain separate from the raw-servo calibration table.

// 2026-10-13 servo_tool `l` 실측 (붙여넣을 때 min/max 가 뒤바뀌어 있어 작은 값 = minimum 으로 정렬함).
inline constexpr JointLimit JOINT_LIMIT[2][NUM_LEG_JOINTS] = {
    {
        {   -18.8f * DEG2RAD,   +16.7f * DEG2RAD },  // L_HIP_ROLL (실측)
        {   -92.0f * DEG2RAD,   +30.1f * DEG2RAD },  // L_HIP_PITCH (실측)
        {   -90.0f * DEG2RAD,   +90.0f * DEG2RAD },  // L_KNEE_PITCH (실측)
        {   -72.8f * DEG2RAD,   +72.7f * DEG2RAD },  // L_ANKLE_PITCH (실측)
    },
    {
        {   -15.6f * DEG2RAD,   +14.6f * DEG2RAD },  // R_HIP_ROLL (실측)
        {   -90.1f * DEG2RAD,   +29.4f * DEG2RAD },  // R_HIP_PITCH (실측)
        {   -90.2f * DEG2RAD,   +89.8f * DEG2RAD },  // R_KNEE_PITCH (실측)
        {   -74.7f * DEG2RAD,   +75.5f * DEG2RAD },  // R_ANKLE_PITCH (실측)
    },
};

// Joint-array helpers. Keep the per-leg stride in one place so adding a joint
// cannot silently shift every right-leg index again.
inline constexpr int legIndexFromJoint(int joint) {
    return joint / NUM_LEG_JOINTS;
}

inline constexpr int jointIndexInLeg(int joint) {
    return joint % NUM_LEG_JOINTS;
}

inline constexpr int jointIndex(int leg, int joint_in_leg) {
    return leg * NUM_LEG_JOINTS + joint_in_leg;
}

inline constexpr const JointLimit& jointLimit(int joint) {
    return JOINT_LIMIT[legIndexFromJoint(joint)][jointIndexInLeg(joint)];
}


}  // namespace tilt
