#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdio>

#include "driver/usb_serial_jtag.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tilt/config/HardwareConfig.h"
#include "tilt/gait/RockingGait.h"
#include "tilt/sts3215/Sts3215Bus.h"
#include "tilt_config.h"
#include "tilt_mpu6050.h"

namespace {

using JointArray = tilt::gait::JointArray;

constexpr std::uint32_t kControlPeriodMs = 10;
constexpr std::int64_t kControlPeriodUs =
    static_cast<std::int64_t>(kControlPeriodMs) * 1000;
constexpr std::uint32_t kStandTransitionMs = 2000;
constexpr std::uint32_t kStopTransitionMs = 600;
constexpr float kEmergencyTiltRad = 35.0f * tilt::DEG2RAD;
constexpr float kRollEditStepRad = 0.5f * tilt::DEG2RAD;
constexpr float kLiftEditStepRad = 1.0f * tilt::DEG2RAD;
constexpr float kLiftEditMinRad = 0.0f;
constexpr float kLiftEditMaxRad = 20.0f * tilt::DEG2RAD;
constexpr std::uint32_t kHalfCycleStepMs = 20;
constexpr std::uint32_t kHalfCycleMinMs = 320;
constexpr std::uint32_t kHalfCycleMaxMs = 800;
constexpr std::size_t kJointCount = tilt::NUM_JOINTS;

enum class ControllerState {
    kUnarmed,
    kArmedHold,
    kMovingToStand,
    kStanding,
    kEditingRollPose,
    kRocking,
};

tilt::sts3215::BusConfig makeBusConfig() {
    tilt::sts3215::BusConfig config;
    config.uart_port = tilt::config::hardware::kServoUartPort;
    config.tx_pin = tilt::config::hardware::kServoUartTxPin;
    config.rx_pin = tilt::config::hardware::kServoUartRxPin;
    config.baud_rate = tilt::config::hardware::kServoUartBaudRate;
    config.response_timeout_ms =
        tilt::config::hardware::kServoResponseTimeoutMs;
    config.rx_buffer_size = tilt::config::hardware::kServoRxBufferSize;
    config.tx_buffer_size = tilt::config::hardware::kServoTxBufferSize;
    return config;
}

tilt::sts3215::Sts3215Bus s_bus(makeBusConfig());
tilt::gait::Preset s_preset = tilt::gait::Preset::kConservative;
tilt::gait::RockingGait s_gait(
    tilt::gait::parametersForPreset(s_preset));
tilt::gait::HipRollMode s_hip_roll_mode =
    tilt::gait::HipRollMode::kPreset;
tilt::ComplementaryFilter s_imu_filter;
tilt::Attitude s_attitude{};

ControllerState s_state = ControllerState::kUnarmed;
JointArray s_command{};
JointArray s_transition_start{};
std::uint32_t s_transition_elapsed_ms = 0;
std::uint32_t s_transition_duration_ms = 0;
bool s_imu_available = false;
std::uint32_t s_overrun_count = 0;
std::uint32_t s_print_divider = 0;
tilt::gait::Phase s_last_phase = tilt::gait::Phase::kIdle;
std::uint64_t s_last_completed_events = 0;
float s_custom_left_support_left_roll_rad = 0.0f;
float s_custom_left_support_right_roll_rad = 0.0f;
float s_edit_left_roll_rad = 0.0f;
float s_edit_right_roll_rad = 0.0f;
float s_custom_lift_rad = 0.0f;
float s_edit_lift_rad = 0.0f;
bool s_custom_lift_enabled = false;
std::uint32_t s_custom_half_cycle_ms = 500;
bool s_custom_half_cycle_enabled = false;
bool s_custom_pose_saved = false;

JointArray standPose() {
    return {
        0.0f,
        -20.0f * tilt::DEG2RAD,
        +40.0f * tilt::DEG2RAD,
        0.0f,
        -20.0f * tilt::DEG2RAD,
        +40.0f * tilt::DEG2RAD,
    };
}

const char* stateName(ControllerState state) {
    switch (state) {
        case ControllerState::kUnarmed: return "UNARMED";
        case ControllerState::kArmedHold: return "ARMED_HOLD";
        case ControllerState::kMovingToStand: return "MOVING_TO_STAND";
        case ControllerState::kStanding: return "STANDING";
        case ControllerState::kEditingRollPose: return "EDITING_ROLL_POSE";
        case ControllerState::kRocking: return "ROCKING";
    }
    return "UNKNOWN";
}

const char* presetName() {
    return s_preset == tilt::gait::Preset::kVisualization ? "VISUAL"
                                                          : "CONSERVATIVE";
}

tilt::gait::RockingParameters configuredGaitParameters() {
    auto parameters = tilt::gait::parametersForPreset(s_preset);
    parameters.hip_roll_mode = s_hip_roll_mode;
    parameters.custom_left_support_left_roll_rad =
        s_custom_left_support_left_roll_rad;
    parameters.custom_left_support_right_roll_rad =
        s_custom_left_support_right_roll_rad;
    if (s_custom_lift_enabled) {
        parameters.lift_hip_delta_rad = -s_custom_lift_rad;
        parameters.lift_knee_delta_rad = +s_custom_lift_rad;
    }
    if (s_custom_half_cycle_enabled) {
        parameters.half_cycle_ms = s_custom_half_cycle_ms;
    }
    return parameters;
}

void applyGaitParameters() {
    s_gait.setParameters(configuredGaitParameters());
}

void printRollEditorStatus() {
    std::printf(
        "custom LEFT-support pose: LHR=%+.1f deg RHR=%+.1f deg "
        "right-leg lift=%.1f deg "
        "| mirrored RIGHT-support: LHR=%+.1f deg RHR=%+.1f deg\n",
        s_edit_left_roll_rad / tilt::DEG2RAD,
        s_edit_right_roll_rad / tilt::DEG2RAD,
        s_edit_lift_rad / tilt::DEG2RAD,
        -s_edit_right_roll_rad / tilt::DEG2RAD,
        -s_edit_left_roll_rad / tilt::DEG2RAD);
}

float smoothstep5(float value) {
    const float s = std::clamp(value, 0.0f, 1.0f);
    return s * s * s * (10.0f + s * (-15.0f + 6.0f * s));
}

bool withinLimits(const JointArray& command, int& bad_joint) {
    for (int joint = 0; joint < tilt::NUM_JOINTS; ++joint) {
        const auto& limit = tilt::JOINT_LIMIT[joint / 3][joint % 3];
        if (!std::isfinite(command[joint]) ||
            command[joint] < limit.minimum_rad ||
            command[joint] > limit.maximum_rad) {
            bad_joint = joint;
            return false;
        }
    }
    bad_joint = -1;
    return true;
}

bool radToTick(int joint, float radians, std::uint16_t& tick) {
    if (joint < 0 || joint >= tilt::NUM_JOINTS ||
        !std::isfinite(radians)) {
        return false;
    }
    const float raw = static_cast<float>(tilt::ZERO_TICK[joint]) +
                      static_cast<float>(tilt::JOINT_SIGN[joint]) *
                          radians / tilt::RAD_PER_TICK;
    const long rounded = std::lround(raw);
    if (rounded < tilt::SERVO_POS_MIN || rounded > tilt::SERVO_POS_MAX) {
        return false;
    }
    tick = static_cast<std::uint16_t>(rounded);
    return true;
}

float tickToRad(int joint, std::uint16_t tick) {
    return static_cast<float>(tilt::JOINT_SIGN[joint]) *
           (static_cast<int>(tick) -
            static_cast<int>(tilt::ZERO_TICK[joint])) *
           tilt::RAD_PER_TICK;
}

bool commandToTicks(const JointArray& command,
                    std::array<std::uint16_t, kJointCount>& ticks,
                    int& bad_joint) {
    if (!withinLimits(command, bad_joint)) return false;
    for (int joint = 0; joint < tilt::NUM_JOINTS; ++joint) {
        if (!radToTick(joint, command[joint], ticks[joint])) {
            bad_joint = joint;
            return false;
        }
    }
    return true;
}

void printHelp() {
    std::printf("\nTILT mk.2 rocking test\n");
    std::printf("  o     arm at measured position\n");
    std::printf("  s     move smoothly to stand\n");
    std::printf("  r     start continuous rocking\n");
    std::printf("  x     stop rocking and return to stand\n");
    std::printf("  1     CONSERVATIVE preset; restore preset lift height\n");
    std::printf("  2     VISUAL preset; restore preset lift height (suspended only)\n");
    std::printf("  -/+   half-cycle -/+20 ms (faster/slower, 320..800 ms)\n");
    std::printf("  m     cycle hip-roll mode: PRESET / PITCH_ONLY / CUSTOM\n");
    std::printf("  e     edit one custom LEFT-support pose (suspended only)\n");
    std::printf("        editor: a/d LHR -/+0.5 deg, j/l RHR -/+0.5 deg\n");
    std::printf("                w/s lift +1/-1 deg (0..20 deg)\n");
    std::printf("                v save+mirror, q cancel, SPACE torque off\n");
    std::printf("  p     print status\n");
    std::printf("  SPACE emergency stop / torque off\n");
    std::printf("state=%s preset=%s roll_mode=%s imu=%s\n\n",
                stateName(s_state), presetName(),
                tilt::gait::hipRollModeName(s_hip_roll_mode),
                s_imu_available ? "ready" : "unavailable");
}

void emergencyStop(const char* reason) {
    const esp_err_t result =
        s_bus.emergencyStop(tilt::SERVO_ID, kJointCount);
    s_gait.stop();
    s_state = ControllerState::kUnarmed;
    std::printf("\n!! EMERGENCY STOP: %s, torque OFF (%s) !!\n",
                reason, esp_err_to_name(result));
}

bool readMeasuredPose(JointArray& measured,
                      std::array<std::uint16_t, kJointCount>& ticks) {
    for (int joint = 0; joint < tilt::NUM_JOINTS; ++joint) {
        const esp_err_t result =
            s_bus.readPosition(tilt::SERVO_ID[joint], ticks[joint]);
        if (result != ESP_OK) {
            std::printf("arm rejected: %s read failed: %s\n",
                        tilt::JOINT_NAME[joint], esp_err_to_name(result));
            return false;
        }
        measured[joint] = tickToRad(joint, ticks[joint]);
        vTaskDelay(pdMS_TO_TICKS(3));
    }
    return true;
}

void armServos() {
    if (s_state != ControllerState::kUnarmed) {
        std::printf("already armed; SPACE first to re-arm\n");
        return;
    }
    std::array<std::uint16_t, kJointCount> ticks{};
    JointArray measured{};
    if (!readMeasuredPose(measured, ticks)) return;

    esp_err_t result = s_bus.syncWritePositions(
        tilt::SERVO_ID, ticks.data(), kJointCount, 0, 0);
    if (result == ESP_OK) {
        result = s_bus.setTorqueAll(tilt::SERVO_ID, kJointCount, true);
    }
    if (result != ESP_OK) {
        std::printf("arm failed: %s\n", esp_err_to_name(result));
        emergencyStop("arm failure");
        return;
    }
    s_command = measured;
    s_state = ControllerState::kArmedHold;
    std::printf("armed at measured pose; press 's' for stand\n");
}

void beginStandTransition(std::uint32_t duration_ms) {
    if (s_state == ControllerState::kUnarmed) {
        std::printf("stand rejected: arm first\n");
        return;
    }
    s_gait.stop();
    s_transition_start = s_command;
    s_transition_elapsed_ms = 0;
    s_transition_duration_ms = duration_ms;
    s_state = ControllerState::kMovingToStand;
}

void selectPreset(tilt::gait::Preset preset) {
    if (s_state == ControllerState::kRocking ||
        s_state == ControllerState::kMovingToStand ||
        s_state == ControllerState::kEditingRollPose) {
        std::printf("preset change rejected while moving\n");
        return;
    }
    s_preset = preset;
    s_custom_lift_enabled = false;
    s_custom_half_cycle_enabled = false;
    applyGaitParameters();
    std::printf("preset=%s, lift height and period restored to preset%s\n",
                presetName(),
                preset == tilt::gait::Preset::kVisualization
                    ? " WARNING: use only while suspended"
                    : "");
}

void adjustHalfCycle(std::int32_t delta_ms) {
    if (s_state == ControllerState::kRocking ||
        s_state == ControllerState::kMovingToStand ||
        s_state == ControllerState::kEditingRollPose) {
        std::printf("period change rejected while moving\n");
        return;
    }
    const std::int32_t current = static_cast<std::int32_t>(
        s_gait.parameters().half_cycle_ms);
    const std::int32_t bounded = std::clamp(
        current + delta_ms,
        static_cast<std::int32_t>(kHalfCycleMinMs),
        static_cast<std::int32_t>(kHalfCycleMaxMs));
    s_custom_half_cycle_ms = static_cast<std::uint32_t>(bounded);
    s_custom_half_cycle_enabled = true;
    applyGaitParameters();
    std::printf("half-cycle=%lu ms full-cycle=%lu ms%s\n",
                static_cast<unsigned long>(s_custom_half_cycle_ms),
                static_cast<unsigned long>(2u * s_custom_half_cycle_ms),
                s_custom_half_cycle_ms <= 360
                    ? " WARNING: aggressive; keep robot suspended" : "");
}

void cycleHipRollMode() {
    if (s_state == ControllerState::kRocking ||
        s_state == ControllerState::kMovingToStand ||
        s_state == ControllerState::kEditingRollPose) {
        std::printf("hip-roll mode change rejected while moving\n");
        return;
    }
    switch (s_hip_roll_mode) {
        case tilt::gait::HipRollMode::kPreset:
            s_hip_roll_mode = tilt::gait::HipRollMode::kDisabled;
            break;
        case tilt::gait::HipRollMode::kDisabled:
            s_hip_roll_mode = tilt::gait::HipRollMode::kCustom;
            break;
        case tilt::gait::HipRollMode::kCustom:
            s_hip_roll_mode = tilt::gait::HipRollMode::kPreset;
            break;
    }
    applyGaitParameters();
    std::printf("hip-roll mode=%s%s\n",
                tilt::gait::hipRollModeName(s_hip_roll_mode),
                s_hip_roll_mode == tilt::gait::HipRollMode::kCustom &&
                        !s_custom_pose_saved
                    ? " (custom pose is still zero)" : "");
}

void enterRollPoseEditor() {
    if (s_state != ControllerState::kStanding) {
        std::printf("editor rejected: reach STANDING first\n");
        return;
    }
    // Always enter at zero to avoid an abrupt jump to an older saved pose.
    s_edit_left_roll_rad = 0.0f;
    s_edit_right_roll_rad = 0.0f;
    s_edit_lift_rad = 0.0f;
    s_state = ControllerState::kEditingRollPose;
    std::printf("custom pose editor started; keep the robot suspended\n");
    printRollEditorStatus();
}

void adjustRollPose(int joint, float delta_rad) {
    if (s_state != ControllerState::kEditingRollPose) return;
    const auto& limit = tilt::JOINT_LIMIT[joint / 3][joint % 3];
    float* value = joint == tilt::L_HIP_ROLL
        ? &s_edit_left_roll_rad : &s_edit_right_roll_rad;
    *value = std::clamp(*value + delta_rad,
                        limit.minimum_rad, limit.maximum_rad);
    printRollEditorStatus();
}

void adjustLiftPose(float delta_rad) {
    if (s_state != ControllerState::kEditingRollPose) return;
    s_edit_lift_rad = std::clamp(s_edit_lift_rad + delta_rad,
                                 kLiftEditMinRad, kLiftEditMaxRad);
    printRollEditorStatus();
}

void finishRollPoseEditor(bool save) {
    if (s_state != ControllerState::kEditingRollPose) return;
    if (save) {
        s_custom_left_support_left_roll_rad = s_edit_left_roll_rad;
        s_custom_left_support_right_roll_rad = s_edit_right_roll_rad;
        s_custom_lift_rad = s_edit_lift_rad;
        s_custom_lift_enabled = true;
        s_custom_pose_saved = true;
        s_hip_roll_mode = tilt::gait::HipRollMode::kCustom;
        applyGaitParameters();
        std::printf("custom pose saved in RAM and CUSTOM mode selected\n");
    } else {
        std::printf("custom pose edit cancelled\n");
    }
    beginStandTransition(kStopTransitionMs);
}

void startRocking() {
    if (s_state != ControllerState::kStanding) {
        std::printf("rocking rejected: reach STANDING first\n");
        return;
    }
    s_gait.start(tilt::gait::Side::kLeft);
    s_last_phase = tilt::gait::Phase::kIdle;
    s_last_completed_events = 0;
    s_state = ControllerState::kRocking;
    std::printf("rocking started preset=%s roll_mode=%s half-cycle=%lu ms\n",
                presetName(),
                tilt::gait::hipRollModeName(s_hip_roll_mode),
                static_cast<unsigned long>(s_gait.parameters().half_cycle_ms));
}

void printStatus() {
    std::printf("state=%s preset=%s roll_mode=%s half-cycle=%lu_ms "
                "full-cycle=%lu_ms overruns=%lu ",
                stateName(s_state), presetName(),
                tilt::gait::hipRollModeName(s_hip_roll_mode),
                static_cast<unsigned long>(s_gait.parameters().half_cycle_ms),
                static_cast<unsigned long>(s_gait.cycleDurationMs()),
                static_cast<unsigned long>(s_overrun_count));
    if (s_imu_available) {
        std::printf("imu_roll=%+.2f imu_pitch=%+.2f ",
                    s_attitude.roll_rad / tilt::DEG2RAD,
                    s_attitude.pitch_rad / tilt::DEG2RAD);
    }
    if (s_state == ControllerState::kRocking) {
        const auto& output = s_gait.output();
        std::printf("phase=%s support=%s events=%llu",
                    tilt::gait::phaseName(output.phase),
                    tilt::gait::sideName(output.support_side),
                    static_cast<unsigned long long>(output.completed_events));
    }
    if (s_custom_pose_saved) {
        std::printf(" custom_left_support=(%+.1f,%+.1f)deg custom_lift=%.1fdeg",
                    s_custom_left_support_left_roll_rad / tilt::DEG2RAD,
                    s_custom_left_support_right_roll_rad / tilt::DEG2RAD,
                    s_custom_lift_rad / tilt::DEG2RAD);
    } else if (s_custom_lift_enabled) {
        std::printf(" custom_lift=%.1fdeg",
                    s_custom_lift_rad / tilt::DEG2RAD);
    }
    std::printf("\n");
}

void handleInput(std::uint8_t input) {
    if (input == ' ') {
        emergencyStop("operator request");
        return;
    }
    const char key = static_cast<char>(std::tolower(input));
    if (s_state == ControllerState::kEditingRollPose) {
        switch (key) {
            case 'a': adjustRollPose(tilt::L_HIP_ROLL, -kRollEditStepRad); break;
            case 'd': adjustRollPose(tilt::L_HIP_ROLL, +kRollEditStepRad); break;
            case 'j': adjustRollPose(tilt::R_HIP_ROLL, -kRollEditStepRad); break;
            case 'l': adjustRollPose(tilt::R_HIP_ROLL, +kRollEditStepRad); break;
            case 'w': adjustLiftPose(+kLiftEditStepRad); break;
            case 's': adjustLiftPose(-kLiftEditStepRad); break;
            case 'v': finishRollPoseEditor(true); break;
            case 'q': finishRollPoseEditor(false); break;
            case 'p': printRollEditorStatus(); break;
            case '?': printHelp(); break;
            default: break;
        }
        return;
    }
    switch (key) {
        case 'o': armServos(); break;
        case 's': beginStandTransition(kStandTransitionMs); break;
        case 'r': startRocking(); break;
        case 'x': beginStandTransition(kStopTransitionMs); break;
        case '1': selectPreset(tilt::gait::Preset::kConservative); break;
        case '2': selectPreset(tilt::gait::Preset::kVisualization); break;
        case '-': adjustHalfCycle(-static_cast<std::int32_t>(kHalfCycleStepMs)); break;
        case '+': adjustHalfCycle(+static_cast<std::int32_t>(kHalfCycleStepMs)); break;
        case 'm': cycleHipRollMode(); break;
        case 'e': enterRollPoseEditor(); break;
        case 'p': printStatus(); break;
        case '?': printHelp(); break;
        default: break;
    }
}

void updateImu() {
    if (!s_imu_available) return;
    tilt::ImuRaw raw{};
    if (tilt::imu_read_raw(raw)) {
        s_attitude = s_imu_filter.update(
            raw, static_cast<float>(kControlPeriodMs) / 1000.0f);
    }
}

void updateCommand() {
    if (s_state == ControllerState::kMovingToStand) {
        s_transition_elapsed_ms = std::min(
            s_transition_elapsed_ms + kControlPeriodMs,
            s_transition_duration_ms);
        const float progress = s_transition_duration_ms == 0
            ? 1.0f
            : static_cast<float>(s_transition_elapsed_ms) /
                  s_transition_duration_ms;
        const float blend = smoothstep5(progress);
        const JointArray stand = standPose();
        for (int joint = 0; joint < tilt::NUM_JOINTS; ++joint) {
            s_command[joint] = s_transition_start[joint] +
                (stand[joint] - s_transition_start[joint]) * blend;
        }
        if (s_transition_elapsed_ms >= s_transition_duration_ms) {
            s_state = ControllerState::kStanding;
            std::printf("reached STANDING\n");
        }
    } else if (s_state == ControllerState::kEditingRollPose) {
        s_command = standPose();
        s_command[tilt::L_HIP_ROLL] = s_edit_left_roll_rad;
        s_command[tilt::R_HIP_ROLL] = s_edit_right_roll_rad;
        // The saved pose is LEFT support, so preview the right swing leg.
        // Equal and opposite hip/knee deltas preserve the fixed foot pitch.
        s_command[tilt::R_HIP_PITCH] -= s_edit_lift_rad;
        s_command[tilt::R_KNEE_PITCH] += s_edit_lift_rad;
    } else if (s_state == ControllerState::kRocking) {
        const auto output = s_gait.update(kControlPeriodMs);
        s_command = output.position_rad;
        if (output.phase != s_last_phase ||
            output.completed_events != s_last_completed_events) {
            std::printf("phase=%s support=%s events=%llu\n",
                        tilt::gait::phaseName(output.phase),
                        tilt::gait::sideName(output.support_side),
                        static_cast<unsigned long long>(output.completed_events));
            s_last_phase = output.phase;
            s_last_completed_events = output.completed_events;
        }
    }
}

void writeCommand() {
    std::array<std::uint16_t, kJointCount> ticks{};
    int bad_joint = -1;
    if (!commandToTicks(s_command, ticks, bad_joint)) {
        emergencyStop(bad_joint >= 0 ? tilt::JOINT_NAME[bad_joint]
                                     : "invalid command");
        return;
    }
    const esp_err_t result = s_bus.syncWritePositions(
        tilt::SERVO_ID, ticks.data(), kJointCount, 0, 0);
    if (result != ESP_OK) {
        emergencyStop("sync write failure");
    }
}

void controlTask(void*) {
    TickType_t last_wake = xTaskGetTickCount();
    while (true) {
        const std::int64_t cycle_start = esp_timer_get_time();

        std::uint8_t input = 0;
        while (usb_serial_jtag_read_bytes(&input, 1, 0) > 0) {
            handleInput(input);
        }

        updateImu();
        if (s_state == ControllerState::kRocking && s_imu_available &&
            (std::fabs(s_attitude.roll_rad) > kEmergencyTiltRad ||
             std::fabs(s_attitude.pitch_rad) > kEmergencyTiltRad)) {
            emergencyStop("IMU tilt limit");
        }

        if (s_state != ControllerState::kUnarmed) {
            updateCommand();
            if (s_state != ControllerState::kUnarmed) writeCommand();
        }

        if (++s_print_divider >= 100) {
            s_print_divider = 0;
            if (s_state == ControllerState::kRocking) printStatus();
        }

        if (esp_timer_get_time() - cycle_start > kControlPeriodUs) {
            ++s_overrun_count;
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(kControlPeriodMs));
    }
}

void initializeServos() {
    for (int joint = 0; joint < tilt::NUM_JOINTS; ++joint) {
        const std::uint8_t id = tilt::SERVO_ID[joint];
        const esp_err_t ping = s_bus.ping(id);
        std::printf("[SERVO] %s ID %u: %s\n",
                    tilt::JOINT_NAME[joint], id,
                    ping == ESP_OK ? "OK" : esp_err_to_name(ping));
        s_bus.setAcceleration(id, 0);
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    s_bus.setTorqueAll(tilt::SERVO_ID, kJointCount, false);
}

}  // namespace

extern "C" void app_main() {
    if (s_bus.initialize() != ESP_OK) {
        std::printf("servo bus initialization failed\n");
        return;
    }

    usb_serial_jtag_driver_config_t console_config =
        USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    const esp_err_t console_result =
        usb_serial_jtag_driver_install(&console_config);
    if (console_result != ESP_OK && console_result != ESP_ERR_INVALID_STATE) {
        std::printf("USB console initialization failed: %s\n",
                    esp_err_to_name(console_result));
        return;
    }

    initializeServos();
    s_imu_available = tilt::imu_init();
    if (!s_imu_available) {
        std::printf("[IMU] unavailable; visual preset must remain suspended\n");
    } else {
        std::printf("[IMU] ready; emergency limit = 35 deg\n");
    }
    printHelp();

    if (xTaskCreate(controlTask, "rocking_control", 8192, nullptr, 5, nullptr) !=
        pdPASS) {
        emergencyStop("task creation failure");
    }
}
