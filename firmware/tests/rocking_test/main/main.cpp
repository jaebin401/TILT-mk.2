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
constexpr std::size_t kJointCount = tilt::NUM_JOINTS;

enum class ControllerState {
    kUnarmed,
    kArmedHold,
    kMovingToStand,
    kStanding,
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
        case ControllerState::kRocking: return "ROCKING";
    }
    return "UNKNOWN";
}

const char* presetName() {
    return s_preset == tilt::gait::Preset::kVisualization ? "VISUAL"
                                                          : "CONSERVATIVE";
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
    std::printf("  1     CONSERVATIVE preset (still unverified on hardware)\n");
    std::printf("  2     VISUAL preset (2 deg stance, 12 deg swing roll, 14 deg lift; suspended only)\n");
    std::printf("  p     print status\n");
    std::printf("  SPACE emergency stop / torque off\n");
    std::printf("state=%s preset=%s imu=%s\n\n",
                stateName(s_state), presetName(),
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
        s_state == ControllerState::kMovingToStand) {
        std::printf("preset change rejected while moving\n");
        return;
    }
    s_preset = preset;
    s_gait.setParameters(tilt::gait::parametersForPreset(preset));
    std::printf("preset=%s%s\n", presetName(),
                preset == tilt::gait::Preset::kVisualization
                    ? " WARNING: use only while suspended"
                    : "");
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
    std::printf("rocking started preset=%s\n", presetName());
}

void printStatus() {
    std::printf("state=%s preset=%s overruns=%lu ",
                stateName(s_state), presetName(),
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
    std::printf("\n");
}

void handleInput(std::uint8_t input) {
    if (input == ' ') {
        emergencyStop("operator request");
        return;
    }
    const char key = static_cast<char>(std::tolower(input));
    switch (key) {
        case 'o': armServos(); break;
        case 's': beginStandTransition(kStandTransitionMs); break;
        case 'r': startRocking(); break;
        case 'x': beginStandTransition(kStopTransitionMs); break;
        case '1': selectPreset(tilt::gait::Preset::kConservative); break;
        case '2': selectPreset(tilt::gait::Preset::kVisualization); break;
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
