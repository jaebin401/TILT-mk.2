#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdio>

#include "BodyPoseController.h"
#include "driver/usb_serial_jtag.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "tilt/config/HardwareConfig.h"
#include "tilt/sts3215/Sts3215Bus.h"
#include "tilt/tb6612/Tb6612Driver.h"
#include "tilt_config.h"

#if !CONFIG_IDF_TARGET_ESP32S3
#error "wheel_base targets ESP32-S3"
#endif

namespace {

using JointArray = tilt::wheel_base::JointArray;

constexpr std::size_t kJointCount = tilt::NUM_JOINTS;
constexpr std::uint32_t kControlPeriodMs = 10;
constexpr float kControlDt = kControlPeriodMs / 1000.0f;
constexpr std::int64_t kControlPeriodUs = kControlPeriodMs * 1000;
constexpr std::uint32_t kPoseDurationMs = 2000;
constexpr float kPositionStepMm = 2.0f;
constexpr float kHipPitchStepRad = 0.5f * tilt::DEG2RAD;
constexpr float kInitialDriveSpeed = 0.30f;
constexpr float kDriveSpeedStep = 0.10f;

enum class SystemState {
    kUnarmed,
    kHolding,
    kPoseTransition,
    kIkControl,
    kEmergencyStop,
};

enum class DriveIntent {
    kStop,
    kForward,
    kReverse,
    kTurnLeft,
    kTurnRight,
};

enum class InputState { kNormal, kEscape, kCsi };
enum class ArrowKey { kUp, kDown, kLeft, kRight };

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

tilt::tb6612::Config makeMotorConfig() {
    tilt::tb6612::Config config;
    config.left.pwm = tilt::config::hardware::kWheelLeftPwmPin;
    config.left.in1 = tilt::config::hardware::kWheelLeftIn1Pin;
    config.left.in2 = tilt::config::hardware::kWheelLeftIn2Pin;
    config.left.inverted = tilt::config::hardware::kWheelLeftInverted;
    config.right.pwm = tilt::config::hardware::kWheelRightPwmPin;
    config.right.in1 = tilt::config::hardware::kWheelRightIn1Pin;
    config.right.in2 = tilt::config::hardware::kWheelRightIn2Pin;
    config.right.inverted = tilt::config::hardware::kWheelRightInverted;
    config.standby = tilt::config::hardware::kWheelStandbyPin;
    config.pwm_frequency_hz = 20'000;
    config.timer_resolution_hz = 10'000'000;
    config.max_accel_per_second = 1.0f;
    config.max_jerk_per_second2 = 3.0f;
    return config;
}

tilt::sts3215::Sts3215Bus s_bus(makeBusConfig());
tilt::tb6612::Tb6612Driver s_motor(makeMotorConfig());
tilt::wheel_base::BodyPoseController s_body;

SystemState s_state = SystemState::kUnarmed;
DriveIntent s_drive_intent = DriveIntent::kStop;
InputState s_input_state = InputState::kNormal;
float s_drive_speed = kInitialDriveSpeed;
JointArray s_command{};
JointArray s_transition_start{};
JointArray s_transition_target{};
std::uint32_t s_transition_elapsed_ms = 0;
const char* s_transition_name = nullptr;
bool s_ik_error_reported = false;
std::uint32_t s_overruns = 0;

const char* stateName(SystemState state) {
    switch (state) {
        case SystemState::kUnarmed: return "UNARMED";
        case SystemState::kHolding: return "HOLDING";
        case SystemState::kPoseTransition: return "POSE_TRANSITION";
        case SystemState::kIkControl: return "IK_CONTROL";
        case SystemState::kEmergencyStop: return "E_STOP";
    }
    return "?";
}

const char* driveName(DriveIntent intent) {
    switch (intent) {
        case DriveIntent::kStop: return "STOP";
        case DriveIntent::kForward: return "FORWARD";
        case DriveIntent::kReverse: return "REVERSE";
        case DriveIntent::kTurnLeft: return "TURN_LEFT";
        case DriveIntent::kTurnRight: return "TURN_RIGHT";
    }
    return "?";
}

JointArray homePose() {
    return {};
}

JointArray standPose() {
    JointArray pose{};
    for (int joint = 0; joint < tilt::NUM_JOINTS; ++joint) {
        pose[joint] = tilt::ZERO_POSE_RAD[joint];
    }
    return pose;
}

float radiansToDegrees(float radians) {
    return radians / tilt::DEG2RAD;
}

float smoothstep5(float value) {
    const float s = std::clamp(value, 0.0f, 1.0f);
    return s * s * s * (10.0f + s * (-15.0f + 6.0f * s));
}

bool withinLimits(const JointArray& command, int& bad_joint) {
    for (int joint = 0; joint < tilt::NUM_JOINTS; ++joint) {
        const tilt::JointLimit& limit = tilt::jointLimit(joint);
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
    if (!std::isfinite(radians)) return false;
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
        static_cast<float>(static_cast<int>(tick) -
                           static_cast<int>(tilt::ZERO_TICK[joint])) *
        tilt::RAD_PER_TICK;
}

bool systemArmed() {
    return s_state == SystemState::kHolding ||
           s_state == SystemState::kPoseTransition ||
           s_state == SystemState::kIkControl;
}

void emergencyStop(const char* reason) {
    const esp_err_t motor_result = s_motor.emergencyStop();
    const esp_err_t servo_result =
        s_bus.emergencyStop(tilt::SERVO_ID, kJointCount);
    s_drive_intent = DriveIntent::kStop;
    s_state = SystemState::kEmergencyStop;
    std::printf("\n!! EMERGENCY STOP: %s | motor=%s servo=%s !!\n",
                reason, esp_err_to_name(motor_result),
                esp_err_to_name(servo_result));
}

void applyDriveIntent() {
    const float speed = s_drive_speed;
    switch (s_drive_intent) {
        case DriveIntent::kStop: s_motor.stop(); break;
        case DriveIntent::kForward: s_motor.setTarget(+speed, +speed); break;
        case DriveIntent::kReverse: s_motor.setTarget(-speed, -speed); break;
        case DriveIntent::kTurnLeft: s_motor.setTarget(-speed, +speed); break;
        case DriveIntent::kTurnRight: s_motor.setTarget(+speed, -speed); break;
    }
}

void setDriveIntent(DriveIntent intent) {
    if (s_state != SystemState::kIkControl) {
        std::printf("wheel command rejected: arm and reach HOME or STAND first\n");
        return;
    }
    s_drive_intent = intent;
    applyDriveIntent();
    std::printf("drive=%s target=%+.0f/%+.0f%%\n",
                driveName(intent), s_motor.leftTarget() * 100.0f,
                s_motor.rightTarget() * 100.0f);
}

void stopWheels() {
    s_drive_intent = DriveIntent::kStop;
    s_motor.stop();
    std::printf("wheel stop requested (ramped)\n");
}

bool wheelsStopped() {
    return std::fabs(s_motor.leftTarget()) < 0.001f &&
           std::fabs(s_motor.rightTarget()) < 0.001f &&
           std::fabs(s_motor.leftOutput()) < 0.02f &&
           std::fabs(s_motor.rightOutput()) < 0.02f;
}

void printHelp() {
    std::printf(
        "\nTILT mk.2 wheel_base\n"
        "  Arrow Up / Down    : wheel forward / reverse (latched)\n"
        "  Arrow Left / Right : spin left / right (latched)\n"
        "  + / -              : wheel target speed +/- 10%%\n"
        "  x                  : ramp wheels to stop\n"
        "  o                  : arm at measured joint positions\n"
        "  h / s              : HOME / STAND, then capture IK reference\n"
        "  i / k              : torso up / down by 2 mm\n"
        "  j / l              : torso back / forward by 2 mm\n"
        "  q / e              : hip pitch -/+ 0.5 deg (knee/ankle solve x/z)\n"
        "  r                  : return body target to captured pose\n"
        "  p                  : print status\n"
        "  ?                  : print this help\n"
        "  SPACE              : immediate motor OFF + servo torque OFF\n"
        "\nArrow commands stay active until x, another arrow, or SPACE.\n"
        "HOME is near a straight-leg singularity; test it while suspended.\n\n");
}

void printStatus() {
    const auto& current = s_body.current();
    const auto& target = s_body.target();
    std::printf(
        "\nstate=%s drive=%s selected=%.0f%% output L/R=%+.1f/%+.1f%% "
        "overruns=%lu\n",
        stateName(s_state), driveName(s_drive_intent), s_drive_speed * 100.0f,
        s_motor.leftOutput() * 100.0f, s_motor.rightOutput() * 100.0f,
        static_cast<unsigned long>(s_overruns));
    if (s_body.initialized()) {
        std::printf(
            "body current x=%+.1f z=%+.1f mm hip_offset=%+.1f deg | "
            "target x=%+.1f z=%+.1f hip_offset=%+.1f deg\n",
            current.x_mm, current.z_mm,
            radiansToDegrees(current.hip_pitch_offset_rad),
            target.x_mm, target.z_mm,
            radiansToDegrees(target.hip_pitch_offset_rad));
        std::printf("hip pitch L/R=%+.1f/%+.1f deg\n",
                    radiansToDegrees(s_body.leftHipPitch()),
                    radiansToDegrees(s_body.rightHipPitch()));
    }
    std::printf("joint command deg:");
    for (int joint = 0; joint < tilt::NUM_JOINTS; ++joint) {
        std::printf(" %s=%+.1f", tilt::JOINT_NAME[joint],
                    radiansToDegrees(s_command[joint]));
    }
    std::printf("\n\n");
}

void armSystem() {
    if (systemArmed()) {
        std::printf("already armed; SPACE before re-arming\n");
        return;
    }

    s_motor.emergencyStop();
    std::array<std::uint16_t, kJointCount> ticks{};
    JointArray measured{};
    for (int joint = 0; joint < tilt::NUM_JOINTS; ++joint) {
        const esp_err_t result =
            s_bus.readPosition(tilt::SERVO_ID[joint], ticks[joint]);
        if (result != ESP_OK) {
            std::printf("arm rejected: %s (ID %u) read failed: %s\n",
                        tilt::JOINT_NAME[joint], tilt::SERVO_ID[joint],
                        esp_err_to_name(result));
            return;
        }
        measured[joint] = tickToRad(joint, ticks[joint]);
        vTaskDelay(pdMS_TO_TICKS(3));
    }

    esp_err_t result = s_bus.syncWritePositions(
        tilt::SERVO_ID, ticks.data(), kJointCount, 0, 0);
    if (result == ESP_OK) {
        result = s_bus.setTorqueAll(tilt::SERVO_ID, kJointCount, true);
    }
    if (result == ESP_OK) result = s_motor.enable();
    if (result != ESP_OK) {
        std::printf("arm failed: %s\n", esp_err_to_name(result));
        emergencyStop("arm failure");
        return;
    }

    s_command = measured;
    s_drive_intent = DriveIntent::kStop;
    s_state = SystemState::kHolding;
    s_ik_error_reported = false;
    std::printf("armed at measured pose; press h or s to enter IK control\n");
}

void beginPose(const JointArray& target, const char* name) {
    if (!systemArmed()) {
        std::printf("pose rejected: press o to arm first\n");
        return;
    }
    if (!wheelsStopped()) {
        stopWheels();
        std::printf("pose deferred: wait for wheel output to reach zero, then press %s again\n",
                    name);
        return;
    }
    int bad_joint = -1;
    if (!withinLimits(target, bad_joint)) {
        std::printf("pose rejected: %s exceeds configured limit\n",
                    bad_joint >= 0 ? tilt::JOINT_NAME[bad_joint] : "joint");
        return;
    }
    if (name[0] == 'H') {
        std::printf("WARN: HOME fully extends the legs; use while suspended\n");
    }
    s_transition_start = s_command;
    s_transition_target = target;
    s_transition_elapsed_ms = 0;
    s_transition_name = name;
    s_state = SystemState::kPoseTransition;
    std::printf("moving to %s\n", name);
}

void adjustDriveSpeed(float delta) {
    s_drive_speed = std::clamp(s_drive_speed + delta, 0.10f, 1.0f);
    if (s_drive_intent != DriveIntent::kStop) applyDriveIntent();
    std::printf("wheel target speed=%.0f%%\n", s_drive_speed * 100.0f);
}

void requireIkAndNudge(float x_mm, float z_mm) {
    if (s_state != SystemState::kIkControl) {
        std::printf("IK command rejected: reach HOME or STAND first\n");
        return;
    }
    s_body.nudgePosition(x_mm, z_mm);
}

void requireIkAndNudgeHip(float radians) {
    if (s_state != SystemState::kIkControl) {
        std::printf("IK command rejected: reach HOME or STAND first\n");
        return;
    }
    s_body.nudgeHipPitch(radians);
}

void handleArrow(ArrowKey arrow) {
    switch (arrow) {
        case ArrowKey::kUp: setDriveIntent(DriveIntent::kForward); break;
        case ArrowKey::kDown: setDriveIntent(DriveIntent::kReverse); break;
        case ArrowKey::kLeft: setDriveIntent(DriveIntent::kTurnLeft); break;
        case ArrowKey::kRight: setDriveIntent(DriveIntent::kTurnRight); break;
    }
}

void handleKey(std::uint8_t raw) {
    if (raw == ' ') {
        emergencyStop("operator request");
        return;
    }
    const char key = static_cast<char>(
        std::tolower(static_cast<unsigned char>(raw)));
    switch (key) {
        case 'o': armSystem(); break;
        case 'h': beginPose(homePose(), "HOME"); break;
        case 's': beginPose(standPose(), "STAND"); break;
        case 'i': requireIkAndNudge(0.0f, +kPositionStepMm); break;
        case 'k': requireIkAndNudge(0.0f, -kPositionStepMm); break;
        case 'j': requireIkAndNudge(-kPositionStepMm, 0.0f); break;
        case 'l': requireIkAndNudge(+kPositionStepMm, 0.0f); break;
        case 'q': requireIkAndNudgeHip(-kHipPitchStepRad); break;
        case 'e': requireIkAndNudgeHip(+kHipPitchStepRad); break;
        case 'r':
            if (s_state == SystemState::kIkControl) s_body.resetTarget();
            else std::printf("reset rejected: IK control is not active\n");
            break;
        case 'x': stopWheels(); break;
        case '+': case '=': adjustDriveSpeed(+kDriveSpeedStep); break;
        case '-': case '_': adjustDriveSpeed(-kDriveSpeedStep); break;
        case 'p': printStatus(); break;
        case '?': printHelp(); break;
        case '\r': case '\n': break;
        default: break;
    }
}

void parseInput(std::uint8_t input) {
    switch (s_input_state) {
        case InputState::kNormal:
            if (input == 0x1B) s_input_state = InputState::kEscape;
            else handleKey(input);
            break;
        case InputState::kEscape:
            s_input_state = input == '[' ? InputState::kCsi
                                         : InputState::kNormal;
            break;
        case InputState::kCsi:
            s_input_state = InputState::kNormal;
            if (input == 'A') handleArrow(ArrowKey::kUp);
            else if (input == 'B') handleArrow(ArrowKey::kDown);
            else if (input == 'C') handleArrow(ArrowKey::kRight);
            else if (input == 'D') handleArrow(ArrowKey::kLeft);
            break;
    }
}

bool writeServoCommand(const JointArray& command) {
    int bad_joint = -1;
    if (!withinLimits(command, bad_joint)) {
        emergencyStop(bad_joint >= 0 ? tilt::JOINT_NAME[bad_joint]
                                     : "invalid joint command");
        return false;
    }
    std::array<std::uint16_t, kJointCount> ticks{};
    for (int joint = 0; joint < tilt::NUM_JOINTS; ++joint) {
        if (!radToTick(joint, command[joint], ticks[joint])) {
            emergencyStop(tilt::JOINT_NAME[joint]);
            return false;
        }
    }
    if (s_bus.syncWritePositions(tilt::SERVO_ID, ticks.data(),
                                 kJointCount, 0, 0) != ESP_OK) {
        emergencyStop("servo sync write failure");
        return false;
    }
    return true;
}

void updatePoseCommand() {
    if (s_state == SystemState::kPoseTransition) {
        s_transition_elapsed_ms = std::min(
            s_transition_elapsed_ms + kControlPeriodMs, kPoseDurationMs);
        const float progress = smoothstep5(
            static_cast<float>(s_transition_elapsed_ms) /
            static_cast<float>(kPoseDurationMs));
        for (int joint = 0; joint < tilt::NUM_JOINTS; ++joint) {
            s_command[joint] = s_transition_start[joint] +
                (s_transition_target[joint] - s_transition_start[joint]) *
                    progress;
        }
        if (s_transition_elapsed_ms >= kPoseDurationMs) {
            s_command = s_transition_target;
            s_body.captureReference(s_command);
            s_state = SystemState::kIkControl;
            std::printf("reached %s; IK reference captured\n",
                        s_transition_name == nullptr ? "pose"
                                                     : s_transition_name);
        }
    } else if (s_state == SystemState::kIkControl) {
        JointArray next{};
        if (s_body.update(kControlDt, next)) {
            s_command = next;
            s_ik_error_reported = false;
        } else if (!s_ik_error_reported) {
            std::printf("IK target rejected: fixed-hip workspace or joint limit\n");
            s_ik_error_reported = true;
        }
    }
}

void controlTask(void*) {
    TickType_t last_wake = xTaskGetTickCount();
    while (true) {
        const std::int64_t cycle_start = esp_timer_get_time();

        std::uint8_t input = 0;
        while (usb_serial_jtag_read_bytes(&input, 1, 0) > 0) {
            parseInput(input);
        }

        if (systemArmed() && s_motor.enabled()) {
            if (s_motor.update(kControlDt) != ESP_OK) {
                emergencyStop("motor update failure");
            }
        }
        if (systemArmed()) {
            updatePoseCommand();
            if (systemArmed()) writeServoCommand(s_command);
        }

        if (esp_timer_get_time() - cycle_start > kControlPeriodUs) {
            ++s_overruns;
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(kControlPeriodMs));
    }
}

void probeServosAndDisableTorque() {
    for (int joint = 0; joint < tilt::NUM_JOINTS; ++joint) {
        const std::uint8_t id = tilt::SERVO_ID[joint];
        const esp_err_t result = s_bus.ping(id);
        std::printf("[SERVO] %s ID %u: %s\n",
                    tilt::JOINT_NAME[joint], id,
                    result == ESP_OK ? "OK" : esp_err_to_name(result));
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    const esp_err_t result =
        s_bus.setTorqueAll(tilt::SERVO_ID, kJointCount, false);
    std::printf("[SERVO] startup torque OFF: %s\n",
                esp_err_to_name(result));
}

}  // namespace

extern "C" void app_main() {
    esp_err_t result = s_motor.initialize();
    if (result != ESP_OK) {
        std::printf("TB6612 initialization failed: %s\n",
                    esp_err_to_name(result));
        return;
    }

    result = s_bus.initialize();
    if (result != ESP_OK) {
        std::printf("servo bus initialization failed: %s\n",
                    esp_err_to_name(result));
        s_motor.emergencyStop();
        return;
    }

    usb_serial_jtag_driver_config_t console_config =
        USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    result = usb_serial_jtag_driver_install(&console_config);
    if (result != ESP_OK && result != ESP_ERR_INVALID_STATE) {
        std::printf("USB console initialization failed: %s\n",
                    esp_err_to_name(result));
        s_motor.emergencyStop();
        return;
    }

    probeServosAndDisableTorque();
    printHelp();
    std::printf(
        "TB6612 pins: PWMA=%d AIN1=%d AIN2=%d PWMB=%d BIN1=%d BIN2=%d STBY=%d\n",
        static_cast<int>(tilt::config::hardware::kWheelLeftPwmPin),
        static_cast<int>(tilt::config::hardware::kWheelLeftIn1Pin),
        static_cast<int>(tilt::config::hardware::kWheelLeftIn2Pin),
        static_cast<int>(tilt::config::hardware::kWheelRightPwmPin),
        static_cast<int>(tilt::config::hardware::kWheelRightIn1Pin),
        static_cast<int>(tilt::config::hardware::kWheelRightIn2Pin),
        static_cast<int>(tilt::config::hardware::kWheelStandbyPin));

    if (xTaskCreate(controlTask, "wheel_base", 8192, nullptr, 5, nullptr) !=
        pdPASS) {
        emergencyStop("task creation failure");
    }
}
