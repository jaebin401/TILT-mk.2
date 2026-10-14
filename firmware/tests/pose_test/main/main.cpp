#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "driver/usb_serial_jtag.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tilt/config/HardwareConfig.h"
#include "tilt/sts3215/Sts3215Bus.h"

#include "JointMap.h"
#include "PosePlayer.h"
#include "PoseTable.h"

namespace {

using tilt::pose::JointArray;
using tilt::pose::PoseCommand;
using tilt::pose::PosePlayer;
using tilt::pose::Posture;
using tilt::pose::Sequence;
using tilt::pose::Side;

constexpr std::size_t kJointCount = tilt::NUM_JOINTS;
constexpr std::int64_t kControlPeriodUs =
    static_cast<std::int64_t>(tilt::CONTROL_PERIOD_MS) * 1000;

struct PoseAvailability {
    bool home = false;
    bool stand = false;
    bool shift_left = false;
    bool shift_right = false;
    bool lift_left = false;
    bool lift_right = false;
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
PosePlayer s_player;
JointArray s_last_valid_command{};
PoseAvailability s_pose_available{};
Posture s_posture = Posture::UNKNOWN;
Posture s_pending_posture = Posture::UNKNOWN;
bool s_posture_pending = false;
bool s_armed = false;
std::uint32_t s_overrun_count = 0;
bool s_write_error_reported = false;

// Stand tuning (sagittal only, both legs identical).
constexpr std::uint32_t kTuneMoveMs = 250;
constexpr float kTuneSteps[] = {0.5f, 1.0f, 0.1f};
std::size_t s_tune_step_index = 0;

const char* postureName(Posture posture) {
    switch (posture) {
        case Posture::HOME: return "HOME";
        case Posture::STAND: return "STAND";
        case Posture::LIFT_L: return "LIFT_L";
        case Posture::LIFT_R: return "LIFT_R";
        case Posture::UNKNOWN: return "UNKNOWN";
    }
    return "UNKNOWN";
}

const tilt::JointLimit& jointLimit(int joint) {
    return tilt::jointLimit(joint);
}

float radiansToDegrees(float radians) {
    return radians / tilt::DEG2RAD;
}

esp_err_t initializeConsoleInput() {
    usb_serial_jtag_driver_config_t config =
        USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    const esp_err_t result = usb_serial_jtag_driver_install(&config);
    return result == ESP_ERR_INVALID_STATE ? ESP_OK : result;
}

void printHelp() {
    std::printf("\nTILT mk.2 pose calibration test\n");
    std::printf("  o     : arm at the measured position\n");
    std::printf("  h     : HOME (all joints 0 rad)\n");
    std::printf("  s     : STAND [0, -20, +40, -20] deg\n");
    std::printf("  l     : LIFT_L (right support, lift left foot)\n");
    std::printf("  r     : LIFT_R (left support, lift right foot)\n");
    std::printf("  p     : print command/measured status\n");
    std::printf("  -- stand tuning (from STAND; both legs move together, roll stays 0) --\n");
    std::printf("  f / b : torso forward / back   (hip pitch +step / -step)\n");
    std::printf("  k / j : knee bend more / less  (knee pitch +step / -step)\n");
    std::printf("  z     : step size 0.5 -> 1.0 -> 0.1 deg\n");
    std::printf("  c     : print the tuned pose as code for standPose()\n");
    std::printf("  u     : back to table STAND [0, %+.1f, %+.1f, %+.1f]\n",
                tilt::pose::kStandHipDeg, tilt::pose::kStandKneeDeg,
                tilt::pose::kStandAnkleDeg);
    std::printf("  ?     : print this help\n");
    std::printf("  SPACE : emergency stop and torque OFF\n");
    std::printf("armed=%s posture=%s\n\n",
                s_armed ? "yes" : "no",
                postureName(s_posture));
}

bool validatePose(const char* name, const JointArray& pose) {
    int bad_joint = -1;
    if (tilt::pose::withinLimits(pose, bad_joint)) {
        std::printf("[POSE] %-8s OK\n", name);
        return true;
    }

    const tilt::JointLimit& limit = jointLimit(bad_joint);
    std::printf("[POSE] %-8s DISABLED: %s=%+.2f deg outside [%+.2f, %+.2f]\n",
                name,
                tilt::JOINT_NAME[bad_joint],
                radiansToDegrees(pose[bad_joint]),
                radiansToDegrees(limit.minimum_rad),
                radiansToDegrees(limit.maximum_rad));
    return false;
}

void validatePoseTable() {
    s_pose_available.home =
        validatePose("HOME", tilt::pose::homePose());
    s_pose_available.stand =
        validatePose("STAND", tilt::pose::standPose());
    s_pose_available.shift_left =
        validatePose("SHIFT_L", tilt::pose::shiftPose(Side::LEFT));
    s_pose_available.shift_right =
        validatePose("SHIFT_R", tilt::pose::shiftPose(Side::RIGHT));
    s_pose_available.lift_left =
        validatePose("LIFT_L", tilt::pose::liftPose(Side::RIGHT));
    s_pose_available.lift_right =
        validatePose("LIFT_R", tilt::pose::liftPose(Side::LEFT));

    const JointArray stand = tilt::pose::standPose();
    const float left_pitch = tilt::pose::footPitchRad(stand, 0);
    const float right_pitch = tilt::pose::footPitchRad(stand, 1);
    if (std::fabs(left_pitch) > 0.5f * tilt::DEG2RAD ||
        std::fabs(right_pitch) > 0.5f * tilt::DEG2RAD) {
        std::printf("[POSE] WARN STAND foot pitch: L=%+.2f deg R=%+.2f deg\n",
                    radiansToDegrees(left_pitch),
                    radiansToDegrees(right_pitch));
    } else {
        std::printf("[POSE] STAND foot pitch: L=%+.2f deg R=%+.2f deg\n",
                    radiansToDegrees(left_pitch),
                    radiansToDegrees(right_pitch));
    }
}

bool segmentEnabled(const char* name) {
    if (name == nullptr) return false;
    if (std::strcmp(name, "HOME") == 0) return s_pose_available.home;
    if (std::strcmp(name, "STAND") == 0) return s_pose_available.stand;
    if (std::strcmp(name, "SHIFT_L") == 0) return s_pose_available.shift_left;
    if (std::strcmp(name, "SHIFT_R") == 0) return s_pose_available.shift_right;
    if (std::strcmp(name, "LIFT_L") == 0) return s_pose_available.lift_left;
    if (std::strcmp(name, "LIFT_R") == 0) return s_pose_available.lift_right;
    return false;
}

void printMeasuredPose(const JointArray& measured) {
    std::printf("joint          measured_deg\n");
    for (int joint = 0; joint < tilt::NUM_JOINTS; ++joint) {
        std::printf("%-14s %+10.2f\n",
                    tilt::JOINT_NAME[joint],
                    radiansToDegrees(measured[joint]));
        const tilt::JointLimit& limit = jointLimit(joint);
        if (measured[joint] < limit.minimum_rad ||
            measured[joint] > limit.maximum_rad) {
            std::printf("  WARN: outside configured limit [%+.2f, %+.2f] deg\n",
                        radiansToDegrees(limit.minimum_rad),
                        radiansToDegrees(limit.maximum_rad));
        }
    }
}

bool convertCommand(const JointArray& command,
                    std::array<std::uint16_t, kJointCount>& ticks,
                    int& bad_joint) {
    for (int joint = 0; joint < tilt::NUM_JOINTS; ++joint) {
        if (!tilt::pose::radToTick(joint, command[joint], ticks[joint])) {
            bad_joint = joint;
            return false;
        }
    }
    bad_joint = -1;
    return true;
}

void emergencyStop() {
    const esp_err_t result =
        s_bus.emergencyStop(tilt::SERVO_ID, kJointCount);
    s_armed = false;
    s_player.clear();
    s_posture = Posture::UNKNOWN;
    s_pending_posture = Posture::UNKNOWN;
    s_posture_pending = false;
    std::printf("\n!! EMERGENCY STOP: torque OFF (%s) !!\n",
                esp_err_to_name(result));
}

void armServos() {
    if (s_armed && !s_player.idle()) {
        std::printf("arm rejected: a pose is moving\n");
        return;
    }

    std::array<std::uint16_t, kJointCount> measured_ticks{};
    JointArray measured{};
    bool all_read = true;
    for (int joint = 0; joint < tilt::NUM_JOINTS; ++joint) {
        const esp_err_t result =
            s_bus.readPosition(tilt::SERVO_ID[joint], measured_ticks[joint]);
        if (result != ESP_OK) {
            std::printf("arm rejected: %s (ID %u) read failed: %s\n",
                        tilt::JOINT_NAME[joint],
                        tilt::SERVO_ID[joint],
                        esp_err_to_name(result));
            all_read = false;
        } else {
            measured[joint] =
                tilt::pose::tickToRad(joint, measured_ticks[joint]);
        }
        vTaskDelay(pdMS_TO_TICKS(3));
    }
    if (!all_read) {
        std::printf("arm rejected: all %u positions are required; torque stays OFF\n",
                    static_cast<unsigned>(kJointCount));
        return;
    }

    printMeasuredPose(measured);
    s_player.reset(measured);
    s_last_valid_command = measured;

    esp_err_t result = s_bus.syncWritePositions(tilt::SERVO_ID,
                                                measured_ticks.data(),
                                                kJointCount,
                                                0,
                                                0);
    if (result != ESP_OK) {
        std::printf("arm rejected: hold-position write failed: %s\n",
                    esp_err_to_name(result));
        return;
    }
    result = s_bus.setTorqueAll(tilt::SERVO_ID, kJointCount, true);
    if (result != ESP_OK) {
        std::printf("arm failed while enabling torque: %s\n",
                    esp_err_to_name(result));
        emergencyStop();
        return;
    }

    s_armed = true;
    s_posture = Posture::UNKNOWN;
    s_pending_posture = Posture::UNKNOWN;
    s_posture_pending = false;
    s_write_error_reported = false;
    std::printf("armed at measured position; posture=UNKNOWN\n");
}

void queueCommand(PoseCommand command) {
    if (command == PoseCommand::HOME) {
        std::printf("WARN: HOME fully extends the legs; use only while suspended\n");
    }
    if (!s_armed) {
        std::printf("command rejected: press 'o' to arm first\n");
        return;
    }
    if (!s_player.idle()) {
        std::printf("command rejected: %s is still moving\n",
                    s_player.activeName() == nullptr
                        ? "pose"
                        : s_player.activeName());
        return;
    }

    Sequence sequence;
    if (!tilt::pose::makeSequence(s_posture, command, sequence)) {
        std::printf("command rejected: %s\n",
                    sequence.error == nullptr ? "invalid sequence"
                                              : sequence.error);
        return;
    }

    for (std::size_t index = 0; index < sequence.count; ++index) {
        const auto& segment = sequence.segments[index];
        int bad_joint = -1;
        if (!segmentEnabled(segment.name) ||
            !tilt::pose::withinLimits(segment.target, bad_joint)) {
            std::printf("command rejected: pose %s is disabled",
                        segment.name == nullptr ? "?" : segment.name);
            if (bad_joint >= 0) {
                std::printf(" (%s outside limit)",
                            tilt::JOINT_NAME[bad_joint]);
            }
            std::printf("\n");
            return;
        }
    }

    for (std::size_t index = 0; index < sequence.count; ++index) {
        if (!s_player.enqueue(sequence.segments[index])) {
            s_player.clear();
            std::printf("command rejected: pose queue is full\n");
            return;
        }
    }

    s_pending_posture = sequence.final_posture;
    s_posture_pending = true;
    std::printf("queued:");
    for (std::size_t index = 0; index < sequence.count; ++index) {
        std::printf(" %s", sequence.segments[index].name);
    }
    std::printf("\n");
}

void printStatus() {
    const JointArray command = s_player.lastCommand();
    std::array<std::uint16_t, kJointCount> command_ticks{};
    int bad_joint = -1;
    const bool command_valid =
        convertCommand(command, command_ticks, bad_joint);
    const bool moving = !s_player.idle();

    std::array<std::uint16_t, kJointCount> measured_ticks{};
    std::array<bool, kJointCount> measured_ok{};
    if (!moving) {
        for (int joint = 0; joint < tilt::NUM_JOINTS; ++joint) {
            measured_ok[joint] =
                s_bus.readPosition(tilt::SERVO_ID[joint],
                                   measured_ticks[joint]) == ESP_OK;
            vTaskDelay(pdMS_TO_TICKS(3));
        }
    }

    std::printf("\nposture=%s armed=%s overruns=%lu",
                postureName(s_posture),
                s_armed ? "yes" : "no",
                static_cast<unsigned long>(s_overrun_count));
    if (moving) {
        std::printf(" active=%s",
                    s_player.activeName() == nullptr ? "?"
                                                     : s_player.activeName());
    }
    std::printf("\n");
    std::printf("joint          cmd_deg  cmd_tick  meas_tick  err_tick  err_deg\n");

    for (int joint = 0; joint < tilt::NUM_JOINTS; ++joint) {
        if (!command_valid) {
            std::printf("%-14s %8.2f      ----       ----      ----      ----\n",
                        tilt::JOINT_NAME[joint],
                        radiansToDegrees(command[joint]));
            continue;
        }

        std::printf("%-14s %8.2f %9u  ",
                    tilt::JOINT_NAME[joint],
                    radiansToDegrees(command[joint]),
                    command_ticks[joint]);
        if (!measured_ok[joint]) {
            std::printf("     ----      ----      ----\n");
            continue;
        }

        const int error = static_cast<int>(measured_ticks[joint]) -
                          static_cast<int>(command_ticks[joint]);
        const float error_degrees = error * tilt::DEG_PER_TICK;
        std::printf("%9u %+9d %+9.2f%s\n",
                    measured_ticks[joint],
                    error,
                    error_degrees,
                    std::abs(error) > 10 ? "  !" : "");
    }

    std::printf("foot_pitch  L=%+.2fdeg  R=%+.2fdeg (command)\n\n",
                radiansToDegrees(tilt::pose::footPitchRad(command, 0)),
                radiansToDegrees(tilt::pose::footPitchRad(command, 1)));
}

// ---------------------------------------------------------------- stand tuning
bool tuneReady() {
    if (!s_armed) {
        std::printf("tune rejected: press 'o' to arm first\n");
        return false;
    }
    if (!s_player.idle()) {
        std::printf("tune rejected: still moving\n");
        return false;
    }
    if (s_posture != Posture::STAND) {
        std::printf("tune rejected: reach STAND first ('s')\n");
        return false;
    }
    return true;
}

// Symmetric sagittal pose with the ankle compensating to keep the sole flat.
JointArray symmetricPose(float hip_rad, float knee_rad) {
    const float ankle_rad = -(hip_rad + knee_rad +
                              tilt::KNEE_OFFSET_RAD +
                              tilt::ANKLE_FIXED_RAD);
    return {0.0f, hip_rad, knee_rad, ankle_rad,
            0.0f, hip_rad, knee_rad, ankle_rad};
}

void moveTo(const JointArray& target) {
    int bad_joint = -1;
    if (!tilt::pose::withinLimits(target, bad_joint)) {
        std::printf("tune rejected: %s would be outside its limit\n",
                    tilt::JOINT_NAME[bad_joint]);
        return;
    }
    tilt::pose::Segment segment;
    segment.target = target;
    segment.duration_ms = kTuneMoveMs;
    segment.name = "TUNE";
    if (!s_player.enqueue(segment)) {
        std::printf("tune rejected: pose queue is full\n");
        return;
    }
    std::printf("stand tune -> hip %+.1f deg, knee %+.1f deg (foot pitch vs torso %+.1f deg)\n",
                radiansToDegrees(target[tilt::L_HIP_PITCH]),
                radiansToDegrees(target[tilt::L_KNEE_PITCH]),
                radiansToDegrees(tilt::pose::footPitchRad(target, 0)));
}

void tuneAdjust(float hip_delta_deg, float knee_delta_deg) {
    if (!tuneReady()) return;
    const JointArray now = s_player.lastCommand();
    // Start from the average of both legs so they stay identical.
    const float hip = 0.5f * (now[tilt::L_HIP_PITCH] + now[tilt::R_HIP_PITCH]) +
                      hip_delta_deg * tilt::DEG2RAD;
    const float knee = 0.5f * (now[tilt::L_KNEE_PITCH] + now[tilt::R_KNEE_PITCH]) +
                       knee_delta_deg * tilt::DEG2RAD;
    moveTo(symmetricPose(hip, knee));
}

void tuneReset() {
    if (!tuneReady()) return;
    moveTo(tilt::pose::standPose());
}

void printTunedPose() {
    const JointArray q = s_player.lastCommand();
    const float hip = radiansToDegrees(q[tilt::L_HIP_PITCH]);
    const float knee = radiansToDegrees(q[tilt::L_KNEE_PITCH]);
    const float ankle = radiansToDegrees(q[tilt::L_ANKLE_PITCH]);

    std::printf("\n=== tuned stand pose ===\n");
    std::printf("command : hip %+.2f deg, knee %+.2f deg, ankle %+.2f deg "
                "(both legs), roll 0\n", hip, knee, ankle);
    std::printf("foot pitch vs torso: %+.2f deg  (0 = sole parallel to the torso; with the feet flat,\n"
                "                     a negative value means the torso is pitched back by that much)\n",
                radiansToDegrees(tilt::pose::footPitchRad(q, 0)));
    if (s_player.idle()) {
        std::printf("measured:");
        for (int joint = 0; joint < tilt::NUM_JOINTS; ++joint) {
            std::uint16_t tick = 0;
            if (s_bus.readPosition(tilt::SERVO_ID[joint], tick) == ESP_OK) {
                std::printf(" %s=%+.2f", tilt::JOINT_NAME[joint],
                            radiansToDegrees(tilt::pose::tickToRad(joint, tick)));
            } else {
                std::printf(" %s=----", tilt::JOINT_NAME[joint]);
            }
            vTaskDelay(pdMS_TO_TICKS(2));
        }
        std::printf("\n");
    }
    std::printf("\n// firmware/components/tilt_config/include/tilt_config.h\n");
    std::printf("// ZERO_POSE_RAD per leg: {0.0f, %.2ff, %.2ff, %.2ff} * DEG2RAD\n",
                hip, knee, ankle);
    std::printf("\n// firmware/tests/pose_test/main/PoseTable.h\n");
    std::printf("inline constexpr float kStandHipDeg = %.2ff;\n", hip);
    std::printf("inline constexpr float kStandKneeDeg = %+.2ff;\n\n", knee);
    std::printf("inline constexpr float kStandAnkleDeg = %+.2ff;\n\n", ankle);
}

void handleInput(std::uint8_t input) {
    if (input == ' ') {
        emergencyStop();
        return;
    }

    const char key = static_cast<char>(std::tolower(input));
    switch (key) {
        case 'o': armServos(); break;
        case 'h': queueCommand(PoseCommand::HOME); break;
        case 's': queueCommand(PoseCommand::STAND); break;
        case 'l': queueCommand(PoseCommand::LIFT_L); break;
        case 'r': queueCommand(PoseCommand::LIFT_R); break;
        case 'p': printStatus(); break;
        case '?': printHelp(); break;
        case 'f': tuneAdjust(+kTuneSteps[s_tune_step_index], 0.0f); break;
        case 'b': tuneAdjust(-kTuneSteps[s_tune_step_index], 0.0f); break;
        case 'k': tuneAdjust(0.0f, +kTuneSteps[s_tune_step_index]); break;
        case 'j': tuneAdjust(0.0f, -kTuneSteps[s_tune_step_index]); break;
        case 'z':
            s_tune_step_index = (s_tune_step_index + 1) %
                                (sizeof(kTuneSteps) / sizeof(kTuneSteps[0]));
            std::printf("tune step = %.1f deg\n", kTuneSteps[s_tune_step_index]);
            break;
        case 'c': printTunedPose(); break;
        case 'u': tuneReset(); break;
        default: break;
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

        if (s_armed) {
            const JointArray command =
                s_player.step(tilt::CONTROL_PERIOD_MS);
            std::array<std::uint16_t, kJointCount> ticks{};
            int bad_joint = -1;
            if (!convertCommand(command, ticks, bad_joint)) {
                if (!s_write_error_reported) {
                    std::printf("pose aborted: %s command is outside servo range\n",
                                bad_joint >= 0 ? tilt::JOINT_NAME[bad_joint]
                                               : "unknown joint");
                    s_write_error_reported = true;
                }
                s_player.reset(s_last_valid_command);
                s_posture = Posture::UNKNOWN;
                s_pending_posture = Posture::UNKNOWN;
                s_posture_pending = false;
            } else {
                const esp_err_t result =
                    s_bus.syncWritePositions(tilt::SERVO_ID,
                                             ticks.data(),
                                             kJointCount,
                                             0,
                                             0);
                if (result == ESP_OK) {
                    s_last_valid_command = command;
                    s_write_error_reported = false;
                    if (s_posture_pending && s_player.idle()) {
                        s_posture = s_pending_posture;
                        s_pending_posture = Posture::UNKNOWN;
                        s_posture_pending = false;
                        std::printf("reached %s\n", postureName(s_posture));
                    }
                } else if (!s_write_error_reported) {
                    std::printf("servo sync write failed: %s; holding last command\n",
                                esp_err_to_name(result));
                    s_write_error_reported = true;
                    s_player.reset(s_last_valid_command);
                    s_posture = Posture::UNKNOWN;
                    s_pending_posture = Posture::UNKNOWN;
                    s_posture_pending = false;
                }
            }
        }

        if (esp_timer_get_time() - cycle_start > kControlPeriodUs) {
            ++s_overrun_count;
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(tilt::CONTROL_PERIOD_MS));
    }
}

void probeServosAndDisableTorque() {
    for (int joint = 0; joint < tilt::NUM_JOINTS; ++joint) {
        const std::uint8_t id = tilt::SERVO_ID[joint];
        const esp_err_t ping_result = s_bus.ping(id);
        if (ping_result == ESP_OK) {
            std::printf("[SERVO] %s ID %u: ping OK\n",
                        tilt::JOINT_NAME[joint], id);
        } else {
            std::printf("[SERVO] %s ID %u: ping FAILED (%s)\n",
                        tilt::JOINT_NAME[joint], id,
                        esp_err_to_name(ping_result));
        }

        const esp_err_t acceleration_result = s_bus.setAcceleration(id, 0);
        if (acceleration_result != ESP_OK) {
            std::printf("[SERVO] %s ID %u: acceleration=0 failed (%s)\n",
                        tilt::JOINT_NAME[joint], id,
                        esp_err_to_name(acceleration_result));
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    const esp_err_t result =
        s_bus.setTorqueAll(tilt::SERVO_ID, kJointCount, false);
    std::printf("[SERVO] startup torque OFF: %s\n", esp_err_to_name(result));
}

}  // namespace

extern "C" void app_main() {
    esp_err_t result = s_bus.initialize();
    if (result != ESP_OK) {
        std::printf("servo bus initialization failed: %s\n",
                    esp_err_to_name(result));
        return;
    }

    result = initializeConsoleInput();
    if (result != ESP_OK) {
        std::printf("USB console initialization failed: %s\n",
                    esp_err_to_name(result));
        return;
    }

    probeServosAndDisableTorque();
    validatePoseTable();
    printHelp();

    const BaseType_t task_result = xTaskCreate(controlTask,
                                               "pose_control",
                                               8192,
                                               nullptr,
                                               5,
                                               nullptr);
    if (task_result != pdPASS) {
        std::printf("failed to create pose control task\n");
        emergencyStop();
    }
}
