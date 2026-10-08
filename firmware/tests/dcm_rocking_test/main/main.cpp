// TILT mk.2 DCM rocking test.
//
// Stepping in place gated by the estimated DCM (tilt_dcm_gait::DcmStepping),
// with the open-loop FLASH pattern available as a baseline. See README.md for
// the hardware procedure. Single-key console over USB Serial/JTAG.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>

#include "ImuMounting.h"
#include "driver/usb_serial_jtag.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tilt/config/HardwareConfig.h"
#include "tilt/dcm_gait/DcmStepping.h"
#include "tilt/estimation/DcmEstimator.h"
#include "tilt/sts3215/Sts3215Bus.h"
#include "tilt_config.h"
#include "tilt_mpu6050.h"

namespace {

namespace gait = tilt::dcm_gait;
namespace est = tilt::estimation;
using JointArray = gait::JointArray;

constexpr std::uint32_t kControlPeriodMs = 10;
constexpr std::int64_t kControlPeriodUs = kControlPeriodMs * 1000;
constexpr float kDt = kControlPeriodMs / 1000.0f;
constexpr std::uint32_t kStandTransitionMs = 2000;
constexpr float kEmergencyTiltRad = 35.0f * tilt::DEG2RAD;
constexpr float kGaitStopTiltRad = 20.0f * tilt::DEG2RAD;
constexpr std::size_t kJointCount = tilt::NUM_JOINTS;
constexpr float kRadToDeg = 57.2957795f;
constexpr int kReadbackFailLimit = 5;  // Consecutive failed cycles -> off.

enum class ControllerState { kUnarmed, kArmedHold, kMovingToStand, kStanding, kStepping };
enum class Monitor { kOff, kImu, kEstimator };

tilt::sts3215::BusConfig makeBusConfig() {
    tilt::sts3215::BusConfig config;
    config.uart_port = tilt::config::hardware::kServoUartPort;
    config.tx_pin = tilt::config::hardware::kServoUartTxPin;
    config.rx_pin = tilt::config::hardware::kServoUartRxPin;
    config.baud_rate = tilt::config::hardware::kServoUartBaudRate;
    config.response_timeout_ms = tilt::config::hardware::kServoResponseTimeoutMs;
    config.rx_buffer_size = tilt::config::hardware::kServoRxBufferSize;
    config.tx_buffer_size = tilt::config::hardware::kServoTxBufferSize;
    return config;
}

struct ImuMap {
    bool swap_xy = tilt_dcm_test::kImuSwapXY;
    int sign[3] = {tilt_dcm_test::kImuSignX, tilt_dcm_test::kImuSignY,
                   tilt_dcm_test::kImuSignZ};
};

// ---------------------------------------------------------------- globals
tilt::sts3215::Sts3215Bus s_bus(makeBusConfig());
tilt::ComplementaryFilter s_filter;
est::DcmEstimator s_estimator;
gait::Params s_params;
gait::DcmStepping s_gait;

ControllerState s_state = ControllerState::kUnarmed;
Monitor s_monitor = Monitor::kOff;
ImuMap s_imu_map;
bool s_imu_available = false;
bool s_imu_verified = tilt_dcm_test::kImuVerified;
bool s_csv = false;
bool s_readback = true;

JointArray s_command{};
JointArray s_measured{};
JointArray s_transition_start{};
std::uint32_t s_transition_elapsed_ms = 0;

tilt::ImuRaw s_imu_robot{};
est::ImuSample s_imu_sample{};
std::int64_t s_last_imu_us = 0;

std::uint32_t s_overruns = 0;
std::uint32_t s_readback_failures = 0;
int s_readback_fail_streak = 0;
std::int64_t s_readback_us = 0;
std::int64_t s_cycle_us = 0;
std::uint32_t s_print_divider = 0;
std::int64_t s_boot_us = 0;

// Stepping statistics.
struct RunStats {
    std::uint32_t swings = 0;
    std::uint32_t dcm_touchdowns = 0;
    std::uint32_t timeouts = 0;
    std::uint32_t swing_ms_sum = 0;
    std::uint32_t swing_ms_min = 0xFFFFFFFFu;
    std::uint32_t swing_ms_max = 0;
    float max_abs_roll = 0.0f;
    std::int64_t start_us = 0;
};
RunStats s_run;
std::int64_t s_swing_start_us = 0;

// ---------------------------------------------------------------- helpers
JointArray standPose() { return gait::standPose(); }

const char* stateName(ControllerState state) {
    switch (state) {
        case ControllerState::kUnarmed: return "UNARMED";
        case ControllerState::kArmedHold: return "ARMED_HOLD";
        case ControllerState::kMovingToStand: return "MOVING_TO_STAND";
        case ControllerState::kStanding: return "STANDING";
        case ControllerState::kStepping: return "STEPPING";
    }
    return "?";
}

const char* supportName(est::Support s) {
    switch (s) {
        case est::Support::kDouble: return "D";
        case est::Support::kLeft: return "L";
        case est::Support::kRight: return "R";
    }
    return "?";
}

float smoothstep5(float value) {
    const float s = std::clamp(value, 0.0f, 1.0f);
    return s * s * s * (10.0f + s * (-15.0f + 6.0f * s));
}

std::int16_t applySign(std::int16_t v, int sign) {
    if (sign >= 0) return v;
    return v == INT16_MIN ? INT16_MAX : static_cast<std::int16_t>(-v);
}

tilt::ImuRaw toRobotFrame(const tilt::ImuRaw& raw, const ImuMap& map) {
    tilt::ImuRaw r = raw;
    if (map.swap_xy) {
        r.ax = raw.ay;
        r.ay = raw.ax;
        r.gx = raw.gy;
        r.gy = raw.gx;
    }
    r.ax = applySign(r.ax, map.sign[0]);
    r.ay = applySign(r.ay, map.sign[1]);
    r.az = applySign(r.az, map.sign[2]);
    r.gx = applySign(r.gx, map.sign[0]);
    r.gy = applySign(r.gy, map.sign[1]);
    r.gz = applySign(r.gz, map.sign[2]);
    return r;
}

bool withinLimits(const JointArray& command, int& bad_joint) {
    for (int joint = 0; joint < tilt::NUM_JOINTS; ++joint) {
        const auto& limit = tilt::JOINT_LIMIT[joint / 3][joint % 3];
        if (!std::isfinite(command[joint]) || command[joint] < limit.minimum_rad ||
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
                      static_cast<float>(tilt::JOINT_SIGN[joint]) * radians /
                          tilt::RAD_PER_TICK;
    const long rounded = std::lround(raw);
    if (rounded < tilt::SERVO_POS_MIN || rounded > tilt::SERVO_POS_MAX) return false;
    tick = static_cast<std::uint16_t>(rounded);
    return true;
}

float tickToRad(int joint, std::uint16_t tick) {
    return static_cast<float>(tilt::JOINT_SIGN[joint]) *
           static_cast<float>(static_cast<int>(tick) -
                              static_cast<int>(tilt::ZERO_TICK[joint])) *
           tilt::RAD_PER_TICK;
}

void configureEstimator() {
    est::EstimatorConfig config = s_estimator.config();
    // Read-back angles already contain servo lag; commanded angles need the model.
    config.joint_lag_s = s_readback ? 0.0f : 0.03f;
    s_estimator.setConfig(config);
}

// ---------------------------------------------------------------- printing
void printParams() {
    const gait::Params& p = s_params;
    std::printf(
        "params: lift=%.0fdeg max_steps=%lu | DCM: touchdown_inside=%.0fmm "
        "lift_past_inner=%+.0fmm swing=%lu..%lums double=%lu..%lums start_hold=%lums "
        "abort_outer=%.0fmm | open-loop: hold=%lums period=%lums\n",
        p.lift_deg, static_cast<unsigned long>(p.max_steps), p.touchdown_dcm_inside_mm,
        p.lift_dcm_past_inner_mm, static_cast<unsigned long>(p.swing_min_ms),
        static_cast<unsigned long>(p.swing_max_ms),
        static_cast<unsigned long>(p.double_min_ms),
        static_cast<unsigned long>(p.double_max_ms),
        static_cast<unsigned long>(p.start_hold_ms), p.abort_outer_mm,
        static_cast<unsigned long>(p.open_hold_ms),
        static_cast<unsigned long>(p.open_period_ms));
}

void printImuMap() {
    std::printf("IMU map: swap_xy=%s sign x=%+d y=%+d z=%+d  verified=%s\n",
                s_imu_map.swap_xy ? "true" : "false", s_imu_map.sign[0],
                s_imu_map.sign[1], s_imu_map.sign[2],
                s_imu_verified ? "yes" : "NO (DCM stepping locked)");
}

void printHelp() {
    std::printf("\nTILT mk.2 DCM rocking test\n");
    std::printf("  o  arm at measured pose      s  move to stand (2 s)      SPACE  E-STOP\n");
    std::printf("  i  IMU monitor (10 Hz)       e  estimator monitor        c  CSV 100 Hz\n");
    std::printf("  X/Y/Z  flip IMU axis sign    W  swap IMU x/y             k  confirm IMU check\n");
    std::printf("  b  servo read-back on/off    p  status                   v  params\n");
    std::printf("  r / l  one open-loop lift (right / left leg)\n");
    std::printf("  f  open-loop FLASH stepping  d  DCM stepping             x  stop -> stand\n");
    std::printf("  + / -  lift +-1deg           [ / ]  touchdown_inside -+1mm\n");
    std::printf("  ; / '  lift_past_inner -+1mm , / .  open-loop period -+10ms\n");
    std::printf("  n / m  max_steps -+2 (0 = unlimited)\n");
    std::printf("state=%s imu=%s readback=%s\n", stateName(s_state),
                s_imu_available ? "ready" : "unavailable", s_readback ? "on" : "off");
    printImuMap();
    printParams();
    std::printf("\n");
}

void printStatus() {
    const est::DcmState& s = s_estimator.state();
    std::printf(
        "state=%s overruns=%lu cycle=%lldus readback=%s (%lldus, failures %lu) "
        "roll=%+.2f pitch=%+.2f h=%.1fmm com_y=%+.1f dcm_y=%+.1f\n",
        stateName(s_state), static_cast<unsigned long>(s_overruns),
        static_cast<long long>(s_cycle_us), s_readback ? "on" : "off",
        static_cast<long long>(s_readback_us),
        static_cast<unsigned long>(s_readback_failures),
        s_imu_sample.roll_rad * kRadToDeg, s_imu_sample.pitch_rad * kRadToDeg,
        s.height_mm, s.com_mm.y, s.dcm_mm[1]);
}

void printCsvHeader() {
    std::printf(
        "CSV,t_ms,state,phase,swing,steps,support,"
        "cmd_LHR,cmd_LHP,cmd_LKP,cmd_RHR,cmd_RHP,cmd_RKP,"
        "meas_LHR,meas_LHP,meas_LKP,meas_RHR,meas_RHP,meas_RKP,"
        "roll,pitch,gx,gy,gz,com_x,com_y,com_z,vel_y,dcm_x,dcm_y,"
        "inner_L,inner_R,outer_L,outer_R,inside_swing,cycle_us,readback_us\n");
}

void printCsvRow() {
    const est::DcmState& s = s_estimator.state();
    const gait::Command& g = s_gait.command();
    const float inside = g.phase == gait::Phase::kSwing
        ? gait::dcmInsideSwingFoot(s, g.swing)
        : NAN;
    const long long t_ms = (esp_timer_get_time() - s_boot_us) / 1000;
    std::printf("CSV,%lld,%d,%s,%s,%lu,%s", t_ms, static_cast<int>(s_state),
                gait::phaseName(g.phase), g.swing == gait::Side::kLeft ? "L" : "R",
                static_cast<unsigned long>(g.steps), supportName(s.support));
    for (std::size_t j = 0; j < kJointCount; ++j) std::printf(",%.2f", s_command[j] * kRadToDeg);
    for (std::size_t j = 0; j < kJointCount; ++j) {
        if (s_readback) {
            std::printf(",%.2f", s_measured[j] * kRadToDeg);
        } else {
            std::printf(",nan");
        }
    }
    std::printf(",%.2f,%.2f,%.1f,%.1f,%.1f", s_imu_sample.roll_rad * kRadToDeg,
                s_imu_sample.pitch_rad * kRadToDeg,
                s_imu_sample.gyro_rad_s[0] * kRadToDeg,
                s_imu_sample.gyro_rad_s[1] * kRadToDeg,
                s_imu_sample.gyro_rad_s[2] * kRadToDeg);
    std::printf(",%.1f,%.1f,%.1f,%.0f,%.1f,%.1f", s.com_mm.x, s.com_mm.y, s.com_mm.z,
                s.com_velocity_mm_s.y, s.dcm_mm[0], s.dcm_mm[1]);
    std::printf(",%.1f,%.1f,%.1f,%.1f,%.1f,%lld,%lld\n", s.dcm_past_inner_edge_mm[0],
                s.dcm_past_inner_edge_mm[1], s.dcm_past_outer_edge_mm[0],
                s.dcm_past_outer_edge_mm[1], inside, static_cast<long long>(s_cycle_us),
                static_cast<long long>(s_readback_us));
}

void printMonitor() {
    if (s_monitor == Monitor::kImu) {
        const tilt::ImuRaw& r = s_imu_robot;
        std::printf(
            "IMU robot-frame acc[g] x=%+.2f y=%+.2f z=%+.2f | gyro[dps] x=%+6.1f y=%+6.1f "
            "z=%+6.1f | roll=%+6.2f pitch=%+6.2f\n",
            tilt::accel_to_g(r.ax), tilt::accel_to_g(r.ay), tilt::accel_to_g(r.az),
            tilt::gyro_to_rad_s(r.gx) * kRadToDeg, tilt::gyro_to_rad_s(r.gy) * kRadToDeg,
            tilt::gyro_to_rad_s(r.gz) * kRadToDeg, s_imu_sample.roll_rad * kRadToDeg,
            s_imu_sample.pitch_rad * kRadToDeg);
    } else if (s_monitor == Monitor::kEstimator) {
        const est::DcmState& s = s_estimator.state();
        std::printf(
            "EST sup=%s h=%.1f w=%.2f com(x=%+.1f y=%+.1f) vy=%+5.0f dcm(x=%+.1f y=%+.1f) "
            "inner L=%+.1f R=%+.1f outer L=%+.1f R=%+.1f roll=%+.2f rb=%s\n",
            supportName(s.support), s.height_mm, s.omega_rad_s, s.com_mm.x, s.com_mm.y,
            s.com_velocity_mm_s.y, s.dcm_mm[0], s.dcm_mm[1], s.dcm_past_inner_edge_mm[0],
            s.dcm_past_inner_edge_mm[1], s.dcm_past_outer_edge_mm[0],
            s.dcm_past_outer_edge_mm[1], s_imu_sample.roll_rad * kRadToDeg,
            s_readback ? "on" : "off");
    }
}

void printEvent(const gait::Command& g) {
    const est::DcmState& s = s_estimator.state();
    const long long t_ms = (esp_timer_get_time() - s_boot_us) / 1000;
    std::printf(
        "EVT t=%lld step=%lu %s%s trig=%s dcm_y=%+.1f inner L=%+.1f R=%+.1f roll=%+.2f",
        t_ms, static_cast<unsigned long>(g.steps), gait::phaseName(g.phase),
        g.phase == gait::Phase::kSwing ? (g.swing == gait::Side::kLeft ? "(L)" : "(R)") : "",
        gait::triggerName(g.last_trigger), s.dcm_mm[1], s.dcm_past_inner_edge_mm[0],
        s.dcm_past_inner_edge_mm[1], s_imu_sample.roll_rad * kRadToDeg);
    if (!g.running) std::printf(" STOP=%s", gait::stopReasonName(g.stop_reason));
    std::printf("\n");
}

void printRunSummary(const gait::Command& g) {
    const float seconds =
        static_cast<float>(esp_timer_get_time() - s_run.start_us) / 1.0e6f;
    std::printf(
        "\n== run: %s, %lu steps in %.2f s, stop=%s | swings %lu (dcm touchdown %lu, "
        "timeout %lu) swing ms min/avg/max %lu/%lu/%lu | max |roll| %.1f deg | "
        "overruns %lu readback failures %lu ==\n\n",
        s_params.mode == gait::Mode::kDcm ? "DCM" : "OPEN-LOOP",
        static_cast<unsigned long>(g.steps), seconds, gait::stopReasonName(g.stop_reason),
        static_cast<unsigned long>(s_run.swings),
        static_cast<unsigned long>(s_run.dcm_touchdowns),
        static_cast<unsigned long>(s_run.timeouts),
        static_cast<unsigned long>(s_run.swings ? s_run.swing_ms_min : 0),
        static_cast<unsigned long>(s_run.swings ? s_run.swing_ms_sum / s_run.swings : 0),
        static_cast<unsigned long>(s_run.swing_ms_max), s_run.max_abs_roll * kRadToDeg,
        static_cast<unsigned long>(s_overruns),
        static_cast<unsigned long>(s_readback_failures));
}

// ---------------------------------------------------------------- actions
void emergencyStop(const char* reason) {
    const esp_err_t result = s_bus.emergencyStop(tilt::SERVO_ID, kJointCount);
    s_gait.stop();
    s_state = ControllerState::kUnarmed;
    std::printf("\n!! EMERGENCY STOP: %s, torque OFF (%s) !!\n", reason,
                esp_err_to_name(result));
}

bool readJoints(JointArray& out) {
    const std::int64_t t0 = esp_timer_get_time();
    bool all_ok = true;
    for (std::size_t j = 0; j < kJointCount; ++j) {
        std::uint16_t tick = 0;
        if (s_bus.readPosition(tilt::SERVO_ID[j], tick) == ESP_OK) {
            out[j] = tickToRad(static_cast<int>(j), tick);
        } else {
            out[j] = s_command[j];
            all_ok = false;
        }
    }
    s_readback_us = esp_timer_get_time() - t0;
    return all_ok;
}

void armServos() {
    if (s_state != ControllerState::kUnarmed) {
        std::printf("already armed; SPACE first to re-arm\n");
        return;
    }
    std::array<std::uint16_t, kJointCount> ticks{};
    JointArray measured{};
    for (std::size_t j = 0; j < kJointCount; ++j) {
        const esp_err_t result = s_bus.readPosition(tilt::SERVO_ID[j], ticks[j]);
        if (result != ESP_OK) {
            std::printf("arm rejected: %s read failed: %s\n", tilt::JOINT_NAME[j],
                        esp_err_to_name(result));
            return;
        }
        measured[j] = tickToRad(static_cast<int>(j), ticks[j]);
        vTaskDelay(pdMS_TO_TICKS(3));
    }
    esp_err_t result =
        s_bus.syncWritePositions(tilt::SERVO_ID, ticks.data(), kJointCount, 0, 0);
    if (result == ESP_OK) result = s_bus.setTorqueAll(tilt::SERVO_ID, kJointCount, true);
    if (result != ESP_OK) {
        std::printf("arm failed: %s\n", esp_err_to_name(result));
        emergencyStop("arm failure");
        return;
    }
    s_command = measured;
    s_measured = measured;
    s_estimator.reset(measured.data());
    s_state = ControllerState::kArmedHold;
    std::printf("armed at measured pose; press 's' for stand\n");
}

void beginStand() {
    if (s_state == ControllerState::kUnarmed) {
        std::printf("stand rejected: arm first ('o')\n");
        return;
    }
    if (s_state == ControllerState::kStepping) s_gait.stop();
    s_transition_start = s_command;
    s_transition_elapsed_ms = 0;
    s_state = ControllerState::kMovingToStand;
}

void startStepping(gait::Mode mode, std::uint32_t max_steps_override,
                   gait::Side first) {
    if (s_state != ControllerState::kStanding) {
        std::printf("rejected: reach STANDING first ('s')\n");
        return;
    }
    if (mode == gait::Mode::kDcm && !s_imu_verified) {
        std::printf("rejected: DCM stepping needs a verified IMU mapping (see README, 'k')\n");
        return;
    }
    if (!s_imu_available) {
        std::printf("rejected: IMU unavailable (tilt safety would be blind)\n");
        return;
    }
    gait::Params p = s_params;
    p.mode = mode;
    p.first_swing = first;
    if (max_steps_override > 0) p.max_steps = max_steps_override;
    s_gait.setParams(p);
    s_run = RunStats{};
    s_run.start_us = esp_timer_get_time();
    s_gait.start();
    s_swing_start_us = esp_timer_get_time();
    s_state = ControllerState::kStepping;
    std::printf("stepping started: %s, first swing %s, max_steps %lu\n",
                mode == gait::Mode::kDcm ? "DCM" : "OPEN-LOOP",
                first == gait::Side::kLeft ? "L" : "R",
                static_cast<unsigned long>(p.max_steps));
    printEvent(s_gait.command());
}

bool idleForEdit() {
    if (s_state == ControllerState::kStepping) {
        std::printf("stop first ('x')\n");
        return false;
    }
    return true;
}

void flipImu(int axis) {
    if (!idleForEdit()) return;
    s_imu_map.sign[axis] = -s_imu_map.sign[axis];
    s_imu_verified = false;
    s_filter.reset();
    printImuMap();
}

template <typename T>
void adjust(T& value, T delta, T low, T high, const char* name) {
    if (!idleForEdit()) return;
    value = std::clamp<T>(static_cast<T>(value + delta), low, high);
    std::printf("%s -> ", name);
    printParams();
}

void adjustPeriod(int delta_ms) {
    if (!idleForEdit()) return;
    const int value = static_cast<int>(s_params.open_period_ms) + delta_ms;
    s_params.open_period_ms = static_cast<std::uint32_t>(std::clamp(value, 120, 600));
    printParams();
}

void handleInput(std::uint8_t input) {
    if (input == ' ') {
        emergencyStop("operator request");
        return;
    }
    gait::Params& p = s_params;
    switch (static_cast<char>(input)) {
        case 'o': armServos(); break;
        case 's': beginStand(); break;
        case 'x':
            if (s_state == ControllerState::kStepping) {
                s_gait.stop();
                s_command = s_gait.command().position_rad;  // Back to stand.
                printEvent(s_gait.command());
                printRunSummary(s_gait.command());
                s_state = ControllerState::kStanding;
            }
            break;
        case 'p': printStatus(); break;
        case 'v': printParams(); printImuMap(); break;
        case '?': case 'h': printHelp(); break;
        case 'i':
            s_monitor = s_monitor == Monitor::kImu ? Monitor::kOff : Monitor::kImu;
            break;
        case 'e':
            s_monitor = s_monitor == Monitor::kEstimator ? Monitor::kOff : Monitor::kEstimator;
            break;
        case 'c':
            s_csv = !s_csv;
            if (s_csv) printCsvHeader();
            break;
        case 'b':
            if (!idleForEdit()) break;
            s_readback = !s_readback;
            s_readback_fail_streak = 0;
            configureEstimator();
            std::printf("servo read-back %s\n", s_readback ? "on" : "off");
            break;
        case 'X': flipImu(0); break;
        case 'Y': flipImu(1); break;
        case 'Z': flipImu(2); break;
        case 'W':
            if (!idleForEdit()) break;
            s_imu_map.swap_xy = !s_imu_map.swap_xy;
            s_imu_verified = false;
            s_filter.reset();
            printImuMap();
            break;
        case 'k':
            s_imu_verified = true;
            std::printf("IMU mapping confirmed for this boot. Copy it into main/ImuMounting.h:\n");
            printImuMap();
            break;
        case 'r': startStepping(gait::Mode::kOpenLoop, 1, gait::Side::kRight); break;
        case 'l': startStepping(gait::Mode::kOpenLoop, 1, gait::Side::kLeft); break;
        case 'f': startStepping(gait::Mode::kOpenLoop, 0, gait::Side::kRight); break;
        case 'd': startStepping(gait::Mode::kDcm, 0, gait::Side::kRight); break;
        case '+': adjust(p.lift_deg, 1.0f, 8.0f, 22.0f, "lift"); break;
        case '-': adjust(p.lift_deg, -1.0f, 8.0f, 22.0f, "lift"); break;
        case '[': adjust(p.touchdown_dcm_inside_mm, -1.0f, 0.0f, 45.0f, "touchdown_inside"); break;
        case ']': adjust(p.touchdown_dcm_inside_mm, 1.0f, 0.0f, 45.0f, "touchdown_inside"); break;
        case ';': adjust(p.lift_dcm_past_inner_mm, -1.0f, -25.0f, 25.0f, "lift_past_inner"); break;
        case '\'': adjust(p.lift_dcm_past_inner_mm, 1.0f, -25.0f, 25.0f, "lift_past_inner"); break;
        case ',': adjustPeriod(-10); break;
        case '.': adjustPeriod(+10); break;
        case 'n':
            if (!idleForEdit()) break;
            p.max_steps = p.max_steps >= 2 ? p.max_steps - 2 : 0;
            printParams();
            break;
        case 'm':
            if (!idleForEdit()) break;
            p.max_steps = std::min<std::uint32_t>(p.max_steps + 2, 100);
            printParams();
            break;
        default: break;
    }
}

// ---------------------------------------------------------------- control loop
void updateImu() {
    if (!s_imu_available) {
        s_imu_sample.valid = false;
        return;
    }
    tilt::ImuRaw raw{};
    if (!tilt::imu_read_raw(raw)) {
        s_imu_sample.valid = false;
        return;
    }
    const std::int64_t now = esp_timer_get_time();
    const float dt = s_last_imu_us == 0 ? kDt : static_cast<float>(now - s_last_imu_us) / 1.0e6f;
    s_last_imu_us = now;
    s_imu_robot = toRobotFrame(raw, s_imu_map);
    const tilt::Attitude a = s_filter.update(s_imu_robot, dt);
    s_imu_sample.roll_rad = a.roll_rad;
    s_imu_sample.pitch_rad = a.pitch_rad;
    s_imu_sample.gyro_rad_s[0] = tilt::gyro_to_rad_s(s_imu_robot.gx);
    s_imu_sample.gyro_rad_s[1] = tilt::gyro_to_rad_s(s_imu_robot.gy);
    s_imu_sample.gyro_rad_s[2] = tilt::gyro_to_rad_s(s_imu_robot.gz);
    s_imu_sample.valid = true;
}

void recordGaitEvent(const gait::Command& g) {
    if (g.last_trigger == gait::Trigger::kDcmTouchdown ||
        g.last_trigger == gait::Trigger::kSwingTimeout ||
        (g.last_trigger == gait::Trigger::kOpenLoopTimer && g.phase != gait::Phase::kSwing) ||
        (!g.running && g.stop_reason == gait::StopReason::kStepLimit)) {
        // A swing just ended.
        const std::uint32_t ms = static_cast<std::uint32_t>(
            (esp_timer_get_time() - s_swing_start_us) / 1000);
        ++s_run.swings;
        s_run.swing_ms_sum += ms;
        s_run.swing_ms_min = std::min(s_run.swing_ms_min, ms);
        s_run.swing_ms_max = std::max(s_run.swing_ms_max, ms);
        if (g.last_trigger == gait::Trigger::kDcmTouchdown) ++s_run.dcm_touchdowns;
        if (g.last_trigger == gait::Trigger::kSwingTimeout) ++s_run.timeouts;
    }
    if (g.phase == gait::Phase::kSwing) s_swing_start_us = esp_timer_get_time();
}

void updateCommand() {
    if (s_state == ControllerState::kMovingToStand) {
        s_transition_elapsed_ms = std::min(s_transition_elapsed_ms + kControlPeriodMs,
                                           kStandTransitionMs);
        const float blend = smoothstep5(static_cast<float>(s_transition_elapsed_ms) /
                                        static_cast<float>(kStandTransitionMs));
        const JointArray stand = standPose();
        for (std::size_t j = 0; j < kJointCount; ++j) {
            s_command[j] = s_transition_start[j] + (stand[j] - s_transition_start[j]) * blend;
        }
        if (s_transition_elapsed_ms >= kStandTransitionMs) {
            s_state = ControllerState::kStanding;
            std::printf("reached STANDING\n");
        }
    } else if (s_state == ControllerState::kStepping) {
        const gait::Command& g = s_gait.update(s_estimator.state(), kControlPeriodMs);
        s_command = g.position_rad;
        if (g.phase_changed) {
            recordGaitEvent(g);
            printEvent(g);
        }
        s_run.max_abs_roll = std::max(s_run.max_abs_roll, std::fabs(s_imu_sample.roll_rad));
        if (!g.running) {
            printRunSummary(g);
            s_state = ControllerState::kStanding;
        }
    }
}

void writeCommand() {
    std::array<std::uint16_t, kJointCount> ticks{};
    int bad_joint = -1;
    if (!withinLimits(s_command, bad_joint)) {
        emergencyStop(bad_joint >= 0 ? tilt::JOINT_NAME[bad_joint] : "invalid command");
        return;
    }
    for (std::size_t j = 0; j < kJointCount; ++j) {
        if (!radToTick(static_cast<int>(j), s_command[j], ticks[j])) {
            emergencyStop(tilt::JOINT_NAME[j]);
            return;
        }
    }
    if (s_bus.syncWritePositions(tilt::SERVO_ID, ticks.data(), kJointCount, 0, 0) != ESP_OK) {
        emergencyStop("sync write failure");
    }
}

void controlTask(void*) {
    TickType_t last_wake = xTaskGetTickCount();
    while (true) {
        const std::int64_t cycle_start = esp_timer_get_time();

        std::uint8_t input = 0;
        while (usb_serial_jtag_read_bytes(&input, 1, 0) > 0) handleInput(input);

        updateImu();
        if (s_state != ControllerState::kUnarmed && s_imu_sample.valid) {
            const float tilt_rad = std::max(std::fabs(s_imu_sample.roll_rad),
                                            std::fabs(s_imu_sample.pitch_rad));
            if (tilt_rad > kEmergencyTiltRad) {
                emergencyStop("IMU tilt > 35 deg");
            } else if (s_state == ControllerState::kStepping && tilt_rad > kGaitStopTiltRad) {
                s_gait.stop();
                s_command = s_gait.command().position_rad;  // Back to stand.
                std::printf("gait stopped: IMU tilt > 20 deg\n");
                printRunSummary(s_gait.command());
                s_state = ControllerState::kStanding;
            }
        }

        if (s_state != ControllerState::kUnarmed) {
            // 1. Joint angles for the estimator.
            if (s_readback) {
                if (readJoints(s_measured)) {
                    s_readback_fail_streak = 0;
                } else {
                    ++s_readback_failures;
                    if (++s_readback_fail_streak >= kReadbackFailLimit) {
                        s_readback = false;
                        configureEstimator();
                        std::printf("servo read-back disabled after %d failed cycles\n",
                                    kReadbackFailLimit);
                    }
                }
            }
            // 2. Estimator with the support of the current gait phase.
            const est::Support support = s_state == ControllerState::kStepping
                ? s_gait.command().support
                : est::Support::kDouble;
            s_estimator.update(s_readback ? s_measured.data() : s_command.data(),
                               s_imu_sample, support, kDt);
            // 3. Gait / stand transition, then write.
            updateCommand();
            if (s_state != ControllerState::kUnarmed) writeCommand();
        } else if (s_monitor == Monitor::kEstimator) {
            // Unarmed: run the estimator on read-back angles so it can be checked by hand.
            readJoints(s_measured);
            s_estimator.update(s_measured.data(), s_imu_sample, est::Support::kDouble, kDt);
        }

        s_cycle_us = esp_timer_get_time() - cycle_start;
        if (s_csv) printCsvRow();
        if (++s_print_divider >= 10) {
            s_print_divider = 0;
            if (!s_csv) printMonitor();
        }
        if (esp_timer_get_time() - cycle_start > kControlPeriodUs) ++s_overruns;
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(kControlPeriodMs));
    }
}

void initializeServos() {
    for (std::size_t j = 0; j < kJointCount; ++j) {
        const std::uint8_t id = tilt::SERVO_ID[j];
        const esp_err_t ping = s_bus.ping(id);
        std::printf("[SERVO] %s ID %u: %s\n", tilt::JOINT_NAME[j], id,
                    ping == ESP_OK ? "OK" : esp_err_to_name(ping));
        s_bus.setAcceleration(id, 0);
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    s_bus.setTorqueAll(tilt::SERVO_ID, kJointCount, false);
}

}  // namespace

extern "C" void app_main() {
    s_boot_us = esp_timer_get_time();
    s_command = standPose();
    s_measured = standPose();
    s_params.max_steps = 10;  // Conservative default for the first runs.
    s_gait.setParams(s_params);
    configureEstimator();

    if (s_bus.initialize() != ESP_OK) {
        std::printf("servo bus initialization failed\n");
        return;
    }
    usb_serial_jtag_driver_config_t console_config = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    const esp_err_t console_result = usb_serial_jtag_driver_install(&console_config);
    if (console_result != ESP_OK && console_result != ESP_ERR_INVALID_STATE) {
        std::printf("USB console initialization failed: %s\n", esp_err_to_name(console_result));
        return;
    }

    initializeServos();
    s_imu_available = tilt::imu_init();
    std::printf(s_imu_available ? "[IMU] ready; E-STOP at 35 deg, gait stop at 20 deg\n"
                                : "[IMU] unavailable; stepping is disabled\n");
    printHelp();

    if (xTaskCreate(controlTask, "dcm_control", 8192, nullptr, 5, nullptr) != pdPASS) {
        emergencyStop("task creation failure");
    }
}
