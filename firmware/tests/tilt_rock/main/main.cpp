// TILT mk.2 rocking with ankle pitch (tilt_rock).
//
// Clone of tests/dcm_rocking_test for the 8-joint legs (hip roll, hip pitch,
// knee pitch, ankle pitch). Stand pose [0, -20, +40, -20] deg (sole flat); every
// lift keeps the sole parallel to the floor (DcmStepping LiftMode, 'V' toggles):
//   flash    hip -L, knee +L        (same leg motion as dcm_rocking_test)
//   vertical hip -L, knee +2L, ankle -L (sole straight up)
// Stepping in place gated by the estimated DCM (tilt_dcm_gait::DcmStepping),
// with the open-loop pattern as a baseline. See README.md for the hardware
// procedure. Single-key console over USB Serial/JTAG.

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
#include "tilt/dcm_gait/GaitFeedback.h"
#include "tilt/dcm_gait/WalkStride.h"
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

enum class ControllerState { kUnarmed, kArmedHold, kMovingToStand, kStanding, kStepping, kStepTest };
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
// IMU feedback layers on top of the stepping pattern (see GaitFeedback.h).
gait::FeedbackParams s_fb_params;
gait::GaitFeedback s_feedback;
bool s_single_lift = false;  // 'r' / 'l': no start ramp.
// Walking: fore-aft foot placement on top of the rocking (see WalkStride.h).
gait::WalkStride s_walk;
bool s_walk_stop_pending = false;  // 'x' while walking: slow to 0, then stop.
float s_walk_saved_target = 0.0f;
float s_stand_blend = 1.0f;  // Fades the posture trim in during the stand transition.
bool s_fade_trim = false;    // Only when the transition starts without trim (after arming).

ControllerState s_state = ControllerState::kUnarmed;
Monitor s_monitor = Monitor::kOff;
ImuMap s_imu_map;
bool s_imu_available = false;
bool s_imu_verified = tilt_dcm_test::kImuVerified;
bool s_csv = false;
bool s_readback = true;

JointArray s_command{};  // Pattern / stand command (before feedback).
JointArray s_output{};   // s_command + feedback: what is written to the servos.
JointArray s_measured{};
JointArray s_transition_start{};
std::uint32_t s_transition_elapsed_ms = 0;

tilt::ImuRaw s_imu_robot{};

// IMU mounting tilt correction (rotation applied to accel and gyro vectors).
constexpr float kAccelLsbPerG = 8192.0f;        // Must match tilt_MPU6050.
constexpr float kGyroLsbPerDps = 65.5f;         // Must match tilt_MPU6050.
float s_mount_roll_rad = tilt_dcm_test::kImuMountRollDeg * tilt::DEG2RAD;
float s_mount_pitch_rad = tilt_dcm_test::kImuMountPitchDeg * tilt::DEG2RAD;
float s_mount_R[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
// Level calibration: average the axis-mapped accelerometer over N samples.
constexpr int kLevelSamples = 100;
int s_level_count = -1;  // -1 = idle
float s_level_sum[3] = {0.0f, 0.0f, 0.0f};
float s_gyro_sum[3] = {0.0f, 0.0f, 0.0f};
// Gyro bias in axis-mapped counts, measured by the level calibration.
float s_gyro_bias[3] = {tilt_dcm_test::kImuGyroBiasCounts[0], tilt_dcm_test::kImuGyroBiasCounts[1],
                        tilt_dcm_test::kImuGyroBiasCounts[2]};
est::ImuSample s_imu_sample{};
std::int64_t s_last_imu_us = 0;

std::uint32_t s_overruns = 0;
std::uint32_t s_readback_failures = 0;
int s_readback_fail_streak = 0;
std::int64_t s_readback_us = 0;
std::int64_t s_cycle_us = 0;
std::uint32_t s_print_divider = 0;
std::int64_t s_boot_us = 0;

// Servo motion settings (diagnostics). Acceleration register 41: 0 is meant to
// disable the ramp; speed is the sync-write goal speed (0 = firmware default).
// 254 = fastest. Register 85 (acceleration limit, factory 50 = ~435 deg/s^2)
// caps register 41, so both are written.
constexpr std::uint8_t kAccOptions[] = {254, 150, 100, 50};
constexpr std::uint16_t kSpeedOptions[] = {0, 3400, 2000, 1000};
std::size_t s_acc_index = 0;
std::size_t s_speed_index = 0;
constexpr std::uint8_t kAddressPresentVoltage = 62;
constexpr std::uint8_t kAddressAccelerationReg = 41;
constexpr std::uint8_t kAddressAccelerationLimit = 85;
constexpr std::uint8_t kAddressLock = 55;

// Step response test: right hip/knee fold for 300 ms, then back, 600 ms total.
constexpr std::uint32_t kStepTestHoldMs = 300;
constexpr std::uint32_t kStepTestTotalMs = 600;
constexpr int kStepTestSamples = kStepTestTotalMs / kControlPeriodMs;
struct StepSample {
    float cmd_knee_deg;
    float meas_knee_deg;
    float meas_hip_deg;
    float volt;
};
StepSample s_step_log[kStepTestSamples];
std::uint32_t s_step_elapsed_ms = 0;

// Stepping statistics.
struct RunStats {
    std::uint32_t swings = 0;
    std::uint32_t dcm_touchdowns = 0;
    std::uint32_t timeouts = 0;
    std::uint32_t dcm_lifts = 0;
    std::uint32_t fallback_lifts = 0;
    std::uint32_t phase_lifts = 0;
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
        case ControllerState::kStepTest: return "STEP_TEST";
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

// R = Ry(pitch) * Rx(roll): maps the board's measured vectors to the robot frame.
void updateMountRotation() {
    const float cr = std::cos(s_mount_roll_rad);
    const float sr = std::sin(s_mount_roll_rad);
    const float cp = std::cos(s_mount_pitch_rad);
    const float sp = std::sin(s_mount_pitch_rad);
    const float R[3][3] = {{cp, sp * sr, sp * cr}, {0.0f, cr, -sr}, {-sp, cp * sr, cp * cr}};
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) s_mount_R[r][c] = R[r][c];
    }
}

std::int16_t toCounts(float value) {
    const long v = std::lround(value);
    return static_cast<std::int16_t>(std::clamp<long>(v, INT16_MIN, INT16_MAX));
}

// Rotates accel and gyro (in counts) by the mounting rotation.
tilt::ImuRaw applyMounting(const tilt::ImuRaw& in) {
    const float a[3] = {static_cast<float>(in.ax), static_cast<float>(in.ay),
                        static_cast<float>(in.az)};
    const float g[3] = {static_cast<float>(in.gx), static_cast<float>(in.gy),
                        static_cast<float>(in.gz)};
    float ra[3];
    float rg[3];
    for (int r = 0; r < 3; ++r) {
        ra[r] = s_mount_R[r][0] * a[0] + s_mount_R[r][1] * a[1] + s_mount_R[r][2] * a[2];
        rg[r] = s_mount_R[r][0] * g[0] + s_mount_R[r][1] * g[1] + s_mount_R[r][2] * g[2];
    }
    tilt::ImuRaw out{};
    out.ax = toCounts(ra[0]);
    out.ay = toCounts(ra[1]);
    out.az = toCounts(ra[2]);
    out.gx = toCounts(rg[0]);
    out.gy = toCounts(rg[1]);
    out.gz = toCounts(rg[2]);
    return out;
}

bool withinLimits(const JointArray& command, int& bad_joint) {
    for (int joint = 0; joint < tilt::NUM_JOINTS; ++joint) {
        const auto& limit =
            tilt::JOINT_LIMIT[joint / tilt::NUM_LEG_JOINTS][joint % tilt::NUM_LEG_JOINTS];
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
        "params: lift=%.0fdeg (%s) max_steps=%lu | DCM: touchdown_inside=%.0fmm "
        "lift_past_inner=%+.0fmm swing=%lu..%lums double=%lu..%lums start_hold=%lums "
        "abort_outer=%.0fmm fallbacks=%lu auto_first=%s | open-loop: hold=%lums period=%lums\n",
        p.lift_deg, gait::liftModeName(p.lift_mode), static_cast<unsigned long>(p.max_steps),
        p.touchdown_dcm_inside_mm,
        p.lift_dcm_past_inner_mm, static_cast<unsigned long>(p.swing_min_ms),
        static_cast<unsigned long>(p.swing_max_ms),
        static_cast<unsigned long>(p.double_min_ms),
        static_cast<unsigned long>(p.double_max_ms),
        static_cast<unsigned long>(p.start_hold_ms), p.abort_outer_mm,
        static_cast<unsigned long>(p.max_double_fallbacks), p.auto_first_swing ? "on" : "off",
        static_cast<unsigned long>(p.open_hold_ms),
        static_cast<unsigned long>(p.open_period_ms));
}

void printFeedbackParams() {
    const gait::FeedbackParams& f = s_fb_params;
    std::printf(
        "feedback: [1] roll trim %s (gain %.2f) [5] pitch trim %s (gain %.2f -> target %+.1fdeg) max %.0f/%.0fdeg | "
        "[2] amplitude %s (target %.1fdeg, gain %.2f, max %.0fdeg) | "
        "[3] damping %s (roll %.3f, pitch %.3f deg per deg/s) | [4] start ramp %lu steps | "
        "[6] phase sync %s (window +-%lums)\n",
        f.trim_enabled ? "ON" : "off", static_cast<double>(f.roll_trim_gain),
        f.pitch_trim_enabled ? "ON" : "off", static_cast<double>(f.pitch_trim_gain), static_cast<double>(f.pitch_target_deg),
        static_cast<double>(f.roll_trim_max_deg), static_cast<double>(f.pitch_trim_max_deg),
        f.amp_enabled ? "ON" : "off", static_cast<double>(f.amp_target_deg),
        static_cast<double>(f.amp_gain), static_cast<double>(f.amp_max_deg),
        f.damp_enabled ? "ON" : "off", static_cast<double>(f.damp_roll_gain),
        static_cast<double>(f.damp_pitch_gain), static_cast<unsigned long>(f.ramp_steps),
        f.phase_sync_enabled ? "ON" : "off",
        static_cast<unsigned long>(s_params.lift_hint_window_ms));
}

void printFeedbackState(const char* prefix) {
    const gait::FeedbackState& f = s_feedback.state();
    std::printf(
        "%s trim roll %+.2f pitch %+.2f (applied %+.2f %+.2f) | lift delta L %+.2f R %+.2f | "
        "last excursion L %.1f R %.1f | cycle mean roll %+.2f pitch %+.2f | phase nom L %s%+.0f R %s%+.0f | "
        "walk %+.1f/%+.0f mm | cycles %lu%s%s%s%s\n",
        prefix, static_cast<double>(f.roll_trim_deg), static_cast<double>(f.pitch_trim_deg),
        static_cast<double>(f.applied_roll_deg), static_cast<double>(f.applied_pitch_deg),
        static_cast<double>(f.lift_delta_deg[0]), static_cast<double>(f.lift_delta_deg[1]),
        static_cast<double>(f.excursion_deg[0]), static_cast<double>(f.excursion_deg[1]),
        static_cast<double>(f.cycle_roll_deg), static_cast<double>(f.cycle_pitch_deg),
        f.phase_learned[0] ? "" : "?", static_cast<double>(f.phase_nominal_deg[0]),
        f.phase_learned[1] ? "" : "?", static_cast<double>(f.phase_nominal_deg[1]),
        static_cast<double>(s_walk.stride_mm()), static_cast<double>(s_walk.params().stride_target_mm),
        static_cast<unsigned long>(f.cycles), f.last_window_frozen ? " (frozen)" : "",
        f.rhythm_stable ? " stable" : "",
        f.roll_trim_saturated ? " !ROLL TRIM SATURATED" : "",
        f.pitch_trim_saturated ? " !PITCH TRIM SATURATED" : "");
}

// Pushes the operator's feedback settings into the controller (the start ramp
// is skipped for single lifts).
void applyFeedbackParams() {
    gait::FeedbackParams p = s_fb_params;
    if (s_single_lift) p.ramp_steps = 0;
    s_feedback.setParams(p);
}

void printImuMap() {
    std::printf("IMU map: swap_xy=%s sign x=%+d y=%+d z=%+d  verified=%s\n",
                s_imu_map.swap_xy ? "true" : "false", s_imu_map.sign[0],
                s_imu_map.sign[1], s_imu_map.sign[2],
                s_imu_verified ? "yes" : "NO (DCM stepping locked)");
    std::printf("IMU mount tilt: roll=%+.2f deg pitch=%+.2f deg  (kImuMountRollDeg / kImuMountPitchDeg)\n",
                static_cast<double>(s_mount_roll_rad * kRadToDeg),
                static_cast<double>(s_mount_pitch_rad * kRadToDeg));
    std::printf("IMU gyro bias [counts]: {%.1ff, %.1ff, %.1ff} = %+.2f %+.2f %+.2f dps  (kImuGyroBiasCounts)\n",
                static_cast<double>(s_gyro_bias[0]), static_cast<double>(s_gyro_bias[1]),
                static_cast<double>(s_gyro_bias[2]),
                static_cast<double>(s_gyro_bias[0] / kGyroLsbPerDps),
                static_cast<double>(s_gyro_bias[1] / kGyroLsbPerDps),
                static_cast<double>(s_gyro_bias[2] / kGyroLsbPerDps));
}

void printHelp() {
    std::printf("\nTILT mk.2 tilt_rock (8 joints: hip roll / hip pitch / knee / ankle)\n");
    std::printf("  o  arm at measured pose      s  move to stand (2 s)      SPACE  E-STOP\n");
    std::printf("  i  IMU monitor (10 Hz)       e  estimator monitor        c  CSV 100 Hz\n");
    std::printf("  X/Y/Z  flip IMU axis sign    W  swap IMU x/y             k  confirm IMU check\n");
    std::printf("  b  servo read-back on/off    p  status                   v  params\n");
    std::printf("  L  level calibration (robot standing level, keep still 1 s)\n");
    std::printf("  T  servo step test (right leg lift, robot hung)  A  cycle acceleration (254/150/100/50)\n");
    std::printf("  S  cycle sync speed                         U  save reg85 to servo EEPROM (permanent)\n");
    std::printf("  r / l  one open-loop lift (right / left leg)\n");
    std::printf("  f  open-loop FLASH stepping  d  DCM stepping             x  stop -> stand\n");
    std::printf("  + / -  lift +-1deg           [ / ]  touchdown_inside -+1mm\n");
    std::printf("  V  lift mode flash (hip -L, knee +L) / vertical (hip -L, knee +2L, ankle -L)\n");
    std::printf("  ; / '  lift_past_inner -+1mm , / .  open-loop period -+10ms\n");
    std::printf("  { / }  lift hold -+10ms (open-loop hold and DCM first lift)\n");
    std::printf("  n / m  max_steps -+2 (0 = unlimited)\n");
    std::printf("  IMU feedback (live): 1 roll trim  5 pitch trim  2 amplitude  3 damping  4 start ramp  6 phase sync\n");
    std::printf("  w  phase sync window 20/30/40/60 ms (not while stepping)\n");
    std::printf("  F / B  walk stride +-2 mm (live, ramps at 4 mm/s; x while walking = slow down, x again = stop)\n");
    std::printf("  g  feedback state   0  reset learned trim/lift (not while stepping)\n");
    std::printf("  u / j  pitch target +-0.5deg   t / y  amplitude target +-0.5deg\n");
    std::printf("  q / a  damping pitch gain +-0.01   Q / z  damping roll gain +-0.005\n");
    std::printf("state=%s imu=%s readback=%s\n", stateName(s_state),
                s_imu_available ? "ready" : "unavailable", s_readback ? "on" : "off");
    printImuMap();
    printParams();
    printFeedbackParams();
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
        "cmd_LHR,cmd_LHP,cmd_LKP,cmd_LAP,cmd_RHR,cmd_RHP,cmd_RKP,cmd_RAP,"
        "meas_LHR,meas_LHP,meas_LKP,meas_LAP,meas_RHR,meas_RHP,meas_RKP,meas_RAP,"
        "roll,pitch,gx,gy,gz,com_x,com_y,com_z,vel_y,dcm_x,dcm_y,"
        "inner_L,inner_R,outer_L,outer_R,inside_swing,cycle_us,readback_us,"
        "fb_roll_trim,fb_pitch_trim,fb_lift_L,fb_lift_R,fb_damp_roll,fb_damp_pitch,"
        "walk_stride,foot_x_L,foot_x_R\n");
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
    for (std::size_t j = 0; j < kJointCount; ++j) std::printf(",%.2f", s_output[j] * kRadToDeg);
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
    std::printf(",%.1f,%.1f,%.1f,%.1f,%.1f,%lld,%lld", s.dcm_past_inner_edge_mm[0],
                s.dcm_past_inner_edge_mm[1], s.dcm_past_outer_edge_mm[0],
                s.dcm_past_outer_edge_mm[1], inside, static_cast<long long>(s_cycle_us),
                static_cast<long long>(s_readback_us));
    const gait::FeedbackState& f = s_feedback.state();
    const float lift_l = g.phase == gait::Phase::kSwing && g.swing == gait::Side::kLeft
        ? s_feedback.extraLiftDeg(g, s_gait.params().lift_deg) : 0.0f;
    const float lift_r = g.phase == gait::Phase::kSwing && g.swing == gait::Side::kRight
        ? s_feedback.extraLiftDeg(g, s_gait.params().lift_deg) : 0.0f;
    std::printf(",%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f\n", f.applied_roll_deg,
                f.applied_pitch_deg, lift_l, lift_r, f.damp_roll_deg, f.damp_pitch_deg,
                s_walk.stride_mm(), s_walk.foot_x_mm(0), s_walk.foot_x_mm(1));
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
    if (g.phase == gait::Phase::kSwing && g.running) {
        std::printf(" ph=%+.0f", static_cast<double>(s_feedback.phaseDeg(g.swing)));
    }
    if (!g.running) std::printf(" STOP=%s", gait::stopReasonName(g.stop_reason));
    std::printf("\n");
}

void printRunSummary(const gait::Command& g) {
    const float seconds =
        static_cast<float>(esp_timer_get_time() - s_run.start_us) / 1.0e6f;
    std::printf(
        "\n== run: %s, %lu steps in %.2f s, stop=%s | swings %lu (dcm touchdown %lu, "
        "timeout %lu) lifts (dcm %lu, timer fallback %lu, phase sync %lu) swing ms min/avg/max %lu/%lu/%lu | max |roll| %.1f deg | "
        "overruns %lu readback failures %lu ==\n\n",
        s_gait.params().mode == gait::Mode::kDcm ? "DCM" : "OPEN-LOOP",
        static_cast<unsigned long>(g.steps), seconds, gait::stopReasonName(g.stop_reason),
        static_cast<unsigned long>(s_run.swings),
        static_cast<unsigned long>(s_run.dcm_touchdowns),
        static_cast<unsigned long>(s_run.timeouts),
        static_cast<unsigned long>(s_run.dcm_lifts),
        static_cast<unsigned long>(s_run.fallback_lifts),
        static_cast<unsigned long>(s_run.phase_lifts),
        static_cast<unsigned long>(s_run.swings ? s_run.swing_ms_min : 0),
        static_cast<unsigned long>(s_run.swings ? s_run.swing_ms_sum / s_run.swings : 0),
        static_cast<unsigned long>(s_run.swing_ms_max), s_run.max_abs_roll * kRadToDeg,
        static_cast<unsigned long>(s_overruns),
        static_cast<unsigned long>(s_readback_failures));
    printFeedbackState("== feedback:");
    std::printf("\n");
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
            out[j] = s_output[j];
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
    s_output = measured;
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
    s_fade_trim = s_state == ControllerState::kArmedHold;
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
    if (mode != gait::Mode::kDcm) p.auto_first_swing = false;  // r / l / f keep their side.
    if (max_steps_override > 0) p.max_steps = max_steps_override;
    s_gait.setParams(p);
    s_single_lift = max_steps_override == 1;
    // Every run starts in place: a stride left over from the last run made the
    // start-up rocking fall over (2026-10-09). F / B change it once it rocks.
    s_walk.params().stride_target_mm = s_walk.params().stride_in_place_mm;
    s_walk_stop_pending = false;
    s_walk.reset();
    applyFeedbackParams();
    s_feedback.beginRun();
    s_run = RunStats{};
    s_run.start_us = esp_timer_get_time();
    s_gait.start(&s_estimator.state());
    s_swing_start_us = esp_timer_get_time();
    s_state = ControllerState::kStepping;
    std::printf("stepping started: %s, first swing %s%s, max_steps %lu\n",
                mode == gait::Mode::kDcm ? "DCM" : "OPEN-LOOP",
                s_gait.command().swing == gait::Side::kLeft ? "L" : "R",
                p.auto_first_swing ? " (auto)" : "",
                static_cast<unsigned long>(p.max_steps));
    printFeedbackParams();
    printFeedbackState("FB start:");
    std::printf("walk: start in place at stride %+.0f mm, F / B to change\n",
                static_cast<double>(s_walk.params().stride_target_mm));
    printEvent(s_gait.command());
}

bool idleForEdit() {
    if (s_state == ControllerState::kStepping || s_state == ControllerState::kStepTest) {
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

// Write then read back (writeRegister does not wait for the status reply, so
// a write can be lost on the half-duplex bus). Returns true when it sticks.
bool writeVerified(std::uint8_t id, std::uint8_t address, std::uint8_t value,
                   std::uint8_t& read_back) {
    read_back = 0xFF;
    for (int attempt = 0; attempt < 5; ++attempt) {
        s_bus.writeRegister(id, address, &value, 1);
        vTaskDelay(pdMS_TO_TICKS(2));
        if (s_bus.readRegister(id, address, &read_back, 1) == ESP_OK && read_back == value) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    return false;
}

// Writes registers 85 (acceleration limit) and 41 (acceleration) on every
// servo and prints what was read back. RAM only: lost when servo power is cut.
void applyAcceleration() {
    const std::uint8_t acc = kAccOptions[s_acc_index];
    std::printf("acceleration <- %u (reg85 limit + reg41)\n", static_cast<unsigned>(acc));
    for (std::size_t j = 0; j < kJointCount; ++j) {
        const std::uint8_t id = tilt::SERVO_ID[j];
        std::uint8_t r85 = 0xFF, r41 = 0xFF, r86 = 0xFF;
        const bool ok85 = writeVerified(id, kAddressAccelerationLimit, acc, r85);
        const bool ok41 = writeVerified(id, kAddressAccelerationReg, acc, r41);
        s_bus.readRegister(id, 86, &r86, 1);
        std::printf("  %s id%u: reg85=%u%s reg41=%u%s reg86=%u\n", tilt::JOINT_NAME[j], id, r85,
                    ok85 ? "" : " (NOT WRITTEN)", r41, ok41 ? "" : " (NOT WRITTEN)", r86);
    }
}

// Saves reg85 to the servo EEPROM (unlock, write, lock). Permanent until
// changed again; use only if the RAM write above does not stick.
void persistAccelerationLimit() {
    const std::uint8_t acc = kAccOptions[s_acc_index];
    std::printf("PERSIST reg85 = %u to servo EEPROM\n", static_cast<unsigned>(acc));
    for (std::size_t j = 0; j < kJointCount; ++j) {
        const std::uint8_t id = tilt::SERVO_ID[j];
        std::uint8_t lock_rb = 0xFF, r85 = 0xFF;
        const bool unlocked = writeVerified(id, kAddressLock, 0, lock_rb);
        const bool ok85 = writeVerified(id, kAddressAccelerationLimit, acc, r85);
        const bool locked = writeVerified(id, kAddressLock, 1, lock_rb);
        std::printf("  %s id%u: unlock=%s reg85=%u%s lock=%s\n", tilt::JOINT_NAME[j], id,
                    unlocked ? "ok" : "FAIL", r85, ok85 ? "" : " (NOT WRITTEN)", locked ? "ok" : "FAIL");
    }
}

void printServoSettings() {
    std::printf("servo settings: acceleration=%u  sync speed=%u\n",
                static_cast<unsigned>(kAccOptions[s_acc_index]),
                static_cast<unsigned>(kSpeedOptions[s_speed_index]));
}

void startStepTest() {
    if (s_state != ControllerState::kStanding) {
        std::printf("rejected: reach STANDING first ('s'); hang the robot so the feet are free\n");
        return;
    }
    s_step_elapsed_ms = 0;
    s_state = ControllerState::kStepTest;
    std::printf("step test: right leg %s lift %.0f deg for %lu ms (",
                gait::liftModeName(s_params.lift_mode), static_cast<double>(s_params.lift_deg),
                static_cast<unsigned long>(kStepTestHoldMs));
    printServoSettings();
}

void updateStepTest() {
    JointArray q = standPose();
    if (s_step_elapsed_ms < kStepTestHoldMs) {
        gait::addLift(q, gait::Side::kRight, s_params.lift_deg * tilt::DEG2RAD, s_params.lift_mode);
    }
    s_command = q;
    const int k = static_cast<int>(s_step_elapsed_ms / kControlPeriodMs);
    if (k < kStepTestSamples) {
        std::uint8_t v = 0;
        const bool ok = s_bus.readRegister(tilt::SERVO_ID[tilt::R_KNEE_PITCH], kAddressPresentVoltage, &v, 1) == ESP_OK;
        s_step_log[k] = {q[tilt::R_KNEE_PITCH] * kRadToDeg, s_measured[tilt::R_KNEE_PITCH] * kRadToDeg,
                         s_measured[tilt::R_HIP_PITCH] * kRadToDeg, ok ? v * 0.1f : NAN};
    }
    s_step_elapsed_ms += kControlPeriodMs;
    if (s_step_elapsed_ms < kStepTestTotalMs) return;

    s_state = ControllerState::kStanding;
    s_command = standPose();
    const float start = s_step_log[0].meas_knee_deg;
    // Knee travel: L (flash) or 2L (vertical).
    const float target = s_params.lift_deg * (s_params.lift_mode == gait::LiftMode::kVertical ? 2.0f : 1.0f);
    int dead = -1, t50 = -1, t90 = -1;
    float peak = 0.0f, vmin = 99.0f;
    for (int i = 0; i < kStepTestSamples; ++i) {
        const float d = s_step_log[i].meas_knee_deg - start;
        if (i * static_cast<int>(kControlPeriodMs) < static_cast<int>(kStepTestHoldMs)) {
            if (dead < 0 && d > 0.5f) dead = i * kControlPeriodMs;
            if (t50 < 0 && d > 0.5f * target) t50 = i * kControlPeriodMs;
            if (t90 < 0 && d > 0.9f * target) t90 = i * kControlPeriodMs;
        }
        peak = std::max(peak, d);
        if (std::isfinite(s_step_log[i].volt)) vmin = std::min(vmin, s_step_log[i].volt);
    }
    std::printf("t_ms  cmd_knee  meas_knee  meas_hip  volt\n");
    for (int i = 0; i < kStepTestSamples; i += 2) {
        std::printf("%4d  %7.1f  %8.1f  %8.1f  %4.1f\n", i * static_cast<int>(kControlPeriodMs),
                    static_cast<double>(s_step_log[i].cmd_knee_deg),
                    static_cast<double>(s_step_log[i].meas_knee_deg),
                    static_cast<double>(s_step_log[i].meas_hip_deg),
                    static_cast<double>(s_step_log[i].volt));
    }
    std::printf("STEP RESULT acc=%u speed=%u: dead time %d ms, 50%% at %d ms, 90%% at %d ms, "
                "peak %.1f/%.0f deg, min voltage %.1f V  (-1 = not reached)\n\n",
                static_cast<unsigned>(kAccOptions[s_acc_index]),
                static_cast<unsigned>(kSpeedOptions[s_speed_index]), dead, t50, t90,
                static_cast<double>(peak), static_cast<double>(target), static_cast<double>(vmin));
}

void adjustHold(int delta_ms) {
    if (!idleForEdit()) return;
    const int value = std::clamp(static_cast<int>(s_params.open_hold_ms) + delta_ms, 60, 300);
    s_params.open_hold_ms = static_cast<std::uint32_t>(value);
    s_params.start_hold_ms = static_cast<std::uint32_t>(value);
    if (s_params.open_period_ms < s_params.open_hold_ms + 20) {
        s_params.open_period_ms = s_params.open_hold_ms + 20;
    }
    printParams();
}

void adjustPeriod(int delta_ms) {
    if (!idleForEdit()) return;
    const int value = static_cast<int>(s_params.open_period_ms) + delta_ms;
    s_params.open_period_ms = static_cast<std::uint32_t>(std::clamp(value, 120, 600));
    printParams();
}

void adjustFeedback(float& value, float delta, float low, float high) {
    value = std::clamp(value + delta, low, high);
    applyFeedbackParams();
    printFeedbackParams();
}

void stopSteppingNow() {
    s_gait.stop();
    s_command = s_gait.command().position_rad;  // Back to stand.
    printEvent(s_gait.command());
    printRunSummary(s_gait.command());
    s_state = ControllerState::kStanding;
    if (s_walk_stop_pending) {
        s_walk.params().stride_target_mm = s_walk_saved_target;  // Kept for the next run.
        s_walk_stop_pending = false;
    }
    s_walk.reset();
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
            if (s_state != ControllerState::kStepping) break;
            if (!s_walk_stop_pending && std::fabs(s_walk.stride_mm()) > 0.5f) {
                // Walking: ramp the stride to 0 first (x again = stop now).
                s_walk_saved_target = s_walk.params().stride_target_mm;
                s_walk.params().stride_target_mm = 0.0f;
                s_walk_stop_pending = true;
                std::printf("walk: slowing to a stop (stride %+.1f mm), 'x' again = stop now\n",
                            static_cast<double>(s_walk.stride_mm()));
                break;
            }
            stopSteppingNow();
            break;
        case 'F':
        case 'B': {
            float& target = s_walk_stop_pending ? s_walk_saved_target : s_walk.params().stride_target_mm;
            target = std::clamp(target + (input == 'F' ? 2.0f : -2.0f), -s_walk.params().stride_max_mm,
                                s_walk.params().stride_max_mm);
            std::printf("walk stride target %+.0f mm (now %+.1f mm, ramp %.0f mm/s)\n",
                        static_cast<double>(target), static_cast<double>(s_walk.stride_mm()),
                        static_cast<double>(s_walk.params().ramp_mm_s));
            break;
        }
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
        case 'L':
            if (!idleForEdit()) break;
            if (!s_imu_available) {
                std::printf("IMU unavailable\n");
                break;
            }
            s_level_count = 0;
            s_level_sum[0] = s_level_sum[1] = s_level_sum[2] = 0.0f;
            s_gyro_sum[0] = s_gyro_sum[1] = s_gyro_sum[2] = 0.0f;
            std::printf("level calibration: keep the robot level and still for 1 s...\n");
            break;
        case 'k':
            s_imu_verified = true;
            std::printf("IMU mapping confirmed for this boot. Copy it into main/ImuMounting.h:\n");
            printImuMap();
            break;
        case 'T': startStepTest(); break;
        case 'A':
            if (!idleForEdit() || s_state == ControllerState::kUnarmed) {
                if (s_state == ControllerState::kUnarmed) std::printf("arm first ('o')\n");
                break;
            }
            s_acc_index = (s_acc_index + 1) % (sizeof(kAccOptions) / sizeof(kAccOptions[0]));
            applyAcceleration();
            break;
        case 'U':
            if (!idleForEdit()) break;
            if (s_state == ControllerState::kUnarmed) {
                std::printf("arm first ('o')\n");
                break;
            }
            persistAccelerationLimit();
            break;
        case 'S':
            if (!idleForEdit()) break;
            s_speed_index = (s_speed_index + 1) % (sizeof(kSpeedOptions) / sizeof(kSpeedOptions[0]));
            printServoSettings();
            break;
        case 'r': startStepping(gait::Mode::kOpenLoop, 1, gait::Side::kRight); break;
        case 'l': startStepping(gait::Mode::kOpenLoop, 1, gait::Side::kLeft); break;
        case 'f': startStepping(gait::Mode::kOpenLoop, 0, gait::Side::kRight); break;
        case 'd': startStepping(gait::Mode::kDcm, 0, gait::Side::kRight); break;
        case '+': adjust(p.lift_deg, 1.0f, 4.0f, 22.0f, "lift"); break;
        case '-': adjust(p.lift_deg, -1.0f, 4.0f, 22.0f, "lift"); break;
        case 'V':
            if (!idleForEdit()) break;
            p.lift_mode = p.lift_mode == gait::LiftMode::kFlash ? gait::LiftMode::kVertical
                                                                : gait::LiftMode::kFlash;
            s_gait.setParams(p);
            printParams();
            break;
        case '[': adjust(p.touchdown_dcm_inside_mm, -1.0f, 0.0f, 45.0f, "touchdown_inside"); break;
        case ']': adjust(p.touchdown_dcm_inside_mm, 1.0f, 0.0f, 45.0f, "touchdown_inside"); break;
        case ';': adjust(p.lift_dcm_past_inner_mm, -1.0f, -25.0f, 25.0f, "lift_past_inner"); break;
        case '\'': adjust(p.lift_dcm_past_inner_mm, 1.0f, -25.0f, 25.0f, "lift_past_inner"); break;
        case '{': adjustHold(-10); break;
        case '}': adjustHold(+10); break;
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
        // ---- IMU feedback (live; settings apply from the next control cycle).
        case '1': s_fb_params.trim_enabled = !s_fb_params.trim_enabled; applyFeedbackParams(); printFeedbackParams(); break;
        case '2': s_fb_params.amp_enabled = !s_fb_params.amp_enabled; applyFeedbackParams(); printFeedbackParams(); break;
        case '3': s_fb_params.damp_enabled = !s_fb_params.damp_enabled; applyFeedbackParams(); printFeedbackParams(); break;
        case 'w': {
            if (!idleForEdit()) break;
            static constexpr std::uint32_t kWindows[] = {20, 30, 40, 60};
            std::size_t k = 0;
            while (k < 4 && kWindows[k] != s_params.lift_hint_window_ms) ++k;
            s_params.lift_hint_window_ms = kWindows[(k + 1) % 4];
            s_gait.setParams(s_params);
            printFeedbackParams();
            break;
        }
        case '6': s_fb_params.phase_sync_enabled = !s_fb_params.phase_sync_enabled; applyFeedbackParams(); printFeedbackParams(); break;
        case '5': s_fb_params.pitch_trim_enabled = !s_fb_params.pitch_trim_enabled; applyFeedbackParams(); printFeedbackParams(); break;
        case '4': s_fb_params.ramp_steps = s_fb_params.ramp_steps > 0 ? 0 : 3; applyFeedbackParams(); printFeedbackParams(); break;
        case '0':
            if (!idleForEdit()) break;
            s_feedback.reset();
            printFeedbackState("FB reset:");
            break;
        case 'g': printFeedbackParams(); printFeedbackState("FB:"); break;
        case 'u': adjustFeedback(s_fb_params.pitch_target_deg, 0.5f, -8.0f, 12.0f); break;
        case 'j': adjustFeedback(s_fb_params.pitch_target_deg, -0.5f, -8.0f, 12.0f); break;
        case 't': adjustFeedback(s_fb_params.amp_target_deg, 0.5f, 2.0f, 12.0f); break;
        case 'y': adjustFeedback(s_fb_params.amp_target_deg, -0.5f, 2.0f, 12.0f); break;
        case 'q': adjustFeedback(s_fb_params.damp_pitch_gain, 0.01f, 0.0f, 0.1f); break;
        case 'a': adjustFeedback(s_fb_params.damp_pitch_gain, -0.01f, 0.0f, 0.1f); break;
        case 'Q': adjustFeedback(s_fb_params.damp_roll_gain, 0.005f, 0.0f, 0.05f); break;
        case 'z': adjustFeedback(s_fb_params.damp_roll_gain, -0.005f, 0.0f, 0.05f); break;
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
    const tilt::ImuRaw mapped = toRobotFrame(raw, s_imu_map);
    if (s_level_count >= 0) {
        s_level_sum[0] += static_cast<float>(mapped.ax);
        s_level_sum[1] += static_cast<float>(mapped.ay);
        s_level_sum[2] += static_cast<float>(mapped.az);
        s_gyro_sum[0] += static_cast<float>(mapped.gx);
        s_gyro_sum[1] += static_cast<float>(mapped.gy);
        s_gyro_sum[2] += static_cast<float>(mapped.gz);
        if (++s_level_count >= kLevelSamples) {
            const float ax = s_level_sum[0] / kLevelSamples;
            const float ay = s_level_sum[1] / kLevelSamples;
            const float az = s_level_sum[2] / kLevelSamples;
            const float norm = std::sqrt(ax * ax + ay * ay + az * az) / kAccelLsbPerG;
            // Same convention as the complementary filter's accelerometer angles.
            s_mount_roll_rad = std::atan2(ay, az);
            s_mount_pitch_rad = std::atan2(-ax, std::sqrt(ay * ay + az * az));
            for (int i = 0; i < 3; ++i) s_gyro_bias[i] = s_gyro_sum[i] / kLevelSamples;
            updateMountRotation();
            s_filter.reset();
            s_level_count = -1;
            std::printf("level calibration done: |g| = %.3f g (expect ~1.0)\n",
                        static_cast<double>(norm));
            if (az <= 0.0f) {
                std::printf("WARNING: robot z reads downward; check the Z sign before trusting this.\n");
            }
            printImuMap();
            std::printf("copy into main/ImuMounting.h to keep it after reboot.\n");
        }
    }
    tilt::ImuRaw unbiased = mapped;
    unbiased.gx = toCounts(static_cast<float>(mapped.gx) - s_gyro_bias[0]);
    unbiased.gy = toCounts(static_cast<float>(mapped.gy) - s_gyro_bias[1]);
    unbiased.gz = toCounts(static_cast<float>(mapped.gz) - s_gyro_bias[2]);
    s_imu_robot = applyMounting(unbiased);
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
    if (g.phase == gait::Phase::kSwing) {
        s_swing_start_us = esp_timer_get_time();
        if (g.last_trigger == gait::Trigger::kDcmLift) ++s_run.dcm_lifts;
        if (g.last_trigger == gait::Trigger::kDoubleTimeout) ++s_run.fallback_lifts;
        if (g.last_trigger == gait::Trigger::kPhaseSync) ++s_run.phase_lifts;
    }
}

void updateCommand() {
    if (s_state == ControllerState::kMovingToStand) {
        s_transition_elapsed_ms = std::min(s_transition_elapsed_ms + kControlPeriodMs,
                                           kStandTransitionMs);
        const float blend = smoothstep5(static_cast<float>(s_transition_elapsed_ms) /
                                        static_cast<float>(kStandTransitionMs));
        s_stand_blend = s_fade_trim ? blend : 1.0f;
        const JointArray stand = standPose();
        for (std::size_t j = 0; j < kJointCount; ++j) {
            s_command[j] = s_transition_start[j] + (stand[j] - s_transition_start[j]) * blend;
        }
        if (s_transition_elapsed_ms >= kStandTransitionMs) {
            s_state = ControllerState::kStanding;
            std::printf("reached STANDING\n");
        }
    } else if (s_state == ControllerState::kStepTest) {
        updateStepTest();
    } else if (s_state == ControllerState::kStepping) {
        s_gait.setLiftHint(s_feedback.liftHint(s_gait.command()));  // Layer 0 (open loop).
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

// s_output = s_command + feedback. Posture trim in stand / stand transition /
// stepping; damping and lift changes only while stepping.
void updateFeedbackAndOutput() {
    const bool stepping = s_state == ControllerState::kStepping;
    const gait::Command& g = s_gait.command();
    s_feedback.update(g, stepping, s_imu_sample.roll_rad, s_imu_sample.pitch_rad,
                      s_imu_sample.gyro_rad_s[0], s_imu_sample.gyro_rad_s[1], kDt);
    s_output = s_command;
    const float lift = s_gait.params().lift_deg;
    switch (s_state) {
        case ControllerState::kStepping:
            s_feedback.apply(s_output, g, gait::FeedbackContext::kStepping, lift, 1.0f,
                             s_gait.params().lift_mode);
            s_walk.update(g, s_gait.params(), kControlPeriodMs);
            s_walk.apply(s_output);
            break;
        case ControllerState::kStanding:
            s_feedback.apply(s_output, g, gait::FeedbackContext::kStand, lift);
            break;
        case ControllerState::kMovingToStand:
            s_feedback.apply(s_output, g, gait::FeedbackContext::kStand, lift, s_stand_blend);
            break;
        default: break;
    }
    if (!stepping) {
        s_walk.reset();
        if (s_walk_stop_pending) {  // Stopped some other way (tilt, E-stop).
            s_walk.params().stride_target_mm = s_walk_saved_target;
            s_walk_stop_pending = false;
        }
    }
    if (stepping && s_feedback.state().cycle_done) printFeedbackState("FB");
    // Graceful stop after walking: stride back to ~0, then stop in double support.
    if (stepping && s_walk_stop_pending && std::fabs(s_walk.stride_mm()) < 0.3f &&
        g.phase == gait::Phase::kDouble) {
        stopSteppingNow();
    }
}

void writeCommand() {
    std::array<std::uint16_t, kJointCount> ticks{};
    int bad_joint = -1;
    if (!withinLimits(s_output, bad_joint)) {
        emergencyStop(bad_joint >= 0 ? tilt::JOINT_NAME[bad_joint] : "invalid command");
        return;
    }
    for (std::size_t j = 0; j < kJointCount; ++j) {
        if (!radToTick(static_cast<int>(j), s_output[j], ticks[j])) {
            emergencyStop(tilt::JOINT_NAME[j]);
            return;
        }
    }
    if (s_bus.syncWritePositions(tilt::SERVO_ID, ticks.data(), kJointCount,
                                 kSpeedOptions[s_speed_index], 0) != ESP_OK) {
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
                s_gait.stop(gait::StopReason::kTilt);
                s_feedback.revertToLastGood();  // Do not start the next run with what the fall taught.
                s_command = s_gait.command().position_rad;  // Back to stand.
                std::printf("gait stopped: IMU tilt > 20 deg\n");
                printRunSummary(s_gait.command());
                printFeedbackState("FB reverted to last settled values:");
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
            s_estimator.update(s_readback ? s_measured.data() : s_output.data(),
                               s_imu_sample, support, kDt);
            // 3. Gait / stand transition, IMU feedback on top, then write.
            updateCommand();
            updateFeedbackAndOutput();
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
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    s_bus.setTorqueAll(tilt::SERVO_ID, kJointCount, false);
    applyAcceleration();  // reg85 + reg41 = 254 (RAM), verified
}

}  // namespace

extern "C" void app_main() {
    s_boot_us = esp_timer_get_time();
    s_command = standPose();
    s_output = standPose();
    s_measured = standPose();
    updateMountRotation();
    s_params.max_steps = 0;  // Unlimited: rock until 'x' / SPACE (or a safety stop).
    // Start with the dcm_rocking_test leg motion (flash fold, 15 deg); the ankle
    // only holds the sole flat. 'V' switches to the vertical lift.
    s_params.lift_mode = gait::LiftMode::kFlash;
    s_params.lift_deg = 15.0f;
    // Measured STS3215 response with reg85 = 254: 40-60 ms dead time, 15 deg
    // reaches 90 % at ~170 ms. Hold the swing long enough for the fold.
    s_params.open_hold_ms = 150;
    s_params.open_period_ms = 300;
    s_params.start_hold_ms = 150;
    s_params.swing_min_ms = 100;
    s_params.swing_max_ms = 240;
    // 2026-10-08 hardware run that rocked well on carpet (see README "Results").
    s_params.touchdown_dcm_inside_mm = 16.0f;
    // DCM mode start-up: lift the leg on the side the robot already leans to,
    // and keep stepping on a timer if the DCM lift gate is missed (max 3 in a row).
    s_params.auto_first_swing = true;
    s_params.max_double_fallbacks = 3;
    // Phase sync (layer 0) may move the open-loop lift by at most this much.
    s_params.lift_hint_window_ms = 40;
    s_gait.setParams(s_params);
    applyFeedbackParams();
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

    if (xTaskCreate(controlTask, "tilt_rock", 8192, nullptr, 5, nullptr) != pdPASS) {
        emergencyStop("task creation failure");
    }
}
