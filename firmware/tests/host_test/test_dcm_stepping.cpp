// Host tests for tilt_dcm_gait::DcmStepping using synthetic estimator states.

#include <cmath>
#include <cstdio>

#include "tilt/dcm_gait/DcmStepping.h"

namespace {

using namespace tilt::dcm_gait;

int s_failures = 0;
void expectTrue(bool c, const char* e, const char* t) {
    if (!c) {
        std::printf("FAIL [%s] %s\n", t, e);
        ++s_failures;
    }
}
#define EXPECT_TRUE(t, e) expectTrue((e), #e, t)

constexpr float rad(float d) { return d * tilt::DEG2RAD; }

// DCM between the feet, far from both inner edges.
DcmState centred() {
    DcmState s;
    s.valid = true;
    s.sole_mm[0] = {0.0f, 55.0f, 0.0f};
    s.sole_mm[1] = {0.0f, -55.0f, 0.0f};
    s.dcm_mm[1] = 0.0f;
    s.dcm_past_inner_edge_mm[0] = -36.0f;
    s.dcm_past_inner_edge_mm[1] = -36.0f;
    s.dcm_past_outer_edge_mm[0] = -74.0f;
    s.dcm_past_outer_edge_mm[1] = -74.0f;
    return s;
}

void runFor(DcmStepping& g, const DcmState& s, int ms) {
    for (int k = 0; k < ms / 10; ++k) g.update(s, 10);
}

void testStartAndFirstSwing() {
    constexpr char kT[] = "G1 start / first swing";
    DcmStepping g;
    g.start();
    const Command& c = g.command();
    EXPECT_TRUE(kT, c.running && c.phase == Phase::kSwing);
    EXPECT_TRUE(kT, c.swing == Side::kRight && c.support == Support::kLeft);
    EXPECT_TRUE(kT, std::fabs(c.position_rad[tilt::R_HIP_PITCH] - (standPose()[tilt::R_HIP_PITCH] - rad(15.0f))) < 1e-5f);
    EXPECT_TRUE(kT, std::fabs(c.position_rad[tilt::R_KNEE_PITCH] - (standPose()[tilt::R_KNEE_PITCH] + rad(15.0f))) < 1e-5f);
    EXPECT_TRUE(kT, std::fabs(c.position_rad[tilt::L_HIP_PITCH] - standPose()[tilt::L_HIP_PITCH]) < 1e-5f);
    // First swing is open loop: DCM conditions are ignored until start_hold_ms.
    DcmState s = centred();
    s.dcm_mm[1] = -60.0f;  // Would trigger touchdown in a normal swing.
    runFor(g, s, 80);
    EXPECT_TRUE(kT, g.command().phase == Phase::kSwing);
    runFor(g, s, 10);
    EXPECT_TRUE(kT, g.command().phase == Phase::kDouble && g.command().steps == 1);
}

void testDcmLift() {
    constexpr char kT[] = "G2 DCM lift";
    DcmStepping g;
    g.start();
    runFor(g, centred(), 90);  // -> DOUBLE, right foot just landed.
    DcmState s = centred();
    s.dcm_past_inner_edge_mm[1] = 2.0f;  // DCM past the right (new stance) inner edge.
    g.update(s, 10);  // 10 ms < double_min_ms
    EXPECT_TRUE(kT, g.command().phase == Phase::kDouble);
    g.update(s, 10);  // 20 ms
    EXPECT_TRUE(kT, g.command().phase == Phase::kSwing);
    EXPECT_TRUE(kT, g.command().swing == Side::kLeft);
    EXPECT_TRUE(kT, g.command().support == Support::kRight);
    EXPECT_TRUE(kT, g.command().last_trigger == Trigger::kDcmLift);
}

void testDcmTouchdownAndTimeout() {
    constexpr char kT[] = "G3 DCM touchdown / timeout";
    Params p;
    DcmStepping g(p);
    g.start();
    runFor(g, centred(), 90);
    DcmState lift = centred();
    lift.dcm_past_inner_edge_mm[1] = 1.0f;
    runFor(g, lift, 20);  // Left leg swings, right stance.
    EXPECT_TRUE(kT, g.command().swing == Side::kLeft);
    // DCM moves toward the left (swing) foot: inside distance = 55 - y.
    DcmState near = centred();
    near.dcm_mm[1] = 40.0f;  // 15 mm inside the left foot centre (< 21).
    runFor(g, near, 50);     // 50 ms < swing_min_ms
    EXPECT_TRUE(kT, g.command().phase == Phase::kSwing);
    runFor(g, near, 10);     // 60 ms
    EXPECT_TRUE(kT, g.command().phase == Phase::kDouble);
    EXPECT_TRUE(kT, g.command().last_trigger == Trigger::kDcmTouchdown);
    EXPECT_TRUE(kT, g.command().steps == 2);

    // Next swing (right) with the DCM never reaching the target -> timeout.
    DcmState lift2 = centred();
    lift2.dcm_past_inner_edge_mm[0] = 0.5f;
    runFor(g, lift2, 20);
    EXPECT_TRUE(kT, g.command().swing == Side::kRight);
    runFor(g, centred(), 160);
    EXPECT_TRUE(kT, g.command().phase == Phase::kDouble);
    EXPECT_TRUE(kT, g.command().last_trigger == Trigger::kSwingTimeout);
}

void testDoubleTimeout() {
    constexpr char kT[] = "G4 double timeout";
    // No fallback allowed: stop at the first timeout.
    Params p;
    p.max_double_fallbacks = 0;
    DcmStepping g(p);
    g.start();
    runFor(g, centred(), 90);
    runFor(g, centred(), 300);
    EXPECT_TRUE(kT, !g.running());
    EXPECT_TRUE(kT, g.command().stop_reason == StopReason::kDoubleTimeout);
    EXPECT_TRUE(kT, std::fabs(g.command().position_rad[tilt::R_KNEE_PITCH] - standPose()[tilt::R_KNEE_PITCH]) < 1e-5f);
}

void testDoubleFallbackLift() {
    constexpr char kT[] = "G9 double timeout fallback";
    Params p;
    p.max_double_fallbacks = 2;
    DcmStepping g(p);
    g.start();
    runFor(g, centred(), 90);    // first swing (R) -> DOUBLE
    runFor(g, centred(), 300);   // no DCM lift -> fallback lift of L
    EXPECT_TRUE(kT, g.running() && g.command().phase == Phase::kSwing);
    EXPECT_TRUE(kT, g.command().swing == Side::kLeft);
    EXPECT_TRUE(kT, g.command().last_trigger == Trigger::kDoubleTimeout);
    runFor(g, centred(), 160);   // swing timeout -> DOUBLE
    runFor(g, centred(), 300);   // second fallback
    EXPECT_TRUE(kT, g.running() && g.command().swing == Side::kRight);
    runFor(g, centred(), 160);
    runFor(g, centred(), 300);   // third consecutive timeout -> stop
    EXPECT_TRUE(kT, !g.running());
    EXPECT_TRUE(kT, g.command().stop_reason == StopReason::kDoubleTimeout);

    // A DCM lift resets the fallback counter.
    DcmStepping h(p);
    h.start();
    runFor(h, centred(), 90);
    runFor(h, centred(), 300);   // fallback 1 (L swing)
    runFor(h, centred(), 160);   // -> DOUBLE, next swing R, stance L
    DcmState lift = centred();
    lift.dcm_past_inner_edge_mm[0] = 1.0f;
    runFor(h, lift, 20);         // DCM lift resets the counter
    EXPECT_TRUE(kT, h.command().last_trigger == Trigger::kDcmLift);
    runFor(h, centred(), 160);
    runFor(h, centred(), 300);   // fallback 1 again
    runFor(h, centred(), 160);
    runFor(h, centred(), 300);   // fallback 2
    EXPECT_TRUE(kT, h.running());
}

void testAutoFirstSwing() {
    constexpr char kT[] = "G10 auto first swing";
    Params p;
    p.auto_first_swing = true;
    DcmState s = centred();
    s.dcm_mm[1] = 3.0f;  // leaning left
    DcmStepping g(p);
    g.start(&s);
    EXPECT_TRUE(kT, g.command().swing == Side::kLeft);
    s.dcm_mm[1] = -3.0f;
    g.start(&s);
    EXPECT_TRUE(kT, g.command().swing == Side::kRight);
    s.dcm_mm[1] = 0.5f;  // inside the deadband -> configured first_swing
    p.first_swing = Side::kLeft;
    g.setParams(p);
    g.start(&s);
    EXPECT_TRUE(kT, g.command().swing == Side::kLeft);
    g.start(nullptr);
    EXPECT_TRUE(kT, g.command().swing == Side::kLeft);
}

void testOuterEdgeAbort() {
    constexpr char kT[] = "G5 outer edge abort";
    DcmStepping g;
    g.start();
    DcmState s = centred();
    s.dcm_past_outer_edge_mm[0] = 20.0f;  // Falling outward over the left (stance) foot.
    g.update(s, 10);
    EXPECT_TRUE(kT, !g.running());
    EXPECT_TRUE(kT, g.command().stop_reason == StopReason::kOuterEdge);
}

void testStepLimit() {
    constexpr char kT[] = "G6 step limit";
    Params p;
    p.max_steps = 1;
    DcmStepping g(p);
    g.start();
    runFor(g, centred(), 90);
    EXPECT_TRUE(kT, !g.running());
    EXPECT_TRUE(kT, g.command().stop_reason == StopReason::kStepLimit);
    EXPECT_TRUE(kT, g.command().steps == 1);
}

void testOpenLoop() {
    constexpr char kT[] = "G7 open loop timing";
    Params p;
    p.mode = Mode::kOpenLoop;
    DcmStepping g(p);
    g.start();
    runFor(g, centred(), 80);
    EXPECT_TRUE(kT, g.command().phase == Phase::kSwing);
    runFor(g, centred(), 10);  // 90 ms
    EXPECT_TRUE(kT, g.command().phase == Phase::kDouble);
    runFor(g, centred(), 100);
    EXPECT_TRUE(kT, g.command().phase == Phase::kDouble);
    runFor(g, centred(), 10);  // period 200 ms
    EXPECT_TRUE(kT, g.command().phase == Phase::kSwing && g.command().swing == Side::kLeft);
    EXPECT_TRUE(kT, g.command().steps == 1);
}

void testSynchronizedLiftTrajectory() {
    constexpr char kT[] = "G11 synchronized pitch trajectory";
    Params p;
    p.mode = Mode::kOpenLoop;
    p.lift_mode = LiftMode::kVertical;
    p.lift_deg = 15.0f;
    p.lift_rise_ms = 100;
    p.lift_fall_ms = 100;
    p.open_hold_ms = 150;
    p.open_period_ms = 300;
    p.open_crossfade = true;
    DcmStepping g(p);
    const JointArray stand = standPose();
    g.start();

    // No joint jumps at phase entry. All three pitch joints share the same
    // quintic progress; at half time smoothstep5(0.5) is exactly 0.5.
    EXPECT_TRUE(kT, std::fabs(g.command().lift_scale) < 1e-6f);
    g.update(centred(), 50);
    const JointArray& half = g.command().position_rad;
    EXPECT_TRUE(kT, std::fabs(g.command().lift_scale - 0.5f) < 1e-5f);
    EXPECT_TRUE(kT, std::fabs(half[tilt::R_HIP_PITCH] -
                              (stand[tilt::R_HIP_PITCH] - rad(7.5f))) < 1e-5f);
    EXPECT_TRUE(kT, std::fabs(half[tilt::R_KNEE_PITCH] -
                              (stand[tilt::R_KNEE_PITCH] + rad(15.0f))) < 1e-5f);
    EXPECT_TRUE(kT, std::fabs(half[tilt::R_ANKLE_PITCH] -
                              (stand[tilt::R_ANKLE_PITCH] - rad(7.5f))) < 1e-5f);
    EXPECT_TRUE(kT, std::fabs(half[tilt::R_HIP_PITCH] + half[tilt::R_KNEE_PITCH] +
                              half[tilt::R_ANKLE_PITCH]) < 1e-5f);

    g.update(centred(), 50);
    EXPECT_TRUE(kT, std::fabs(g.command().lift_scale - 1.0f) < 1e-6f);
    g.update(centred(), 200);  // At 300 ms: R starts landing while L starts rising.
    EXPECT_TRUE(kT, g.command().phase == Phase::kSwing);
    EXPECT_TRUE(kT, g.command().swing == Side::kLeft && g.command().steps == 1);
    EXPECT_TRUE(kT, std::fabs(g.command().lift_scale_by_leg[0]) < 1e-6f);
    EXPECT_TRUE(kT, std::fabs(g.command().lift_scale_by_leg[1] - 1.0f) < 1e-6f);

    g.update(centred(), 50);  // Symmetric crossfade: both legs at half scale.
    EXPECT_TRUE(kT, std::fabs(g.command().lift_scale_by_leg[0] - 0.5f) < 1e-5f);
    EXPECT_TRUE(kT, std::fabs(g.command().lift_scale_by_leg[1] - 0.5f) < 1e-5f);
    EXPECT_TRUE(kT, std::fabs(g.command().position_rad[tilt::L_KNEE_PITCH] -
                              (stand[tilt::L_KNEE_PITCH] + rad(15.0f))) < 1e-5f);
    EXPECT_TRUE(kT, std::fabs(g.command().position_rad[tilt::R_KNEE_PITCH] -
                              (stand[tilt::R_KNEE_PITCH] + rad(15.0f))) < 1e-5f);

    g.update(centred(), 50);
    EXPECT_TRUE(kT, std::fabs(g.command().lift_scale_by_leg[0] - 1.0f) < 1e-6f);
    EXPECT_TRUE(kT, std::fabs(g.command().lift_scale_by_leg[1]) < 1e-6f);
}

void testStepLimitWaitsForLanding() {
    constexpr char kT[] = "G12 step limit waits for landing";
    Params p;
    p.mode = Mode::kOpenLoop;
    p.max_steps = 1;
    p.lift_rise_ms = 100;
    p.lift_fall_ms = 100;
    p.open_hold_ms = 100;
    DcmStepping g(p);
    g.start();
    runFor(g, centred(), 100);
    EXPECT_TRUE(kT, g.running() && g.command().phase == Phase::kDouble);
    EXPECT_TRUE(kT, g.command().steps == 1 && std::fabs(g.command().lift_scale - 1.0f) < 1e-6f);
    runFor(g, centred(), 50);
    EXPECT_TRUE(kT, g.running() && std::fabs(g.command().lift_scale - 0.5f) < 1e-5f);
    runFor(g, centred(), 50);
    EXPECT_TRUE(kT, !g.running() && g.command().stop_reason == StopReason::kStepLimit);
    EXPECT_TRUE(kT, std::fabs(g.command().lift_scale) < 1e-6f);
}

void testInsideDistance() {
    constexpr char kT[] = "G8 inside distance";
    DcmState s = centred();
    s.dcm_mm[1] = -40.0f;
    EXPECT_TRUE(kT, std::fabs(dcmInsideSwingFoot(s, Side::kRight) - 15.0f) < 1e-5f);
    s.dcm_mm[1] = 40.0f;
    EXPECT_TRUE(kT, std::fabs(dcmInsideSwingFoot(s, Side::kLeft) - 15.0f) < 1e-5f);
}

}  // namespace

// 8 joints (with ankle): stand pose and both lift modes keep the sole parallel.
void testLiftModesAndStand() {
    constexpr char kT[] = "G0 stand / lift modes (ankle)";
    const JointArray stand = standPose();
    for (int leg = 0; leg < 2; ++leg) {
        const int b = leg * tilt::NUM_LEG_JOINTS;
        EXPECT_TRUE(kT, std::fabs(stand[b + 1] - rad(-20.0f)) < 1e-6f);
        EXPECT_TRUE(kT, std::fabs(stand[b + 2] - rad(40.0f)) < 1e-6f);
        EXPECT_TRUE(kT, std::fabs(stand[b + 3] - rad(-20.0f)) < 1e-6f);
    }
    for (const LiftMode mode : {LiftMode::kFlash, LiftMode::kVertical}) {
        Params p;
        p.lift_mode = mode;
        DcmStepping g(p);
        g.start();  // First swing: right leg.
        const JointArray& q = g.command().position_rad;
        const float sum_r = q[tilt::R_HIP_PITCH] + q[tilt::R_KNEE_PITCH] + q[tilt::R_ANKLE_PITCH];
        const float sum_stand = stand[tilt::R_HIP_PITCH] + stand[tilt::R_KNEE_PITCH] +
                                stand[tilt::R_ANKLE_PITCH];
        EXPECT_TRUE(kT, std::fabs(sum_r - sum_stand) < 1e-5f);  // Sole parallel.
        EXPECT_TRUE(kT, std::fabs(q[tilt::R_HIP_PITCH] - (stand[tilt::R_HIP_PITCH] - rad(15.0f))) < 1e-5f);
        const float knee = mode == LiftMode::kVertical ? 30.0f : 15.0f;
        const float ankle = mode == LiftMode::kVertical ? -15.0f : 0.0f;
        EXPECT_TRUE(kT, std::fabs(q[tilt::R_KNEE_PITCH] - (stand[tilt::R_KNEE_PITCH] + rad(knee))) < 1e-5f);
        EXPECT_TRUE(kT, std::fabs(q[tilt::R_ANKLE_PITCH] - (stand[tilt::R_ANKLE_PITCH] + rad(ankle))) < 1e-5f);
        EXPECT_TRUE(kT, q[tilt::R_HIP_ROLL] == stand[tilt::R_HIP_ROLL]);
        for (int j = 0; j < tilt::NUM_LEG_JOINTS; ++j) EXPECT_TRUE(kT, q[j] == stand[j]);  // Left untouched.
    }
}

int main() {
    testLiftModesAndStand();
    testStartAndFirstSwing();
    testDcmLift();
    testDcmTouchdownAndTimeout();
    testDoubleTimeout();
    testOuterEdgeAbort();
    testStepLimit();
    testOpenLoop();
    testSynchronizedLiftTrajectory();
    testStepLimitWaitsForLanding();
    testInsideDistance();
    testDoubleFallbackLift();
    testAutoFirstSwing();
    if (s_failures == 0) {
        std::printf("All DCM stepping tests passed.\n");
        return 0;
    }
    std::printf("%d DCM stepping assertion(s) failed.\n", s_failures);
    return 1;
}
