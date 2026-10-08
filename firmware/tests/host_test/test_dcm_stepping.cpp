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
    EXPECT_TRUE(kT, std::fabs(c.position_rad[tilt::R_HIP_PITCH] - rad(-35.0f)) < 1e-5f);
    EXPECT_TRUE(kT, std::fabs(c.position_rad[tilt::R_KNEE_PITCH] - rad(55.0f)) < 1e-5f);
    EXPECT_TRUE(kT, std::fabs(c.position_rad[tilt::L_HIP_PITCH] - rad(-20.0f)) < 1e-5f);
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
    DcmStepping g;
    g.start();
    runFor(g, centred(), 90);
    runFor(g, centred(), 300);
    EXPECT_TRUE(kT, !g.running());
    EXPECT_TRUE(kT, g.command().stop_reason == StopReason::kDoubleTimeout);
    EXPECT_TRUE(kT, std::fabs(g.command().position_rad[tilt::R_KNEE_PITCH] - rad(40.0f)) < 1e-5f);
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

int main() {
    testStartAndFirstSwing();
    testDcmLift();
    testDcmTouchdownAndTimeout();
    testDoubleTimeout();
    testOuterEdgeAbort();
    testStepLimit();
    testOpenLoop();
    testInsideDistance();
    if (s_failures == 0) {
        std::printf("All DCM stepping tests passed.\n");
        return 0;
    }
    std::printf("%d DCM stepping assertion(s) failed.\n", s_failures);
    return 1;
}
