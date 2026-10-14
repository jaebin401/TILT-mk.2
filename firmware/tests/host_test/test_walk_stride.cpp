// Host tests for tilt_dcm_gait::WalkStride on the open-loop pattern.

#include <cmath>
#include <cstdio>

#include "tilt/dcm_gait/WalkStride.h"

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

constexpr float kDeg = tilt::DEG2RAD;

Params openLoop() {
    Params p;
    p.mode = Mode::kOpenLoop;
    p.lift_deg = 15.0f;
    p.open_hold_ms = 150;
    p.open_period_ms = 300;
    return p;
}

void testZeroStride() {
    constexpr char kT[] = "W1 zero stride changes nothing";
    DcmStepping g(openLoop());
    WalkParams zero;
    zero.stride_target_mm = 0.0f;
    WalkStride w(zero);
    g.start();
    DcmState s;
    for (int k = 0; k < 200; ++k) {
        const Command& c = g.update(s, 10);
        w.update(c, g.params(), 10);
        JointArray q = c.position_rad;
        const JointArray before = q;
        w.apply(q);
        for (int j = 0; j < tilt::NUM_JOINTS; ++j) {
            EXPECT_TRUE(kT, q[j] == before[j]);
        }
    }
}

void testWalking() {
    constexpr char kT[] = "W2 forward stride: ramp, swing ahead, stance sweeps back";
    DcmStepping g(openLoop());
    WalkParams wp;
    wp.stride_target_mm = 10.0f;
    WalkStride w(wp);
    g.start();
    DcmState s;
    float prev_x[2] = {0.0f, 0.0f};
    bool prev_swing[2] = {false, false};
    int t = 0;
    for (int k = 0; k < 600; ++k, t += 10) {
        const Command& c = g.update(s, 10);
        w.update(c, g.params(), 10);
        if (t == 1000) EXPECT_TRUE(kT, w.stride_mm() > 3.5f && w.stride_mm() < 4.5f);  // 4 mm/s.
        for (int i = 0; i < 2; ++i) {
            const bool swing = c.phase == Phase::kSwing && (c.swing == Side::kLeft ? 0 : 1) == i;
            if (t > 3000) {
                if (swing) EXPECT_TRUE(kT, std::fabs(w.foot_x_mm(i) - 5.0f) < 1e-3f);
                // Stance: never moves forward.
                if (!swing && !prev_swing[i]) EXPECT_TRUE(kT, w.foot_x_mm(i) <= prev_x[i] + 1e-4f);
                EXPECT_TRUE(kT, w.foot_x_mm(i) >= -5.0f - 1e-3f && w.foot_x_mm(i) <= 5.0f + 1e-3f);
            }
            prev_x[i] = w.foot_x_mm(i);
            prev_swing[i] = swing;
        }
    }
    EXPECT_TRUE(kT, std::fabs(w.stride_mm() - 10.0f) < 1e-3f);
    // Joint deltas: foot forward = hip pitch more negative (about -2.1 deg at +5 mm).
    JointArray q = standPose();
    const JointArray stand = q;
    // Force a known state: run until the right leg swings.
    for (int k = 0; k < 60; ++k) {
        const Command& c = g.update(s, 10);
        w.update(c, g.params(), 10);
        if (c.phase == Phase::kSwing && c.swing == Side::kRight) break;
    }
    w.apply(q);
    const float dhip = (q[tilt::R_HIP_PITCH] - stand[tilt::R_HIP_PITCH]) / kDeg;
    EXPECT_TRUE(kT, dhip < -1.8f && dhip > -2.4f);
    EXPECT_TRUE(kT, std::fabs(q[tilt::R_HIP_ROLL] - stand[tilt::R_HIP_ROLL]) < 0.05f * kDeg);
}

void testStopResets() {
    constexpr char kT[] = "W3 stopping the gait resets the stride";
    DcmStepping g(openLoop());
    WalkParams wp;
    wp.stride_target_mm = 10.0f;
    WalkStride w(wp);
    g.start();
    DcmState s;
    for (int k = 0; k < 400; ++k) w.update(g.update(s, 10), g.params(), 10);
    EXPECT_TRUE(kT, w.stride_mm() > 9.0f);
    g.stop();
    w.update(g.command(), g.params(), 10);
    EXPECT_TRUE(kT, w.stride_mm() == 0.0f && w.foot_x_mm(0) == 0.0f && w.foot_x_mm(1) == 0.0f);
}

}  // namespace

int main() {
    testZeroStride();
    testWalking();
    testStopResets();
    if (s_failures == 0) std::printf("test_walk_stride: all passed\n");
    return s_failures == 0 ? 0 : 1;
}
