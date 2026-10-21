#include <vector>
#include <cstdlib>
#include <algorithm>
// Host tests for tilt_dcm_gait::GaitFeedback driven by the open-loop pattern
// and a synthetic rocking roll signal.

#include <cmath>
#include <cstdio>

#include "tilt/dcm_gait/GaitFeedback.h"

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
constexpr float kPi = 3.14159265f;

Params openLoop() {
    Params p;
    p.mode = Mode::kOpenLoop;
    p.lift_deg = 15.0f;
    p.open_hold_ms = 150;
    p.open_period_ms = 300;
    return p;
}

// Synthetic rocking: 600 ms period. Right swings (first 300 ms of the cycle)
// rock to -amp_r, left swings to +amp_l, plus a constant offset.
struct Body {
    float amp_r = 7.0f;
    float amp_l = 7.0f;
    float roll_offset = 0.0f;
    float pitch = 1.0f;  // = default pitch target.
    int shift_ms = 0;  // Body phase lag relative to the gait timer.
    bool period2 = false;  // Right-side amplitude alternates 4 / 9 deg per cycle.
    float ampR(int t_ms) const {
        if (!period2) return amp_r;
        return ((t_ms - shift_ms) / 600) % 2 == 0 ? 4.0f : 9.0f;
    }
    float roll(int t_ms) const {
        const float ph = static_cast<float>(((t_ms - shift_ms) % 600 + 600) % 600) / 600.0f;
        const float s = std::sin(2.0f * kPi * ph);
        return roll_offset + (s > 0.0f ? -ampR(t_ms) * s : -amp_l * s);
    }
    float rate(int t_ms) const {  // deg/s
        const float ph = static_cast<float>(((t_ms - shift_ms) % 600 + 600) % 600) / 600.0f;
        const float s = std::sin(2.0f * kPi * ph);
        const float c = std::cos(2.0f * kPi * ph) * 2.0f * kPi / 0.6f;
        return s > 0.0f ? -ampR(t_ms) * c : -amp_l * c;
    }
};

struct Run {
    DcmStepping gait{openLoop()};
    GaitFeedback fb;
    int t = 0;
    void start() {
        gait.start();
        fb.beginRun();
        fb.update(gait.command(), true, 0.0f, 0.0f, 0.0f, 0.0f, 0.01f);
    }
    std::vector<int> lifts;  // Times of swing starts.
    void step(const Body& b, int ms) {
        DcmState s;
        for (int k = 0; k < ms / 10; ++k) {
            t += 10;
            gait.setLiftHint(fb.liftHint(gait.command()));
            const Command& c = gait.update(s, 10);
            if (c.phase_changed && c.phase == Phase::kSwing) lifts.push_back(t);
            fb.update(c, true, b.roll(t) * kDeg, b.pitch * kDeg, b.rate(t) * kDeg, 0.0f, 0.01f);
        }
    }
};

void testSymmetricNoChange() {
    constexpr char kT[] = "F1 symmetric rocking learns nothing";
    Run r;
    r.start();
    r.step(Body{}, 6000);
    const FeedbackState& s = r.fb.state();
    EXPECT_TRUE(kT, s.cycles >= 8);
    EXPECT_TRUE(kT, std::fabs(s.roll_trim_deg) < 0.2f);
    EXPECT_TRUE(kT, std::fabs(s.pitch_trim_deg) < 0.05f);
    EXPECT_TRUE(kT, std::fabs(s.lift_delta_deg[0]) < 0.3f && std::fabs(s.lift_delta_deg[1]) < 0.3f);
}

void testRollTrimSign() {
    constexpr char kT[] = "F2 mean roll < 0 (leaning left) -> -hip roll trim";
    Run r;
    r.start();
    Body b;
    b.roll_offset = -1.5f;
    r.step(b, 3000);
    const float trim = r.fb.state().roll_trim_deg;
    EXPECT_TRUE(kT, trim < -0.5f);
    r.step(b, 30000);
    EXPECT_TRUE(kT, std::fabs(r.fb.state().roll_trim_deg + 4.0f) < 1e-4f);  // Clamped.
    EXPECT_TRUE(kT, r.fb.state().roll_trim_saturated);
    // The applied offset appears on both hip roll joints with the same sign.
    JointArray q = standPose();
    r.fb.apply(q, r.gait.command(), FeedbackContext::kStand, 15.0f);
    EXPECT_TRUE(kT, q[tilt::L_HIP_ROLL] < -3.0f * kDeg && q[tilt::R_HIP_ROLL] < -3.0f * kDeg);
    // A large mean error (> 5 deg) is not learned from.
    Run far;
    far.start();
    Body tilted;
    tilted.roll_offset = -6.0f;
    tilted.amp_r = tilted.amp_l = 3.0f;
    far.step(tilted, 3000);
    EXPECT_TRUE(kT, far.fb.state().roll_trim_deg == 0.0f);
}

void testPitchTrimSign() {
    constexpr char kT[] = "F3 pitch above target -> +hip pitch trim";
    FeedbackParams p;
    p.pitch_trim_enabled = true;
    Run r;
    r.fb.setParams(p);
    r.start();
    Body b;
    b.pitch = 4.5f;
    r.step(b, 3000);
    EXPECT_TRUE(kT, r.fb.state().pitch_trim_deg > 0.5f);
    Run r2;
    r2.fb.setParams(p);
    r2.start();
    b.pitch = 0.5f;
    r2.step(b, 3000);
    EXPECT_TRUE(kT, r2.fb.state().pitch_trim_deg < -0.5f);
}

void testAmplitude() {
    constexpr char kT[] = "F4 weak left rocking -> more lift on the left swing";
    Run r;
    r.start();
    Body b;
    b.amp_l = 4.0f;
    r.step(b, 6000);
    const FeedbackState& s = r.fb.state();
    // Excursions are measured from the cycle mean ((4 - 7) / pi = -0.95 deg).
    EXPECT_TRUE(kT, s.lift_delta_deg[0] > s.lift_delta_deg[1] + 1.0f);  // Left lifts more.
    EXPECT_TRUE(kT, s.lift_delta_deg[0] > 1.0f && s.lift_delta_deg[0] <= 3.0f + 1e-4f);
    EXPECT_TRUE(kT, std::fabs(s.excursion_deg[0] - 4.95f) < 0.3f);
    EXPECT_TRUE(kT, std::fabs(s.excursion_deg[1] - 6.05f) < 0.3f);
}

void testRampAndApply() {
    constexpr char kT[] = "F5 start ramp and lift application";
    FeedbackParams p;
    p.ramp_steps = 3;  // Off by default; test the mechanism.
    Run r;
    r.fb.setParams(p);
    r.start();
    JointArray q = r.gait.command().position_rad;
    r.fb.apply(q, r.gait.command(), FeedbackContext::kStepping, 15.0f);
    const JointArray stand = standPose();
    // First swing (right): 15 * 1/4 = 3.75 deg fold.
    EXPECT_TRUE(kT, std::fabs(q[tilt::R_HIP_PITCH] - (stand[tilt::R_HIP_PITCH] - 3.75f * kDeg)) < 1e-4f);
    EXPECT_TRUE(kT, std::fabs(q[tilt::R_KNEE_PITCH] - (stand[tilt::R_KNEE_PITCH] + 3.75f * kDeg)) < 1e-4f);
    EXPECT_TRUE(kT, std::fabs(q[tilt::L_HIP_PITCH] - stand[tilt::L_HIP_PITCH]) < 1e-6f);
    r.step(Body{}, 1200);  // Past the ramp (swing index 4 at t=1200).
    const Command& c = r.gait.command();
    EXPECT_TRUE(kT, c.phase == Phase::kSwing && c.steps >= 3);
    const float fade = std::min(1.0f, static_cast<float>(r.fb.state().cycles) / 8.0f);
    EXPECT_TRUE(kT, std::fabs(r.fb.extraLiftDeg(c, 15.0f) -
                              fade * r.fb.state().lift_delta_deg[c.swing == Side::kLeft ? 0 : 1]) < 1e-6f);
}

void testFeedbackFollowsLandingTrajectory() {
    constexpr char kT[] = "F12 lift feedback stays continuous through landing";
    Params gp = openLoop();
    gp.lift_rise_ms = 100;
    gp.lift_fall_ms = 100;
    gp.max_steps = 1;
    FeedbackParams fp;
    fp.ramp_steps = 3;
    DcmStepping gait(gp);
    GaitFeedback fb(fp);
    DcmState state;
    const JointArray stand = standPose();
    gait.start();
    gait.update(state, 100);  // Full commanded lift; ramp reduces it to 1/4.
    JointArray q = gait.command().position_rad;
    fb.apply(q, gait.command(), FeedbackContext::kStepping, gp.lift_deg);
    EXPECT_TRUE(kT, std::fabs(q[tilt::R_KNEE_PITCH] -
                              (stand[tilt::R_KNEE_PITCH] + 3.75f * kDeg)) < 1e-4f);

    gait.update(state, 50);  // Enter DOUBLE at the same full effective lift.
    q = gait.command().position_rad;
    fb.apply(q, gait.command(), FeedbackContext::kStepping, gp.lift_deg);
    EXPECT_TRUE(kT, gait.command().phase == Phase::kDouble);
    EXPECT_TRUE(kT, std::fabs(q[tilt::R_KNEE_PITCH] -
                              (stand[tilt::R_KNEE_PITCH] + 3.75f * kDeg)) < 1e-4f);

    gait.update(state, 50);  // Halfway down: the complete effective fold halves.
    q = gait.command().position_rad;
    fb.apply(q, gait.command(), FeedbackContext::kStepping, gp.lift_deg);
    EXPECT_TRUE(kT, std::fabs(q[tilt::R_KNEE_PITCH] -
                              (stand[tilt::R_KNEE_PITCH] + 1.875f * kDeg)) < 1e-4f);
}

void testFeedbackFollowsCrossfade() {
    constexpr char kT[] = "F13 lift feedback follows both crossfade legs";
    Params gp = openLoop();
    gp.lift_rise_ms = 100;
    gp.lift_fall_ms = 100;
    gp.open_crossfade = true;
    FeedbackParams fp;
    fp.ramp_steps = 3;
    DcmStepping gait(gp);
    GaitFeedback fb(fp);
    DcmState state;
    const JointArray stand = standPose();
    gait.start();
    gait.update(state, 300);  // L starts rising; first R lift starts descending.
    EXPECT_TRUE(kT, gait.command().swing == Side::kLeft && gait.command().steps == 1);
    gait.update(state, 50);   // Both trajectory scales are 0.5.
    JointArray q = gait.command().position_rad;
    fb.apply(q, gait.command(), FeedbackContext::kStepping, gp.lift_deg);
    // R is swing index 0: 15 * 1/4 * 0.5 = 1.875 deg.
    // L is swing index 1: 15 * 2/4 * 0.5 = 3.75 deg.
    EXPECT_TRUE(kT, std::fabs(q[tilt::R_KNEE_PITCH] -
                              (stand[tilt::R_KNEE_PITCH] + 1.875f * kDeg)) < 1e-4f);
    EXPECT_TRUE(kT, std::fabs(q[tilt::L_KNEE_PITCH] -
                              (stand[tilt::L_KNEE_PITCH] + 3.75f * kDeg)) < 1e-4f);
}

void testWarmStartFade() {
    constexpr char kT[] = "F8 learned lift is faded in on a new run";
    FeedbackParams p;
    p.ramp_steps = 0;
    Run r;
    r.fb.setParams(p);
    r.start();
    Body weak;
    weak.amp_l = weak.amp_r = 3.0f;  // Drives both deltas to +3.
    r.step(weak, 12000);
    EXPECT_TRUE(kT, r.fb.state().lift_delta_deg[0] > 2.9f);
    r.gait.stop();
    r.fb.update(r.gait.command(), false, 0.0f, 0.0f, 0.0f, 0.0f, 0.01f);
    r.start();  // Second run keeps the learned +3 but starts without it.
    EXPECT_TRUE(kT, std::fabs(r.fb.extraLiftDeg(r.gait.command(), 15.0f)) < 1e-6f);
    r.step(weak, 2400);  // 4 cycles: half way.
    const Command& c = r.gait.command();
    const float e = r.fb.extraLiftDeg(c, 15.0f);
    EXPECT_TRUE(kT, c.phase != Phase::kSwing || (e > 0.5f && e < 2.9f));
}

void testPhaseSync() {
    constexpr char kT[] = "F9 phase sync keeps a locked rhythm and follows a shifted body";
    FeedbackParams p;
    p.phase_sync_enabled = true;
    Run r;
    r.fb.setParams(p);
    r.start();
    Body b;
    r.step(b, 15000);
    EXPECT_TRUE(kT, r.fb.state().phase_learned[0] && r.fb.state().phase_learned[1]);
    // Locked body: lifts stay on the 300 ms grid.
    const std::size_t n0 = r.lifts.size();
    r.step(b, 3000);
    for (std::size_t i = n0 + 1; i < r.lifts.size(); ++i) {
        EXPECT_TRUE(kT, std::abs(r.lifts[i] - r.lifts[i - 1] - 300) <= 10);
    }
    // Body lags by 50 ms: the lifts follow it (within the 60 ms window).
    for (int shift : {50, -40}) {
        b.shift_ms = shift;
        r.step(b, 3000);
        const std::size_t n1 = r.lifts.size();
        r.step(b, 1800);
        int worst = 0;
        for (std::size_t i = n1; i < r.lifts.size(); ++i) {
            const int off = ((r.lifts[i] - shift) % 300 + 300) % 300;
            const int err = std::min(off, 300 - off);  // Distance to the shifted grid.
            worst = std::max(worst, err);
        }
        if (worst > 15) std::printf("  shift %d: worst grid error %d ms\n", shift, worst);
        EXPECT_TRUE(kT, worst <= 15);
    }
    // Without the layer the hint is kNone.
    FeedbackParams off;
    GaitFeedback none(off);
    DcmStepping g(openLoop());
    g.start();
    EXPECT_TRUE(kT, none.liftHint(g.command()) == LiftHint::kNone);
}

void testPhaseLearnGate() {
    constexpr char kT[] = "F10 no phase learning from a period-2 rhythm";
    FeedbackParams p;
    p.phase_sync_enabled = true;
    p.amp_enabled = false;  // Keep the synthetic pattern as is.
    Run r;
    r.fb.setParams(p);
    r.start();
    Body b;
    b.period2 = true;
    r.step(b, 15000);
    EXPECT_TRUE(kT, !r.fb.state().phase_learned[0] && !r.fb.state().phase_learned[1]);
    EXPECT_TRUE(kT, !r.fb.state().rhythm_stable);
    b.period2 = false;  // Settles -> learns.
    r.step(b, 12000);
    EXPECT_TRUE(kT, r.fb.state().phase_learned[0] && r.fb.state().phase_learned[1]);
}

void testRevertToLastGood() {
    constexpr char kT[] = "F11 a fall reverts to the last settled values";
    FeedbackParams p;
    Run r;
    r.fb.setParams(p);
    r.start();
    Body b;
    b.roll_offset = -0.5f;  // Small offset: stays 'stable', trim learns a bit.
    r.step(b, 9000);
    const float good_trim = r.fb.state().roll_trim_deg;
    const float good_lift = r.fb.state().lift_delta_deg[0];
    EXPECT_TRUE(kT, r.fb.state().rhythm_stable);
    Body bad;
    bad.amp_l = 2.0f;  // Rhythm breaks: left lift is pushed up.
    bad.roll_offset = 3.0f;
    r.step(bad, 3000);
    EXPECT_TRUE(kT, r.fb.state().lift_delta_deg[0] > good_lift + 0.5f);
    r.fb.revertToLastGood();
    EXPECT_TRUE(kT, std::fabs(r.fb.state().lift_delta_deg[0] - good_lift) < 0.5f);
    EXPECT_TRUE(kT, std::fabs(r.fb.state().roll_trim_deg - good_trim) < 0.5f);
}

void testDisabledAndFreeze() {
    constexpr char kT[] = "F6 layers off / freeze on big tilt";
    FeedbackParams p;
    p.trim_enabled = false;
    p.amp_enabled = false;
    p.ramp_steps = 0;
    Run r;
    r.fb.setParams(p);
    r.start();
    Body b;
    b.roll_offset = -2.0f;
    b.amp_l = 3.0f;
    r.step(b, 6000);
    JointArray q = r.gait.command().position_rad;
    const JointArray before = q;
    r.fb.apply(q, r.gait.command(), FeedbackContext::kStepping, 15.0f);
    for (int j = 0; j < tilt::NUM_JOINTS; ++j) {
        EXPECT_TRUE(kT, std::fabs(q[j] - before[j]) < 1e-6f);
    }

    Run big;
    big.start();
    Body wild;
    wild.amp_r = 15.0f;  // Beyond freeze_tilt_deg: nothing learned.
    wild.roll_offset = -1.0f;
    big.step(wild, 6000);
    EXPECT_TRUE(kT, std::fabs(big.fb.state().roll_trim_deg) < 1e-6f);
    EXPECT_TRUE(kT, std::fabs(big.fb.state().lift_delta_deg[1]) < 1e-6f);
}

void testDamping() {
    constexpr char kT[] = "F7 damping signs and limit";
    FeedbackParams p;
    p.damp_enabled = true;
    p.damp_roll_gain = 0.02f;
    p.damp_pitch_gain = 0.03f;
    GaitFeedback fb(p);
    DcmStepping g(openLoop());
    g.start();
    for (int k = 0; k < 20; ++k) fb.update(g.command(), true, 0.0f, 0.0f, 50.0f * kDeg, 40.0f * kDeg, 0.01f);
    EXPECT_TRUE(kT, std::fabs(fb.state().damp_roll_deg - 1.0f) < 0.05f);
    EXPECT_TRUE(kT, std::fabs(fb.state().damp_pitch_deg - 1.2f) < 0.05f);
    for (int k = 0; k < 20; ++k) fb.update(g.command(), true, 0.0f, 0.0f, 0.0f, 400.0f * kDeg, 0.01f);
    EXPECT_TRUE(kT, std::fabs(fb.state().damp_pitch_deg - 3.0f) < 1e-4f);
    fb.update(g.command(), false, 0.0f, 0.0f, 0.0f, 400.0f * kDeg, 0.01f);
    EXPECT_TRUE(kT, fb.state().damp_pitch_deg == 0.0f);
}

}  // namespace

int main() {
    testSymmetricNoChange();
    testRollTrimSign();
    testPitchTrimSign();
    testAmplitude();
    testRampAndApply();
    testFeedbackFollowsLandingTrajectory();
    testFeedbackFollowsCrossfade();
    testDisabledAndFreeze();
    testDamping();
    testWarmStartFade();
    testPhaseSync();
    testPhaseLearnGate();
    testRevertToLastGood();
    if (s_failures == 0) std::printf("test_gait_feedback: all passed\n");
    return s_failures == 0 ? 0 : 1;
}
