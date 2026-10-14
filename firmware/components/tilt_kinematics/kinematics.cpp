#include "tilt_kinematics.h"

#include <cmath>
#include <cstdint>

#include "tilt/kinematics/RobotModel.h"

namespace tilt {
namespace {

constexpr float PI_RAD = 3.14159265358979323846f;
constexpr float REACH_CLAMP_EPS_MM = 0.001f;

Mat3 identity() {
    return {{{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}}};
}

Mat3 multiply(const Mat3& a, const Mat3& b) {
    Mat3 out{};
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            out.m[r][c] = a.m[r][0] * b.m[0][c] + a.m[r][1] * b.m[1][c] +
                          a.m[r][2] * b.m[2][c];
        }
    }
    return out;
}

Mat3 from_array(const float m[3][3]) {
    Mat3 out{};
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) out.m[r][c] = m[r][c];
    }
    return out;
}

Vec3 rotate(const Mat3& R, const float v[3]) {
    return {
        R.m[0][0] * v[0] + R.m[0][1] * v[1] + R.m[0][2] * v[2],
        R.m[1][0] * v[0] + R.m[1][1] * v[1] + R.m[1][2] * v[2],
        R.m[2][0] * v[0] + R.m[2][1] * v[1] + R.m[2][2] * v[2],
    };
}

// Rodrigues rotation about a unit axis.
Mat3 axis_angle(const float axis[3], float angle) {
    const float x = axis[0];
    const float y = axis[1];
    const float z = axis[2];
    const float c = std::cos(angle);
    const float s = std::sin(angle);
    const float t = 1.0f - c;
    return {{
        {t * x * x + c, t * x * y - s * z, t * x * z + s * y},
        {t * x * y + s * z, t * y * y + c, t * y * z - s * x},
        {t * x * z - s * y, t * y * z + s * x, t * z * z + c},
    }};
}

// child = parent * Translate(origin) * rotation * Rot(axis, q)
Pose apply_joint(const Pose& parent, const model::JointSpec& joint, float q) {
    const Vec3 offset = rotate(parent.R, joint.origin_mm);
    Pose child{};
    child.p = {parent.p.x + offset.x, parent.p.y + offset.y,
               parent.p.z + offset.z};
    child.R = multiply(parent.R, from_array(joint.rotation));
    if (q != 0.0f) child.R = multiply(child.R, axis_angle(joint.axis, q));
    return child;
}

float wrap_pi(float angle) {
    return std::remainder(angle, 2.0f * PI_RAD);
}

// Foot rotation about the ankle axis relative to the URDF's fixed foot.
constexpr float kAnkleModelOffsetRad = ANKLE_FIXED_RAD - model::SOLE_PITCH_AT_ZERO_RAD;

// Ankle axis point in the calf frame (see the header): on the calf axis at
// CALF_LENGTH_MM, at the sole frame's lateral position.
Vec3 ankle_point_in_calf(int i) {
    return {CALF_LENGTH_MM, 0.0f, model::SOLE_FIXED[i].origin_mm[2]};
}

// Sagittal (x, z) vectors in the roll frame with every joint at 0:
// H -> knee, knee -> ankle axis, ankle axis -> sole point.
struct SagittalChain {
    float thigh[2];
    float shank[2];
    float foot[2];
};

LegFrames fk_leg_impl(int i, const float theta[NUM_LEG_JOINTS]);

const SagittalChain& sagittal_chain(int i) {
    static SagittalChain chains[2];
    static bool ready = false;
    if (!ready) {
        for (int leg = 0; leg < 2; ++leg) {
            const float zero[NUM_LEG_JOINTS] = {};
            const LegFrames f = fk_leg_impl(leg, zero);
            const Vec3 h = {model::HIP_POINT_MM[leg][0], model::HIP_POINT_MM[leg][1],
                            model::HIP_POINT_MM[leg][2]};
            const Vec3 k = f.calf.p;
            const Vec3 a = f.ankle.p;
            const Vec3 s = f.sole.p;
            chains[leg] = {{k.x - h.x, k.z - h.z}, {a.x - k.x, a.z - k.z}, {s.x - a.x, s.z - a.z}};
        }
        ready = true;
    }
    return chains[i];
}

LegFrames fk_leg_impl(int i, const float theta[NUM_LEG_JOINTS]) {
    const Pose base{identity(), {0.0f, 0.0f, 0.0f}};
    LegFrames frames{};
    frames.hip = apply_joint(base, model::LEG_JOINT[i][0], theta[0]);
    frames.thigh = apply_joint(frames.hip, model::LEG_JOINT[i][1], theta[1]);
    frames.calf = apply_joint(frames.thigh, model::LEG_JOINT[i][2], theta[2]);

    // Ankle: rotate the URDF foot about an axis parallel to the knee axis.
    const Vec3 a = ankle_point_in_calf(i);
    frames.ankle.p = transform_point(frames.calf, a);
    frames.ankle.R = multiply(frames.calf.R,
                              axis_angle(model::LEG_JOINT[i][2].axis,
                                         theta[3] + kAnkleModelOffsetRad));
    const float* o = model::SOLE_FIXED[i].origin_mm;
    frames.sole.p = transform_point(frames.ankle, {o[0] - a.x, o[1] - a.y, o[2] - a.z});
    frames.sole.R = multiply(frames.ankle.R, from_array(model::SOLE_FIXED[i].rotation));
    return frames;
}

}  // namespace

Pose base_pose() {
    return {identity(), {0.0f, 0.0f, 0.0f}};
}

Pose joint_child_pose(const Pose& parent, const model::JointSpec& joint,
                      float q) {
    return apply_joint(parent, joint, q);
}

Vec3 transform_point(const Pose& pose, const Vec3& local) {
    const float v[3] = {local.x, local.y, local.z};
    const Vec3 r = rotate(pose.R, v);
    return {pose.p.x + r.x, pose.p.y + r.y, pose.p.z + r.z};
}

LegFrames fk_leg(Leg leg, const float theta[NUM_LEG_JOINTS]) {
    return fk_leg_impl(static_cast<int>(leg), theta);
}

Vec3 fk_foot(Leg leg, const float theta[NUM_LEG_JOINTS]) {
    return fk_leg(leg, theta).sole.p;
}

float sole_pitch_rad(const float theta[NUM_LEG_JOINTS]) {
    return ANKLE_FIXED_RAD + theta[1] + theta[2] + theta[3];
}

IkResult ik_foot(Leg leg, const Vec3& sole_target, float sole_pitch) {
    const int i = static_cast<int>(leg);
    if (!std::isfinite(sole_target.x) || !std::isfinite(sole_target.y) ||
        !std::isfinite(sole_target.z) || !std::isfinite(sole_pitch)) {
        return {{0.0f, 0.0f, 0.0f, 0.0f}, false};
    }
    bool reachable = true;

    // Target relative to the hip point H.
    const float rx = sole_target.x - model::HIP_POINT_MM[i][0];
    const float ry = sole_target.y - model::HIP_POINT_MM[i][1];
    const float rz = sole_target.z - model::HIP_POINT_MM[i][2];

    // Frontal plane: in the roll frame the sole sits at (y', z') = (d, zp)
    // with a constant lateral offset d, so |(ry, rz)| = |(d, zp)|.
    const float d = model::SOLE_LATERAL_MM[i];
    float rho = std::sqrt(ry * ry + rz * rz);
    const float rho_min = std::fabs(d) + REACH_CLAMP_EPS_MM;
    if (rho < rho_min) {
        rho = rho_min;
        reachable = false;
    }
    const float zp = -std::sqrt(rho * rho - d * d);  // Foot below the hip.
    const float roll = wrap_pi(std::atan2(rz, ry) - std::atan2(zp, d));

    // Sagittal plane: pitch joints rotate (x, z) about +y (Ry rotates by -q):
    //   x' = x cos q + z sin q,  z' = -x sin q + z cos q.
    // The foot vector (ankle axis -> sole) is rotated by the total pitch
    // phi = hip + knee + ankle; subtract it to get the ankle-axis target, then
    // solve the two-link (thigh, shank) problem for it.
    const SagittalChain& chain = sagittal_chain(i);
    const float phi = sole_pitch - ANKLE_FIXED_RAD;
    const float cphi = std::cos(phi);
    const float sphi = std::sin(phi);
    const float fx = chain.foot[0] * cphi + chain.foot[1] * sphi;
    const float fz = -chain.foot[0] * sphi + chain.foot[1] * cphi;

    const float tx = chain.thigh[0];
    const float tz = chain.thigh[1];
    const float sx = chain.shank[0];
    const float sz = chain.shank[1];
    const float l1 = std::sqrt(tx * tx + tz * tz);
    const float l2 = std::sqrt(sx * sx + sz * sz);

    float px = rx - fx;
    float pz = zp - fz;
    float dist = std::sqrt(px * px + pz * pz);
    const float max_reach = l1 + l2 - REACH_CLAMP_EPS_MM;
    const float min_reach = std::fabs(l1 - l2) + REACH_CLAMP_EPS_MM;
    float clamped = dist;
    if (dist > max_reach) clamped = max_reach;
    if (dist < min_reach) clamped = min_reach;
    if (clamped != dist) {
        reachable = false;
        if (dist > 0.0f) {
            px *= clamped / dist;
            pz *= clamped / dist;
        } else {
            px = 0.0f;
            pz = -clamped;
        }
        dist = clamped;
    }

    float cosine = (dist * dist - l1 * l1 - l2 * l2) / (2.0f * l1 * l2);
    if (cosine > 1.0f) cosine = 1.0f;
    if (cosine < -1.0f) cosine = -1.0f;
    const float angle_thigh = std::atan2(tz, tx);
    const float angle_shank = std::atan2(sz, sx);
    const float knee = angle_shank - angle_thigh + std::acos(cosine);

    // w = thigh + Ry(knee) * shank ; Ry(hip) * w must point at (px, pz).
    const float ck = std::cos(knee);
    const float sk = std::sin(knee);
    const float wx = tx + (sx * ck + sz * sk);
    const float wz = tz + (-sx * sk + sz * ck);
    const float hip = wrap_pi(std::atan2(wz, wx) - std::atan2(pz, px));
    const float ankle = wrap_pi(phi - hip - knee);

    return {{roll, hip, knee, ankle}, reachable};
}

bool clamp_to_limits(Leg leg, float theta[NUM_LEG_JOINTS]) {
    if (theta == nullptr) {
        return false;
    }

    const std::uint8_t leg_index = static_cast<std::uint8_t>(leg);
    if (leg_index > static_cast<std::uint8_t>(Leg::RIGHT)) {
        return false;
    }

    bool unchanged = true;
    for (std::uint8_t joint = 0; joint < NUM_LEG_JOINTS; ++joint) {
        const JointLimit& limit = JOINT_LIMIT[leg_index][joint];
        if (!std::isfinite(theta[joint])) {
            if (0.0f < limit.minimum_rad) {
                theta[joint] = limit.minimum_rad;
            } else if (0.0f > limit.maximum_rad) {
                theta[joint] = limit.maximum_rad;
            } else {
                theta[joint] = 0.0f;
            }
            unchanged = false;
        } else if (theta[joint] < limit.minimum_rad) {
            theta[joint] = limit.minimum_rad;
            unchanged = false;
        } else if (theta[joint] > limit.maximum_rad) {
            theta[joint] = limit.maximum_rad;
            unchanged = false;
        }
    }
    return unchanged;
}

}  // namespace tilt
