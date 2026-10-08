"""Validate the firmware DCM estimator (C++) against MuJoCo ground truth.

Runs FLASH-style stepping in place in MuJoCo with the STS3215 torque model from
simulation/rocking_sim.py, feeds the C++ estimator the same inputs the ESP32
will see (commanded joints, noisy IMU attitude + gyro, contact-based support)
and compares its CoM / DCM with the simulator's true values.

Build the bridge first:
  cmake -S simulation/dcm -B simulation/dcm/build && cmake --build simulation/dcm/build
Run:
  python3 simulation/dcm/estimator_check.py [--lib PATH] [--csv out.csv]
"""
import argparse
import ctypes
import math
import os
import sys

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import rocking_sim as rs  # noqa: E402  (torque model, model path)

FW_ORDER = ["l_hip_roll", "l_hip_pitch", "l_knee_pitch",
            "r_hip_roll", "r_hip_pitch", "r_knee_pitch"]
STAND = np.deg2rad([0.0, -20.0, 40.0, 0.0, -20.0, 40.0])
G = 9806.65  # mm/s^2
SUPPORT_DOUBLE, SUPPORT_LEFT, SUPPORT_RIGHT = 0, 1, 2


def find_library(path):
    if path:
        return path
    for name in ("libtilt_dcm.dylib", "libtilt_dcm.so"):
        candidate = os.path.join(HERE, "build", name)
        if os.path.exists(candidate):
            return candidate
    sys.exit("bridge library not found; build simulation/dcm first or pass --lib")


class Estimator:
    def __init__(self, lib_path, joint_lag, velocity_tau):
        self.lib = ctypes.CDLL(lib_path)
        f6 = ctypes.c_float * 6
        self.lib.tilt_dcm_create.restype = ctypes.c_void_p
        self.lib.tilt_dcm_create.argtypes = [ctypes.c_float, ctypes.c_float]
        self.lib.tilt_dcm_reset.argtypes = [ctypes.c_void_p, f6]
        self.lib.tilt_dcm_update.argtypes = [
            ctypes.c_void_p, f6, ctypes.c_float, ctypes.c_float,
            ctypes.c_float * 3, ctypes.c_int, ctypes.c_int, ctypes.c_float,
            ctypes.c_float * 18]
        self.lib.tilt_dcm_destroy.argtypes = [ctypes.c_void_p]
        self.h = self.lib.tilt_dcm_create(joint_lag, velocity_tau)
        self.f6 = f6

    def reset(self, theta):
        self.lib.tilt_dcm_reset(self.h, self.f6(*theta))

    def update(self, theta, roll, pitch, gyro, support, dt):
        out = (ctypes.c_float * 18)()
        self.lib.tilt_dcm_update(self.h, self.f6(*theta), roll, pitch,
                                 (ctypes.c_float * 3)(*gyro), 1, support, dt, out)
        return np.array(out[:])

    def close(self):
        self.lib.tilt_dcm_destroy(self.h)


class World:
    def __init__(self):
        self.m = mujoco.MjModel.from_xml_path(str(rs.MODEL_PATH))
        self.d = mujoco.MjData(self.m)
        m = self.m
        self.qadr = np.array([m.jnt_qposadr[m.joint(n).id] for n in FW_ORDER])
        self.vadr = np.array([m.jnt_dofadr[m.joint(n).id] for n in FW_ORDER])
        m.actuator_gainprm[:6, :] = 0.0
        m.actuator_biasprm[:6, :] = 0.0
        self.torso = m.body("torso").id
        self.floor = m.geom("floor").id
        self.foot = [m.geom("l_foot_collision").id, m.geom("r_foot_collision").id]
        self.sole = [m.site("l_sole").id, m.site("r_sole").id]

    def reset(self):
        m, d = self.m, self.d
        mujoco.mj_resetData(m, d)
        d.qpos[3:7] = (1, 0, 0, 0)
        d.qpos[self.qadr] = STAND
        mujoco.mj_forward(m, d)
        d.qpos[2] += 0.001 - min(d.site_xpos[s, 2] for s in self.sole)
        mujoco.mj_forward(m, d)

    def step(self, target, kp=28.0, kd=0.65):
        q = self.d.qpos[self.qadr]
        qd = self.d.qvel[self.vadr]
        tau = rs.torque_speed_limit(kp * (target - q) - kd * qd, qd)
        self.d.qfrc_applied[:] = 0.0
        self.d.qfrc_applied[self.vadr] = tau
        self.d.ctrl[6:] = 0.0
        mujoco.mj_step(self.m, self.d)

    def contacts(self):
        touch = [False, False]
        for i in range(self.d.ncon):
            c = self.d.contact[i]
            for leg in (0, 1):
                if {c.geom1, c.geom2} == {self.foot[leg], self.floor}:
                    touch[leg] = True
        return touch

    def attitude(self):
        R = self.d.xmat[self.torso].reshape(3, 3)
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.asin(max(-1.0, min(1.0, -R[2, 0])))
        yaw = math.atan2(R[1, 0], R[0, 0])
        return roll, pitch, yaw

    def truth(self, support):
        """True CoM, velocity, DCM relative to the reference sole (mm, gravity aligned)."""
        m, d = self.m, self.d
        mujoco.mj_subtreeVel(m, d)
        sole = [d.site_xpos[s] * 1000.0 for s in self.sole]
        ref = {SUPPORT_LEFT: sole[0], SUPPORT_RIGHT: sole[1]}.get(
            support, 0.5 * (sole[0] + sole[1]))
        _, _, yaw = self.attitude()
        cz, sz = math.cos(-yaw), math.sin(-yaw)
        Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
        r = Rz @ (d.subtree_com[self.torso] * 1000.0 - ref)
        v = Rz @ (d.subtree_linvel[self.torso] * 1000.0)
        omega = math.sqrt(G / max(r[2], 50.0))
        return r, v, r[:2] + v[:2] / omega, omega


def flash_command(t, lift_deg=15.0, hold_ms=90.0, period_ms=200.0, events=12,
                  start=0.5):
    """FLASH: right leg first, swing leg folded for hold_ms at the start of each event."""
    target = STAND.copy()
    ms = (t - start) * 1000.0
    if ms < 0:
        return target
    k = int(ms // period_ms)
    if k >= events or ms - k * period_ms >= hold_ms:
        return target
    base = 3 if k % 2 == 0 else 0  # swing leg: right on even events
    target[base + 1] -= math.radians(lift_deg)
    target[base + 2] += math.radians(lift_deg)
    return target


def run(lib, readback, roll_noise_deg, gyro_noise, gyro_bias, imu_delay_steps,
        encoder_noise_deg=0.0, joint_lag=0.03, joint_velocity_tau=0.03, seed=1,
        csv=None, duration=3.4):
    """readback=True feeds measured joint angles (servo read-back, no lag model);
    readback=False feeds commanded angles through the first-order lag model."""
    rng = np.random.default_rng(seed)
    world = World()
    world.reset()
    est = Estimator(lib, 0.0 if readback else joint_lag, joint_velocity_tau)
    est.reset(STAND)
    dt = 0.01
    sub = int(round(dt / world.m.opt.timestep))
    t = 0.0
    history = []
    errors = {"dcm_y": [], "dcm_x": [], "com_y": [], "vel_y": [], "dcm_y_ssp": []}
    rows = []
    while t < duration:
        cmd = flash_command(t)
        for _ in range(sub):
            world.step(cmd)
        t += dt
        touch = world.contacts()
        support = (SUPPORT_DOUBLE if touch[0] and touch[1] else
                   SUPPORT_LEFT if touch[0] else
                   SUPPORT_RIGHT if touch[1] else SUPPORT_DOUBLE)
        roll, pitch, _ = world.attitude()
        gyro = world.d.qvel[3:6].copy()  # free joint angular velocity: body frame
        history.append((roll, pitch, gyro))
        r_meas, p_meas, g_meas = history[max(0, len(history) - 1 - imu_delay_steps)]
        r_meas += math.radians(rng.normal(0.0, roll_noise_deg))
        p_meas += math.radians(rng.normal(0.0, roll_noise_deg))
        g_meas = g_meas + rng.normal(0.0, gyro_noise, 3) + gyro_bias
        if readback:
            joints = world.d.qpos[world.qadr].copy()
            joints += np.radians(rng.normal(0.0, encoder_noise_deg, 6)) if encoder_noise_deg else 0.0
        else:
            joints = cmd
        out = est.update(joints, r_meas, p_meas, g_meas, support, dt)
        r, v, xi, omega = world.truth(support)
        if t > 0.6:
            errors["dcm_y"].append(out[9] - xi[1])
            errors["dcm_x"].append(out[8] - xi[0])
            errors["com_y"].append(out[1] - r[1])
            errors["vel_y"].append(out[4] - v[1])
            if support != SUPPORT_DOUBLE:
                errors["dcm_y_ssp"].append(out[9] - xi[1])
        rows.append([t, support, xi[0], xi[1], out[8], out[9], r[1], out[1],
                     v[1], out[4], math.degrees(roll)])
    est.close()
    stats = {k: (float(np.sqrt(np.mean(np.square(e)))), float(np.max(np.abs(e))))
             for k, e in errors.items() if e}
    if csv:
        np.savetxt(csv, np.array(rows), delimiter=",", fmt="%.4f",
                   header="t,support,true_dcm_x,true_dcm_y,est_dcm_x,est_dcm_y,"
                          "true_com_y,est_com_y,true_vel_y,est_vel_y,roll_deg")
    return stats, np.array(rows)


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--lib")
    p.add_argument("--csv")
    args = p.parse_args()
    lib = find_library(args.lib)
    # (name, readback, attitude noise deg, gyro noise rad/s, gyro bias rad/s,
    #  IMU delay [control steps], encoder noise deg)
    cases = [
        ("read-back joints, ideal IMU", True, 0.0, 0.0, 0.0, 0, 0.0),
        ("read-back joints, noisy IMU + 10 ms delay + enc 0.1deg", True, 0.3, 0.02, 0.01, 1, 0.1),
        ("commanded joints + lag model, ideal IMU", False, 0.0, 0.0, 0.0, 0, 0.0),
        ("commanded joints + lag model, noisy IMU + 10 ms delay", False, 0.3, 0.02, 0.01, 1, 0.0),
    ]
    print("FLASH stepping in place (15 deg, hold 90 ms, period 200 ms), errors in mm or mm/s, rms/max")
    print(f"{'case':55s} {'DCM y rms/max':>16s} {'DCM y SSP':>14s} {'DCM x':>14s} "
          f"{'CoM y':>14s} {'vel y mm/s':>16s}")
    for i, (name, readback, rn, gn, gb, delay, enc) in enumerate(cases):
        stats, _ = run(lib, readback, rn, gn, gb, delay, encoder_noise_deg=enc,
                       csv=args.csv if (args.csv and i == 1) else None)
        fmt = lambda k: f"{stats[k][0]:6.2f}/{stats[k][1]:6.2f}"
        print(f"{name:55s} {fmt('dcm_y'):>16s} {fmt('dcm_y_ssp'):>14s} "
              f"{fmt('dcm_x'):>14s} {fmt('com_y'):>14s} {fmt('vel_y'):>16s}")


if __name__ == "__main__":
    main()
