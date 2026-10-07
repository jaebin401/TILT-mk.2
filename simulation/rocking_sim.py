#!/usr/bin/env python3
"""Dynamic rocking/brief-SSP test for TILT mk.2.

The leg servos are torque controlled in Python so the STS3215-C001 7.4 V
torque-speed envelope can be enforced.  This is deliberately a best-case
battery model; voltage sag, bus latency, backlash and thermal derating are
reported as validation gaps rather than silently guessed.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import time
from dataclasses import asdict, dataclass
from pathlib import Path

import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = ROOT / "model" / "mjcf" / "scene.xml"
RESULTS_DIR = ROOT / "simulation" / "results"

LEG_JOINTS = (
    "r_hip_roll",
    "r_hip_pitch",
    "r_knee_pitch",
    "l_hip_roll",
    "l_hip_pitch",
    "l_knee_pitch",
)
STAND_DEG = np.array([0.0, -20.0, 40.0, 0.0, -20.0, 40.0])

# FEETECH STS3215-C001, 7.4 V, vendor specification A/0.
STALL_TORQUE_NM = 19.5 * 0.0980665
RATED_TORQUE_NM = 5.0 * 0.0980665
NO_LOAD_SPEED_RAD_S = 52.0 * 2.0 * math.pi / 60.0
KT_NM_PER_A = 8.0 * 0.0980665
STALL_CURRENT_A = 2.5
PROTECTION_CURRENT_A = 2.0
ENCODER_STEP_RAD = 2.0 * math.pi / 4096.0


@dataclass(frozen=True)
class RockingConfig:
    support: str = "left"
    roll_deg: float = 11.0
    roll_time: float = 0.15
    lift_deg: float = 10.0
    lift_time: float = 0.20
    lift_offset: float = -0.05
    hold_time: float = 0.08
    return_time: float = 0.30

    @property
    def duration(self) -> float:
        lift_end = self.roll_time + self.lift_offset + 2.0 * self.lift_time + self.hold_time
        return max(self.roll_time, lift_end) + self.return_time


@dataclass
class Metrics:
    support: str
    success: bool
    ssp_duration_s: float
    max_swing_clearance_mm: float
    max_stance_slip_mm: float
    peak_torque_nm: float
    peak_torque_ratio: float
    peak_speed_rad_s: float
    peak_current_a: float
    peak_total_leg_current_a: float
    longest_over_rated_s: float
    longest_over_2a_s: float
    min_capture_margin_mm: float | None
    max_torso_roll_deg: float
    max_torso_pitch_deg: float
    nonfoot_floor_contact: bool
    fell: bool


def smoothstep5(x: float) -> float:
    x = min(1.0, max(0.0, x))
    return x**3 * (10.0 + x * (-15.0 + 6.0 * x))


def pulse_target(local_t: float, cfg: RockingConfig) -> np.ndarray:
    """Return six leg joint targets for one support-foot event."""
    target = np.deg2rad(STAND_DEG.copy())
    lift_start = cfg.roll_time + cfg.lift_offset
    lift_top = lift_start + cfg.lift_time
    lift_down = lift_top + cfg.hold_time
    lift_end = lift_down + cfg.lift_time
    roll_return = max(cfg.roll_time, lift_end)

    if local_t < 0.0:
        roll_scale = lift_scale = 0.0
    else:
        if local_t < cfg.roll_time:
            roll_scale = smoothstep5(local_t / cfg.roll_time)
        elif local_t < roll_return:
            roll_scale = 1.0
        elif local_t < roll_return + cfg.return_time:
            roll_scale = 1.0 - smoothstep5((local_t - roll_return) / cfg.return_time)
        else:
            roll_scale = 0.0

        if local_t < lift_start:
            lift_scale = 0.0
        elif local_t < lift_top:
            lift_scale = smoothstep5((local_t - lift_start) / cfg.lift_time)
        elif local_t < lift_down:
            lift_scale = 1.0
        elif local_t < lift_end:
            lift_scale = 1.0 - smoothstep5((local_t - lift_down) / cfg.lift_time)
        else:
            lift_scale = 0.0

    sign = 1.0 if cfg.support == "left" else -1.0
    target[[0, 3]] = math.radians(cfg.roll_deg) * sign * roll_scale
    swing_hip, swing_knee = (1, 2) if cfg.support == "left" else (4, 5)
    lift = math.radians(cfg.lift_deg) * lift_scale
    target[swing_hip] -= lift
    target[swing_knee] += lift
    return target


def torque_speed_limit(torque: np.ndarray, speed: np.ndarray) -> np.ndarray:
    """Clamp torque to a linear ideal-DC-motor voltage/current envelope.

    Motoring torque falls to zero at the 7.4 V no-load speed.  Opposing
    (braking) torque remains available up to the 2.5 A stall-current bound.
    """
    upper = STALL_TORQUE_NM * (1.0 - speed / NO_LOAD_SPEED_RAD_S)
    lower = -STALL_TORQUE_NM * (1.0 + speed / NO_LOAD_SPEED_RAD_S)
    upper = np.clip(upper, -STALL_TORQUE_NM, STALL_TORQUE_NM)
    lower = np.clip(lower, -STALL_TORQUE_NM, STALL_TORQUE_NM)
    return np.minimum(np.maximum(torque, lower), upper)


def quat_to_roll_pitch_deg(quat: np.ndarray) -> tuple[float, float]:
    matrix = np.empty(9)
    mujoco.mju_quat2Mat(matrix, quat)
    r = matrix.reshape(3, 3)
    roll = math.atan2(r[2, 1], r[2, 2])
    pitch = math.asin(float(np.clip(-r[2, 0], -1.0, 1.0)))
    return math.degrees(roll), math.degrees(pitch)


class Simulator:
    def __init__(self, control_hz: float = 100.0, kp: float = 28.0, kd: float = 0.65):
        self.model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
        self.data = mujoco.MjData(self.model)
        self.control_period = 1.0 / control_hz
        self.kp = kp
        self.kd = kd

        self.joint_ids = np.array([self.model.joint(name).id for name in LEG_JOINTS])
        self.qpos_ids = self.model.jnt_qposadr[self.joint_ids]
        self.dof_ids = self.model.jnt_dofadr[self.joint_ids]
        self.floor_id = self.model.geom("floor").id
        self.foot_ids = {
            "left": self.model.geom("l_foot_collision").id,
            "right": self.model.geom("r_foot_collision").id,
        }
        self.site_ids = {
            "left": self.model.site("l_sole").id,
            "right": self.model.site("r_sole").id,
        }
        self.torso_id = self.model.body("torso").id

        # Leg position actuators are replaced by the torque controller below.
        # Head/arm position actuators remain active at zero to hold their pose.
        self.model.actuator_gainprm[:6, :] = 0.0
        self.model.actuator_biasprm[:6, :] = 0.0

    def reset(self) -> None:
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[3:7] = (1.0, 0.0, 0.0, 0.0)
        self.data.qpos[self.qpos_ids] = np.deg2rad(STAND_DEG)
        mujoco.mj_forward(self.model, self.data)
        lowest_sole = min(self.data.site_xpos[sid, 2] for sid in self.site_ids.values())
        self.data.qpos[2] += 0.001 - lowest_sole
        mujoco.mj_forward(self.model, self.data)

    def foot_forces(self) -> tuple[dict[str, float], bool]:
        forces = {"left": 0.0, "right": 0.0}
        nonfoot_floor = False
        wrench = np.zeros(6)
        for i in range(self.data.ncon):
            contact = self.data.contact[i]
            pair = {contact.geom1, contact.geom2}
            if self.floor_id not in pair:
                continue
            other = contact.geom2 if contact.geom1 == self.floor_id else contact.geom1
            side = next((key for key, gid in self.foot_ids.items() if gid == other), None)
            if side is None:
                nonfoot_floor = True
                continue
            mujoco.mj_contactForce(self.model, self.data, i, wrench)
            forces[side] += max(0.0, wrench[0])
        return forces, nonfoot_floor

    def run(
        self,
        configs: list[RockingConfig],
        settle_s: float = 1.0,
        recovery_s: float = 0.45,
        viewer: bool = False,
        record: bool = True,
    ) -> tuple[list[Metrics], dict[str, np.ndarray]]:
        self.reset()
        dt = self.model.opt.timestep
        gap = 0.25
        starts: list[float] = []
        cursor = settle_s
        for cfg in configs:
            starts.append(cursor)
            cursor += cfg.duration + gap
        total_time = cursor + recovery_s

        active_viewer = None
        if viewer:
            from mujoco import viewer as mj_viewer

            active_viewer = mj_viewer.launch_passive(self.model, self.data)

        q_target = np.deg2rad(STAND_DEG)
        next_control = 0.0
        logs: dict[str, list] = {
            key: []
            for key in (
                "time",
                "q_target",
                "q_actual",
                "torque",
                "speed",
                "left_force",
                "right_force",
                "left_z",
                "right_z",
                "com_y",
                "com_z",
                "roll_deg",
                "pitch_deg",
            )
        }
        event_state = [
            {
                "ssp": 0.0,
                "ssp_run": 0.0,
                "clearance": 0.0,
                "slip": 0.0,
                "slip_origin": None,
                "capture_margin": math.inf,
                "nonfoot": False,
                "fell": False,
                "peak_torque": 0.0,
                "peak_speed": 0.0,
                "peak_current": 0.0,
                "peak_total_current": 0.0,
                "over_rated": np.zeros(6),
                "over_rated_max": 0.0,
                "over_2a": np.zeros(6),
                "over_2a_max": 0.0,
                "roll": 0.0,
                "pitch": 0.0,
                "last_com_y": None,
            }
            for _ in configs
        ]

        wall_start = time.perf_counter()
        while self.data.time < total_time:
            if self.data.time + 1e-12 >= next_control:
                q_target = np.deg2rad(STAND_DEG)
                for start, cfg in zip(starts, configs):
                    local_t = self.data.time - start
                    if 0.0 <= local_t <= cfg.duration:
                        q_target = pulse_target(local_t, cfg)
                        break
                q_target = np.round(q_target / ENCODER_STEP_RAD) * ENCODER_STEP_RAD
                next_control += self.control_period

            q = self.data.qpos[self.qpos_ids].copy()
            qd = self.data.qvel[self.dof_ids].copy()
            desired_torque = self.kp * (q_target - q) - self.kd * qd
            torque = torque_speed_limit(desired_torque, qd)
            self.data.qfrc_applied[:] = 0.0
            self.data.qfrc_applied[self.dof_ids] = torque
            self.data.ctrl[6:] = 0.0
            mujoco.mj_step(self.model, self.data)

            foot_force, nonfoot = self.foot_forces()
            sole_z = {side: self.data.site_xpos[sid, 2] for side, sid in self.site_ids.items()}
            com = self.data.subtree_com[self.torso_id].copy()
            roll_deg, pitch_deg = quat_to_roll_pitch_deg(self.data.qpos[3:7])

            if record:
                logs["time"].append(self.data.time)
                logs["q_target"].append(q_target.copy())
                logs["q_actual"].append(q.copy())
                logs["torque"].append(torque.copy())
                logs["speed"].append(qd.copy())
                logs["left_force"].append(foot_force["left"])
                logs["right_force"].append(foot_force["right"])
                logs["left_z"].append(sole_z["left"])
                logs["right_z"].append(sole_z["right"])
                logs["com_y"].append(com[1])
                logs["com_z"].append(com[2])
                logs["roll_deg"].append(roll_deg)
                logs["pitch_deg"].append(pitch_deg)

            for i, (start, cfg) in enumerate(zip(starts, configs)):
                if not (start <= self.data.time <= start + cfg.duration + recovery_s):
                    continue
                state = event_state[i]
                stance, swing = cfg.support, "right" if cfg.support == "left" else "left"
                mg = self.model.body_subtreemass[self.torso_id] * abs(self.model.opt.gravity[2])
                is_ssp = (
                    foot_force[swing] < 0.03 * mg
                    and foot_force[stance] > 0.30 * mg
                    and sole_z[swing] > 0.001
                )
                if is_ssp:
                    state["ssp_run"] += dt
                    state["ssp"] = max(state["ssp"], state["ssp_run"])
                    state["clearance"] = max(state["clearance"], sole_z[swing])
                    stance_xy = self.data.site_xpos[self.site_ids[stance], :2].copy()
                    if state["slip_origin"] is None:
                        state["slip_origin"] = stance_xy
                    state["slip"] = max(
                        state["slip"], float(np.linalg.norm(stance_xy - state["slip_origin"]))
                    )
                else:
                    state["ssp_run"] = 0.0

                if state["last_com_y"] is None:
                    com_vy = 0.0
                else:
                    com_vy = (com[1] - state["last_com_y"]) / dt
                state["last_com_y"] = com[1]
                omega = math.sqrt(9.81 / max(com[2], 0.03))
                capture_y = com[1] + com_vy / omega
                sole_y = self.data.site_xpos[self.site_ids[stance], 1]
                lateral_error = abs(capture_y - sole_y)
                if is_ssp:
                    state["capture_margin"] = min(state["capture_margin"], 0.019 - lateral_error)

                current = np.abs(torque) / KT_NM_PER_A
                over_rated = np.abs(torque) > RATED_TORQUE_NM
                over_2a = current > PROTECTION_CURRENT_A
                state["over_rated"] = np.where(over_rated, state["over_rated"] + dt, 0.0)
                state["over_2a"] = np.where(over_2a, state["over_2a"] + dt, 0.0)
                state["over_rated_max"] = max(state["over_rated_max"], float(np.max(state["over_rated"])))
                state["over_2a_max"] = max(state["over_2a_max"], float(np.max(state["over_2a"])))
                state["peak_torque"] = max(state["peak_torque"], float(np.max(np.abs(torque))))
                state["peak_speed"] = max(state["peak_speed"], float(np.max(np.abs(qd))))
                state["peak_current"] = max(state["peak_current"], float(np.max(current)))
                state["peak_total_current"] = max(state["peak_total_current"], float(np.sum(current)))
                state["roll"] = max(state["roll"], abs(roll_deg))
                state["pitch"] = max(state["pitch"], abs(pitch_deg))
                state["nonfoot"] = state["nonfoot"] or nonfoot
                state["fell"] = state["fell"] or abs(roll_deg) > 45.0 or abs(pitch_deg) > 45.0 or com[2] < 0.12

            if active_viewer is not None:
                active_viewer.sync()
                elapsed = time.perf_counter() - wall_start
                if self.data.time > elapsed:
                    time.sleep(min(self.data.time - elapsed, 0.01))

        if active_viewer is not None:
            active_viewer.close()

        metrics: list[Metrics] = []
        for cfg, state in zip(configs, event_state):
            success = bool(
                state["ssp"] >= 0.05
                and state["clearance"] >= 0.001
                and state["slip"] <= 0.005
                and not state["nonfoot"]
                and not state["fell"]
                and state["over_2a_max"] < 2.0
            )
            metrics.append(
                Metrics(
                    support=cfg.support,
                    success=success,
                    ssp_duration_s=state["ssp"],
                    max_swing_clearance_mm=state["clearance"] * 1000.0,
                    max_stance_slip_mm=state["slip"] * 1000.0,
                    peak_torque_nm=state["peak_torque"],
                    peak_torque_ratio=state["peak_torque"] / STALL_TORQUE_NM,
                    peak_speed_rad_s=state["peak_speed"],
                    peak_current_a=state["peak_current"],
                    peak_total_leg_current_a=state["peak_total_current"],
                    longest_over_rated_s=state["over_rated_max"],
                    longest_over_2a_s=state["over_2a_max"],
                    min_capture_margin_mm=(
                        state["capture_margin"] * 1000.0
                        if math.isfinite(state["capture_margin"])
                        else None
                    ),
                    max_torso_roll_deg=state["roll"],
                    max_torso_pitch_deg=state["pitch"],
                    nonfoot_floor_contact=bool(state["nonfoot"]),
                    fell=bool(state["fell"]),
                )
            )
        return metrics, {key: np.asarray(value) for key, value in logs.items()}

    def run_forever(self, configs: list[RockingConfig], settle_s: float = 1.0) -> None:
        """Repeat a rocking sequence in real time until the viewer is closed."""
        from mujoco import viewer as mj_viewer

        self.reset()
        gap = 0.25
        starts: list[float] = []
        cursor = 0.0
        for cfg in configs:
            starts.append(cursor)
            cursor += cfg.duration + gap
        cycle_duration = cursor

        q_target = np.deg2rad(STAND_DEG)
        next_control = 0.0
        active_viewer = mj_viewer.launch_passive(self.model, self.data)
        wall_start = time.perf_counter()
        while active_viewer.is_running():
            if self.data.time + 1e-12 >= next_control:
                q_target = np.deg2rad(STAND_DEG)
                if self.data.time >= settle_s:
                    cycle_t = (self.data.time - settle_s) % cycle_duration
                    for start, cfg in zip(starts, configs):
                        local_t = cycle_t - start
                        if 0.0 <= local_t <= cfg.duration:
                            q_target = pulse_target(local_t, cfg)
                            break
                q_target = np.round(q_target / ENCODER_STEP_RAD) * ENCODER_STEP_RAD
                next_control += self.control_period

            q = self.data.qpos[self.qpos_ids].copy()
            qd = self.data.qvel[self.dof_ids].copy()
            desired_torque = self.kp * (q_target - q) - self.kd * qd
            torque = torque_speed_limit(desired_torque, qd)
            self.data.qfrc_applied[:] = 0.0
            self.data.qfrc_applied[self.dof_ids] = torque
            self.data.ctrl[6:] = 0.0
            mujoco.mj_step(self.model, self.data)

            active_viewer.sync()
            elapsed = time.perf_counter() - wall_start
            if self.data.time > elapsed:
                time.sleep(min(self.data.time - elapsed, 0.01))
        active_viewer.close()


def score(metric: Metrics) -> float:
    return (
        500.0 * metric.ssp_duration_s
        + 2.0 * min(metric.max_swing_clearance_mm, 5.0)
        - 4.0 * metric.max_stance_slip_mm
        - 12.0 * metric.peak_torque_ratio
        - (100.0 if metric.fell else 0.0)
        - (50.0 if metric.nonfoot_floor_contact else 0.0)
    )


def search_parameters(sim: Simulator, output: Path) -> list[dict]:
    rows: list[dict] = []
    for support in ("left", "right"):
        roll_values = (8.0, 11.0, 14.0, 16.0) if support == "left" else (7.0, 9.0, 11.0, 12.0)
        for roll_deg in roll_values:
            for roll_time in (0.15, 0.25, 0.40):
                for lift_deg in (10.0, 15.0, 20.0):
                    for lift_offset in (-0.10, -0.05, 0.0):
                        cfg = RockingConfig(
                            support=support,
                            roll_deg=roll_deg,
                            roll_time=roll_time,
                            lift_deg=lift_deg,
                            lift_offset=lift_offset,
                        )
                        metric = sim.run([cfg], record=False)[0][0]
                        rows.append({**asdict(cfg), **asdict(metric), "score": score(metric)})
    rows.sort(key=lambda row: (row["success"], row["score"]), reverse=True)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    return rows


def save_plot(logs: dict[str, np.ndarray], output: Path) -> None:
    import matplotlib.pyplot as plt

    output.parent.mkdir(parents=True, exist_ok=True)
    t = logs["time"]
    fig, axes = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
    axes[0].plot(t, logs["left_force"], label="left")
    axes[0].plot(t, logs["right_force"], label="right")
    axes[0].set_ylabel("normal force [N]")
    axes[0].legend()
    axes[1].plot(t, logs["left_z"] * 1000.0, label="left")
    axes[1].plot(t, logs["right_z"] * 1000.0, label="right")
    axes[1].axhline(1.0, color="k", linewidth=0.7, linestyle="--")
    axes[1].set_ylabel("sole height [mm]")
    axes[2].plot(t, logs["roll_deg"], label="roll")
    axes[2].plot(t, logs["pitch_deg"], label="pitch")
    axes[2].set_ylabel("torso [deg]")
    axes[2].legend()
    axes[3].plot(t, np.max(np.abs(logs["torque"]), axis=1), label="peak joint torque")
    axes[3].axhline(RATED_TORQUE_NM, color="tab:orange", linestyle="--", label="rated")
    axes[3].axhline(STALL_TORQUE_NM, color="tab:red", linestyle="--", label="stall")
    axes[3].set_ylabel("torque [Nm]")
    axes[3].set_xlabel("time [s]")
    axes[3].legend()
    fig.tight_layout()
    fig.savefig(output, dpi=160)
    plt.close(fig)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--support", choices=("left", "right", "both"), default="both")
    parser.add_argument("--cycles", type=int, default=1)
    parser.add_argument("--roll-deg", type=float, default=11.0)
    parser.add_argument("--roll-time", type=float, default=0.15)
    parser.add_argument("--lift-deg", type=float, default=10.0)
    parser.add_argument("--lift-time", type=float, default=0.20)
    parser.add_argument("--lift-offset", type=float, default=-0.05)
    parser.add_argument("--control-hz", type=float, default=100.0)
    parser.add_argument("--kp", type=float, default=28.0)
    parser.add_argument("--kd", type=float, default=0.65)
    parser.add_argument("--search", action="store_true")
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--forever", action="store_true", help="repeat until the viewer window is closed")
    parser.add_argument("--plot", type=Path)
    parser.add_argument("--output", type=Path, default=RESULTS_DIR / "rocking_search.csv")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    sim = Simulator(control_hz=args.control_hz, kp=args.kp, kd=args.kd)
    if args.search:
        rows = search_parameters(sim, args.output)
        print(json.dumps({"output": str(args.output), "top": rows[:10]}, indent=2))
        return

    sides = [args.support] if args.support != "both" else ["left", "right"]
    repeat_count = 1 if args.forever else args.cycles
    configs = [
        RockingConfig(
            support=side,
            roll_deg=args.roll_deg if side == "left" else min(args.roll_deg, 12.0),
            roll_time=args.roll_time,
            lift_deg=args.lift_deg,
            lift_time=args.lift_time,
            lift_offset=args.lift_offset,
        )
        for _ in range(repeat_count)
        for side in sides
    ]
    if args.forever:
        sim.run_forever(configs)
        return
    metrics, logs = sim.run(configs, viewer=args.viewer)
    if args.plot:
        save_plot(logs, args.plot)
    print(json.dumps([asdict(metric) for metric in metrics], indent=2))


if __name__ == "__main__":
    main()
