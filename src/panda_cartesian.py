"""Plan and track a collision-checked Cartesian path for the MuJoCo Panda."""

import argparse
import csv
from contextlib import nullcontext
from dataclasses import dataclass
from pathlib import Path
from time import perf_counter, sleep
from typing import NamedTuple

import mujoco
import numpy as np
from numpy.typing import NDArray

from panda_position import ROOT, load_panda, require_position_servo


JOINT_NAMES = tuple(f"joint{i}" for i in range(1, 8))
WAYPOINT_LABELS = ("home", "horizontal", "lower", "raise", "second target")
# The finger pad center is ~0.0584 + 0.0445 m along the hand's local Z axis.
TCP_OFFSET_M = np.array([0.0, 0.0, 0.1034])
WAYPOINT_OFFSETS_M = np.array([
    [0.0, 0.0, 0.0],
    [0.045, 0.12, 0.0],
    [0.045, 0.12, -0.13],
    [0.045, 0.12, 0.0],
    [-0.09, 0.20, 0.055],
])
SEGMENT_DURATIONS_S = (2.2, 2.0, 2.0, 2.5)
FINAL_HOLD_S = 1.3
JOINT_MARGIN_RAD = 0.05
MAX_REFERENCE_SPEED = 0.8  # rad/s
MAX_REFERENCE_ACCEL = 1.8  # rad/s²
MAX_TRACKING_SPEED = 1.0  # rad/s
MAX_TRACKING_ACCEL = 2.5  # rad/s²
ORIENTATION_WEIGHT = 0.25  # meters per radian in the weighted pose error
IK_DAMPING = 0.015
IK_TOLERANCE = 1e-8
FINAL_ERROR_LIMIT_M = 0.02

FloatArray = NDArray[np.float64]


@dataclass(frozen=True)
class Arm:
    hand: int
    qpos: NDArray[np.intp]
    dofs: NDArray[np.intp]
    actuators: NDArray[np.intp]
    joint_range: FloatArray
    ctrl_range: FloatArray
    torque_range: FloatArray
    kp: FloatArray
    kd: FloatArray


def read_arm(model: mujoco.MjModel) -> Arm:
    joints = np.array([model.joint(name).id for name in JOINT_NAMES])
    actuators = np.array([model.actuator(f"actuator{i}").id for i in range(1, 8)])
    for joint, actuator in zip(joints, actuators):
        require_position_servo(model, model.joint(int(joint)).name, model.actuator(int(actuator)).name)
    if not (
        model.jnt_limited[joints].all()
        and model.actuator_ctrllimited[actuators].all()
        and model.actuator_forcelimited[actuators].all()
    ):
        raise ValueError("All seven arm joints must have position, control, and output limits")
    return Arm(
        model.body("hand").id,
        model.jnt_qposadr[joints],
        model.jnt_dofadr[joints],
        actuators,
        model.jnt_range[joints],
        model.actuator_ctrlrange[actuators],
        model.actuator_forcerange[actuators],
        model.actuator_gainprm[actuators, 0],
        -model.actuator_biasprm[actuators, 2],
    )


def tcp_position(data: mujoco.MjData, arm: Arm) -> FloatArray:
    return data.xpos[arm.hand] + data.xmat[arm.hand].reshape(3, 3) @ TCP_OFFSET_M


def check_contacts(model: mujoco.MjModel, data: mujoco.MjData, stage: str) -> None:
    for contact in data.contact:
        if contact.dist < -1e-6:
            bodies = [model.body(int(model.geom_bodyid[geom])).name for geom in contact.geom]
            raise RuntimeError(f"{stage}: collision {bodies[0]} / {bodies[1]} (dist={contact.dist:.6f} m)")


class InverseKinematics:
    def __init__(self, model: mujoco.MjModel, home: mujoco.MjData, arm: Arm):
        self.model = model
        self.arm = arm
        # Candidate poses live in separate data. Only mj_step moves the simulated arm.
        self.scratch = mujoco.MjData(model)
        mujoco.mj_copyData(self.scratch, model, home)
        self.orientation = home.xquat[arm.hand].copy()
        self.lower = np.maximum(arm.joint_range[:, 0], arm.ctrl_range[:, 0]) + JOINT_MARGIN_RAD
        self.upper = np.minimum(arm.joint_range[:, 1], arm.ctrl_range[:, 1]) - JOINT_MARGIN_RAD
        self.jacp = np.empty((3, model.nv))
        self.jacr = np.empty((3, model.nv))
        self.regularization = IK_DAMPING**2 * np.eye(6)

    def pose_error(self, angles: FloatArray, target: FloatArray) -> tuple[FloatArray, FloatArray]:
        self.scratch.qpos[self.arm.qpos] = angles
        mujoco.mj_fwdPosition(self.model, self.scratch)
        point = tcp_position(self.scratch, self.arm)
        inverse = self.scratch.xquat[self.arm.hand].copy()
        inverse[1:] *= -1
        rotation = np.empty(4)
        mujoco.mju_mulQuat(rotation, self.orientation, inverse)
        if rotation[0] < 0:
            rotation *= -1
        error = np.empty(6)
        error[:3] = target - point
        mujoco.mju_quat2Vel(error[3:], rotation, 1.0)
        error[3:] *= ORIENTATION_WEIGHT
        return point, error

    def solve(self, target: FloatArray, seed: FloatArray, *, stage: str = "IK") -> FloatArray:
        if target.shape != (3,) or not np.isfinite(target).all():
            raise ValueError("IK target must be a finite world XYZ vector")
        angles = np.clip(seed, self.lower, self.upper)
        point, error = self.pose_error(angles, target)
        for _ in range(80):
            norm = np.linalg.norm(error)
            if norm < IK_TOLERANCE:
                check_contacts(self.model, self.scratch, stage)
                return angles
            mujoco.mj_jac(self.model, self.scratch, self.jacp, self.jacr, point, self.arm.hand)
            jac = np.vstack((self.jacp[:, self.arm.dofs], ORIENTATION_WEIGHT * self.jacr[:, self.arm.dofs]))
            delta = jac.T @ np.linalg.solve(jac @ jac.T + self.regularization, error)
            delta *= min(1.0, 0.12 / max(0.12, np.max(np.abs(delta))))
            for fraction in (1.0, 0.5, 0.25, 0.125, 0.0625):
                trial = np.clip(angles + fraction * delta, self.lower, self.upper)
                trial_point, trial_error = self.pose_error(trial, target)
                if np.linalg.norm(trial_error) < norm:
                    angles, point, error = trial, trial_point, trial_error
                    break
            else:
                break
        remaining = np.linalg.norm(error[:3])
        raise ValueError(f"{stage}: unreachable with joint limits and fixed orientation; XYZ residual {remaining:.6f} m")


@dataclass(frozen=True)
class Trajectory:
    waypoints: FloatArray
    waypoint_steps: NDArray[np.intp]
    xyz: FloatArray
    angles: FloatArray
    velocities: FloatArray
    peak_speed: float
    peak_accel: float


def plan_trajectory(model: mujoco.MjModel, home: mujoco.MjData, arm: Arm) -> Trajectory:
    dt = model.opt.timestep
    waypoints = tcp_position(home, arm) + WAYPOINT_OFFSETS_M
    segment_steps = np.round(np.array(SEGMENT_DURATIONS_S) / dt).astype(int)
    waypoint_steps = np.r_[0, np.cumsum(segment_steps)]
    ik = InverseKinematics(model, home, arm)
    # Quintic time scaling gives zero velocity and acceleration at every junction.
    targets = [waypoints[:1]]
    for start, end, steps in zip(waypoints[:-1], waypoints[1:], segment_steps):
        u = np.arange(1, steps + 1) / steps
        blend = u**3 * (10 + u * (-15 + 6 * u))
        targets.append(start + blend[:, None] * (end - start))
    xyz = np.concatenate(targets)
    angles = np.empty((len(xyz), len(JOINT_NAMES)))
    seed = home.qpos[arm.qpos]
    waypoint_labels = dict(zip(waypoint_steps, WAYPOINT_LABELS))
    for step, target in enumerate(xyz):
        stage = f"path at {step * dt:.3f} s"
        if step in waypoint_labels:
            stage += f" ({waypoint_labels[step]})"
        seed = ik.solve(target, seed, stage=stage)
        angles[step] = seed

    peak_speed = float(np.max(np.abs(np.diff(angles, axis=0))) / dt)
    peak_accel = float(np.max(np.abs(np.diff(angles, n=2, axis=0))) / dt**2)
    if peak_speed > MAX_REFERENCE_SPEED or peak_accel > MAX_REFERENCE_ACCEL:
        raise ValueError(f"Joint reference exceeds velocity or acceleration limits: {peak_speed:.3f} rad/s, "
                         f"{peak_accel:.3f} rad/s^2")

    hold = round(FINAL_HOLD_S / dt)
    xyz = np.concatenate((xyz, np.repeat(xyz[-1:], hold, axis=0)))
    angles = np.concatenate((angles, np.repeat(angles[-1:], hold, axis=0)))
    velocities = np.gradient(angles, dt, axis=0)
    velocities[0] = 0.0
    return Trajectory(waypoints, waypoint_steps, xyz, angles, velocities, peak_speed, peak_accel)


class TrackingRun(NamedTuple):
    times: FloatArray
    xyz: FloatArray
    angles: FloatArray
    velocities: FloatArray
    torques: FloatArray


def joint_controls(data: mujoco.MjData, arm: Arm, angle: FloatArray, velocity: FloatArray) -> FloatArray:
    position = data.qpos[arm.qpos]
    speed = data.qvel[arm.dofs]
    torque = arm.kp * (angle - position) + arm.kd * (velocity - speed) + data.qfrc_bias[arm.dofs]
    torque = np.clip(torque, arm.torque_range[:, 0], arm.torque_range[:, 1])
    # Invert the verified general actuator law: tau = kp * (ctrl - q) - kd * dq.
    control = position + (torque + arm.kd * speed) / arm.kp
    return np.clip(control, arm.ctrl_range[:, 0], arm.ctrl_range[:, 1])


def add_connector(scene: mujoco.MjvScene, start: FloatArray, end: FloatArray, *, measured: bool) -> None:
    if scene.ngeom >= scene.maxgeom or np.linalg.norm(end - start) < 1e-5:
        return
    kind = mujoco.mjtGeom.mjGEOM_CAPSULE if measured else mujoco.mjtGeom.mjGEOM_LINE
    width = 0.0024 if measured else 3.0
    color = (1.0, 0.40, 0.05, 1.0) if measured else (0.06, 0.92, 1.0, 1.0)
    geom = scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(geom, kind, np.zeros(3), np.zeros(3), np.eye(3).ravel(), color)
    mujoco.mjv_connector(geom, kind, width, start, end)
    scene.ngeom += 1


def draw_waypoints(scene: mujoco.MjvScene, points: FloatArray) -> None:
    for start, end in zip(points[:-1], points[1:]):
        add_connector(scene, start, end, measured=False)
    for point in points:
        if scene.ngeom >= scene.maxgeom:
            return
        mujoco.mjv_initGeom(
            scene.geoms[scene.ngeom], mujoco.mjtGeom.mjGEOM_SPHERE, (0.006, 0, 0), point,
            np.eye(3).ravel(), (0.06, 0.92, 1.0, 1.0),
        )
        scene.ngeom += 1


def track_trajectory(
    model: mujoco.MjModel, data: mujoco.MjData, arm: Arm, plan: Trajectory, *, viewer: bool = False
) -> TrackingRun:
    dt = model.opt.timestep
    count = len(plan.xyz)
    run = TrackingRun(np.empty(count), np.empty((count, 3)), np.empty_like(plan.angles),
                      np.empty_like(plan.angles), np.zeros_like(plan.angles))
    view_context = nullcontext()
    if viewer:
        from mujoco.viewer import launch_passive

        view_data = mujoco.MjData(model)
        mujoco.mj_copyData(view_data, model, data)
        view_context = launch_passive(model, view_data, show_left_ui=False, show_right_ui=False)

    with view_context as window:
        start = 0.0
        frame_steps = max(1, round(1 / (40 * dt)))
        trail_point = tcp_position(data, arm)
        if window is not None:
            with window.lock():
                window.cam.lookat[:] = (0.36, 0.06, 0.44)
                window.cam.distance = 1.85
                window.cam.azimuth = 120
                window.cam.elevation = -27
                draw_waypoints(window.user_scn, plan.waypoints)
            window.sync()
            start = perf_counter() + 1.5

        for i in range(count):
            if i:
                data.ctrl[arm.actuators] = joint_controls(data, arm, plan.angles[i - 1], plan.velocities[i - 1])
                mujoco.mj_step(model, data)
                run.torques[i] = data.actuator_force[arm.actuators]
                # Refresh world kinematics and contacts for the integrated state.
                mujoco.mj_forward(model, data)

            stage = f"simulation at {i * dt:.3f} s"
            if not (np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all()
                    and np.isfinite(run.torques[i]).all()
                    and np.isclose(data.time, i * dt, atol=dt / 4, rtol=0)
                    and not any(warning.number for warning in data.warning)):
                raise RuntimeError(f"{stage}: invalid or reset dynamics state")
            check_contacts(model, data, stage)
            run.times[i] = data.time
            run.xyz[i] = tcp_position(data, arm)
            run.angles[i] = data.qpos[arm.qpos]
            run.velocities[i] = data.qvel[arm.dofs]
            if (np.any(run.angles[i] < arm.joint_range[:, 0] - 1e-6)
                    or np.any(run.angles[i] > arm.joint_range[:, 1] + 1e-6)
                    or np.max(np.abs(run.velocities[i])) > MAX_TRACKING_SPEED
                    or (i and np.max(np.abs(run.velocities[i] - run.velocities[i - 1])) / dt > MAX_TRACKING_ACCEL)):
                raise RuntimeError(f"{stage}: measured joint position, speed, or acceleration limit exceeded")

            if window is not None and window.is_running() and (i % frame_steps == 0 or i == count - 1):
                sleep(max(0.0, start + data.time - perf_counter()))
                with window.lock():
                    mujoco.mj_copyData(view_data, model, data)
                    add_connector(window.user_scn, trail_point, run.xyz[i], measured=True)
                window.sync(state_only=True)
                trail_point = run.xyz[i]
    return run


def save_results(plan: Trajectory, run: TrackingRun, output_dir: Path) -> tuple[Path, Path, Path]:
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib.figure import Figure

    figures = output_dir / "figures"
    figures.mkdir(parents=True, exist_ok=True)
    csv_path = output_dir / "panda_cartesian_tracking.csv"
    error_path = figures / "panda_cartesian_xyz_error.png"
    angles_path = figures / "panda_cartesian_joint_angles.png"

    xyz_error = plan.xyz - run.xyz
    error_norm = np.linalg.norm(xyz_error, axis=1)
    columns = ["time_s"]
    for kind in ("target", "actual", "error"):
        columns.extend(f"{kind}_{axis}_m" for axis in "xyz")
    columns.append("error_norm_m")
    for kind, unit in (("target", "rad"), ("actual", "rad"), ("velocity", "rad_s"), ("applied_torque", "nm")):
        columns.extend(f"{kind}_{joint}_{unit}" for joint in JOINT_NAMES)
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(columns)
        writer.writerows(np.column_stack((run.times, plan.xyz, run.xyz, xyz_error, error_norm,
                                          plan.angles, run.angles, run.velocities, run.torques)))

    figure = Figure(figsize=(9, 6), layout="constrained")
    FigureCanvasAgg(figure)
    figure.suptitle("Panda end-effector tracking error in world coordinates")
    axes = figure.subplots(2, 2, sharex=True).flat
    for ax, values, name, color in zip(axes, np.column_stack((xyz_error, error_norm)).T,
                                      ("X", "Y", "Z", "3D norm"), ("#1568a0", "#16856d", "#973f93", "#b14e17")):
        ax.plot(run.times, 1000 * values, color=color)
        ax.set(title=name, ylabel="Error (mm)")
        ax.grid(alpha=0.25)
    figure.supxlabel("Simulation time (s)")
    figure.savefig(error_path, dpi=150)

    figure = Figure(figsize=(10, 8.5), layout="constrained")
    FigureCanvasAgg(figure)
    figure.suptitle("Panda arm joint angles")
    axes = figure.subplots(4, 2, sharex=True).ravel()
    for i, joint in enumerate(JOINT_NAMES):
        ax = axes[i]
        ax.plot(run.times, run.angles[:, i], color="#bd531e", label="Measured", linewidth=2.2)
        ax.plot(run.times, plan.angles[:, i], "--", color="#174b7a", label="IK target", linewidth=1.0)
        ax.set(title=joint, ylabel="Angle (rad)")
        ax.grid(alpha=0.25)
    axes[-1].legend(*axes[0].get_legend_handles_labels(), loc="center", frameon=False)
    axes[-1].set_axis_off()
    figure.supxlabel("Simulation time (s)")
    figure.savefig(angles_path, dpi=150)
    return csv_path, error_path, angles_path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--viewer", action="store_true", help="show the target path and live measured trail")
    parser.add_argument("--output-dir", type=Path, default=ROOT / "outputs", help="directory for CSV and figures")
    args = parser.parse_args()
    try:
        model, data = load_panda()
        arm = read_arm(model)
        plan = plan_trajectory(model, data, arm)
        print(f"MuJoCo {mujoco.__version__} | {len(arm.actuators)} arm actuators / {model.nu} total", flush=True)
        for name, position in zip(WAYPOINT_LABELS, plan.waypoints):
            print(f"{name}: {np.array2string(position.round(4), suppress_small=True)} m")
        print(f"Preflight passed | peak reference speed {plan.peak_speed:.3f} rad/s | "
              f"peak acceleration {plan.peak_accel:.3f} rad/s^2", flush=True)
        run = track_trajectory(model, data, arm, plan, viewer=args.viewer)
    except (ValueError, RuntimeError) as error:
        parser.exit(1, f"Error: {error}\n")

    errors = np.linalg.norm(plan.xyz - run.xyz, axis=1)
    for path in save_results(plan, run, args.output_dir):
        print(path.resolve())
    print(f"{len(run.times) - 1} physics steps | {run.times[-1]:.3f} s | "
          f"final XYZ error {1000 * errors[-1]:.6f} mm | peak {1000 * errors.max():.3f} mm")
    print(f"Measured peak joint speed {np.max(np.abs(run.velocities)):.3f} rad/s | "
          f"peak acceleration {np.max(np.abs(np.diff(run.velocities, axis=0))) / model.opt.timestep:.3f} rad/s^2")
    if errors[-1] >= FINAL_ERROR_LIMIT_M:
        parser.exit(1, "Error: final XYZ error must be below 20 mm\n")


if __name__ == "__main__":
    main()
