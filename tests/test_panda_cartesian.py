import csv

import mujoco
import numpy as np
import pytest

from panda_cartesian import (
    FINAL_ERROR_LIMIT_M,
    JOINT_MARGIN_RAD,
    MAX_REFERENCE_ACCEL,
    MAX_REFERENCE_SPEED,
    MAX_TRACKING_ACCEL,
    MAX_TRACKING_SPEED,
    InverseKinematics,
    load_panda,
    plan_trajectory,
    read_arm,
    save_results,
    tcp_position,
    track_trajectory,
)
from panda_position import SCENE


def test_ik_rejects_an_unreachable_target():
    model, home = load_panda()
    arm = read_arm(model)
    ik = InverseKinematics(model, home, arm)
    with pytest.raises(ValueError, match="unreachable.*XYZ residual"):
        ik.solve(np.array([2.0, 0.0, 1.2]), home.qpos[arm.qpos])


def test_preflight_rejects_a_colliding_home_pose():
    spec = mujoco.MjSpec.from_file(str(SCENE))
    spec.worldbody.add_geom(
        name="collision_fixture", type=mujoco.mjtGeom.mjGEOM_SPHERE,
        pos=[0.5545, 0, 0.61], size=[0.08, 0, 0],
    )
    model = spec.compile()
    home = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, home, model.key("home").id)
    mujoco.mj_forward(model, home)
    arm = read_arm(model)
    with pytest.raises(RuntimeError, match=r"path at 0\.000 s \(home\): collision"):
        plan_trajectory(model, home, arm)


def test_cartesian_path_tracks_through_dynamics_and_exports(monkeypatch, tmp_path):
    model, data = load_panda()
    arm = read_arm(model)
    plan = plan_trajectory(model, data, arm)
    dt = model.opt.timestep
    assert len(arm.actuators) == 7
    assert plan.peak_speed <= MAX_REFERENCE_SPEED
    assert plan.peak_accel <= MAX_REFERENCE_ACCEL
    assert np.all(plan.angles >= arm.joint_range[:, 0] + JOINT_MARGIN_RAD)
    assert np.all(plan.angles <= arm.joint_range[:, 1] - JOINT_MARGIN_RAD)
    np.testing.assert_allclose(plan.waypoints[0], tcp_position(data, arm), atol=1e-12)
    delta_xyz = np.diff(plan.waypoints, axis=0)
    assert delta_xyz[0, 2] == pytest.approx(0)
    np.testing.assert_allclose(delta_xyz[1:3, :2], 0, atol=1e-12)
    assert delta_xyz[1, 2] < 0 < delta_xyz[2, 2]

    step_physics = mujoco.mj_step
    previous_qpos = data.qpos.copy()
    grip = data.ctrl[model.actuator("actuator8").id]
    steps = 0

    def checked_step(step_model, step_data):
        nonlocal steps
        assert step_data is data
        np.testing.assert_array_equal(step_data.qpos, previous_qpos)
        assert step_data.actuator("actuator8").ctrl[0] == grip
        assert np.all(step_data.ctrl[arm.actuators] >= arm.ctrl_range[:, 0])
        assert np.all(step_data.ctrl[arm.actuators] <= arm.ctrl_range[:, 1])
        step_physics(step_model, step_data)
        previous_qpos[:] = step_data.qpos
        steps += 1

    monkeypatch.setattr(mujoco, "mj_step", checked_step)
    run = track_trajectory(model, data, arm, plan)
    assert steps == len(run.times) - 1 >= 1000
    assert run.times[-1] == pytest.approx(10.0)
    assert np.isfinite(run.angles).all() and np.isfinite(run.velocities).all() and np.isfinite(run.xyz).all()
    assert np.all(np.ptp(run.angles, axis=0) > 0.02)
    assert np.all(np.ptp(run.xyz, axis=0) > 0.05)
    assert np.max(np.abs(run.velocities)) <= MAX_TRACKING_SPEED
    assert np.max(np.abs(np.diff(run.velocities, axis=0))) / dt <= MAX_TRACKING_ACCEL
    assert np.all(run.torques >= arm.torque_range[:, 0] - 1e-9)
    assert np.all(run.torques <= arm.torque_range[:, 1] + 1e-9)
    error = np.linalg.norm(plan.xyz - run.xyz, axis=1)
    assert np.all(error[plan.waypoint_steps] < FINAL_ERROR_LIMIT_M)
    assert error[-1] < FINAL_ERROR_LIMIT_M

    csv_path, *figures = save_results(plan, run, tmp_path)
    with csv_path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        assert reader.fieldnames[:7] == [
            "time_s", "target_x_m", "target_y_m", "target_z_m", "actual_x_m", "actual_y_m", "actual_z_m"
        ]
        rows = list(reader)
    assert len(rows) == steps + 1
    assert float(rows[-1]["error_norm_m"]) == pytest.approx(error[-1])
    for figure in figures:
        assert figure.parent == tmp_path / "figures"
        with figure.open("rb") as stream:
            assert stream.read(8) == b"\x89PNG\r\n\x1a\n"
