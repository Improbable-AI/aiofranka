"""Warm control computations before torque control; never access robot transport."""

from copy import copy
import threading
import time
from types import SimpleNamespace

import mujoco
import numpy as np


def warmup_control_math(controller, iterations=100):
    """Exercise real control laws with separate MuJoCo data and a dummy sink.

    Only the already-synchronized local model/data are read. A shadow controller
    owns its targets, gains and data; its robot has only a dummy step method.
    No live state reads, torque writes, IPC publication or mode changes occur.
    Returned timings describe local computation, excluding communication.
    """
    if not isinstance(iterations, int) or iterations < 1:
        raise ValueError("Warmup iterations must be a positive integer")
    began = time.perf_counter()
    model, source_data = controller.robot.model, controller.robot.data
    data = mujoco.MjData(model)
    for name in ("qpos", "qvel", "ctrl", "act", "mocap_pos", "mocap_quat"):
        getattr(data, name)[:] = getattr(source_data, name)
    if not np.isfinite(np.r_[data.qpos, data.qvel, data.ctrl]).all():
        raise ValueError("Warmup needs finite local robot state")

    shadow = copy(controller)
    shadow.state_lock = threading.Lock()
    shadow._shm = shadow.error_callback = None
    for name in ("kp", "kd", "ee_kp", "ee_kd", "null_kp", "null_kd", "torque_limit"):
        setattr(shadow, name, np.array(getattr(controller, name), dtype=float, copy=True))
    sink_count = 0

    def discard_torque(torque):
        nonlocal sink_count
        torque = np.asarray(torque)
        if torque.shape != (7,) or not np.isfinite(torque).all():
            raise ValueError("Warmup produced invalid torques")
        sink_count += 1

    shadow.robot = SimpleNamespace(step=discard_torque)
    ee, jac, mm = np.eye(4), np.zeros((6, 7)), np.zeros((7, 7))
    site_id = controller.robot.site_id
    state = {"qpos": data.qpos.copy(), "qvel": data.qvel.copy(), "last_torque": data.ctrl.copy(),
             "ee": ee, "jac": jac, "mm": mm}
    shadow.state = state
    shadow.initial_qpos = shadow.q_desired = data.qpos.copy()
    shadow.ee_desired = np.eye(4)
    shadow.torque = np.zeros(7)

    # Exercise both LAPACK branches regardless of the current robot pose.
    for size in (6, 7):
        np.linalg.inv(np.eye(size))
        np.linalg.pinv(np.eye(size))

    timings = {name: np.empty(iterations) for name in ("mujoco", "impedance", "osc")}
    for index in range(iterations):
        started = time.perf_counter()
        mujoco.mj_forward(model, data)
        ee[:3, :3] = data.site_xmat[site_id].reshape(3, 3)
        ee[:3, 3] = data.site_xpos[site_id]
        mujoco.mj_jacSite(model, data, jac[:3], jac[3:], site_id)
        mujoco.mj_fullM(model, data, mm)
        timings["mujoco"][index] = time.perf_counter() - started
        shadow.ee_desired[:] = ee
        for name in ("impedance", "osc"):
            started = time.perf_counter()
            getattr(shadow, f"_{name}_step")(state)
            timings[name][index] = time.perf_counter() - started

    return {"iterations": iterations, "dummy_torque_writes": sink_count,
            "elapsed_s": time.perf_counter() - began,
            "timing_scope": "local computations only; no robot communication or IPC",
            "timing_ms": {name: {"first": float(values[0] * 1000),
                                 "median": float(np.median(values) * 1000),
                                 "max": float(np.max(values) * 1000)}
                          for name, values in timings.items()}}
