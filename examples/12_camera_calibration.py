#!/usr/bin/env python3
"""Calibrate a fixed PyCAAS RealSense stream against a Franka with an AprilCube.

Setup (Linux, Python >= 3.10)::

    python -m pip install -e '.[t-pushing]'
    python -m pip install 'numpy>=1.24' 'pyzmq>=25' 'click>=8' 'pyrealsense2>=2.50'
    python -m pip install --no-deps -e /path/to/pycaas
    pycaas start
    # Put your printed cube's config at assets/aprilcube/config.json, or pass
    # --cube-config /path/to/config.json. Dimensions must match the actual print.
    python examples/12_camera_calibration.py list-cameras
    python examples/12_camera_calibration.py preview
    aiofranka unlock
    python examples/12_camera_calibration.py calibrate --robot-ip 172.16.0.2
    # If multiple cameras are running, add --stream rs_SERIAL_color.
    python examples/12_camera_calibration.py fit --session camera_calibration/RUN

These install commands target PyCAAS 0.1.0. Its dependencies are installed
explicitly so OpenCV's contrib wheel supplies cv2 for both PyCAAS and AprilCube;
do not install the overlapping opencv-python wheel alongside it.

Open the URL printed at startup in your browser, including when running over
SSH. The web UI binds to 0.0.0.0; --host and --port override the address. No
desktop display is needed. Only one calibration client should control a run.

Keep the camera fixed and the cube rigidly attached throughout collection. Move
other targets with overlapping marker IDs out of view. Set
the robot's payload for your gripper/cube before starting. Calibration starts a
separate aiofranka server in damped gravity compensation; move the arm by hand.
Hold still, press Space, and remain still for one second. Capture 15-25 poses
spread across the image and depth, with substantial wrist rotations about at
least two axes. The browser's Finish button (Enter) fits; Stop (Q) exits with
samples preserved. After fitting, MuJoCo overlays are generated automatically
and shown alongside the matrix. The result page stays until Stop or Ctrl+C.
Overlays and calibration_summary.jpg are also saved in SESSION/verification.
Use --cube-xml for custom cube assets, or --no-overlay to skip rendering.

PyCAAS owns camera capture; this script neither starts/stops nor reconfigures
its daemon. Frames use the stream's native resolution and factory intrinsics.
--width/--height are optional profile checks. Configure resolution/FPS with
pycaas before running. PyCAAS timestamps daemon read completion, not exposure;
captures use a stationary interval to reduce sensitivity to camera latency.

The fit estimates both camera-to-base and cube-to-end-effector transforms. It
keeps the active color stream's factory intrinsics/distortion fixed. Distances
are meters; T_A_B maps B coordinates into A. Camera axes are right/down/forward.
The EE frame is aiofranka's MuJoCo attachment_site (state['ee']), not a Desk TCP.
Raw images, corners, robot poses, and config are saved for hardware-free refits.
Inspect held-out reprojection error before using the result; it measures image
agreement, not independently verified physical accuracy.

Workflow informed by https://github.com/tonibronars/flexiv_deploy/tree/main/camera.
Uses upstream AprilCube geometry, without the reference repo's local patches.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import importlib.util
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import mimetypes
import os
from pathlib import Path
import queue
import socket
import subprocess
import sys
import threading
import time
from urllib.parse import unquote

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


ROOT = Path(__file__).resolve().parents[1]


def write_json(path, value):
    """Atomically replace our session index so interrupted runs remain usable."""
    path = Path(path)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")
    temporary.replace(path)


def load_example(filename, name):
    """Load a neighboring example without depending on the caller's working directory."""
    spec = importlib.util.spec_from_file_location(name, Path(__file__).with_name(filename))
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


_capture = load_example("camera_calibration_capture.py", "_aiofranka_calibration_capture")
PycaasCamera = _capture.PycaasCamera
list_cameras = _capture.list_cameras


class CubeDetector:
    """Independent AprilCube measurements; output transforms/geometry use meters."""

    def __init__(self, config_path, camera_metadata, min_tags=2, max_rms=2.0):
        try:
            from aprilcube.detect import load_cube_config, build_tag_corner_map, create_detector
            from aprilcube.generate import FACE_DEFS
        except ImportError as exc:
            raise RuntimeError("Install AprilCube in this environment: python -m pip install 'aprilcube>=0.3'") from exc

        config, faces = load_cube_config(str(config_path))
        if len(set(config.tag_ids)) != len(config.tag_ids):
            raise ValueError("Cube config contains duplicate tag IDs")
        self.points = {int(tag): np.asarray(corners, dtype=float) * 0.001
                       for tag, corners in build_tag_corner_map(config).items()}
        if not self.points or any(p.shape != (4, 3) or not np.isfinite(p).all()
                                  for p in self.points.values()):
            raise ValueError("Cube config needs finite 4x3 corner geometry per tag")
        self.normals = {}
        if config.marker_corners is None:
            for name, axis, sign, *_ in FACE_DEFS:
                normal = np.zeros(3)
                normal[axis] = sign
                for tag in faces.get(name, []):
                    self.normals[int(tag)] = normal
        self.normals.update({int(tag): np.asarray(normal, dtype=float)
                             for tag, normal in (config.marker_normals or {}).items()})
        self.matrix = np.asarray(camera_metadata["camera_matrix"], dtype=float)
        self.distortion = np.asarray(camera_metadata["dist_coeffs"], dtype=float)
        self.detector = create_detector(config.dict_id, fast=False)  # SUBPIX refinement.
        self.min_tags, self.max_rms = min_tags, max_rms
        if min_tags < 1 or not np.isfinite(max_rms) or max_rms <= 0:
            raise ValueError("Minimum tags and maximum reprojection RMS must be positive")

    def detect(self, image):
        """Decode current pixels only; do not reuse tracking or filtered poses."""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
        quads, ids, _ = self.detector.detectMarkers(gray)
        observed = {}
        for quad, tag in zip(quads, np.asarray(ids).reshape(-1) if ids is not None else []):
            tag = int(tag)
            if tag not in self.points:
                continue
            if tag in observed:
                raise ValueError(f"Duplicate visible tag ID {tag}; remove other matching targets")
            observed[tag] = np.asarray(quad, dtype=float).reshape(4, 2)
        tags = sorted(observed)
        if len(tags) < self.min_tags:
            raise ValueError(f"Need {self.min_tags} cube tags; detected {len(tags)}")
        points = np.concatenate([self.points[tag] for tag in tags])
        pixels = np.concatenate([observed[tag] for tag in tags])
        planar = np.linalg.matrix_rank(points - points.mean(axis=0), tol=1e-9) == 2
        flags = [cv2.SOLVEPNP_SQPNP] + ([cv2.SOLVEPNP_IPPE] if planar else [])
        candidates = []
        for flag in flags:
            try:
                solved, rotations, translations, *_ = cv2.solvePnPGeneric(
                    points, pixels, self.matrix, self.distortion, flags=flag)
            except cv2.error:
                continue
            if not solved:
                continue
            for rotation, translation in zip(rotations, translations):
                try:
                    rotation, translation = cv2.solvePnPRefineLM(
                        points, pixels, self.matrix, self.distortion,
                        rotation.copy(), translation.copy())
                    matrix = cv2.Rodrigues(rotation)[0]
                    offset = translation.reshape(3)
                    camera_points = points @ matrix.T + offset
                    if not np.isfinite(camera_points).all() or np.any(camera_points[:, 2] <= 0):
                        continue
                    if any(np.dot(matrix @ self.normals[tag],
                                  camera_points[4*i:4*i+4].mean(axis=0)) >= 0
                           for i, tag in enumerate(tags) if tag in self.normals):
                        continue
                    projected = cv2.projectPoints(points, rotation, translation,
                                                  self.matrix, self.distortion)[0].reshape(-1, 2)
                    rms = float(np.sqrt(np.mean(np.sum((projected - pixels)**2, axis=1))))
                except cv2.error:
                    continue
                if np.isfinite(rms):
                    pose = np.eye(4)
                    pose[:3, :3], pose[:3, 3] = matrix, offset
                    candidates.append((rms, pose))
        if not candidates:
            raise ValueError("PnP found no valid front-facing cube pose")
        candidates.sort(key=lambda item: item[0])
        rms, pose = candidates[0]
        if rms > self.max_rms:
            raise ValueError(f"Cube reprojection RMS {rms:.2f}px exceeds {self.max_rms:g}px")
        for other_rms, other_pose in candidates[1:]:
            angle = np.linalg.norm(cv2.Rodrigues(pose[:3, :3].T @ other_pose[:3, :3])[0])
            if planar and other_rms - rms < 0.1 and angle > np.deg2rad(5):
                raise ValueError("Ambiguous planar pose; tilt the cube to expose another face")
        return {"T_camera_cube": pose.tolist(), "object_points_m": points.tolist(),
                "image_points_px": pixels.tolist(), "tag_ids": tags,
                "reprojection_rms_px": rms}


def _calib_transform(rotation, translation):
    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = translation
    return matrix


def _calib_unpack(parameters):
    return tuple(
        _calib_transform(Rotation.from_rotvec(parameters[i:i + 3]).as_matrix(),
                         parameters[i + 3:i + 6])
        for i in (0, 6)
    )


def _calib_pack(*matrices):
    return np.concatenate([
        np.r_[Rotation.from_matrix(matrix[:3, :3]).as_rotvec(), matrix[:3, 3]]
        for matrix in matrices
    ])


def _calib_validate_pose(value, name):
    matrix = np.asarray(value, dtype=float)
    if (matrix.shape != (4, 4) or not np.isfinite(matrix).all()
            or not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-6)
            or not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-4)
            or not np.isclose(np.linalg.det(matrix[:3, :3]), 1, atol=1e-4)):
        raise ValueError(f"{name} must be a finite rigid 4x4 transform.")
    return matrix


def _calib_prepare(dataset):
    camera = dataset["camera"]
    K = np.asarray(camera["camera_matrix"], dtype=float)
    D = np.asarray(camera["dist_coeffs"], dtype=float).reshape(-1)
    if (K.shape != (3, 3) or D.shape != (5,) or not np.isfinite(K).all()
            or not np.isfinite(D).all() or K[0, 0] <= 0 or K[1, 1] <= 0
            or not np.allclose(K[2], [0, 0, 1])):
        raise ValueError("Expected a valid camera matrix and five distortion coefficients.")
    views = []
    for index, view in enumerate(dataset["views"]):
        points = np.asarray(view["object_points_m"], dtype=float)
        pixels = np.asarray(view["image_points_px"], dtype=float)
        if (points.ndim != 2 or points.shape[1] != 3 or len(points) < 4
                or pixels.shape != (len(points), 2)
                or not np.isfinite(points).all() or not np.isfinite(pixels).all()):
            raise ValueError(f"View {index}: expected at least four matching 3D/2D points.")
        views.append({
            "index": index,
            "G": _calib_validate_pose(view["T_base_ee"], f"View {index} T_base_ee"),
            "C": _calib_validate_pose(view["T_camera_cube"], f"View {index} T_camera_cube"),
            "points": points,
            "pixels": pixels,
        })
    return K, D, views


def _calib_check_motion(views, minimum_distinct=12):
    distinct = []
    for view in views:
        pose = view["G"]
        if all(np.linalg.norm(pose[:3, 3] - old[:3, 3]) > 0.002
               or Rotation.from_matrix(pose[:3, :3] @ old[:3, :3].T).magnitude()
               > np.deg2rad(2) for old in distinct):
            distinct.append(pose)
    if len(distinct) < minimum_distinct:
        raise ValueError(f"Need at least {minimum_distinct} distinct poses; got {len(distinct)}. "
                         "Move at least 2 mm or rotate at least 2 degrees between captures.")
    # Pairwise rotation axes expose the unconstrained mount translation in single-axis data.
    rotation_vectors = [
        Rotation.from_matrix(a[:3, :3].T @ b[:3, :3]).as_rotvec()
        for i, a in enumerate(distinct) for b in distinct[i + 1:]
    ]
    singular = np.linalg.svd(rotation_vectors, compute_uv=False) / np.sqrt(len(rotation_vectors))
    if singular[0] < np.deg2rad(15) or singular[1] < np.deg2rad(5):
        raise ValueError("Insufficient rotation diversity: tilt the cube about at least two "
                         "nonparallel axes (roughly 20–40 degrees each), then recapture.")
    return {"distinct_poses": len(distinct),
            "rotation_excitation_deg": np.rad2deg(singular).tolist()}


def _calib_initialize(views):
    # OpenCV solves A_i X C_i = constant. With A_i = inverse(T_base_ee),
    # X is T_base_camera and the constant is the unknown T_ee_cube.
    inverse_robot = [np.linalg.inv(view["G"]) for view in views]
    if callable(getattr(cv2, "calibrateHandEye", None)):
        rotation, translation = cv2.calibrateHandEye(
            [pose[:3, :3] for pose in inverse_robot],
            [pose[:3, 3] for pose in inverse_robot],
            [view["C"][:3, :3] for view in views],
            [view["C"][:3, 3] for view in views],
            method=cv2.CALIB_HAND_EYE_PARK,
        )
    else:
        # OpenCV 5 wheels may omit calibrateHandEye. Park's AX=XB solve uses
        # log(R_A) = R_X log(R_B), followed by a linear translation solve.
        inverse_cube = [np.linalg.inv(view["C"]) for view in views]
        pairs = [(views[j]["G"] @ inverse_robot[i], views[j]["C"] @ inverse_cube[i])
                 for i in range(len(views)) for j in range(i + 1, len(views))]
        alpha = np.array([Rotation.from_matrix(A[:3, :3]).as_rotvec() for A, _ in pairs])
        beta = np.array([Rotation.from_matrix(B[:3, :3]).as_rotvec() for _, B in pairs])
        if not pairs or np.linalg.matrix_rank(alpha) < 2 or np.linalg.matrix_rank(beta) < 2:
            raise ValueError("Hand-eye initialization needs rotations about multiple axes.")
        rotation = Rotation.align_vectors(alpha, beta)[0].as_matrix()
        system = np.vstack([A[:3, :3] - np.eye(3) for A, _ in pairs])
        rhs = np.concatenate([rotation @ B[:3, 3] - A[:3, 3] for A, B in pairs])
        translation, _, rank, _ = np.linalg.lstsq(system, rhs, rcond=None)
        if rank < 3:
            raise ValueError("Hand-eye translation is unconstrained; collect more varied orientations.")
    X = _calib_transform(rotation, np.asarray(translation).reshape(3))
    if not np.isfinite(X).all():
        raise ValueError("Hand-eye initialization failed; collect more varied cube orientations.")
    mounts = [inverse @ X @ view["C"] for inverse, view in zip(inverse_robot, views)]
    Y = _calib_transform(
        Rotation.from_matrix(np.array([pose[:3, :3] for pose in mounts])).mean().as_matrix(),
        np.mean([pose[:3, 3] for pose in mounts], axis=0),
    )
    return _calib_pack(X, Y)


def _calib_errors(parameters, views, K, D, with_depth_penalty=False):
    X, Y = _calib_unpack(parameters)
    inverse_camera = np.linalg.inv(X)
    residuals = []
    for view in views:
        camera_cube = inverse_camera @ view["G"] @ Y
        projected, _ = cv2.projectPoints(
            view["points"], Rotation.from_matrix(camera_cube[:3, :3]).as_rotvec(),
            camera_cube[:3, 3], K, D,
        )
        residuals.append((projected.reshape(-1, 2) - view["pixels"]).ravel())
        if with_depth_penalty:
            depths = (view["points"] @ camera_cube[:3, :3].T + camera_cube[:3, 3])[:, 2]
            residuals.append(10000 * np.minimum(depths - 0.01, 0))
    return np.concatenate(residuals)


def _calib_solve(views, K, D, initial=None):
    result = least_squares(
        _calib_errors, _calib_initialize(views) if initial is None else initial,
        args=(views, K, D, True), loss="soft_l1", f_scale=2.0,
        x_scale="jac", max_nfev=500, ftol=1e-10, xtol=1e-10, gtol=1e-8,
    )
    if not result.success or not np.isfinite(result.x).all():
        raise ValueError(f"Calibration optimization failed: {result.message}")
    X, Y = _calib_unpack(result.x)
    for view in views:
        C = np.linalg.inv(X) @ view["G"] @ Y
        if np.any((view["points"] @ C[:3, :3].T + C[:3, 3])[:, 2] <= 0):
            raise ValueError("Calibration places target points behind the camera; inspect detections.")
    return result


def _calib_diagnostics(parameters, views, K, D):
    errors = []
    per_view = []
    for view in views:
        distances = np.linalg.norm(_calib_errors(parameters, [view], K, D).reshape(-1, 2), axis=1)
        errors.extend(distances)
        per_view.append({"view_index": view["index"], "point_count": len(distances),
                         "rms_px": float(np.sqrt(np.mean(distances ** 2)))})
    errors = np.asarray(errors)
    return {"view_count": len(views), "point_count": len(errors),
            "rms_px": float(np.sqrt(np.mean(errors ** 2))),
            "median_px": float(np.median(errors)), "p95_px": float(np.percentile(errors, 95)),
            "max_px": float(np.max(errors)), "per_view": per_view}


def fit_calibration(dataset):
    """Fit X=T_base_camera and Y=T_ee_cube such that G_i Y = X C_i.

    Input coordinates use meters. Intrinsics stay fixed. Every fifth view is
    reserved to measure prediction error from a fit that has not seen those
    observations; returned transforms are subsequently fitted on every view.
    """
    K, D, views = _calib_prepare(dataset)
    motion = _calib_check_motion(views)
    held_out = [view for i, view in enumerate(views) if i % 5 == 4]
    training = [view for i, view in enumerate(views) if i % 5 != 4]
    _calib_check_motion(training, minimum_distinct=8)
    training_fit = _calib_solve(training, K, D)
    held_out_metrics = _calib_diagnostics(training_fit.x, held_out, K, D)
    final_fit = _calib_solve(views, K, D, initial=training_fit.x)
    X, Y = _calib_unpack(final_fit.x)
    singular = np.linalg.svd(final_fit.jac, compute_uv=False)
    if singular[-1] <= singular[0] * 1e-8:
        raise ValueError("Calibration is poorly constrained; collect more varied cube orientations.")
    return {
        "T_base_camera": X.tolist(), "T_camera_base": np.linalg.inv(X).tolist(),
        "T_ee_cube": Y.tolist(), "camera_matrix": K.tolist(), "dist_coeffs": D.tolist(),
        "metrics": {
            "motion": motion,
            "training": _calib_diagnostics(training_fit.x, training, K, D),
            "held_out": held_out_metrics,
            "all_views": _calib_diagnostics(final_fit.x, views, K, D),
            "jacobian_singular_values": singular.tolist(),
            "jacobian_condition_number": float(singular[0] / singular[-1]),
            "optimizer_evaluations": int(final_fit.nfev),
            "intrinsics_refined": False,
        },
    }


def robot_state(controller):
    state = controller.state
    if not -0.05 <= time.time() - state["timestamp"] <= 0.2:
        raise RuntimeError("Robot state is stale; capture stopped.")
    if not all(np.isfinite(state[key]).all() for key in ("qpos", "qvel", "ee")):
        raise RuntimeError("Robot state contains nonfinite values.")
    return state


def capture_settled(camera, controller):
    """Bracket the source frame timestamp with 0.4 s of measured stationarity."""
    states = []
    start = time.monotonic()
    selected = None
    while time.monotonic() - start < 2.0:
        states.append(robot_state(controller))
        image, timing = camera.read()
        state = robot_state(controller)
        states.append(state)
        timestamps = np.asarray([s["timestamp"] for s in states])
        if np.any(np.diff(timestamps) < 0) or np.max(np.diff(timestamps)) > 0.15:
            raise ValueError("Gap in robot measurements during capture; try again.")
        if selected is None and timing["camera_timestamp_s"] - timestamps[0] >= 0.4:
            selected = image, timing, state
        if np.max(np.abs(state["qvel"])) > 0.005:
            raise ValueError("Arm is moving. Hold still, press Space, and stay still for one second.")
        if selected is not None and timestamps[-1] - selected[1]["camera_timestamp_s"] >= 0.4:
            break
    if selected is None or len(states) < 12 or timestamps[-1] - selected[1]["camera_timestamp_s"] < 0.4:
        raise ValueError("Not enough fresh measurements around the image; try again.")
    joints = np.asarray([s["qpos"] for s in states])
    speeds = np.asarray([s["qvel"] for s in states])
    poses = np.asarray([s["ee"] for s in states])
    angles = Rotation.from_matrix(poses[0, :3, :3].T @ poses[:, :3, :3]).magnitude()
    if (np.max(np.ptp(joints, axis=0)) > 0.001 or np.max(np.abs(speeds)) > 0.005
            or np.max(np.linalg.norm(poses[:, :3, 3] - poses[0, :3, 3], axis=1)) > 0.0005
            or np.max(angles) > np.deg2rad(0.15)):
        raise ValueError("Arm moved during capture; sample rejected.")
    image, timing, state = selected
    # PyCAAS timestamps daemon read completion, not sensor exposure. The settled
    # window reduces sensitivity to transport latency; these are not hardware-synced poses.
    if not states[0]["timestamp"] <= timing["camera_timestamp_s"] <= states[-1]["timestamp"]:
        raise ValueError("Frame timestamp falls outside the stationary robot interval.")
    return image, {**timing, "robot_timestamp_s": state["timestamp"],
                   "qpos": state["qpos"].tolist(), "T_base_ee": state["ee"].tolist(),
                   "max_joint_speed_rad_s": float(np.max(np.abs(speeds))),
                   "max_joint_motion_rad": float(np.max(np.ptp(joints, axis=0)))}


WEB_UI_HTML = """<!doctype html>
<html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Camera calibration</title><style>
*{box-sizing:border-box}body{margin:0;background:#10151b;color:#edf3fa;font:16px system-ui,sans-serif}
main{max-width:1500px;margin:auto;padding:24px}header{display:flex;align-items:center;justify-content:space-between;gap:20px}
h1{font-size:25px;margin:0 0 8px}p{color:#a7b6c6;line-height:1.5}.count{white-space:nowrap;color:#90baff}
.camera{background:#080b0f;border-radius:12px;overflow:hidden;min-height:180px;margin:18px 0}
img{display:block;width:100%;height:auto}#status{min-height:28px;overflow-wrap:anywhere}
.actions{display:flex;flex-wrap:wrap;gap:12px;margin:18px 0}button,a{font:inherit;border:0;border-radius:8px;padding:12px 18px}
button{background:#273443;color:#edf3fa;cursor:pointer}button.primary{background:#287bdc}button.stop{background:#653535}
button:disabled{opacity:.4;cursor:default}a{display:inline-block;background:#27663d;color:white;text-decoration:none}
.hint{font-size:14px}[hidden]{display:none!important}.result-grid{display:grid;grid-template-columns:minmax(350px,1fr) 2fr;gap:20px;margin-top:20px}
pre{font-size:14px;overflow:auto;padding:18px;border-radius:8px;background:#080b0f;line-height:1.8}h2{font-size:18px}
@media(max-width:850px){main{padding:16px}header{align-items:flex-start}.result-grid{grid-template-columns:1fr}}
</style><main><header><div><h1>Camera calibration</h1><p>PyCAAS RealSense · robot-held AprilCube</p></div>
<div class="count" id="count">0 samples</div></header><div class="camera"><img id="frame" alt="Waiting for camera image"></div>
<div id="status" role="status">Connecting…</div><div class="actions">
<button class="primary" id="capture" disabled>Capture · Space</button><button id="finish" disabled>Finish &amp; fit · Enter</button>
<button class="stop" id="quit">Stop · Q</button></div>
<a id="download" href="/calibration.json" download="calibration.json" hidden>Download calibration</a>
<section id="results" hidden><div class="result-grid"><div><h2>Camera → robot base</h2>
<p class="hint">T_base_camera · translation in meters</p><pre id="matrix"></pre></div>
<div id="verification" hidden><h2>Recorded image / MuJoCo / overlays</h2><img id="overlayImage" alt="Recorded image and MuJoCo comparison">
<p><a href="/verification/" target="_blank">Inspect all recorded views</a>
<a href="/verification/calibration_summary.jpg" download>Download overlay + matrix</a></p></div></div></section>
<p class="hint" id="hint">Hold the arm still before capturing and keep it still for one second. Capture 15–25 varied poses,
including wrist rotations about at least two axes. Keep the camera and cube mounting fixed.</p></main><script>
const el=id=>document.getElementById(id);let sending=false,latest={busy:true},closed=false;
function render(s){latest=s;el('status').textContent=s.status;el('count').textContent=s.sample_count+' samples';
const disabled=s.busy||sending||s.preview_only||s.result_ready;
el('capture').disabled=disabled;el('finish').disabled=disabled;el('download').hidden=!s.result_ready;
el('results').hidden=!s.result_ready;document.querySelector('.camera').hidden=s.result_ready;
if(s.T_base_camera)el('matrix').textContent=s.T_base_camera.map(row=>'[ '+row.map(x=>x.toFixed(6).padStart(10)).join(' ')+' ]').join(String.fromCharCode(10));
el('verification').hidden=!s.verification_ready;
if(s.overlay_url&&el('overlayImage').dataset.src!==s.overlay_url){el('overlayImage').dataset.src=s.overlay_url;el('overlayImage').src=s.overlay_url;}
if(s.preview_only)el('hint').textContent='Preview only. Check that the cube markers are visible; start calibrate to collect robot poses.';}
async function status(){if(closed)return;try{const r=await fetch('/status',{cache:'no-store'});render(await r.json());}
catch(e){el('status').textContent='Disconnected. Check the calibration terminal.';}setTimeout(status,250);}
async function action(name){if(name!=='quit'&&(sending||latest.busy||latest.preview_only||latest.result_ready))return;
sending=true;render(latest);try{const r=await fetch('/action',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({action:name})});
const answer=await r.json();if(!r.ok)el('status').textContent=answer.error;
if(r.ok&&name==='quit'){closed=true;el('status').textContent='Stopping; saved samples are retained.';}}
catch(e){el('status').textContent='Connection lost. Check the calibration terminal.';}finally{sending=false;}}
for(const name of ['capture','finish','quit'])el(name).onclick=()=>action(name);
document.addEventListener('keydown',e=>{if(e.repeat||e.ctrlKey||e.metaKey||e.altKey||/INPUT|TEXTAREA|SELECT/.test(e.target.tagName)||e.target.isContentEditable)return;
const name=e.code==='Space'?'capture':e.key==='Enter'?'finish':e.key.toLowerCase()==='q'?'quit':null;if(name){e.preventDefault();action(name);}});
function frame(){if(!closed)el('frame').src='/frame.jpg?t='+Date.now();}
el('frame').onload=()=>setTimeout(frame,120);el('frame').onerror=()=>setTimeout(frame,500);frame();status();
</script></html>"""


class WebUI:
    """Serve cached preview frames; HTTP clients never access robot/camera APIs."""

    def __init__(self, host="0.0.0.0", port=8080, preview_only=False):
        self.host, self.port = host, port
        self._lock = threading.Lock()
        self._actions = queue.Queue(maxsize=1)
        self._jpeg = self._result = None
        self._verification = None
        self._pending = False
        self._state = {"status": "Starting camera…", "sample_count": 0, "busy": True,
                       "preview_only": preview_only, "result_ready": False, "verification_ready": False}

    def __enter__(self):
        ui = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *_):
                pass

            def send(self, code, mime, body):
                try:
                    self.send_response(code)
                    self.send_header("Content-Type", mime)
                    self.send_header("Content-Length", str(len(body)))
                    self.send_header("Cache-Control", "no-store")
                    self.end_headers()
                    self.wfile.write(body)
                except (BrokenPipeError, ConnectionResetError):
                    pass

            def do_GET(self):
                route = self.path.split("?", 1)[0]
                with ui._lock:
                    jpeg, result, state = ui._jpeg, ui._result, dict(ui._state)
                    verification = ui._verification
                if route == "/":
                    self.send(200, "text/html; charset=utf-8", WEB_UI_HTML.encode())
                elif route == "/status":
                    self.send(200, "application/json", json.dumps(state).encode())
                elif route == "/frame.jpg" and jpeg is not None:
                    self.send(200, "image/jpeg", jpeg)
                elif route == "/calibration.json" and result is not None:
                    self.send(200, "application/json", result)
                elif route.startswith("/verification/") and verification is not None:
                    relative = unquote(route[len("/verification/"):]) or "index.html"
                    path = (verification / relative).resolve()
                    if path.is_relative_to(verification) and path.is_file():
                        mime = mimetypes.guess_type(path.name)[0] or "application/octet-stream"
                        self.send(200, mime, path.read_bytes())
                    else:
                        self.send(404, "text/plain", b"Not available")
                else:
                    self.send(404, "text/plain", b"Not available")

            def do_POST(self):
                if self.path != "/action":
                    self.send(404, "application/json", b'{"error":"Unknown endpoint"}')
                    return
                try:
                    length = int(self.headers.get("Content-Length", "0"))
                    if not 0 < length <= 1024:
                        raise ValueError("Invalid action request")
                    action = json.loads(self.rfile.read(length))["action"]
                    if action not in ("capture", "finish", "quit"):
                        raise ValueError("Unknown action")
                    with ui._lock:
                        blocked = action != "quit" and (ui._state["busy"] or ui._state["preview_only"]
                                                        or ui._state["result_ready"])
                        if not blocked:
                            if action == "quit":  # Stop takes priority over a queued capture.
                                try:
                                    ui._actions.get_nowait()
                                except queue.Empty:
                                    pass
                            ui._actions.put_nowait(action)
                            ui._pending = True
                            ui._state["busy"] = True
                    self.send(409 if blocked else 202, "application/json",
                              b'{"error":"Controls are busy or unavailable"}' if blocked else b'{"accepted":true}')
                except (ValueError, KeyError, TypeError, queue.Full):
                    self.send(400, "application/json", b'{"error":"Invalid or already queued action"}')

        self.server = ThreadingHTTPServer((self.host, self.port), Handler)
        self.server.daemon_threads = True
        self.port = self.server.server_address[1]
        self._thread = threading.Thread(target=self.server.serve_forever,
                                        kwargs={"poll_interval": 0.1}, daemon=True)
        self._thread.start()
        return self

    def update(self, image, status, sample_count=0, busy=None):
        ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if not ok:
            raise RuntimeError("Could not encode camera preview")
        with self._lock:
            self._jpeg = encoded.tobytes()
            self._state.update(status=status, sample_count=sample_count)
            if busy is not None:
                self._state["busy"] = busy or self._pending

    def set_status(self, status, busy=False):
        with self._lock:
            self._state.update(status=status, busy=busy or self._pending)

    def set_result(self, result, status):
        payload = (json.dumps(result, indent=2, allow_nan=False) + "\n").encode()
        with self._lock:
            self._result = payload
            self._state.update(status=status, busy=self._pending, result_ready=True,
                               T_base_camera=result["T_base_camera"])

    def set_verification(self, directory):
        directory = Path(directory).resolve()
        report = json.loads((directory / "report.json").read_text())
        selected = report["summary"]["view_index"]
        comparison = report["views"][selected]["files"]["comparison"]
        with self._lock:
            self._verification = directory
            self._state.update(verification_ready=True, overlay_url=f"/verification/{comparison}")

    def poll_action(self):
        with self._lock:
            try:
                action = self._actions.get_nowait()
            except queue.Empty:
                return None
            self._pending = False
            return action

    def __exit__(self, *_):
        self.server.shutdown()
        self.server.server_close()
        self._thread.join(timeout=1)


def annotate_frame(image, detection):
    display = image.copy()
    if detection is not None:
        for tag, corners in zip(detection["tag_ids"], np.asarray(detection["image_points_px"]).reshape(-1, 4, 2)):
            cv2.polylines(display, [np.round(corners).astype(np.int32)], True, (0, 220, 0), 2)
            cv2.putText(display, str(tag), tuple(np.round(corners[0]).astype(int)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 220, 0), 1)
    scale = min(1.0, 1280 / display.shape[1])
    return cv2.resize(display, None, fx=scale, fy=scale)


def collect_samples(args, ui):
    """Return a session to fit only after camera and robot resources are closed."""
    config_path = args.cube_config.expanduser().resolve()
    if config_path.is_dir():
        config_path /= "config.json"
    if not config_path.is_file():
        raise ValueError(f"Cube config missing: {config_path}. Pass --cube-config for your actual printed cube.")
    config_text = config_path.read_text()
    controller = None
    dataset = None
    output = None
    finish = False
    try:
        with PycaasCamera(args) as camera:
            detector = CubeDetector(config_path, camera.metadata)
            image, _ = camera.read()
            ui.update(image, "Camera ready", busy=True)
            if args.command == "calibrate":
                import aiofranka
                from aiofranka import FrankaRemoteController

                output = args.output or ROOT / "camera_calibration" / datetime.now().strftime("%Y%m%d_%H%M%S")
                output = output.expanduser().resolve()
                output.mkdir(parents=True, exist_ok=False)
                (output / "images").mkdir()
                (output / "cube_config.json").write_text(config_text)
                model = Path(aiofranka.__file__).parent / "model/fr3.xml"
                dataset = {
                    "schema_version": 1, "setup": "fixed_camera_robot_held_cube",
                    "created_at": datetime.now(timezone.utc).isoformat(),
                    "transform_convention": "T_A_B maps B to A; meters; optical camera right/down/forward",
                    "camera": camera.metadata, "robot_ip": args.robot_ip,
                    "ee_frame": "aiofranka state['ee']: MuJoCo attachment_site",
                    "robot_model_sha256": hashlib.sha256(model.read_bytes()).hexdigest(),
                    "cube_config": json.loads(config_text),
                    "cube_config_sha256": hashlib.sha256(config_text.encode()).hexdigest(),
                    "views": [],
                }
                write_json(output / "views.json", dataset)
                print(f"Session: {output}\nStarting damped gravity compensation. Move the arm by hand.", flush=True)
                controller = FrankaRemoteController(args.robot_ip, home=False)
                controller.start()
                controller.kd = np.full(7, args.damping)
                controller.ki = np.zeros(7)
                controller.kp = np.zeros(7)
                controller.switch("impedance")
            message = "Preview only" if dataset is None else "0 samples; rotate the wrist about multiple axes"
            ui.set_status(message, busy=False)
            while True:
                image, _ = camera.read()
                if controller is not None:
                    robot_state(controller)  # Fail promptly if the control server stops.
                try:
                    detection = detector.detect(image)
                    status = f"{len(detection['tag_ids'])} tags, {detection['reprojection_rms_px']:.2f}px | {message}"
                except ValueError as exc:
                    detection = None
                    status = f"{exc} | {message}"
                ui.update(annotate_frame(image, detection), status,
                          sample_count=len(dataset["views"]) if dataset else 0)
                action = ui.poll_action()
                if action == "quit":
                    break
                if dataset is None:
                    continue
                if action == "finish":
                    try:
                        _, _, views = _calib_prepare(dataset)
                        _calib_check_motion(views)
                        _calib_check_motion([v for i, v in enumerate(views) if i % 5 != 4], minimum_distinct=8)
                    except ValueError as exc:
                        message = str(exc)
                        ui.set_status(message, busy=False)
                        print(message, flush=True)
                        continue
                    finish = True
                    break
                if action != "capture":
                    continue
                try:
                    ui.set_status("Hold still for one second...", busy=True)
                    print("Hold still...", flush=True)
                    image, view = capture_settled(camera, controller)
                    view.update(detector.detect(image))
                    pose = np.asarray(view["T_base_ee"])
                    for old in dataset["views"]:
                        previous = np.asarray(old["T_base_ee"])
                        angle = Rotation.from_matrix(previous[:3, :3].T @ pose[:3, :3]).magnitude()
                        if np.linalg.norm(previous[:3, 3] - pose[:3, 3]) < 0.005 and angle < np.deg2rad(3):
                            raise ValueError("Pose already captured; move or rotate the cube farther.")
                except ValueError as exc:
                    message = str(exc)
                    ui.set_status(message, busy=False)
                    print(f"Rejected: {message}", flush=True)
                    continue
                view["id"] = len(dataset["views"])
                view["image"] = f"images/{view['id']:04d}.png"
                if not cv2.imwrite(str(output / view["image"]), image):
                    raise RuntimeError("Could not save the raw calibration image.")
                dataset["views"].append(view)
                write_json(output / "views.json", dataset)
                message = f"Saved {len(dataset['views'])} samples"
                ui.update(annotate_frame(image, view), message, len(dataset["views"]), busy=False)
                print(message, flush=True)
    finally:
        try:
            if controller is not None:
                controller.stop()
        finally:
            if output is not None:
                print(f"Samples preserved in {output}", flush=True)
    return output if finish else None


def web_view_url(host, port):
    """Use the SSH server address or the default route for a wildcard bind."""
    if host == "0.0.0.0":
        connection = os.environ.get("SSH_CONNECTION", "").split()
        address = connection[2] if len(connection) == 4 else ""
        try:
            socket.inet_pton(socket.AF_INET, address)
        except OSError:
            address = ""
        if address == "0.0.0.0" or address.startswith("127."):
            address = ""
        if not address:
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
                    # Select the local interface from the routing table; no packets are sent.
                    probe.connect(("192.0.2.1", 9))
                    address = probe.getsockname()[0]
            except OSError:
                address = socket.gethostname()
        host = address
    return f"http://{host}:{port}/"


def collect(args):
    with WebUI(args.host, args.port, preview_only=args.command == "preview") as ui:
        print(f"\nCalibration web view (listening on {args.host}):\n"
              f"{web_view_url(args.host, ui.port)}\n", flush=True)
        code = 0
        try:
            session = collect_samples(args, ui)
            if session is None:
                return 0
            ui.set_status("Collection finished; robot stopped. Fitting calibration...", busy=True)
            output, result = save_fit(session / "views.json", session / "calibration.json", False)
            rms = result["metrics"]["held_out"]["rms_px"]
            status = f"Saved {output}. Held-out RMS: {rms:.3f} px."
            if rms > 2.0:
                status += " Error exceeds 2 px; inspect this fit before using it."
            ui.set_result(result, status)
            if not args.no_overlay:
                try:
                    verification = generate_overlays(args, session, output, ui)
                    ui.set_verification(verification)
                    ui.set_status(status + " MuJoCo overlays are ready.", busy=False)
                except (OSError, RuntimeError, ValueError, KeyError) as exc:
                    code = 1
                    message = f"{status} Overlay rendering failed: {exc}. The calibration is still available."
                    ui.set_status(message, busy=False)
                    print(message, file=sys.stderr, flush=True)
        except (ImportError, OSError, RuntimeError, ValueError, KeyError, cv2.error) as exc:
            code = 1
            ui.set_status(f"Calibration error: {exc}. Saved samples are retained. Stop to exit.", busy=True)
            print(f"Calibration error: {exc}", file=sys.stderr, flush=True)
        while ui.poll_action() != "quit":
            time.sleep(0.1)
        return code


def generate_overlays(args, session, calibration_path, ui=None):
    """Render in a clean EGL process after collection/fit; preserve a saved fit on failure."""
    session = Path(session).expanduser().resolve()
    if session.is_file():
        session = session.parent
    calibration_path = Path(calibration_path).expanduser().resolve()
    cube_xml = args.cube_xml
    if cube_xml is None:
        config = args.cube_config.expanduser().resolve()
        cube_xml = (config if config.is_dir() else config.parent) / "mujoco/cube.xml"
    folder = "verification" if calibration_path.name == "calibration.json" else f"verification_{calibration_path.stem}"
    output = session / folder
    command = [sys.executable, str(Path(__file__).with_name("13_verify_camera_calibration.py")),
               "--session", str(session), "--calibration", str(calibration_path),
               "--cube-xml", str(cube_xml), "--output", str(output), "--no-serve"]
    environment = os.environ.copy()
    environment.setdefault("MUJOCO_GL", "egl")
    if ui is not None:
        ui.set_status("Calibration saved. Rendering recorded images with MuJoCo...", busy=True)
    print("Generating MuJoCo overlays...", flush=True)
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                               text=True, bufsize=1, env=environment)
    tail = []
    try:
        for line in process.stdout:
            line = line.strip()
            if line:
                tail = (tail + [line])[-8:]
                print(line, flush=True)
                if ui is not None:
                    ui.set_status(f"Calibration saved. {line}", busy=True)
        if process.wait() != 0:
            raise RuntimeError("MuJoCo export failed: " + " | ".join(tail))
    finally:
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        process.stdout.close()
    if not (output / "calibration_summary.jpg").is_file():
        raise RuntimeError("MuJoCo export did not produce calibration_summary.jpg")
    print(f"Overlay + calibration matrix: {output / 'calibration_summary.jpg'}", flush=True)
    return output


def save_fit(session, output, overwrite):
    session = session.expanduser().resolve()
    if session.is_dir():
        session /= "views.json"
    output = output.expanduser().resolve() if output else session.parent / "calibration.json"
    if output == session or output.name in ("views.json", "cube_config.json") or output.suffix != ".json":
        raise ValueError("Fit output must be a separate calibration .json file.")
    if output.exists() and not overwrite:
        raise ValueError(f"{output} exists. Choose --output or use --overwrite.")
    dataset = json.loads(session.read_text())
    if dataset.get("schema_version") != 1 or dataset.get("setup") != "fixed_camera_robot_held_cube":
        raise ValueError("Expected a views.json dataset produced by this script.")
    print(f"Fitting {len(dataset['views'])} samples...", flush=True)
    result = fit_calibration(dataset)
    result.update({"schema_version": 1, "setup": dataset["setup"],
                   "created_at": datetime.now(timezone.utc).isoformat(),
                   "transform_convention": dataset["transform_convention"],
                   "ee_frame": dataset["ee_frame"], "camera": dataset["camera"],
                   "robot_model_sha256": dataset["robot_model_sha256"],
                   "cube_config": dataset["cube_config"],
                   "dataset": str(session), "dataset_sha256": hashlib.sha256(session.read_bytes()).hexdigest()})
    output.parent.mkdir(parents=True, exist_ok=True)
    write_json(output, result)
    print(f"Calibration saved: {output}")
    print("T_base_camera (camera coordinates -> robot base, meters):")
    print(np.array2string(np.asarray(result["T_base_camera"]), precision=6, suppress_small=True))
    for name in ("training", "held_out", "all_views"):
        metric = result["metrics"][name]
        print(f"{name.replace('_', ' ').capitalize()}: {metric['rms_px']:.3f} px RMS ({metric['view_count']} views)")
    if result["metrics"]["held_out"]["rms_px"] > 2.0:
        print("Held-out error exceeds 2 px. Inspect per-view errors and collect better poses before using this fit.")
    return output, result


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("command", choices=("calibrate", "preview", "fit", "list-cameras"),
                        nargs="?", default="calibrate")
    parser.add_argument("--cube-config", type=Path, default=ROOT / "assets/aprilcube/config.json")
    parser.add_argument("--cube-xml", type=Path, help="MuJoCo target model (default: mujoco/cube.xml beside cube config)")
    parser.add_argument("--no-overlay", action="store_true", help="Skip automatic MuJoCo overlay export")
    parser.add_argument("--robot-ip", default="172.16.0.2")
    parser.add_argument("--stream", help="PyCAAS color stream, e.g. rs_908212070730_color")
    parser.add_argument("--serial", help="RealSense serial (alternative to --stream)")
    parser.add_argument("--pycaas-endpoint", default="ipc:///tmp/pycaas.sock", help="Local PyCAAS IPC endpoint")
    parser.add_argument("--width", type=int, help="Require this native width; does not resize/reconfigure PyCAAS")
    parser.add_argument("--height", type=int, help="Require this native height; does not resize/reconfigure PyCAAS")
    parser.add_argument("--fps", type=int, help="Require this FPS if the PyCAAS daemon reports it")
    parser.add_argument("--host", default="0.0.0.0", help="Web UI bind address (default: 0.0.0.0)")
    parser.add_argument("--port", type=int, default=8080, help="Web UI port (default: 8080)")
    parser.add_argument("--damping", type=float, default=2.0, help="Freedrive joint damping, Nm s/rad (default: 2)")
    parser.add_argument("--session", type=Path, help="Saved session directory or views.json, for fit")
    parser.add_argument("--output", type=Path, help="New session directory; for fit, a result .json path")
    parser.add_argument("--overwrite", action="store_true", help="Allow replacing a fitted result (fit only)")
    args = parser.parse_args()
    if any(value is not None and value <= 0 for value in (args.width, args.height, args.fps)) or not np.isfinite(args.damping) or args.damping < 0:
        parser.error("Resolution/FPS must be positive and damping finite and nonnegative.")
    if not 1 <= args.port <= 65535:
        parser.error("--port must be between 1 and 65535")
    if args.command == "fit" and args.session is None:
        parser.error("fit requires --session")
    if args.command != "fit" and (args.session is not None or args.overwrite):
        parser.error("--session and --overwrite apply only to fit")
    cv2.setNumThreads(1)
    try:
        if args.command == "list-cameras":
            list_cameras(args)
        elif args.command == "fit":
            output, _ = save_fit(args.session, args.output, args.overwrite)
            if not args.no_overlay:
                try:
                    generate_overlays(args, args.session, output)
                except (OSError, RuntimeError, ValueError) as exc:
                    raise RuntimeError(f"Calibration saved to {output}; overlay rendering failed: {exc}") from exc
        else:
            return collect(args)
    except KeyboardInterrupt:
        print("\nCalibration interrupted; saved samples are retained.")
        return 130
    except (ImportError, OSError, RuntimeError, ValueError, KeyError, cv2.error) as exc:
        print(f"Calibration error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
