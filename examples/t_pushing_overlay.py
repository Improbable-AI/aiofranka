"""Draw saved AprilCube results offline, without running detection again."""

import json
from pathlib import Path

import cv2
import numpy as np


class AprilCubeOverlay:
    """Use AprilCube's own renderer with the pose and corners recorded per frame."""

    def __init__(self, run_directory: Path):
        import aprilcube

        run_directory = Path(run_directory)
        calibration = json.loads((run_directory / "calibration.json").read_text())
        T_base_camera = np.asarray(calibration["T_base_camera"], dtype=float)
        self.T_camera_base = np.linalg.inv(T_base_camera)
        self.estimator = aprilcube.detector(
            run_directory / "target_config.json",
            np.asarray(calibration["camera_matrix"], dtype=float),
            dist_coeffs=np.asarray(calibration["dist_coeffs"], dtype=float),
            extrinsic=T_base_camera, enable_filter=False)
        self.metadata = {
            "renderer": "aprilcube.CubePoseEstimator.draw_result",
            "detector_rerun": False,
            "pose_source": "Saved native camera pose; saved base pose transformed to camera if needed",
            "overlay_contents": ["Recorded marker corners when available", "Object coordinate axes",
                                 "Native bounding cuboid", "Native tag and reprojection statistics",
                                 "TRACKED / PREDICTED / NO POSE status"],
            "missing_corners": "Omitted and labeled; never inferred or detected again",
        }

    def _restore_result(self, record):
        success = bool(record["valid"])
        T = rvec = tvec = None
        if success:
            native_pose = record.get("aprilcube_T_camera_object_mm")
            if native_pose is not None:
                T = np.asarray(native_pose, dtype=float)
            elif record.get("T_base_object") is not None:
                T = self.T_camera_base @ np.asarray(record["T_base_object"], dtype=float)
                T[:3, 3] *= 1000.  # The native renderer uses millimeters.
            else:
                raise ValueError("Valid camera record has no saved AprilCube pose")
            rvec = cv2.Rodrigues(T[:3, :3])[0]
            tvec = T[:3, 3:4]

        tag_ids = record.get("tag_ids", [])
        faces = record.get("aprilcube_visible_faces")
        if faces is None:
            faces = [face for face, ids in self.estimator.face_id_sets.items()
                     if ids.intersection(tag_ids)]
        error = record.get("reprojection_rms_px")
        return {
            "success": success, "rvec": rvec, "tvec": tvec, "T": T,
            "detections": [(int(tag_id), np.asarray(corners, dtype=float).reshape(4, 2))
                           for tag_id, corners in record.get("aprilcube_detections", [])],
            "visible_faces": set(faces), "tag_ids": tag_ids,
            "n_tags": record.get("n_tags", len(tag_ids)),
            "n_inliers": record.get("n_inliers", 0),
            "reproj_error": float(error) if error is not None else float("inf"),
            "predicted": bool(record.get("predicted", False)),
        }

    def __call__(self, image_bgr, record):
        result = self._restore_result(record)
        overlay = self.estimator.draw_result(image_bgr, result)
        status = ("PREDICTED" if result["predicted"] else "TRACKED") if result["success"] else "NO POSE"
        color = (0, 255, 0) if status == "TRACKED" else (0, 200, 255) if status == "PREDICTED" else (0, 80, 255)
        missing_corners = "aprilcube_detections" not in record
        cv2.rectangle(overlay, (0, 0), (overlay.shape[1] - 1, 92 if missing_corners else 66), (0, 0, 0), -1)
        cv2.putText(overlay, "Recorded AprilCube result", (12, 26), cv2.FONT_HERSHEY_SIMPLEX,
                    .65, (255, 255, 255), 2)
        cv2.putText(overlay, status, (12, 54), cv2.FONT_HERSHEY_SIMPLEX, .65, color, 2)
        if missing_corners:
            cv2.putText(overlay, "Observed tag corners were not recorded", (12, 82),
                        cv2.FONT_HERSHEY_SIMPLEX, .6, (220, 220, 220), 1)
        return overlay
