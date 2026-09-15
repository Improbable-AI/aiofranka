"""Geometry for a flat AprilCube T target and the Pocky tool; no hardware access."""

from itertools import product
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import cv2
import numpy as np


def rigid_transform(value):
    pose = np.asarray(value, dtype=float)
    if (pose.shape != (4, 4) or not np.isfinite(pose).all()
            or not np.allclose(pose[3], [0, 0, 0, 1], atol=1e-8)
            or not np.allclose(pose[:3, :3].T @ pose[:3, :3], np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(pose[:3, :3]), 1, atol=1e-6)):
        raise ValueError("Expected a finite rigid 4x4 object transform")
    return pose.copy()


class TShape:
    """Exact voxel-union footprint in local XZ, with thickness along local Y."""

    def __init__(self, config_path):
        config = json.loads(Path(config_path).read_text())
        target = config["target"]
        if target.get("type") != "voxel_cuboids":
            raise ValueError("Expected an AprilCube voxel_cuboids target config")
        self.voxel_size_m = float(target["voxel_size_mm"]) * 0.001
        if not np.isfinite(self.voxel_size_m) or self.voxel_size_m <= 0:
            raise ValueError("Voxel size must be positive and finite")
        occupied = set()
        for cuboid in target["cuboids"]:
            origin, size = np.asarray(cuboid["origin"]), np.asarray(cuboid["size"])
            if (origin.shape != (3,) or size.shape != (3,) or not np.isfinite(origin).all()
                    or not np.isfinite(size).all() or np.any(origin != origin.astype(int))
                    or np.any(size != size.astype(int)) or np.any(size <= 0)):
                raise ValueError("Cuboids need integer voxel origins and positive integer sizes")
            occupied.update(product(*(range(int(o), int(o + s)) for o, s in zip(origin, size))))
        if not occupied:
            raise ValueError("Target contains no occupied voxels")
        self.cells = np.asarray(sorted(occupied), dtype=int)
        minimum, maximum = self.cells.min(axis=0), self.cells.max(axis=0)
        if minimum[1] != maximum[1]:
            raise ValueError("Planar T target must have one voxel layer along local Y")
        dimensions = (maximum - minimum + 1) * self.voxel_size_m
        if not np.allclose(np.asarray(config["box_dims"]) * .001, dimensions):
            raise ValueError("Target box dimensions disagree with its occupied voxels")
        center = (minimum + maximum + 1) / 2
        lower = (self.cells - center) * self.voxel_size_m
        upper = lower + self.voxel_size_m
        self.rectangles_xz = np.array([
            [[a[0], a[2]], [b[0], a[2]], [b[0], b[2]], [a[0], b[2]]]
            for a, b in zip(lower, upper)
        ])
        self.corners_m = np.unique(np.array([
            [a[0] if i == 0 else b[0], a[1] if j == 0 else b[1], a[2] if k == 0 else b[2]]
            for a, b in zip(lower, upper) for i, j, k in product((0, 1), repeat=3)
        ]), axis=0)
        self.area_m2 = len(self.cells) * self.voxel_size_m ** 2
        self.half_thickness_m = dimensions[1] / 2

    def support_height(self, pose):
        """Assume the operator placed the T flat: center Z minus half its thickness."""
        return float(rigid_transform(pose)[2, 3] - self.half_thickness_m)

    def footprint(self, pose):
        """Return disjoint convex voxel polygons projected into robot-base XY."""
        pose = rigid_transform(pose)
        points = np.zeros((*self.rectangles_xz.shape[:2], 3))
        points[:, :, 0], points[:, :, 2] = self.rectangles_xz[:, :, 0], self.rectangles_xz[:, :, 1]
        return (points @ pose[:3, :3].T + pose[:3, 3])[:, :, :2]

    def overlap(self, goal, actual):
        """Projected XY intersection / goal area, retaining the T's concavity."""
        goal_polygons, actual_polygons = self.footprint(goal), self.footprint(actual)
        # Shift before float32 clipping to preserve precision away from the base origin.
        origin = goal_polygons.reshape(-1, 2).mean(axis=0)
        goal_polygons = np.asarray(goal_polygons - origin, dtype=np.float32)
        actual_polygons = np.asarray(actual_polygons - origin, dtype=np.float32)
        area = sum(cv2.contourArea(polygon) for polygon in goal_polygons)
        intersection = sum(cv2.intersectConvexConvex(a, b, handleNested=True)[0]
                           for a in goal_polygons for b in actual_polygons)
        return float(np.clip(intersection / area, 0, 1))

    def distance_xy(self, pose, xy):
        """Distance in meters outside the exact footprint, or zero inside/on it."""
        xy = np.asarray(xy, dtype=float)
        if xy.shape != (2,) or not np.isfinite(xy).all():
            raise ValueError("XY point must contain two finite coordinates")
        polygons = self.footprint(pose)
        origin = polygons.reshape(-1, 2).mean(axis=0)
        point = tuple((xy - origin).tolist())
        signed = [cv2.pointPolygonTest(np.asarray(polygon - origin, dtype=np.float32), point, True)
                  for polygon in polygons]
        return float(max(0, -max(signed)))

    def fixed_start_xy(self, source_pose, destination_pose, clearance=.075):
        """Fixed point outside the saved source, on its side away from the destination.

        Offset a support vertex by the requested clearance. Every source point
        lies behind its supporting line, so the nearest outline distance is
        exactly the clearance, including for the concave T shape.
        """
        if not np.isfinite(clearance) or clearance <= 0:
            raise ValueError("Start clearance must be positive and finite")
        source = rigid_transform(source_pose)
        destination = rigid_transform(destination_pose)
        direction = source[:2, 3] - destination[:2, 3]
        distance = np.linalg.norm(direction)
        direction = direction / distance if distance >= 1e-9 else np.array([0., -1.])
        vertices = self.footprint(source).reshape(-1, 2)
        boundary = vertices[np.argmax(vertices @ direction)]
        return boundary + clearance * direction

    def sample_start_xy(self, pose, rng, max_distance=.10, min_distance=.05):
        """Sample uniformly by area at min_distance < distance <= max_distance.

        Distance is from the tip's XY point to the nearest part of the exact
        T outline, including its concave sections.
        """
        if not np.isfinite([min_distance, max_distance]).all() or not 0 <= min_distance < max_distance:
            raise ValueError("Start distances must be finite and satisfy 0 <= min < max")
        points = self.footprint(pose).reshape(-1, 2)
        lower, upper = points.min(axis=0) - max_distance, points.max(axis=0) + max_distance
        for _ in range(10000):
            xy = rng.uniform(lower, upper)
            if min_distance < self.distance_xy(pose, xy) <= max_distance:
                return xy
        raise ValueError("Could not sample a point in the requested outside band")


def load_stick_tip(xml_path):
    """Read nominal tip and mesh apex in stick-body coordinates, not a calibrated EE mount."""
    xml_path = Path(xml_path).resolve()
    root = ET.parse(xml_path).getroot()
    body = root.find(".//body[@name='stick']")
    site = body.find("site[@name='stick_tip']")
    visual = body.find("geom[@name='stick_visual']")
    mesh = root.find(f"./asset/mesh[@name='{visual.get('mesh')}']")
    if any(visual.get(name) for name in ("quat", "euler", "axisangle", "xyaxes", "zaxis")):
        raise ValueError("Stick tip loader expects the provided unrotated visual mesh")
    compiler = root.find("compiler")
    mesh_path = xml_path.parent / (compiler.get("meshdir", "") if compiler is not None else "") / mesh.get("file")
    vertices = np.array([[float(x) for x in line.split()[1:4]]
                         for line in mesh_path.read_text().splitlines() if line.startswith("v ")])
    vertices *= np.fromstring(mesh.get("scale", "1 1 1"), sep=" ")
    vertices += np.fromstring(visual.get("pos", "0 0 0"), sep=" ")
    apex = vertices[np.isclose(vertices[:, 2], vertices[:, 2].max(), atol=1e-9, rtol=0)].mean(axis=0)
    nominal = np.fromstring(site.get("pos", "0 0 0"), sep=" ")
    sphere = body.find("geom[@name='stick_collision_sphere']")
    return {"frame": "stick body; mounting transform to attachment_site must be supplied",
            "site_tip_m": nominal.tolist(), "geometry_tip_m": apex.tolist(),
            "discrepancy_m": float(np.linalg.norm(apex - nominal)),
            "contact_radius_m": float(sphere.get("size").split()[0]) if sphere is not None else None,
            "sphere_center_m": np.fromstring(sphere.get("pos", "0 0 0"), sep=" ").tolist() if sphere is not None else None}
