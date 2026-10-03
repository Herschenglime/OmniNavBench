"""PhysX Raycast 2D LiDAR implementation for OmniNavBench."""

from __future__ import annotations

from collections import OrderedDict
import math
from typing import Any, Dict, Optional, Tuple

import numpy as np

from OmniNav.core.robot.robot import BaseRobot
from OmniNav.core.scene.scene import IScene
from OmniNav.core.sensor.sensor import BaseSensor
from OmniNavExt.configs.sensors import PhysXRaycastLidarCfg


def _quat_to_rot_matrix(w: float, x: float, y: float, z: float) -> np.ndarray:
    """Convert quaternion (w, x, y, z) to 3x3 rotation matrix."""
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm > 0:
        w, x, y, z = w / norm, x / norm, y / norm, z / norm
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ], dtype=np.float64)


def _quat_to_yaw(w: float, x: float, y: float, z: float) -> float:
    """Extract yaw angle around Z axis from quaternion (w, x, y, z)."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _parse_raycast_result(hit: Any) -> Tuple[bool, Optional[float]]:
    """Parse PhysX scene query raycast hit result."""
    if hit is None:
        return False, None
    if isinstance(hit, dict):
        ok = bool(hit.get("hit") or hit.get("hasHit") or hit.get("is_hit"))
        dist = hit.get("distance")
        return ok, None if dist is None else float(dist)
    if isinstance(hit, (list, tuple)) and hit:
        ok = bool(hit[0])
        dist = None
        if len(hit) >= 2:
            try:
                dist = float(hit[1])
            except Exception:
                dist = None
        return ok, dist
    return False, None


@BaseSensor.register("PhysXRaycastLidar")
@BaseSensor.register("Lidar2D")
class PhysXRaycastLidar(BaseSensor):
    """2D planar LiDAR sensor simulated using PhysX raycasting.

    Performs radial raycasts in the horizontal plane around the sensor origin,
    producing range readings compatible with standard 2D laser scans.
    """

    def __init__(self, config: PhysXRaycastLidarCfg, robot: BaseRobot, scene: Optional[IScene] = None):
        super().__init__(config, robot, scene)
        self.config: PhysXRaycastLidarCfg = config

        self._carb = None
        self._physx_query = None

        # Precompute ray angles in sensor local frame (radians)
        fov_deg = float(self.config.horizontal_fov_deg)
        fov_rad = math.radians(fov_deg)
        num_rays = max(1, int(self.config.num_rays))
        endpoint = False if fov_deg >= 360.0 else True

        self._angles = np.linspace(-fov_rad / 2.0, fov_rad / 2.0, num_rays, endpoint=endpoint, dtype=np.float32)

    def post_reset(self):
        """Initialize PhysX query interface after simulation resets."""
        super().post_reset()
        try:
            import carb  # type: ignore
            self._carb = carb
        except Exception:
            self._carb = None

        try:
            import omni.physx  # type: ignore
            self._physx_query = omni.physx.get_physx_scene_query_interface()
        except Exception:
            self._physx_query = None

    def restore_sensor_info(self):
        self.post_reset()

    def cleanup(self):
        self._physx_query = None
        self._carb = None

    def get_data(self) -> OrderedDict:
        """Execute radial raycasts and return range measurements.

        Returns:
            OrderedDict containing:
                - 'ranges': np.ndarray of shape (num_rays,) with range readings (meters).
                - 'angles': np.ndarray of shape (num_rays,) with ray angles (radians).
                - 'min_range_m': near detection limit.
                - 'max_range_m': far detection limit.
                - 'fov_deg': scan field of view in degrees.
                - 'num_rays': total number of beams.
                - 'sensor_pose': dictionary containing world position and yaw.
        """
        num_rays = len(self._angles)
        min_range = float(self.config.min_range_m)
        max_range = float(self.config.max_range_m)

        # 1. Determine sensor origin and orientation in world coordinates
        robot_pos = (0.0, 0.0, 0.0)
        robot_quat = (1.0, 0.0, 0.0, 0.0)
        try:
            pos, quat = self._robot.get_pose()
            robot_pos = tuple(pos.tolist() if isinstance(pos, np.ndarray) else pos)
            robot_quat = tuple(quat.tolist() if isinstance(quat, np.ndarray) else quat)
        except Exception:
            if hasattr(self._robot, "articulation") and self._robot.articulation is not None:
                try:
                    pos, quat = self._robot.articulation.get_world_poses()
                    robot_pos = tuple(pos[0].tolist())
                    robot_quat = tuple(quat[0].tolist())
                except Exception:
                    pass

        rw, rx, ry, rz = robot_quat
        r_rot = _quat_to_rot_matrix(rw, rx, ry, rz)
        robot_yaw = _quat_to_yaw(rw, rx, ry, rz)

        # Apply sensor mount translation relative to robot base
        mount_trans = np.array(self.config.translation or (0.0, 0.0, 0.0), dtype=np.float64)
        if self.config.vertical_offset_m:
            mount_trans[2] += float(self.config.vertical_offset_m)

        sensor_origin_world = np.array(robot_pos, dtype=np.float64) + r_rot.dot(mount_trans)

        # Sensor relative rotation
        sensor_yaw_offset = 0.0
        if self.config.orientation is not None:
            sw, sx, sy, sz = self.config.orientation
            sensor_yaw_offset = _quat_to_yaw(sw, sx, sy, sz)

        total_sensor_yaw = robot_yaw + sensor_yaw_offset

        # 2. Perform PhysX raycasting
        ranges = np.full(num_rays, max_range, dtype=np.float32)

        if self._physx_query is not None and self._carb is not None:
            origin_carb = self._carb.Float3(
                float(sensor_origin_world[0]),
                float(sensor_origin_world[1]),
                float(sensor_origin_world[2]),
            )

            for i, local_angle in enumerate(self._angles):
                global_angle = total_sensor_yaw + float(local_angle)
                dir_x = math.cos(global_angle)
                dir_y = math.sin(global_angle)
                ray_dir = self._carb.Float3(dir_x, dir_y, 0.0)

                hit = self._physx_query.raycast_closest(origin_carb, ray_dir, max_range)
                hit_ok, dist = _parse_raycast_result(hit)
                if hit_ok and dist is not None:
                    if dist >= min_range:
                        ranges[i] = dist
                    else:
                        ranges[i] = min_range

            # Optional Gaussian noise
            if self.config.noise_std and float(self.config.noise_std) > 0.0:
                noise = np.random.normal(0.0, float(self.config.noise_std), size=num_rays).astype(np.float32)
                ranges = np.clip(ranges + noise, min_range, max_range)

        data = {
            "ranges": ranges,
            "angles": self._angles.copy(),
            "min_range_m": min_range,
            "max_range_m": max_range,
            "fov_deg": float(self.config.horizontal_fov_deg),
            "num_rays": num_rays,
            "sensor_pose": {
                "position": [float(v) for v in sensor_origin_world],
                "yaw": float(total_sensor_yaw),
            },
        }

        return self._make_ordered(data)
