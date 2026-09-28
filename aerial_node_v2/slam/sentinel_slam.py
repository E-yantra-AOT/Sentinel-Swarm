"""
=============================================================================
  Sentinel Swarm v2 — SLAM & Navigation Engine
  Hardware: Jetson Orin + Intel RealSense D435i + Livox Mid-360
  Framework: Isaac ROS Visual SLAM + LiDAR Map Fusion
=============================================================================

NAVIGATION ARCHITECTURE:
  Phase 1: VIO (Visual-Inertial Odometry)
    - RealSense D435i RGB-D feeds Isaac ROS Visual SLAM
    - IMU data from flight controller fused via Extended Kalman Filter
    - Provides 6-DoF pose at 200Hz: X, Y, Z, Roll, Pitch, Yaw

  Phase 2: LiDAR Dense Mapping
    - Livox Mid-360 provides 360° non-repetitive scanning at 200,000 pts/sec
    - Faster-LIO2 algorithm builds 3D occupancy map in real-time
    - Map used for path planning and obstacle avoidance

  Phase 3: Path Planning
    - RRT* (Rapidly-exploring Random Tree) for initial path to target
    - Dynamic Potential Field for reactive obstacle avoidance mid-flight

GPS-DENIED OPERATION:
  The system bootstraps purely on VIO. When LiDAR is available,
  both maps are fused using a shared voxel grid. The drone maintains
  a full 3D occupancy map of the disaster zone enabling autonomous
  search patterns, loop-closure detection, and safe return-to-base.
"""

import time
import threading
import numpy as np
from dataclasses import dataclass, field
from typing import Optional, Tuple, List
from collections import deque
import heapq

# ---------------------------------------------------------------------------
# Data Structures
# ---------------------------------------------------------------------------

@dataclass
class Pose6DOF:
    """Full 6-Degree-of-Freedom drone pose in world frame."""
    x: float = 0.0    # meters
    y: float = 0.0
    z: float = 0.0
    roll: float  = 0.0  # radians
    pitch: float = 0.0
    yaw: float   = 0.0
    timestamp: float = field(default_factory=time.time)

    def as_matrix(self) -> np.ndarray:
        """Returns 4x4 homogeneous transformation matrix."""
        cr, sr = np.cos(self.roll),  np.sin(self.roll)
        cp, sp = np.cos(self.pitch), np.sin(self.pitch)
        cy, sy = np.cos(self.yaw),   np.sin(self.yaw)
        R = np.array([
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [-sp,   cp*sr,            cp*cr            ]
        ])
        T = np.eye(4)
        T[:3, :3] = R
        T[:3,  3] = [self.x, self.y, self.z]
        return T


@dataclass
class Waypoint:
    x: float
    y: float
    z: float
    hover_time: float = 0.0     # seconds to hover at this point (0 = pass-through)
    priority: int = 0           # Lower = higher priority


# ---------------------------------------------------------------------------
# Extended Kalman Filter for VIO Fusion
# ---------------------------------------------------------------------------

class VIOFusionEKF:
    """
    Extended Kalman Filter fusing RealSense Visual Odometry with
    IMU data from the ESP32 flight controller.
    State vector: [x, y, z, vx, vy, vz, roll, pitch, yaw]
    """
    STATE_DIM  = 9
    OBS_DIM    = 6   # Position + orientation from VIO

    def __init__(self, dt: float = 0.005):  # 200Hz
        self.dt = dt
        # State
        self.X = np.zeros(self.STATE_DIM)
        # Covariance
        self.P = np.eye(self.STATE_DIM) * 0.01
        # Process noise
        self.Q = np.eye(self.STATE_DIM) * 0.001
        # Measurement noise (VIO)
        self.R_vio = np.eye(self.OBS_DIM) * 0.005
        # Measurement noise (IMU)
        self.R_imu = np.eye(3) * 0.01

    def predict(self, accel: np.ndarray, gyro: np.ndarray):
        """Propagate state using IMU readings."""
        # State transition: integrate kinematics
        ax, ay, az = accel
        self.X[0] += self.X[3] * self.dt + 0.5 * ax * self.dt**2
        self.X[1] += self.X[4] * self.dt + 0.5 * ay * self.dt**2
        self.X[2] += self.X[5] * self.dt + 0.5 * az * self.dt**2
        self.X[3] += ax * self.dt
        self.X[4] += ay * self.dt
        self.X[5] += az * self.dt
        self.X[6] += gyro[0] * self.dt
        self.X[7] += gyro[1] * self.dt
        self.X[8] += gyro[2] * self.dt
        # Jacobian (linearized)
        F = np.eye(self.STATE_DIM)
        F[0, 3] = self.dt; F[1, 4] = self.dt; F[2, 5] = self.dt
        self.P = F @ self.P @ F.T + self.Q

    def update_vio(self, vio_pose: np.ndarray):
        """Correct state with VIO measurement [x, y, z, roll, pitch, yaw]."""
        H = np.zeros((self.OBS_DIM, self.STATE_DIM))
        H[0, 0]=1; H[1, 1]=1; H[2, 2]=1
        H[3, 6]=1; H[4, 7]=1; H[5, 8]=1
        y = vio_pose - H @ self.X
        S = H @ self.P @ H.T + self.R_vio
        K = self.P @ H.T @ np.linalg.inv(S)
        self.X = self.X + K @ y
        self.P = (np.eye(self.STATE_DIM) - K @ H) @ self.P

    @property
    def current_pose(self) -> Pose6DOF:
        return Pose6DOF(
            x=self.X[0], y=self.X[1], z=self.X[2],
            roll=self.X[6], pitch=self.X[7], yaw=self.X[8]
        )


# ---------------------------------------------------------------------------
# 3D Voxel Occupancy Map
# ---------------------------------------------------------------------------

class VoxelMap:
    """
    Lightweight 3D occupancy grid built from LiDAR point clouds.
    Voxel size: 0.1m x 0.1m x 0.1m
    """
    VOXEL_SIZE = 0.1   # meters

    def __init__(self, map_size: float = 100.0):
        n = int(map_size / self.VOXEL_SIZE)
        # -1 = never observed, 0 = lidar-cleared, 1 = lidar hit.
        self.grid = np.full((n, n, n), -1, dtype=np.int8)
        self.origin = np.array([map_size/2, map_size/2, map_size/2])
        self._lock = threading.Lock()

    def update(self, points: np.ndarray, drone_pose: Pose6DOF,
               hit_mask: Optional[np.ndarray] = None):
        """Ray-clear observed space and mark returns occupied in world frame."""
        if points.size == 0:
            return
        transform = drone_pose.as_matrix()
        world_points = points @ transform[:3, :3].T + transform[:3, 3]
        sensor = transform[:3, 3]
        if hit_mask is None:
            hit_mask = np.ones(len(world_points), dtype=bool)
        hit_mask = np.asarray(hit_mask, dtype=bool)
        # Carve each measured ray through observed free space. A small lateral
        # dilation covers the angular spacing of the simulated scan.
        free_indices = []
        for endpoint in world_points:
            length = float(np.linalg.norm(endpoint - sensor))
            steps = max(1, int(length / (self.VOXEL_SIZE * 0.75)))
            ray = sensor + (endpoint - sensor) * np.linspace(0.0, 1.0, steps, endpoint=False)[:, None]
            free_indices.append(np.floor((ray + self.origin) / self.VOXEL_SIZE).astype(int))
        free_indices = np.concatenate(free_indices, axis=0)
        shape = np.asarray(self.grid.shape)
        valid = np.all((free_indices >= 0) & (free_indices < shape), axis=1)
        free_indices = np.unique(free_indices[valid], axis=0)
        # Fill angular gaps between adjacent horizontal scan rays (the sensor
        # publishes at 0.5 degrees); retain the same observed height slice.
        if len(free_indices):
            offsets = np.array([[dx, dy, 0] for dx in (-1, 0, 1)
                                for dy in (-1, 0, 1)])
            free_indices = (free_indices[:, None, :] + offsets[None, :, :]).reshape(-1, 3)
            valid = np.all((free_indices >= 0) & (free_indices < shape), axis=1)
            free_indices = np.unique(free_indices[valid], axis=0)
        hit_indices = np.floor((world_points[hit_mask] + self.origin) / self.VOXEL_SIZE).astype(int)
        valid = np.all((hit_indices >= 0) & (hit_indices < shape), axis=1)
        hit_indices = np.unique(hit_indices[valid], axis=0)
        if len(hit_indices):
            offsets = np.array([[dx, dy, dz] for dx in (-1, 0, 1)
                                for dy in (-1, 0, 1) for dz in (-1, 0, 1)])
            hit_indices = (hit_indices[:, None, :] + offsets[None, :, :]).reshape(-1, 3)
            valid = np.all((hit_indices >= 0) & (hit_indices < shape), axis=1)
            hit_indices = np.unique(hit_indices[valid], axis=0)
        with self._lock:
            if len(free_indices):
                self.grid[free_indices[:, 0], free_indices[:, 1], free_indices[:, 2]] = 0
            if len(hit_indices):
                self.grid[hit_indices[:, 0], hit_indices[:, 1], hit_indices[:, 2]] = 1

    def is_occupied(self, xyz: np.ndarray, threshold: int = 10) -> bool:
        idx = np.floor((xyz + self.origin) / self.VOXEL_SIZE).astype(int)
        if np.any(idx < 0) or np.any(idx >= np.asarray(self.grid.shape)):
            return True  # Treat out-of-bounds as occupied (safe)
        # Unknown space is blocked: the planner may use only lidar-cleared voxels.
        return int(self.grid[idx[0], idx[1], idx[2]]) != 0

    def segment_is_free(self, start: np.ndarray, end: np.ndarray) -> bool:
        """Check the whole segment at voxel resolution, not just its endpoint."""
        distance = float(np.linalg.norm(end - start))
        samples = max(2, int(distance / self.VOXEL_SIZE) + 1)
        return not any(
            self.is_occupied(start + (end - start) * t)
            for t in np.linspace(0.0, 1.0, samples)
        )

    def nearby_obstacles(self, xyz: np.ndarray, radius: float = 2.0) -> List[np.ndarray]:
        """Return occupied voxel centers near a world-frame position."""
        center = np.floor((xyz + self.origin) / self.VOXEL_SIZE).astype(int)
        cells = int(np.ceil(radius / self.VOXEL_SIZE))
        low = np.maximum(center - cells, 0)
        high = np.minimum(center + cells + 1, np.asarray(self.grid.shape))
        with self._lock:
            region = self.grid[low[0]:high[0], low[1]:high[1], low[2]:high[2]]
            occupied = np.argwhere(region == 1)
        if not len(occupied):
            return []
        occupied += low
        world = occupied * self.VOXEL_SIZE - self.origin + self.VOXEL_SIZE / 2
        distances = np.linalg.norm(world - xyz, axis=1)
        world = world[distances <= radius]
        # Limit the reactive field cost while preserving nearby surface coverage.
        if len(world) > 256:
            world = world[::int(np.ceil(len(world) / 256))]
        return [point for point in world]


# ---------------------------------------------------------------------------
# Path Planning: RRT* + Dynamic Potential Field
# ---------------------------------------------------------------------------

class PathPlanner:
    """
    Two-stage planner:
      Stage 1: RRT* for global path from current pos to target.
      Stage 2: Dynamic Potential Field for reactive obstacle avoidance.
    """

    def __init__(self, voxel_map: VoxelMap, step_size: float = 0.5,
                 max_iter: int = 2000):
        self.map       = voxel_map
        self.step      = step_size
        self.max_iter  = max_iter

    def _sample_free(self, bounds: np.ndarray) -> Optional[np.ndarray]:
        """Sample a collision-free point within the operating volume."""
        for _ in range(100):
            pt = np.random.uniform(bounds[0], bounds[1])
            if not self.map.is_occupied(pt):
                return pt
        return None

    def rrt_star(self, start: np.ndarray,
                 goal: np.ndarray) -> List[np.ndarray]:
        """Return an RRT* path, or an empty list when no safe path was found."""
        start = np.asarray(start, dtype=float)
        goal = np.asarray(goal, dtype=float)
        if self.map.segment_is_free(start, goal):
            return [start.copy(), goal.copy()]

        bounds = np.array([[-14.0, -14.0, 1.5], [14.0, 14.0, 6.0]])
        # The simulated scanner is planar. For level routes, sample in that
        # observed slice instead of wasting samples in unobserved altitudes.
        if abs(start[2] - goal[2]) <= 0.2:
            bounds[:, 2] = goal[2]
        nodes = [start]
        parent = {0: None}
        cost = {0: 0.0}
        children = {0: set()}

        connected_to_goal = []
        for _ in range(self.max_iter):
            sample = goal if np.random.rand() < 0.15 else self._sample_free(bounds)
            if sample is None:
                continue

            dists = np.linalg.norm(np.asarray(nodes) - sample, axis=1)
            nearest_i = int(np.argmin(dists))
            nearest = nodes[nearest_i]

            direction = sample - nearest
            dist = np.linalg.norm(direction)
            if dist > self.step:
                new_pt = nearest + direction / dist * self.step
            else:
                new_pt = sample.copy()

            if not self.map.segment_is_free(nearest, new_pt):
                continue

            near_radius = min(2.0, max(self.step * 2, 1.5))
            new_dists = np.linalg.norm(np.asarray(nodes) - new_pt, axis=1)
            near_ids = np.flatnonzero(new_dists <= near_radius).tolist()
            new_i = len(nodes)
            parent_i = nearest_i
            best_cost = cost[nearest_i] + float(np.linalg.norm(new_pt - nearest))
            for candidate_i in near_ids:
                candidate = nodes[candidate_i]
                candidate_cost = cost[candidate_i] + float(np.linalg.norm(new_pt - candidate))
                if candidate_cost < best_cost and self.map.segment_is_free(candidate, new_pt):
                    parent_i, best_cost = candidate_i, candidate_cost

            nodes.append(new_pt)
            parent[new_i] = parent_i
            cost[new_i] = best_cost
            children[new_i] = set()
            children[parent_i].add(new_i)

            # Rewire nearby nodes through the new, cheaper branch. Skip its ancestors.
            ancestors = set()
            ancestor_i = parent_i
            while ancestor_i is not None:
                ancestors.add(ancestor_i)
                ancestor_i = parent[ancestor_i]
            for candidate_i in near_ids:
                if candidate_i == parent_i or candidate_i in ancestors:
                    continue
                candidate = nodes[candidate_i]
                edge_cost = float(np.linalg.norm(candidate - new_pt))
                if cost[new_i] + edge_cost < cost[candidate_i] and self.map.segment_is_free(new_pt, candidate):
                    old_parent = parent[candidate_i]
                    children[old_parent].remove(candidate_i)
                    children[new_i].add(candidate_i)
                    parent[candidate_i] = new_i
                    delta = cost[new_i] + edge_cost - cost[candidate_i]
                    stack = [candidate_i]
                    while stack:
                        child_i = stack.pop()
                        cost[child_i] += delta
                        stack.extend(children[child_i])

            if self.map.segment_is_free(new_pt, goal):
                connected_to_goal.append(new_i)

        if connected_to_goal:
            best_i = min(connected_to_goal,
                         key=lambda i: cost[i] + float(np.linalg.norm(nodes[i] - goal)))
            path = [goal.copy()]
            path_i = best_i
            while path_i is not None:
                path.append(nodes[path_i])
                path_i = parent[path_i]
            path.reverse()
            return self._shortcut_path(path)

        return []

    def _shortcut_path(self, path: List[np.ndarray]) -> List[np.ndarray]:
        """Greedily remove redundant waypoints while preserving collision checks."""
        if len(path) < 3:
            return path
        result = [path[0]]
        anchor = 0
        while anchor < len(path) - 1:
            next_i = len(path) - 1
            while next_i > anchor + 1 and not self.map.segment_is_free(path[anchor], path[next_i]):
                next_i -= 1
            result.append(path[next_i])
            anchor = next_i
        return result

    def potential_field_step(self, pos: np.ndarray, goal: np.ndarray,
                              nearby_obstacles: List[np.ndarray],
                              k_att: float = 1.0,
                              k_rep: float = 2.0,
                              d0: float = 1.5) -> np.ndarray:
        """
        Returns a velocity vector from potential field forces.
        Attractive force towards goal, repulsive from obstacles.
        """
        F_att = k_att * (goal - pos)
        F_rep = np.zeros(3)
        for obs in nearby_obstacles:
            d = np.linalg.norm(pos - obs)
            if 0 < d < d0:
                F_rep += k_rep * (1/d - 1/d0) * (1/d**2) * (pos - obs) / d
        return F_att + F_rep


# ---------------------------------------------------------------------------
# Full SLAM Navigation Manager
# ---------------------------------------------------------------------------

class SLAMNavigationManager:
    """
    Orchestrates VIO, LiDAR mapping, and path planning.
    Runs on CPU Cores 1-2 of the Jetson Orin.
    Outputs 6-DoF pose at 200Hz to the flight bridge.
    """

    def __init__(self):
        self.ekf       = VIOFusionEKF(dt=0.005)
        # The operating zone is 30m across. A 32m map at 10cm resolution uses
        # about 33MB per drone; the former 100m map allocated 1GB per drone.
        self.voxel_map = VoxelMap(map_size=32.0)
        self.planner   = PathPlanner(self.voxel_map)

        self.current_pose    : Pose6DOF           = Pose6DOF()
        self.active_waypoints: List[Waypoint]     = []
        self.path_to_target  : List[np.ndarray]   = []

        self._pose_lock = threading.Lock()
        self.running    = True

    def register_imu(self, accel: np.ndarray, gyro: np.ndarray):
        """Called by flight bridge at 200Hz with IMU data from ESP32."""
        self.ekf.predict(accel, gyro)
        with self._pose_lock:
            self.current_pose = self.ekf.current_pose

    def register_vio_pose(self, pose_vec: np.ndarray):
        """Called by Isaac ROS Visual SLAM at 30Hz."""
        self.ekf.update_vio(pose_vec)
        with self._pose_lock:
            self.current_pose = self.ekf.current_pose

    def register_lidar_frame(self, points: np.ndarray,
                             hit_mask: Optional[np.ndarray] = None):
        """Called by Livox driver at 10Hz with new point cloud."""
        with self._pose_lock:
            pose = self.current_pose
        self.voxel_map.update(points, pose, hit_mask)

    def plan_to(self, target_xyz: np.ndarray):
        """Compute global path to a target (e.g., a detected casualty)."""
        with self._pose_lock:
            start = np.array([self.current_pose.x,
                               self.current_pose.y,
                               self.current_pose.z])
        print(f"[SLAM] Planning path: {start} → {target_xyz}")
        self.path_to_target = self.planner.rrt_star(start, target_xyz)
        print(f"[SLAM] Path computed: {len(self.path_to_target)} waypoints")
        return bool(self.path_to_target)

    def get_next_velocity_command(self) -> Tuple[float, float, float]:
        """
        Returns (vx, vy, vz) velocity command for the flight bridge,
        using potential field for reactive obstacle avoidance.
        """
        if not self.path_to_target:
            return (0.0, 0.0, 0.0)

        with self._pose_lock:
            pos = np.array([self.current_pose.x,
                             self.current_pose.y,
                             self.current_pose.z])

        target = self.path_to_target[0]

        # Pop waypoints as they are reached
        if np.linalg.norm(pos - target) < 0.3:
            self.path_to_target.pop(0)
            if not self.path_to_target:
                return (0.0, 0.0, 0.0)
            target = self.path_to_target[0]

        obstacles = self.voxel_map.nearby_obstacles(pos)
        vel = self.planner.potential_field_step(pos, target, obstacles)
        speed = np.linalg.norm(vel)
        if speed > 1.5:   # clamp to 1.5 m/s for safety
            vel = vel / speed * 1.5

        return float(vel[0]), float(vel[1]), float(vel[2])
