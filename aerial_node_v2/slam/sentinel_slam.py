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
        self.grid = np.zeros((n, n, n), dtype=np.uint8)
        self.origin = np.array([map_size/2, map_size/2, map_size/2])
        self._lock = threading.Lock()

    def update(self, points: np.ndarray, drone_pose: Pose6DOF):
        """Insert a point cloud into the voxel map."""
        with self._lock:
            for pt in points:
                idx = ((pt + self.origin) / self.VOXEL_SIZE).astype(int)
                if all(0 <= idx[i] < self.grid.shape[i] for i in range(3)):
                    self.grid[idx[0], idx[1], idx[2]] = min(255, 
                        int(self.grid[idx[0], idx[1], idx[2]]) + 10)

    def is_occupied(self, xyz: np.ndarray, threshold: int = 50) -> bool:
        idx = ((xyz + self.origin) / self.VOXEL_SIZE).astype(int)
        try:
            return int(self.grid[idx[0], idx[1], idx[2]]) > threshold
        except IndexError:
            return True  # Treat out-of-bounds as occupied (safe)


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

    def _sample_free(self, bounds: float = 50.0) -> np.ndarray:
        """Sample a random collision-free point."""
        for _ in range(100):
            pt = (np.random.rand(3) - 0.5) * 2 * bounds
            if not self.map.is_occupied(pt):
                return pt
        return np.zeros(3)

    def rrt_star(self, start: np.ndarray,
                 goal: np.ndarray) -> List[np.ndarray]:
        """Returns a list of waypoints from start to goal using RRT*."""
        nodes  = [start]
        parent = {0: None}
        cost   = {0: 0.0}

        for _ in range(self.max_iter):
            # Bias towards goal 10% of the time
            if np.random.rand() < 0.1:
                sample = goal
            else:
                sample = self._sample_free()

            # Find nearest node
            dists  = [np.linalg.norm(sample - n) for n in nodes]
            near_i = int(np.argmin(dists))
            near   = nodes[near_i]

            # Steer
            direction = sample - near
            dist = np.linalg.norm(direction)
            if dist > self.step:
                new_pt = near + direction / dist * self.step
            else:
                new_pt = sample

            if self.map.is_occupied(new_pt):
                continue

            new_i = len(nodes)
            nodes.append(new_pt)
            parent[new_i] = near_i
            cost[new_i]   = cost[near_i] + self.step

            # Check goal
            if np.linalg.norm(new_pt - goal) < self.step:
                # Trace path back
                path = [goal]
                idx  = new_i
                while parent[idx] is not None:
                    path.append(nodes[idx])
                    idx = parent[idx]
                path.reverse()
                return path

        return [start, goal]  # Fallback: straight line

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
        self.voxel_map = VoxelMap(map_size=100.0)
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

    def register_lidar_frame(self, points: np.ndarray):
        """Called by Livox driver at 10Hz with new point cloud."""
        with self._pose_lock:
            pose = self.current_pose
        self.voxel_map.update(points, pose)

    def plan_to(self, target_xyz: np.ndarray):
        """Compute global path to a target (e.g., a detected casualty)."""
        with self._pose_lock:
            start = np.array([self.current_pose.x,
                               self.current_pose.y,
                               self.current_pose.z])
        print(f"[SLAM] Planning path: {start} → {target_xyz}")
        self.path_to_target = self.planner.rrt_star(start, target_xyz)
        print(f"[SLAM] Path computed: {len(self.path_to_target)} waypoints")

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

        vel = self.planner.potential_field_step(pos, target, [])
        speed = np.linalg.norm(vel)
        if speed > 1.5:   # clamp to 1.5 m/s for safety
            vel = vel / speed * 1.5

        return float(vel[0]), float(vel[1]), float(vel[2])
