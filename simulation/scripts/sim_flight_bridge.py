#!/usr/bin/env python3
"""
=============================================================================
  Sentinel Swarm — Simulation Flight Bridge
  Wires sentinel_slam.py + sentinel_swarm.py into Gazebo Harmonic per drone.

  One instance runs per drone. Launched by aerial_v1_spawn.launch.py.

  Data flow:
    Gazebo /<drone_id>/imu        → SLAM EKF (register_imu)
    Gazebo /model/<id>/pose       → SLAM ground-truth VIO substitute
    SLAM get_next_velocity_command → SwarmManager → /sentinel/cmd_vel (Twist)
    /sentinel/enable              → Bool (arms the velocity controller)

  Swarm coordination:
    Each drone shares its DroneState JSON on /swarm/state/<drone_id>
    Each drone listens to all peers on /swarm/state/+ and updates SwarmManager

  Usage (normally spawned by the launch file):
    python3 sim_flight_bridge.py --drone-id sentinel_01 --drone-index 0
                                 --num-drones 5 --formation circle
=============================================================================
"""

import argparse
import os
import sys
import math
import json
import threading
import time
import numpy as np

# ── ROS2 ─────────────────────────────────────────────────────────────────────
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from sensor_msgs.msg import Image, Imu, LaserScan
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
from std_msgs.msg import Bool, String

# ── Add project root to path so we can import aerial_node_v2 ─────────────────
# Walk up from __file__ until we find the aerial_node_v2 directory.
# Works whether launched from source tree or colcon install prefix.
def _find_repo_root() -> str:
    candidate = os.path.dirname(os.path.abspath(__file__))
    for _ in range(10):
        if os.path.isdir(os.path.join(candidate, 'aerial_node_v2')):
            return candidate
        candidate = os.path.dirname(candidate)
    raise RuntimeError(
        'Cannot find aerial_node_v2 package. '
        'Ensure the repo root is accessible from the launched script path.'
    )

sys.path.insert(0, _find_repo_root())

from aerial_node_v2.slam.sentinel_slam import SLAMNavigationManager
from aerial_node_v2.swarm.sentinel_swarm import SwarmManager, DroneState, DroneRole


# ── Constants ─────────────────────────────────────────────────────────────────
TAKEOFF_ALT   = 3.0      # metres — hover altitude
SEARCH_ALT    = 3.0      # metres — keep routes in the horizontal lidar scan plane
CASUALTY_XYZ  = np.array([-0.5, -9.0, 0.0])   # from disaster_zone.sdf
ZONE_BOUNDS   = [[-15.0, -15.0], [15.0, 15.0]]  # world boundary
CONTROL_HZ    = 20.0     # Hz

# Flight phases
PHASE_ARM     = "ARM"
PHASE_TAKEOFF = "TAKEOFF"
PHASE_SEARCH  = "SEARCH"
PHASE_RESCUE  = "RESCUE"
PHASE_HOVER   = "HOVER"
PHASE_RTB     = "RETURN_TO_HOME"


class SimFlightBridge(Node):
    """
    Per-drone ROS2 node that runs the full autonomy stack in simulation.
    Subscribes to Gazebo sensor topics, runs SLAM + swarm logic,
    and publishes velocity commands back into Gazebo.
    """

    def __init__(self, drone_id: str, drone_index: int,
                 num_drones: int, formation: str):
        super().__init__(f'sim_flight_bridge_{drone_id}')

        self.drone_id    = drone_id
        self.drone_index = drone_index
        self.num_drones  = num_drones
        self.phase       = PHASE_ARM

        self.current_pose   = np.zeros(3)   # x, y, z from Gazebo ground truth
        self.current_roll   = 0.0
        self.current_pitch  = 0.0
        self.current_yaw    = 0.0
        self.imu_received   = False
        self.pose_received  = False
        self.pose_last_received_at = None
        self.pose_stale_warned = False
        self.camera_last_received_at = None
        self.casualty_visible = False
        self.armed          = False
        self.casualty_found = False
        self.casualty_xyz   = None
        self.last_plan_attempt_at = 0.0
        self.home_xyz = None
        self.previous_pose_for_battery = None
        self.distance_flown_m = 0.0
        self.battery_pct = 1.0
        self.battery_range_m = 1000.0  # nominal simulated range, configurable in code

        # ── SLAM (EKF + RRT* path planner) ───────────────────────────────────
        self.slam = SLAMNavigationManager()

        # ── Swarm manager (behavior tree + Voronoi zones) ─────────────────────
        self.swarm = SwarmManager(
            drone_id   = drone_id,
            zone_bounds= np.array(ZONE_BOUNDS)
        )
        # Assign Voronoi zone based on index
        self._assign_voronoi_zone()

        # ── QoS — best-effort for high-rate sensor data ───────────────────────
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )

        # ── Subscribers ───────────────────────────────────────────────────────
        # IMU — model-scoped topic: sentinel_01/imu, sentinel_02/imu etc.
        self.sub_imu = self.create_subscription(
            Imu, f'/{drone_id}/imu',
            self._imu_callback, sensor_qos
        )

        # Ground-truth pose from Gazebo (replaces VIO in sim)
        self.sub_pose = self.create_subscription(
            PoseStamped, f'/model/{drone_id}/pose',
            self._pose_callback, sensor_qos
        )
        self.sub_lidar = self.create_subscription(
            LaserScan, f'/{drone_id}/lidar/scan',
            self._lidar_callback, sensor_qos
        )
        self.sub_camera = self.create_subscription(
            Image, f'/{drone_id}/camera/image_raw',
            self._camera_callback, sensor_qos
        )

        # Swarm state from peer drones
        self.sub_peers = []
        for i in range(1, num_drones + 1):
            peer_id = f'sentinel_{i:02d}'
            if peer_id != drone_id:
                sub = self.create_subscription(
                    String, f'/swarm/state/{peer_id}',
                    self._peer_state_callback, 10
                )
                self.sub_peers.append(sub)

        # ── Publishers ────────────────────────────────────────────────────────
        # Velocity command → MulticopterVelocityControl plugin
        # Velocity command → MulticopterVelocityControl
        # Matches official Gazebo X3 example: namespace/gazebo/command/twist
        self.tf_broadcaster = TransformBroadcaster(self)
        self.pub_cmd_vel = self.create_publisher(
            Twist, f'/{self.drone_id}/gazebo/command/twist', 10
        )

        # Arm/disarm the velocity controller
        self.pub_enable = self.create_publisher(
            Bool, f'/{self.drone_id}/enable', 10
        )

        # Broadcast own state to swarm peers
        self.pub_state = self.create_publisher(
            String, f'/swarm/state/{drone_id}', 10
        )

        # ── Main control loop timer ───────────────────────────────────────────
        self.timer = self.create_timer(
            1.0 / CONTROL_HZ, self._control_loop
        )

        # ── State broadcast timer (2Hz like real LoRa) ───────────────────────
        self.state_timer = self.create_timer(0.5, self._broadcast_state)

        self.get_logger().info(
            f'[{drone_id}] SimFlightBridge ready | '
            f'index={drone_index} | drones={num_drones}'
        )

    # ─────────────────────────────────────────────────────────────────────────
    # Voronoi zone assignment — splits the 30×30m zone evenly among drones
    # ─────────────────────────────────────────────────────────────────────────
    def _assign_voronoi_zone(self):
        """Assign each drone a non-overlapping grid cell and lawnmower route."""
        cols = math.ceil(math.sqrt(self.num_drones))
        rows = math.ceil(self.num_drones / cols)
        row, col = divmod(self.drone_index, cols)
        cell_width = 27.0 / cols
        cell_height = 27.0 / rows
        x_min = -13.5 + col * cell_width
        y_min = -13.5 + row * cell_height
        x_lanes = max(1, math.ceil(cell_width / 4.0))
        lane_xs = np.linspace(
            x_min + cell_width / (2 * x_lanes),
            x_min + cell_width - cell_width / (2 * x_lanes),
            x_lanes,
        )

        self.search_waypoints = []
        for lane, x in enumerate(lane_xs):
            y_start, y_end = y_min + 0.5, y_min + cell_height - 0.5
            if lane % 2:
                y_start, y_end = y_end, y_start
            self.search_waypoints.extend([
                np.array([x, y_start, SEARCH_ALT]),
                np.array([x, y_end, SEARCH_ALT]),
            ])
        self.search_waypoint_index = 0
        self.zone_seed = self.search_waypoints[0]
        self.get_logger().info(
            f'[{self.drone_id}] Search cell row={row}, col={col}; '
            f'{len(self.search_waypoints)} sweep waypoints'
        )

    # ─────────────────────────────────────────────────────────────────────────
    # Callbacks
    # ─────────────────────────────────────────────────────────────────────────
    def _imu_callback(self, msg: Imu):
        accel = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])
        gyro = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])
        self.slam.register_imu(accel, gyro)
        self.imu_received = True

    def _pose_callback(self, msg: PoseStamped):
        """Gazebo ground-truth pose — substitute for VIO in sim."""
        # Ignore any relative/link pose if one leaks through; only update state
        # from the world-frame model pose published by the URDF plugin.
        if msg.header.frame_id == self.drone_id:
            return

        p = msg.pose.position
        q = msg.pose.orientation
        self.current_pose = np.array([p.x, p.y, p.z])

        # Euler angles from the world pose quaternion.
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        self.current_roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        self.current_pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = f'{self.drone_id}/base_link'
        t.transform.translation.x = p.x
        t.transform.translation.y = p.y
        t.transform.translation.z = p.z
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)
    

        # Feed into SLAM as VIO update (6DOF pose vector)
        pose_vec = np.array([
            p.x, p.y, p.z,
            self.current_roll, self.current_pitch, self.current_yaw,
        ])
        self.slam.register_vio_pose(pose_vec)
        self.pose_received = True
        self.pose_last_received_at = time.monotonic()

    def _peer_state_callback(self, msg: String):
        try:
            state = DroneState.from_json(msg.data)
            # Inject directly into the swarm transport's peer table
            if hasattr(self.swarm, 'transport') and self.swarm.transport:
                self.swarm.transport.peers[state.drone_id] = state
        except Exception:
            pass

    def _lidar_callback(self, msg: LaserScan):
        """Project an oblique GPU LiDAR scan into the world-frame voxel map."""
        if not self.pose_received:
            return
        ranges = np.asarray(msg.ranges, dtype=float)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        valid = (~np.isnan(ranges) & (ranges >= msg.range_min))
        hit_mask = np.isfinite(ranges[valid]) & (ranges[valid] < msg.range_max)
        ranges = np.minimum(ranges[valid], msg.range_max)
        angles = angles[valid]
        if not len(ranges):
            return
        # The URDF mounts the scan horizontally so free-space rays cover the
        # same altitude slice used by the 2D navigation route.
        scan_points = np.column_stack((ranges * np.cos(angles),
                                       ranges * np.sin(angles),
                                       np.zeros_like(ranges)))
        pitch = 0.0
        pitch_rotation = np.array([
            [math.cos(pitch), 0.0, math.sin(pitch)],
            [0.0, 1.0, 0.0],
            [-math.sin(pitch), 0.0, math.cos(pitch)],
        ])
        points = scan_points @ pitch_rotation.T + np.array([0.0, 0.0, 0.035])
        self.slam.register_lidar_frame(points, hit_mask)

    def _camera_callback(self, msg: Image):
        """Detect the synthetic casualty's distinct torso color in rendered RGB."""
        self.casualty_visible = False
        self.camera_last_received_at = time.monotonic()
        if msg.encoding not in ('rgb8', 'bgr8') or msg.step < msg.width * 3:
            return
        try:
            rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
            frame = rows[:, :msg.width * 3].reshape(msg.height, msg.width, 3)[::4, ::4]
            if msg.encoding == 'bgr8':
                red, green, blue = frame[:, :, 2], frame[:, :, 1], frame[:, :, 0]
            else:
                red, green, blue = frame[:, :, 0], frame[:, :, 1], frame[:, :, 2]
            # Synthetic SDF torso color cue; deployed detection still needs
            # the learned vision model and a depth based localizer.
            torso_pixels = (red > 55) & (red > green * 1.35) & (green > blue * 1.15)
            self.casualty_visible = int(np.count_nonzero(torso_pixels)) >= 12
        except (ValueError, BufferError):
            self.casualty_visible = False

    # ─────────────────────────────────────────────────────────────────────────
    # Broadcast own state to peers at 2Hz
    # ─────────────────────────────────────────────────────────────────────────
    def _broadcast_state(self):
        role = (DroneRole.RTB if self.phase == PHASE_RTB else
                DroneRole.RESCUING if self.phase in (PHASE_RESCUE, PHASE_HOVER)
                else DroneRole.SEARCHING)
        state = DroneState(
            drone_id      = self.drone_id,
            role          = role,
            pos_xyz       = self.current_pose,
            battery_pct   = self.battery_pct,
            casualty_found= self.casualty_found,
            casualty_xyz  = self.casualty_xyz,
            casualty_conf = 0.9 if self.casualty_found else 0.0
        )
        msg = String()
        msg.data = state.to_json()
        self.pub_state.publish(msg)

    # ─────────────────────────────────────────────────────────────────────────
    # Main 20Hz control loop
    # ─────────────────────────────────────────────────────────────────────────
    def _control_loop(self):
        cmd = Twist()  # default: hover in place (zero velocity)

        if self.phase == PHASE_ARM:
            self._arm()

        # Ensure Gazebo Velocity Control plugin remains enabled.
        if self.armed:
            enable_msg = Bool()
            enable_msg.data = True
            self.pub_enable.publish(enable_msg)

        pose_age = (
            time.monotonic() - self.pose_last_received_at
            if self.pose_last_received_at is not None else None
        )
        if pose_age is None or pose_age > 1.0:
            if not self.pose_stale_warned:
                self.get_logger().warn(
                    f'[{self.drone_id}] Waiting for fresh Gazebo pose; '
                    'holding zero velocity'
                )
                self.pose_stale_warned = True
            self.pub_cmd_vel.publish(cmd)
            return
        self.pose_stale_warned = False

        if self.home_xyz is None:
            self.home_xyz = self.current_pose.copy()
            self.previous_pose_for_battery = self.current_pose.copy()
        else:
            self.distance_flown_m += float(np.linalg.norm(
                self.current_pose - self.previous_pose_for_battery
            ))
            self.previous_pose_for_battery = self.current_pose.copy()
            self.battery_pct = max(0.0, 1.0 - self.distance_flown_m / self.battery_range_m)
        if self.battery_pct <= 0.20 and self.armed and self.phase != PHASE_RTB:
            self.phase = PHASE_RTB
            self.slam.path_to_target = []
            self.get_logger().warn(f'[{self.drone_id}] Battery at 20%; returning home')

        peers = self.swarm.transport.get_live_peers()
        self.swarm.update_state(
            pos_xyz=self.current_pose.copy(), battery=self.battery_pct,
            casualty_found=self.casualty_found,
            casualty_xyz=self.casualty_xyz,
            casualty_conf=0.9 if self.casualty_found else 0.0,
        )
        self.swarm.bt.tick(self.swarm.state, peers)
        if self.phase == PHASE_SEARCH:
            reports = [peer for peer in peers.values()
                       if peer.casualty_found and peer.casualty_conf > 0.4
                       and peer.casualty_xyz is not None]
            if reports:
                report = max(reports, key=lambda peer: peer.casualty_conf)
                self.casualty_found = True
                self.casualty_xyz = report.casualty_xyz.copy()
                if self._should_handle_casualty(self.casualty_xyz, peers):
                    self._request_path(np.array([*self.casualty_xyz[:2], 3.0]))
                    self.phase = PHASE_RESCUE
                    self.get_logger().warn(
                        f'[{self.drone_id}] Taking rescue assignment for report '
                        f'from {report.drone_id}'
                    )

        if self.phase == PHASE_TAKEOFF:
            self._run_takeoff(cmd)

        elif self.phase == PHASE_SEARCH:
            self._run_search(cmd)

        elif self.phase == PHASE_RESCUE:
            self._run_rescue(cmd)

        elif self.phase == PHASE_HOVER:
            pass  # hold position — cmd stays zero

        elif self.phase == PHASE_RTB:
            self._run_return_home(cmd)

        self._apply_peer_separation(cmd)
        self.pub_cmd_vel.publish(cmd)

    def _apply_peer_separation(self, cmd: Twist):
        """Add a bounded horizontal escape velocity when a peer is too close."""
        minimum_distance = 2.0
        repulsion = np.zeros(2)
        for peer in self.swarm.transport.get_live_peers().values():
            delta = self.current_pose[:2] - peer.pos_xyz[:2]
            distance = float(np.linalg.norm(delta))
            if distance >= minimum_distance:
                continue
            if distance < 1e-3:
                angle = 2.0 * math.pi * self.drone_index / max(self.num_drones, 1)
                delta = np.array([math.cos(angle), math.sin(angle)])
                distance = 0.0
            repulsion += delta / max(distance, 1e-3) * min(1.0, minimum_distance - distance)

        velocity = np.array([cmd.linear.x, cmd.linear.y]) + repulsion
        speed = float(np.linalg.norm(velocity))
        if speed > 1.5:
            velocity *= 1.5 / speed
        cmd.linear.x = float(velocity[0])
        cmd.linear.y = float(velocity[1])

    def _should_handle_casualty(self, casualty_xyz: np.ndarray, peers) -> bool:
        """Choose one nearest searching drone; preserve an existing rescuer."""
        active_rescuers = [
            peer for peer in peers.values()
            if peer.casualty_found and peer.role == DroneRole.RESCUING
            and peer.casualty_xyz is not None
            and np.linalg.norm(peer.casualty_xyz[:2] - casualty_xyz[:2]) < 2.0
        ]
        if active_rescuers:
            return False

        candidates = [(float(np.linalg.norm(self.current_pose[:2] - casualty_xyz[:2])),
                       self.drone_id)]
        candidates.extend(
            (float(np.linalg.norm(peer.pos_xyz[:2] - casualty_xyz[:2])), peer.drone_id)
            for peer in peers.values()
            if peer.role == DroneRole.SEARCHING
        )
        return min(candidates)[1] == self.drone_id

    # ─────────────────────────────────────────────────────────────────────────

    # Pose accessor — Gazebo ground truth if available, EKF fallback otherwise
    # ─────────────────────────────────────────────────────────────────────────
    def _get_current_pos(self) -> np.ndarray:
        if self.pose_received:
            return self.current_pose.copy()
        # Fall back to SLAM EKF dead-reckoning (IMU integration)
        p = self.slam.current_pose
        return np.array([p.x, p.y, p.z])

    # ─────────────────────────────────────────────────────────────────────────
    # Phase logic
    # ─────────────────────────────────────────────────────────────────────────
    def _arm(self):
        """Arm the velocity controller and transition to takeoff."""
        enable_msg = Bool()
        enable_msg.data = True
        self.pub_enable.publish(enable_msg)
        self.armed = True
        self.phase = PHASE_TAKEOFF
        self.get_logger().info(f'[{self.drone_id}] ARMED → TAKEOFF')

    def _run_takeoff(self, cmd: Twist):
        """Climb until ground-truth altitude confirms the target is reached."""
        alt = self.current_pose[2]

        # Never enter search based on elapsed time: a slow or missing pose
        # stream must not make the bridge assume the drone has taken off.
        if self.pose_received and alt >= TAKEOFF_ALT - 0.2:
            self.get_logger().info(f'[{self.drone_id}] TAKEOFF → SEARCH')
            self._request_path(self.zone_seed)
            self.phase = PHASE_SEARCH
        else:
            # Climb at up to 1.5 m/s, proportional to remaining distance
            cmd.linear.z = min(1.5, (TAKEOFF_ALT - alt) * 1.2)

    def _run_search(self, cmd: Twist):
        """
        Follow the assigned lawnmower sweep and check camera visibility cues.
        """
        target = self.search_waypoints[self.search_waypoint_index]
        pos = self._get_current_pos()
        if np.linalg.norm(pos[:2] - target[:2]) < 0.6:
            self.search_waypoint_index = (self.search_waypoint_index + 1) % len(self.search_waypoints)
            target = self.search_waypoints[self.search_waypoint_index]
        if not self.slam.path_to_target:
            self._request_path(target)
        vx, vy, vz = self.slam.get_next_velocity_command()

        # Clamp to safe search speed (1.5 m/s max)
        speed = math.sqrt(vx**2 + vy**2 + vz**2)
        if speed > 1.5:
            scale = 1.5 / speed
            vx, vy, vz = vx * scale, vy * scale, vz * scale

        cmd.linear.x = float(vx)
        cmd.linear.y = float(vy)
        cmd.linear.z = float(vz)

        # Require a fresh rendered image cue; use the known SDF coordinate only
        # as the simulator's localization proxy because this camera has no depth.
        image_fresh = (
            self.camera_last_received_at is not None
            and time.monotonic() - self.camera_last_received_at <= 1.0
        )
        if self.casualty_visible and image_fresh and pos[2] > 1.5:
            self.casualty_found = True
            self.casualty_xyz   = CASUALTY_XYZ.copy()
            peers = self.swarm.transport.get_live_peers()
            if self._should_handle_casualty(self.casualty_xyz, peers):
                self.get_logger().warn(
                    f'[{self.drone_id}] 🚨 Visible casualty cue at '
                    f'({CASUALTY_XYZ[0]:.1f}, {CASUALTY_XYZ[1]:.1f}) → RESCUE'
                )
                self._request_path(np.array([CASUALTY_XYZ[0], CASUALTY_XYZ[1], 3.0]))
                self.phase = PHASE_RESCUE

    def _run_rescue(self, cmd: Twist):
        """Fly to casualty location and hold a 3m hover directly above."""
        casualty_xyz = self.casualty_xyz if self.casualty_xyz is not None else CASUALTY_XYZ
        target = np.array([casualty_xyz[0], casualty_xyz[1], 3.0])
        pos    = self._get_current_pos()
        diff   = target - pos
        dist   = np.linalg.norm(diff)

        if dist > 0.5:
            if not self.slam.path_to_target:
                self._request_path(target)
            vx, vy, vz = self.slam.get_next_velocity_command()
            cmd.linear.x = float(vx)
            cmd.linear.y = float(vy)
            cmd.linear.z = float(vz)
        else:
            self.phase = PHASE_HOVER
            self.get_logger().info(
                f'[{self.drone_id}] ✅ RESCUE HOVER — marking casualty'
            )

    def _run_return_home(self, cmd: Twist):
        """Navigate to the launch position, then descend and disarm."""
        if self.home_xyz is None:
            return
        pos = self.current_pose.copy()
        if np.linalg.norm(pos[:2] - self.home_xyz[:2]) > 0.5:
            target = self.home_xyz.copy()
            target[2] = TAKEOFF_ALT
            if not self.slam.path_to_target:
                self._request_path(target)
            vx, vy, vz = self.slam.get_next_velocity_command()
            cmd.linear.x, cmd.linear.y, cmd.linear.z = float(vx), float(vy), float(vz)
        elif pos[2] > 0.15:
            cmd.linear.z = -0.35
        else:
            self.pub_enable.publish(Bool(data=False))
            self.armed = False
            self.phase = PHASE_HOVER
            cmd.linear.z = 0.0

    def _request_path(self, target: np.ndarray):
        """Rate-limit replanning while the LiDAR map fills or routes are blocked."""
        now = time.monotonic()
        if now - self.last_plan_attempt_at < 1.0:
            return False
        self.last_plan_attempt_at = now
        return self.slam.plan_to(target)


def main():
    parser = argparse.ArgumentParser(description='Sentinel Sim Flight Bridge')
    parser.add_argument('--drone-id',    required=True,
                        help='e.g. sentinel_01')
    parser.add_argument('--drone-index', type=int, required=True,
                        help='0-based index in the swarm')
    parser.add_argument('--num-drones',  type=int, default=5)
    parser.add_argument('--formation',   default='circle')

    # ROS2 passes extra args; ignore them
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = SimFlightBridge(
        drone_id    = args.drone_id,
        drone_index = args.drone_index,
        num_drones  = args.num_drones,
        formation   = args.formation,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass  # already shut down by SIGINT handler — safe to ignore


if __name__ == '__main__':
    main()
