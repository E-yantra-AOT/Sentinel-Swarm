#!/usr/bin/env python3
"""
=============================================================================
  Sentinel Swarm — Simulation Flight Bridge
  Wires sentinel_slam.py + sentinel_swarm.py into Gazebo Harmonic per drone.

  One instance runs per drone. Launched by aerial_v1_spawn.launch.py.

  Data flow:
    Gazebo /sentinel/imu          → SLAM EKF (register_imu)
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
import time
import json
import threading
import numpy as np

# ── ROS2 ─────────────────────────────────────────────────────────────────────
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from sensor_msgs.msg import Imu
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
SEARCH_ALT    = 4.0      # metres — search altitude
CASUALTY_XYZ  = np.array([-0.5, -9.0, 0.0])   # from disaster_zone.sdf
ZONE_BOUNDS   = [[-15.0, -15.0], [15.0, 15.0]]  # world boundary
CONTROL_HZ    = 20.0     # Hz

# Flight phases
PHASE_ARM     = "ARM"
PHASE_TAKEOFF = "TAKEOFF"
PHASE_SEARCH  = "SEARCH"
PHASE_RESCUE  = "RESCUE"
PHASE_HOVER   = "HOVER"


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
        self.current_yaw    = 0.0
        self.imu_received   = False
        self.pose_received  = False
        self.armed          = False
        self.casualty_found = False
        self.casualty_xyz   = None
        self.takeoff_start_time = None   # for time-based takeoff fallback

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
        """Give each drone a Voronoi seed waypoint based on its index."""
        n   = self.num_drones
        idx = self.drone_index
        # Circle of seeds, radius 8m, starting from north
        angle = (2 * math.pi / n) * idx - math.pi / 2
        r     = 8.0
        self.zone_seed = np.array([r * math.cos(angle),
                                   r * math.sin(angle),
                                   SEARCH_ALT])
        self.get_logger().info(
            f'[{self.drone_id}] Zone seed: '
            f'({self.zone_seed[0]:.1f}, {self.zone_seed[1]:.1f})'
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
        p = msg.pose.position
        q = msg.pose.orientation
        self.current_pose = np.array([p.x, p.y, p.z])

        # Yaw from quaternion
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
        pose_vec = np.array([p.x, p.y, p.z, 0.0, 0.0, self.current_yaw])
        self.slam.register_vio_pose(pose_vec)
        self.pose_received = True

    def _peer_state_callback(self, msg: String):
        try:
            state = DroneState.from_json(msg.data)
            # Inject directly into the swarm transport's peer table
            if hasattr(self.swarm, 'transport') and self.swarm.transport:
                self.swarm.transport.peers[state.drone_id] = state
        except Exception:
            pass

    # ─────────────────────────────────────────────────────────────────────────
    # Broadcast own state to peers at 2Hz
    # ─────────────────────────────────────────────────────────────────────────
    def _broadcast_state(self):
        state = DroneState(
            drone_id      = self.drone_id,
            role          = DroneRole.SEARCHING,
            pos_xyz       = self.current_pose,
            battery_pct   = 0.95,
            casualty_found= self.casualty_found,
            casualty_xyz  = self.casualty_xyz,
            casualty_conf = 0.0
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
        # Ensure Gazebo Velocity Control plugin is enabled
        if self.armed:
            enable_msg = Bool()
            enable_msg.data = True
            self.pub_enable.publish(enable_msg)

        if self.phase == PHASE_TAKEOFF:
            self._run_takeoff(cmd)

        elif self.phase == PHASE_SEARCH:
            self._run_search(cmd)

        elif self.phase == PHASE_RESCUE:
            self._run_rescue(cmd)

        elif self.phase == PHASE_HOVER:
            pass  # hold position — cmd stays zero

        # Ensure Gazebo Velocity Control plugin is enabled
        if self.armed:
            enable_msg = Bool()
            enable_msg.data = True
            self.pub_enable.publish(enable_msg)

        self.pub_cmd_vel.publish(cmd)
        self.get_logger().info(f"Publishing Twist: z={cmd.linear.z}")
        self.get_logger().info(f"Publishing Twist: z={cmd.linear.z}")

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
        self.takeoff_start_time = time.time()
        self.phase = PHASE_TAKEOFF
        self.get_logger().info(f'[{self.drone_id}] ARMED → TAKEOFF')

    def _run_takeoff(self, cmd: Twist):
        """Climb to TAKEOFF_ALT. Uses pose altitude or 6s timer as fallback."""
        alt = self.current_pose[2]
        elapsed = time.time() - (self.takeoff_start_time or time.time())

        # Transition if altitude reached (pose-based) OR 6s elapsed (timer fallback)
        if alt >= TAKEOFF_ALT - 0.2 or elapsed > 6.0:
            if not self.pose_received:
                self.get_logger().warn(
                    f'[{self.drone_id}] Pose data not received — '
                    'using timer fallback for takeoff transition'
                )
            self.get_logger().info(f'[{self.drone_id}] TAKEOFF → SEARCH')
            self.slam.plan_to(self.zone_seed)
            self.phase = PHASE_SEARCH
        else:
            # Climb at up to 1.5 m/s, proportional to remaining distance
            cmd.linear.z = min(1.5, (TAKEOFF_ALT - alt) * 1.2)

    def _run_search(self, cmd: Twist):
        """
        Run the SLAM navigator toward the Voronoi zone seed.
        Detects casualty by proximity. Uses ground-truth pose or EKF fallback.
        """
        vx, vy, vz = self.slam.get_next_velocity_command()

        # Clamp to safe search speed (1.5 m/s max)
        speed = math.sqrt(vx**2 + vy**2 + vz**2)
        if speed > 1.5:
            scale = 1.5 / speed
            vx, vy, vz = vx * scale, vy * scale, vz * scale

        cmd.linear.x = float(vx)
        cmd.linear.y = float(vy)
        cmd.linear.z = float(vz)

        pos = self._get_current_pos()

        # Casualty detection by proximity
        dist_to_casualty = np.linalg.norm(pos[:2] - CASUALTY_XYZ[:2])
        if dist_to_casualty < 5.0 and pos[2] > 1.5:
            self.casualty_found = True
            self.casualty_xyz   = CASUALTY_XYZ.copy()
            self.get_logger().warn(
                f'[{self.drone_id}] 🚨 CASUALTY DETECTED at '
                f'({CASUALTY_XYZ[0]:.1f}, {CASUALTY_XYZ[1]:.1f}) → RESCUE'
            )
            self.slam.plan_to(
                np.array([CASUALTY_XYZ[0], CASUALTY_XYZ[1], 3.0])
            )
            self.phase = PHASE_RESCUE

    def _run_rescue(self, cmd: Twist):
        """Fly to casualty location and hold a 3m hover directly above."""
        target = np.array([CASUALTY_XYZ[0], CASUALTY_XYZ[1], 3.0])
        pos    = self._get_current_pos()
        diff   = target - pos
        dist   = np.linalg.norm(diff)

        if dist > 0.5:
            vx, vy, vz = self.slam.get_next_velocity_command()
            cmd.linear.x = float(vx)
            cmd.linear.y = float(vy)
            cmd.linear.z = float(vz)
        else:
            self.phase = PHASE_HOVER
            self.get_logger().info(
                f'[{self.drone_id}] ✅ RESCUE HOVER — marking casualty'
            )


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
