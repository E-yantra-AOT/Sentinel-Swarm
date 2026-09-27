"""
=============================================================================
  Sentinel Swarm — ROS2 Launch File
  Spawns N aerial_v1 drones in Gazebo disaster zone

  Usage:
    # Single drone (default)
    ros2 launch sentinel_sim aerial_v1_spawn.launch.py

    # Spawn 5 drones in formation
    ros2 launch sentinel_sim aerial_v1_spawn.launch.py num_drones:=5

    # Open RViz2 as well
    ros2 launch sentinel_sim aerial_v1_spawn.launch.py num_drones:=3 rviz:=true
=============================================================================
"""

import os
import math
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, OpaqueFunction,
    IncludeLaunchDescription, TimerAction
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


# ── Paths ─────────────────────────────────────────────────────────────────────
SIM_DIR   = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
URDF_PATH = os.path.join(SIM_DIR, 'urdf', 'aerial_v1.urdf')
WORLD_PATH = os.path.join(SIM_DIR, 'worlds', 'disaster_zone.sdf')


def compute_spawn_positions(n: int, formation: str = 'line') -> list:
    """
    Computes spawn positions for N drones.

    Formations:
      'line'    — drones spawn in a horizontal line, 3m apart
      'grid'    — drones spawn in a square grid, 5m apart
      'circle'  — drones spawn on a circle of radius 8m
    """
    positions = []

    if formation == 'line':
        for i in range(n):
            positions.append({
                'x': -((n - 1) * 1.5) + i * 3.0,
                'y': 0.0,
                'z': 0.5,       # 0.5m above ground (sitting on battery)
                'yaw': 0.0,
            })

    elif formation == 'grid':
        cols = math.ceil(math.sqrt(n))
        for i in range(n):
            row = i // cols
            col = i %  cols
            positions.append({
                'x': col * 5.0 - (cols * 2.5),
                'y': row * 5.0,
                'z': 0.5,
                'yaw': 0.0,
            })

    elif formation == 'circle':
        radius = max(4.0, n * 1.2)
        for i in range(n):
            angle = (2 * math.pi / n) * i
            positions.append({
                'x': radius * math.cos(angle),
                'y': radius * math.sin(angle),
                'z': 0.5,
                'yaw': angle + math.pi,   # Face inward
            })

    return positions


def launch_setup(context, *args, **kwargs):
    """OpaqueFunction that reads launch args and builds the node list."""

    num_drones  = int(LaunchConfiguration('num_drones').perform(context))
    formation   = LaunchConfiguration('formation').perform(context)
    rviz_flag   = LaunchConfiguration('rviz').perform(context).lower() == 'true'
    world_path  = LaunchConfiguration('world').perform(context)

    nodes = []

    # ── 1. Gazebo Simulator ────────────────────────────────────────────────
    nodes.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('ros_gz_sim'),
                    'launch', 'gz_sim.launch.py'
                )
            ),
            launch_arguments={
                'gz_args': f'-r {world_path}',
                'on_exit_shutdown': 'true',
            }.items(),
        )
    )

    # ── 2. Spawn each drone with a unique namespace ────────────────────────
    positions = compute_spawn_positions(num_drones, formation)

    # Read URDF once
    with open(URDF_PATH, 'r') as f:
        urdf_content = f.read()

    for i, pos in enumerate(positions):
        drone_id  = f'sentinel_{i+1:02d}'
        namespace = f'/sentinel/{drone_id}'

        # Robot State Publisher (one per drone)
        nodes.append(
            Node(
                package    = 'robot_state_publisher',
                executable = 'robot_state_publisher',
                namespace  = namespace,
                name       = 'rsp',
                parameters = [{'robot_description': urdf_content,
                               'use_sim_time': True}],
                output     = 'screen',
            )
        )

        # Spawn entity in Gazebo (staggered by 0.5s each to avoid physics explosions)
        nodes.append(
            TimerAction(
                period = float(i) * 0.5,
                actions = [
                    Node(
                        package    = 'ros_gz_sim',
                        executable = 'create',
                        arguments  = [
                            '-name',  drone_id,
                            '-topic', f'{namespace}/robot_description',
                            '-x',     str(pos['x']),
                            '-y',     str(pos['y']),
                            '-z',     str(pos['z']),
                            '-Y',     str(pos['yaw']),
                        ],
                        output = 'screen',
                    )
                ]
            )
        )

        # ROS ↔ Gazebo Bridge (IMU + Camera per drone)
        nodes.append(
            Node(
                package    = 'ros_gz_bridge',
                executable = 'parameter_bridge',
                namespace  = namespace,
                name       = f'gz_bridge_{drone_id}',
                arguments  = [
                    f'/sentinel/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                    f'/sentinel/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                    f'/model/{drone_id}/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose',
                    f'/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                ],
                output = 'screen',
            )
        )

    # ── 3. Swarm State Broadcaster (publishes all drone poses on one topic) ─
    nodes.append(
        Node(
            package    = 'sentinel_sim',
            executable = 'swarm_state_broadcaster',
            name       = 'swarm_broadcaster',
            parameters = [{'num_drones': num_drones,
                           'use_sim_time': True}],
            output = 'screen',
        )
    )

    # ── 4. Optional RViz2 ──────────────────────────────────────────────────
    if rviz_flag:
        rviz_config = os.path.join(SIM_DIR, 'config', 'sentinel_rviz.rviz')
        nodes.append(
            Node(
                package    = 'rviz2',
                executable = 'rviz2',
                name       = 'rviz2',
                arguments  = ['-d', rviz_config] if os.path.exists(rviz_config) else [],
                parameters = [{'use_sim_time': True}],
                output     = 'screen',
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription([
        # ── Launch Arguments ───────────────────────────────────────────────
        DeclareLaunchArgument(
            'num_drones', default_value='1',
            description='Number of Sentinel aerial drones to spawn (1-100)'
        ),
        DeclareLaunchArgument(
            'formation', default_value='line',
            description='Spawn formation: line | grid | circle'
        ),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='Launch RViz2 alongside Gazebo'
        ),
        DeclareLaunchArgument(
            'world', default_value=WORLD_PATH,
            description='Path to Gazebo world SDF file'
        ),

        # ── Dynamic setup ──────────────────────────────────────────────────
        OpaqueFunction(function=launch_setup),
    ])
