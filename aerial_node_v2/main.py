"""
=============================================================================
  Sentinel Swarm v2 — Main Entry Point
  Boots all subsystems on Jetson Orin Nano Super
=============================================================================
"""
import time
import threading
import numpy as np

from vision.sentinel_vision    import SentinelVisionPipeline
from slam.sentinel_slam        import SLAMNavigationManager
from swarm.sentinel_swarm      import SwarmManager
from flight_bridge.flight_bridge import FlightBridge


DRONE_ID = "SENTINEL_AERIAL_1"

# Search zone: 100m x 100m area centered at origin
ZONE_BOUNDS = np.array([[-50, -50], [50, 50]], dtype=float)


def main():
    print("=" * 60)
    print("  SENTINEL SWARM v2 — AERIAL NODE BOOT")
    print("  Hardware: Jetson Orin Nano Super (67 TOPS)")
    print("=" * 60)

    # 1. Initialize SLAM (starts its own update loops)
    slam = SLAMNavigationManager()

    # 2. Initialize Flight Bridge with IMU callback → SLAM EKF
    bridge = FlightBridge(
        port='/dev/ttyS0',
        imu_callback=lambda a, g: slam.register_imu(a, g)
    )

    # 3. Initialize Swarm Manager
    swarm = SwarmManager(DRONE_ID, ZONE_BOUNDS)

    # 4. Initialize Vision Pipeline
    vision = SentinelVisionPipeline()

    # 5. Start vision in background thread (dedicated Core 3 via taskset)
    def vision_thread():
        vision.run(slam_pose_provider=lambda: slam.current_pose.as_matrix(),
                   camera_source=0)

    t_vision = threading.Thread(target=vision_thread, daemon=True)
    t_vision.start()

    # 6. Start swarm manager in background thread
    t_swarm = threading.Thread(target=swarm.run, daemon=True)
    t_swarm.start()

    print("[MAIN] All subsystems nominal. Entering control loop.")

    try:
        while True:
            # Read current SLAM pose
            pose = slam.current_pose
            pos  = np.array([pose.x, pose.y, pose.z])

            # Check if vision detected any casualty
            with vision._lock:
                casualties = list(vision.confirmed_casualties)

            casualty_found = len(casualties) > 0
            best_casualty  = max(casualties, key=lambda c: c.confidence) if casualties else None

            # Update swarm state
            swarm.update_state(
                pos_xyz        = pos,
                battery        = 0.95,   # In production: read from FC telemetry
                casualty_found = casualty_found,
                casualty_xyz   = best_casualty.world_xyz if best_casualty else None,
                casualty_conf  = best_casualty.confidence if best_casualty else 0.0,
            )

            # Plan path to casualty if found
            if casualty_found and best_casualty:
                slam.plan_to(best_casualty.world_xyz)

            # Get velocity command from SLAM and send to FC
            vx, vy, vz = slam.get_next_velocity_command()
            bridge.send_velocity(vx, vy, vz, current_yaw=pose.yaw)

            time.sleep(0.05)   # 20Hz main loop

    except KeyboardInterrupt:
        print("\n[MAIN] Shutdown initiated.")
        bridge.send_hover()
        vision.running = False
        swarm.running  = False


if __name__ == "__main__":
    main()
