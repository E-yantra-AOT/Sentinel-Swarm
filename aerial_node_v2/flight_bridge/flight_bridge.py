"""
=============================================================================
  Sentinel Swarm v2 — Flight Bridge
  Hardware: Jetson Orin → ESP32-S31 FC via UART (MAVLink-lite)
=============================================================================

Receives velocity commands from SLAM, converts to attitude setpoints,
and sends over UART to the ESP32-S31 flight controller.
Also reads back telemetry (IMU, battery) and feeds it to the SLAM EKF.
"""

import serial
import time
import struct
import threading
import numpy as np
from typing import Callable, Optional

class FlightBridge:
    """
    Serial bridge between Jetson Orin and ESP32-S31 flight controller.
    Protocol: ASCII for simplicity, binary framing for telemetry.

    Uplink (Jetson → ESP32):   CMD,<pitch_deg>,<roll_deg>,<thrust_pct>\n
    Downlink (ESP32 → Jetson): TEL,<ax>,<ay>,<az>,<gx>,<gy>,<gz>,<bat_v>\n
    """

    MAX_PITCH  = 20.0   # degrees — safety clamp
    MAX_ROLL   = 20.0
    UART_BAUD  = 115200

    def __init__(self, port: str = '/dev/ttyS0',
                 imu_callback: Optional[Callable] = None):
        """
        imu_callback: called with (accel_xyz, gyro_xyz) on each telemetry packet.
                      Used to feed the SLAM EKF.
        """
        self.imu_callback = imu_callback
        self.running       = True
        self._lock         = threading.Lock()
        self._last_cmd     = (0.0, 0.0, 0.0)   # pitch, roll, thrust

        try:
            self.ser = serial.Serial(port, self.UART_BAUD, timeout=0.05)
            print(f"[FlightBridge] UART open on {port} @ {self.UART_BAUD}")
        except Exception as e:
            print(f"[FlightBridge] UART unavailable: {e} — running in simulation mode")
            self.ser = None

        # Start telemetry reader thread
        t = threading.Thread(target=self._telemetry_loop, daemon=True)
        t.start()

    def send_velocity(self, vx: float, vy: float, vz: float,
                      current_yaw: float, target_yaw: float = None):
        """
        Converts X/Y/Z velocity vectors from SLAM into pitch/roll attitude commands.
        Body-frame: +X = forward (pitch forward), +Y = right (roll right).
        """
        # Simple P-controller to convert velocity to attitude
        Kv = 2.5   # velocity → degrees gain
        pitch_cmd = float(np.clip(Kv * vx, -self.MAX_PITCH, self.MAX_PITCH))
        roll_cmd  = float(np.clip(Kv * vy, -self.MAX_ROLL,  self.MAX_ROLL))
        # Thrust is held constant in cruise; altitude is handled by the FC
        thrust    = 50.0  # percent — placeholder

        self._send_command(pitch_cmd, roll_cmd, thrust)

    def send_hover(self):
        """Command the drone to stop all horizontal movement and hover."""
        self._send_command(0.0, 0.0, 50.0)

    def _send_command(self, pitch: float, roll: float, thrust: float):
        with self._lock:
            self._last_cmd = (pitch, roll, thrust)
        if self.ser and self.ser.is_open:
            cmd = f"CMD,{pitch:.2f},{roll:.2f},{thrust:.1f}\n"
            self.ser.write(cmd.encode('ascii'))

    def _telemetry_loop(self):
        """Reads telemetry from ESP32 and calls imu_callback."""
        while self.running:
            if not self.ser or not self.ser.is_open:
                time.sleep(0.1)
                continue
            try:
                line = self.ser.readline().decode('ascii', errors='ignore').strip()
                if line.startswith("TEL,"):
                    parts = line.split(",")
                    if len(parts) == 8:
                        ax, ay, az = float(parts[1]), float(parts[2]), float(parts[3])
                        gx, gy, gz = float(parts[4]), float(parts[5]), float(parts[6])
                        bat_v       = float(parts[7])
                        if self.imu_callback:
                            self.imu_callback(
                                np.array([ax, ay, az]),
                                np.array([gx, gy, gz])
                            )
            except Exception:
                pass
