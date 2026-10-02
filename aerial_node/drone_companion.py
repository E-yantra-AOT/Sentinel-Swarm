"""
=============================================================================
  Sentinel Swarm — Aerial Node Companion Computer
  Raspberry Pi 5 (8GB) + MicoAir H743 V2 (ArduPilot)

  Hardware:
    FC:     MicoAir H743 V2 (STM32H743, ArduPilot)
    IMU:    BMI088 + BMI270 (onboard FC)
    Baro:   SPL06 (onboard FC)
    GPS:    NEO-6M on FC UART3 (SERIAL3)
    Comms:  Pi UART (GPIO14/15) → FC UART1 (SERIAL1) @ 921600
    XBee:   USB XBee PRO S2C → /dev/ttyUSB0 @ 9600 baud (PAN 3333)
    Camera: USB Webcam 1080p → /dev/video0

  Data Flow:
    FC UART1 ←MAVLink2→ Pi /dev/serial0
    XBee USB ←JSON/9600→ Ground Bot A, Ground Bot B
    USB Webcam → YOLO11n NCNN → casualty detection

  Swarm Protocol:
    Robot ID: 'DRONE'
    Joins same XBee PAN 3333 mesh as ground bots
    Broadcasts detections as: {'r':'DRONE','cx':...,'cy':...,'cf':...,'sz':...,'st':'T'/'L'}
    Receives ground bot detections for cross-node awareness

  State Machine:
    DISARMED  → waiting for arm command / all pre-checks pass
    ARMED     → motors armed, waiting for takeoff
    TAKEOFF   → climbing to CRUISE_ALT (3m default)
    SEARCHING → lawnmower sweep pattern, broadcasting XBee
    TRACKING  → casualty locked, descending for closer inspection
    RELAYING  → hovering, broadcasting GPS coords of casualty to ground bots
    RTB       → returning to home, battery < RTB_THRESHOLD
    LANDING   → final descent and disarm

  Usage:
    python3 drone_companion.py
    python3 drone_companion.py --sim          # SITL mode (no real FC)
    python3 drone_companion.py --no-xbee      # disable XBee (solo mode)
    python3 drone_companion.py --alt 5.0      # cruise altitude override
=============================================================================
"""

import sys
import os
import time
import http.server
import socketserver
import io
from PIL import Image
import math
import threading
import argparse
import logging
from pathlib import Path
from enum import Enum
from dataclasses import dataclass, field
from typing import Optional

import cv2
import numpy as np

# ── MAVLink (pymavlink) ───────────────────────────────────────────────────────
try:
    from pymavlink import mavutil
    MAVLINK_AVAILABLE = True
except ImportError:
    MAVLINK_AVAILABLE = False
    print("[WARN] pymavlink not installed. Run: pip install pymavlink")

# ── Shared modules (same as ground bots) ─────────────────────────────────────
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "ground_node" / "shared"))

from ground_node.shared.xbee_transport import XBeeTransport
from ground_node.shared.yolo_ncnn import YoloNcnn

# ── Logging ───────────────────────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format='[%(asctime)s] [%(levelname)s] %(message)s',
    datefmt='%H:%M:%S'
)
log = logging.getLogger(__name__)


# =============================================================================
# Configuration
# =============================================================================

# Hardware ports
FC_SERIAL_PORT   = '/dev/ttyACM0'      # FC connected via USB Type-C      # Pi GPIO14/15 → FC UART1
FC_BAUD_RATE     = 115200              # Standard baud for ArduPilot USB (MAVLink)              # Must match SERIAL1_BAUD=921 in ArduPilot
XBEE_PORT        = '/dev/ttyUSB0'      # USB XBee module
CAMERA_DEVICE    = 0                   # /dev/video0

# YOLO
YOLO_MODEL_DIR   = os.path.expanduser('~/yolo11n_ncnn_model')
YOLO_CONF        = 0.45                # Slightly higher threshold than ground (aerial view)
YOLO_THREADS     = 4                   # Pi 5 has 4 full cores — use all of them

# Swarm
ROBOT_ID         = 'DRONE'
XBEE_BROADCAST_HZ = 2.0               # 2Hz broadcast (matches LoRa sim rate)

# Flight parameters
CRUISE_ALT       = 3.0                 # metres AGL for search pattern
TRACKING_ALT     = 1.5                 # descend to this when casualty locked
TAKEOFF_ALT      = 3.0                 # initial takeoff altitude
RTB_BATTERY_PCT  = 20.0               # % battery → force RTB
HOVER_LOITER_S   = 5.0                # seconds to hover after casualty found
WAYPOINT_RADIUS  = 0.5                 # metres — consider waypoint reached

# Lawnmower search pattern (relative to home, metres)
SEARCH_PATTERN = [
    ( 0.0,  5.0), ( 5.0,  5.0), ( 5.0,  0.0),
    (10.0,  0.0), (10.0,  5.0), (15.0,  5.0),
    (15.0,  0.0), (20.0,  0.0), (20.0,  5.0),
]

# Velocity control gains (for GUIDED mode SET_POSITION_TARGET_LOCAL_NED)
KV_HORIZONTAL   = 1.5                  # m/s per unit of normalised error
MAX_VEL_XY      = 2.0                  # m/s horizontal
MAX_VEL_Z       = 1.0                  # m/s vertical

# Safety
HEARTBEAT_TIMEOUT_S = 3.0              # declare FC link dead after this
CONTROL_HZ          = 10              # main control loop rate


# =============================================================================
# State Machine
# =============================================================================

class DroneState(Enum):
    DISARMED  = "DISARMED"
    ARMED     = "ARMED"
    TAKEOFF   = "TAKEOFF"
    SEARCHING = "SEARCHING"
    TRACKING  = "TRACKING"
    RELAYING  = "RELAYING"
    RTB       = "RTB"
    LANDING   = "LANDING"


# =============================================================================
# MAVLink Flight Controller Interface
# =============================================================================

class FCInterface:
    """
    Handles all MAVLink communication with the MicoAir H743 V2.
    Wraps pymavlink in a clean, thread-safe API.
    """

    def __init__(self, port: str, baud: int, sim_mode: bool = False):
        self.sim_mode      = sim_mode
        self._mav          = None
        self._armed        = False
        self._mode         = "STABILIZE"
        self._alt_rel      = 0.0          # relative altitude from FC baro (m)
        self._battery_pct  = 100.0
        self._home_lat     = 0.0
        self._home_lon     = 0.0
        self._home_set     = False
        self._gps_fix      = 0
        self._lock         = threading.Lock()
        self._last_hb_time = time.time()

        if sim_mode:
            log.info("[FC] SIM MODE — no real FC connection")
            return

        if not MAVLINK_AVAILABLE:
            log.error("[FC] pymavlink not available. Install: pip install pymavlink")
            return

        log.info(f"[FC] Connecting to {port} @ {baud} baud...")
        try:
            self._mav = mavutil.mavlink_connection(
                port, baud=baud,
                source_system=1,
                source_component=1,
                autoreconnect=True
            )
            log.info("[FC] Waiting for heartbeat...")
            self._mav.wait_heartbeat(timeout=10)
            self._last_hb_time = time.time()
            log.info(f"[FC] ✅ Heartbeat received — FC is alive "
                     f"(system={self._mav.target_system}, "
                     f"component={self._mav.target_component})")
        except Exception as e:
            log.error(f"[FC] Connection failed: {e}")
            self._mav = None

        # Start telemetry reader thread
        t = threading.Thread(target=self._telemetry_loop, daemon=True)
        t.start()

    # ── Properties ────────────────────────────────────────────────────────────

    @property
    def connected(self) -> bool:
        if self.sim_mode: return True
        return self._mav is not None

    @property
    def link_alive(self) -> bool:
        if self.sim_mode: return True
        return (time.time() - self._last_hb_time) < HEARTBEAT_TIMEOUT_S

    @property
    def armed(self) -> bool:
        return self._armed

    @property
    def altitude(self) -> float:
        return self._alt_rel

    @property
    def battery_pct(self) -> float:
        return self._battery_pct

    @property
    def gps_fix(self) -> int:
        return self._gps_fix

    @property
    def home_set(self) -> bool:
        return self._home_set

    # ── MAVLink Commands ──────────────────────────────────────────────────────

    def set_mode(self, mode: str):
        """Set ArduPilot flight mode (e.g. 'GUIDED', 'LOITER', 'RTL')."""
        if self.sim_mode:
            log.info(f"[FC][SIM] Mode → {mode}")
            self._mode = mode
            return
        if not self._mav: return
        mode_id = self._mav.mode_mapping().get(mode)
        if mode_id is None:
            log.error(f"[FC] Unknown mode: {mode}")
            return
        self._mav.mav.set_mode_send(
            self._mav.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        log.info(f"[FC] Mode → {mode}")
        self._mode = mode

    def arm(self):
        """Arm the motors (requires GUIDED mode and GPS fix >= 3D)."""
        if self.sim_mode:
            log.info("[FC][SIM] ARMED")
            self._armed = True
            return
        if not self._mav: return
        self._mav.mav.command_long_send(
            self._mav.target_system,
            self._mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0   # param1=1 → arm
        )
        log.info("[FC] ARM command sent")

    def disarm(self):
        """Disarm the motors."""
        if self.sim_mode:
            log.info("[FC][SIM] DISARMED")
            self._armed = False
            return
        if not self._mav: return
        self._mav.mav.command_long_send(
            self._mav.target_system,
            self._mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 0, 0, 0, 0, 0, 0   # param1=0 → disarm
        )
        log.info("[FC] DISARM command sent")

    def takeoff(self, altitude_m: float):
        """Command takeoff to altitude_m above home (requires GUIDED + ARMED)."""
        if self.sim_mode:
            log.info(f"[FC][SIM] TAKEOFF to {altitude_m}m")
            self._alt_rel = altitude_m
            return
        if not self._mav: return
        self._mav.mav.command_long_send(
            self._mav.target_system,
            self._mav.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, altitude_m
        )
        log.info(f"[FC] TAKEOFF → {altitude_m}m")

    def send_velocity_ned(self, vx: float, vy: float, vz: float):
        """
        Send NED velocity setpoint to FC in GUIDED mode.
        vx = North (m/s), vy = East (m/s), vz = Down (m/s, positive = descend)

        ArduPilot will hold this velocity until next command.
        Must be called at >2Hz or FC falls back to LOITER.
        """
        if self.sim_mode:
            return
        if not self._mav: return

        # Clamp velocities
        vx = max(-MAX_VEL_XY, min(MAX_VEL_XY, vx))
        vy = max(-MAX_VEL_XY, min(MAX_VEL_XY, vy))
        vz = max(-MAX_VEL_Z,  min(MAX_VEL_Z,  vz))

        self._mav.mav.set_position_target_local_ned_send(
            int(time.time() * 1000) & 0xFFFFFFFF,  # timestamp ms
            self._mav.target_system,
            self._mav.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            # Type mask: ignore position + acceleration, USE velocity only
            0b0000_111111000111,
            0, 0, 0,              # x, y, z position (ignored)
            vx, vy, vz,           # velocity NED (m/s)
            0, 0, 0,              # acceleration (ignored)
            0, 0                  # yaw, yaw_rate (ignored)
        )

    def hover(self):
        """Stop in place — send zero velocity."""
        self.send_velocity_ned(0.0, 0.0, 0.0)

    def rtl(self):
        """Trigger Return To Launch mode."""
        self.set_mode('RTL')

    def land(self):
        """Trigger auto-land at current position."""
        self.set_mode('LAND')

    # ── Telemetry Loop ─────────────────────────────────────────────────────────

    def _telemetry_loop(self):
        """Background thread: reads MAVLink telemetry from FC."""
        if not self._mav: return
        log.info("[FC] Telemetry thread started")
        while True:
            try:
                msg = self._mav.recv_match(blocking=True, timeout=1.0)
                if msg is None:
                    continue

                msg_type = msg.get_type()

                if msg_type == 'HEARTBEAT':
                    self._last_hb_time = time.time()
                    self._armed = bool(msg.base_mode &
                                       mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

                elif msg_type == 'GLOBAL_POSITION_INT':
                    with self._lock:
                        self._alt_rel = msg.relative_alt / 1000.0   # mm → m
                        if not self._home_set and self._gps_fix >= 3:
                            self._home_lat = msg.lat / 1e7
                            self._home_lon = msg.lon / 1e7
                            self._home_set = True
                            log.info(f"[FC] Home set: "
                                     f"{self._home_lat:.6f}, {self._home_lon:.6f}")

                elif msg_type == 'GPS_RAW_INT':
                    self._gps_fix = msg.fix_type

                elif msg_type == 'BATTERY_STATUS':
                    if msg.battery_remaining >= 0:
                        self._battery_pct = float(msg.battery_remaining)

                elif msg_type == 'SYS_STATUS':
                    # Fallback battery from SYS_STATUS if BATTERY_STATUS not sent
                    if self._battery_pct == 100.0 and msg.battery_remaining >= 0:
                        self._battery_pct = float(msg.battery_remaining)

            except Exception as e:
                log.error(f"[FC] Telemetry error: {e}")
                time.sleep(0.1)


# =============================================================================
# Vision Thread — YOLO on USB Webcam
# =============================================================================

@dataclass
class CasualtyDetection:
    """Result of YOLO detection on current frame."""
    norm_cx    : float   = 0.0
    norm_cy    : float   = 0.0
    confidence : float   = 0.0
    bbox_area  : float   = 0.0   # normalised area
    detected   : bool    = False
    timestamp  : float   = field(default_factory=time.time)

    @property
    def is_fresh(self) -> bool:
        return (time.time() - self.timestamp) < 1.5


class VisionEngine:
    """
    Runs YOLO11n NCNN on USB webcam.
    Outputs thread-safe CasualtyDetection.
    Reuses exact same YoloNcnn wrapper as ground bots.
    """

    def __init__(self, camera_idx: int = 0, model_dir: str = YOLO_MODEL_DIR):
        self._cap       = None
        self._det       = CasualtyDetection()
        self._lock      = threading.Lock()
        self._running   = False
        self._camera_idx = camera_idx
        self._model_dir  = model_dir

        # Start inference thread
        t = threading.Thread(target=self._inference_loop, daemon=True)
        t.start()

    @property
    def latest(self) -> CasualtyDetection:
        with self._lock:
            return self._det

    def _inference_loop(self):
        log.info("[Vision] Loading YOLO11n NCNN model...")
        try:
            yolo = YoloNcnn(
                model_dir      = self._model_dir,
                input_size     = 320,
                conf_threshold = YOLO_CONF,
                num_threads    = YOLO_THREADS,   # Pi 5 — use all 4 cores
            )
        except Exception as e:
            log.error(f"[Vision] YOLO load failed: {e}")
            return

        log.info("[Vision] Opening camera...")
        cap = cv2.VideoCapture(self._camera_idx)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self._running = True
        log.info("[Vision] Inference loop active")

        frame_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        frame_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        frame_area = max(frame_w * frame_h, 1)

        while True:
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.1)
                continue

            detections = yolo.detect(frame)
            
            with stream_state.lock:
                disp = frame.copy()
                cv2.line(disp, (frame_w//2, 0), (frame_w//2, frame_h), (0,255,0), 1)
                cv2.line(disp, (0, frame_h//2), (frame_w, frame_h//2), (0,255,0), 1)
                for d in detections:
                    cx, cy = int(d.norm_cx * frame_w), int(d.norm_cy * frame_h)
                    cv2.circle(disp, (cx, cy), 15, (0,0,255), 2)
                    cv2.putText(disp, f"CASUALTY {d.confidence:.2f}", (cx-20, cy-20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)
                stream_state.frame = disp
            persons    = [d for d in detections if d.class_id == 0]

            if persons:
                # Pick highest confidence detection (aerial view strategy)
                best = max(persons, key=lambda d: d.confidence)
                bbox_area = (best.bbox_width * best.bbox_height) / frame_area
                det = CasualtyDetection(
                    norm_cx    = best.norm_cx,
                    norm_cy    = best.norm_cy,
                    confidence = best.confidence,
                    bbox_area  = bbox_area,
                    detected   = True,
                    timestamp  = time.time(),
                )
            else:
                det = CasualtyDetection(detected=False)

            with self._lock:
                self._det = det


# =============================================================================
# Swarm Mesh — XBee peer integration
# =============================================================================

class SwarmMesh:
    """
    Manages the 3-node XBee swarm (DRONE + Ground Bot A + Ground Bot B).
    Broadcasts drone's YOLO detections and receives ground bot detections.
    """

    def __init__(self, port: str, robot_id: str = ROBOT_ID):
        self.robot_id  = robot_id
        self._xbee     = None
        self._peers    = {}      # robot_id → {'cx', 'cy', 'cf', 'time'}
        self._lock     = threading.Lock()

        try:
            self._xbee = XBeeTransport(port)
            log.info(f"[Swarm] XBee connected on {port}")
        except Exception as e:
            log.warning(f"[Swarm] XBee not available: {e} — solo mode")

        # Start receive thread
        t = threading.Thread(target=self._recv_loop, daemon=True)
        t.start()

    @property
    def connected(self) -> bool:
        return self._xbee is not None and self._xbee.is_connected

    def broadcast(self, det: CasualtyDetection):
        """Broadcast this drone's detection to ground bots."""
        if not self.connected: return
        if det.detected:
            self._xbee.send({
                'r'  : self.robot_id,
                'cx' : round(det.norm_cx, 3),
                'cy' : round(det.norm_cy, 3),
                'cf' : round(det.confidence, 3),
                'sz' : round(det.bbox_area, 4),
                'st' : 'T',
            })
        else:
            self._xbee.send({'r': self.robot_id, 'st': 'L'})

    def get_swarm_best(self) -> Optional[dict]:
        """Return the freshest, highest-confidence detection across all ground peers."""
        with self._lock:
            now = time.time()
            fresh = {k: v for k, v in self._peers.items()
                     if (now - v['time']) < 2.0}
            if not fresh: return None
            return max(fresh.values(), key=lambda v: v['cf'])

    def _recv_loop(self):
        """Background thread: receive XBee packets from ground bots."""
        while True:
            if not self.connected:
                time.sleep(0.5)
                continue
            packet = self._xbee.recv()
            if packet and packet.get('r') != self.robot_id:
                peer_id = packet.get('r', '?')
                if packet.get('st') == 'T':
                    with self._lock:
                        self._peers[peer_id] = {
                            'cx'  : packet.get('cx', 0.5),
                            'cy'  : packet.get('cy', 0.5),
                            'cf'  : packet.get('cf', 0.0),
                            'time': time.time(),
                        }
                elif packet.get('st') == 'L':
                    with self._lock:
                        self._peers.pop(peer_id, None)
            time.sleep(0.02)


# =============================================================================
# Main Drone Controller
# =============================================================================

class DroneController:
    """
    Top-level controller. Runs the drone state machine.
    Integrates FC (MAVLink), Vision (YOLO), and Swarm (XBee) into
    a single coherent autonomous behaviour loop.
    """

    def __init__(self, args):
        self.args      = args
        self.state     = DroneState.DISARMED
        self._wp_index = 0                # current search waypoint index
        self._casualty_pos  = None        # (norm_cx, norm_cy) when found
        self._hover_start   = None        # timestamp when hover started
        self._last_xbee_tx  = 0.0

        log.info("=" * 60)
        log.info("  Sentinel Swarm — Drone Companion Computer")
        log.info(f"  FC Port:   {FC_SERIAL_PORT} @ {FC_BAUD_RATE} baud")
        log.info(f"  XBee Port: {XBEE_PORT}")
        log.info(f"  Camera:    /dev/video{CAMERA_DEVICE}")
        log.info(f"  Robot ID:  {ROBOT_ID}")
        log.info(f"  Cruise Alt:{args.alt}m | RTB at {RTB_BATTERY_PCT}% battery")
        log.info("=" * 60)

        # Initialise subsystems
        self.fc    = FCInterface(FC_SERIAL_PORT, FC_BAUD_RATE, sim_mode=args.sim)
        self.vision = VisionEngine(CAMERA_DEVICE, YOLO_MODEL_DIR)
        self.swarm  = SwarmMesh(XBEE_PORT) if not args.no_xbee else None

        # Wait for FC link
        if not args.sim:
            log.info("[Main] Waiting for FC link...")
            for _ in range(30):
                if self.fc.link_alive:
                    break
                time.sleep(0.5)
            if not self.fc.link_alive:
                log.error("[Main] FC not responding! Check wiring.")
                sys.exit(1)

    # ── State Machine ─────────────────────────────────────────────────────────

    def run(self):
        """Main control loop — runs at CONTROL_HZ."""
        log.info("[Main] Control loop started")
        dt = 1.0 / CONTROL_HZ

        while True:
            t0 = time.time()

            # Safety watchdog — FC link dead → hover
            if not self.args.sim and not self.fc.link_alive:
                log.warning("[Main] FC link lost! Hovering...")
                self.fc.hover()
                time.sleep(dt)
                continue

            # Get latest detections
            det = self.vision.latest

            # XBee broadcast at 2Hz
            now = time.time()
            if self.swarm and (now - self._last_xbee_tx) > (1.0 / XBEE_BROADCAST_HZ):
                self.swarm.broadcast(det)
                self._last_xbee_tx = now

            # ── State transitions ──
            new_state = self._step(det)
            if new_state != self.state:
                log.info(f"[Main] State: {self.state.value} → {new_state.value}")
                self.state = new_state

            elapsed = time.time() - t0
            sleep_t = max(0, dt - elapsed)
            time.sleep(sleep_t)

    def _step(self, det: CasualtyDetection) -> DroneState:
        """One control tick. Returns the next state."""
        state = self.state
        batt  = self.fc.battery_pct
        alt   = self.fc.altitude

        # ── DISARMED ──────────────────────────────────────────────────────────
        if state == DroneState.DISARMED:
            log.info(f"[DISARMED] GPS fix={self.fc.gps_fix} "
                     f"batt={batt:.0f}% armed={self.fc.armed}")
            if self.fc.gps_fix >= 3:
                self.fc.set_mode('GUIDED')
                time.sleep(1.0)
                self.fc.arm()
                return DroneState.ARMED
            return DroneState.DISARMED

        # ── ARMED ─────────────────────────────────────────────────────────────
        elif state == DroneState.ARMED:
            if self.fc.armed:
                time.sleep(2.0)  # Wait for ESCs to spin up
                self.fc.takeoff(self.args.alt)
                return DroneState.TAKEOFF
            return DroneState.ARMED

        # ── TAKEOFF ───────────────────────────────────────────────────────────
        elif state == DroneState.TAKEOFF:
            log.info(f"[TAKEOFF] Alt={alt:.1f}m / {self.args.alt}m")
            if alt >= self.args.alt * 0.9:
                self._wp_index = 0
                return DroneState.SEARCHING
            return DroneState.TAKEOFF

        # ── SEARCHING ─────────────────────────────────────────────────────────
        elif state == DroneState.SEARCHING:
            # RTB check
            if batt < RTB_BATTERY_PCT:
                return DroneState.RTB

            # Casualty found by own camera?
            if det.detected and det.is_fresh and det.confidence > 0.6:
                self._casualty_pos = (det.norm_cx, det.norm_cy)
                return DroneState.TRACKING

            # Ground bot reports casualty via XBee?
            if self.swarm:
                peer_best = self.swarm.get_swarm_best()
                if peer_best and peer_best['cf'] > 0.7:
                    log.info(f"[SEARCHING] Ground bot has target cf={peer_best['cf']:.2f}"
                             f" — pivoting to investigate")
                    return DroneState.TRACKING

            # Fly lawnmower pattern
            self._fly_search_pattern(alt)
            return DroneState.SEARCHING

        # ── TRACKING ──────────────────────────────────────────────────────────
        elif state == DroneState.TRACKING:
            if batt < RTB_BATTERY_PCT:
                return DroneState.RTB

            if det.detected and det.is_fresh:
                # Proportional control: steer toward casualty centre
                error_x = det.norm_cx - 0.5     # negative = left, positive = right
                error_y = det.norm_cy - 0.5     # negative = up in frame = forward

                vx =  error_y * KV_HORIZONTAL   # forward/back
                vy =  error_x * KV_HORIZONTAL   # left/right

                # Descend slowly while tracking if above TRACKING_ALT
                vz = 0.3 if alt > TRACKING_ALT else 0.0

                self.fc.send_velocity_ned(vx, vy, vz)

                log.info(f"[TRACKING] conf={det.confidence:.2f} "
                         f"err=({error_x:+.2f},{error_y:+.2f}) "
                         f"vel=({vx:.1f},{vy:.1f},{vz:.1f})")

                # Close enough + confident enough → switch to RELAYING
                if det.confidence > 0.75 and det.bbox_area > 0.05:
                    self._hover_start = time.time()
                    return DroneState.RELAYING

            else:
                # Lost target — hover and wait
                self.fc.hover()
                if not det.detected:
                    log.info("[TRACKING] Target lost — returning to search")
                    return DroneState.SEARCHING

            return DroneState.TRACKING

        # ── RELAYING ──────────────────────────────────────────────────────────
        elif state == DroneState.RELAYING:
            self.fc.hover()
            elapsed = time.time() - (self._hover_start or time.time())

            # Broadcast GPS coords of casualty to ground bots
            if det.detected:
                log.info(f"[RELAYING] 🚨 Casualty confirmed! "
                         f"Hovering {elapsed:.1f}s / {HOVER_LOITER_S}s")

            if elapsed > HOVER_LOITER_S:
                return DroneState.RTB

            if batt < RTB_BATTERY_PCT:
                return DroneState.RTB

            return DroneState.RELAYING

        # ── RTB ───────────────────────────────────────────────────────────────
        elif state == DroneState.RTB:
            log.info(f"[RTB] Battery={batt:.0f}% — returning to launch")
            self.fc.rtl()
            # Wait until altitude drops (FC is handling return)
            if alt < 0.5 and self.fc.armed:
                return DroneState.LANDING
            return DroneState.RTB

        # ── LANDING ───────────────────────────────────────────────────────────
        elif state == DroneState.LANDING:
            log.info("[LANDING] Touchdown — disarming")
            time.sleep(3.0)
            self.fc.disarm()
            return DroneState.DISARMED

        return state

    def _fly_search_pattern(self, current_alt: float):
        """
        Executes a simple lawnmower velocity sweep.
        Commands velocity toward next waypoint in SEARCH_PATTERN list.
        Advances waypoint index when within WAYPOINT_RADIUS.
        """
        if not SEARCH_PATTERN:
            self.fc.hover()
            return

        # Wrap around when pattern complete
        wp_idx = self._wp_index % len(SEARCH_PATTERN)
        target_x, target_y = SEARCH_PATTERN[wp_idx]

        # Without absolute positioning, use timed velocity steps
        # (Real GPS-based navigation would use SET_POSITION_TARGET_GLOBAL_INT)
        vx = 1.0   # fly at 1 m/s North for simple pattern
        vy = 0.0

        # Altitude hold: correct if drifted
        vz = 0.3 if current_alt < self.args.alt - 0.3 else (
            -0.2 if current_alt > self.args.alt + 0.3 else 0.0
        )

        self.fc.send_velocity_ned(vx, vy, vz)

        # Advance waypoint every 5 seconds (approximate)
        if not hasattr(self, '_wp_timer'):
            self._wp_timer = time.time()
        if time.time() - self._wp_timer > 5.0:
            self._wp_index = (self._wp_index + 1) % len(SEARCH_PATTERN)
            self._wp_timer = time.time()
            log.info(f"[SEARCH] Waypoint → {self._wp_index}/{len(SEARCH_PATTERN)}")


# =============================================================================
# Entry Point
# =============================================================================

# =============================================================================
# MJPEG Debug Stream Server
# =============================================================================
class StreamState:
    def __init__(self):
        self.frame = None
        self.lock = threading.Lock()

stream_state = StreamState()

class StreamingHandler(http.server.BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Age', 0)
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
            self.end_headers()
            try:
                while True:
                    with stream_state.lock:
                        frame = stream_state.frame
                    if frame is not None:
                        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                        img = Image.fromarray(rgb)
                        buf = io.BytesIO()
                        img.save(buf, format='JPEG', quality=60)
                        buf_bytes = buf.getvalue()

                        self.wfile.write(b'--FRAME\r\n')
                        self.send_header('Content-Type', 'image/jpeg')
                        self.send_header('Content-Length', len(buf_bytes))
                        self.end_headers()
                        self.wfile.write(buf_bytes)
                        self.wfile.write(b'\r\n')
                    time.sleep(0.05)
            except Exception as e:
                pass
        else:
            self.send_error(404)
            self.end_headers()

def stream_thread_func(port=5000):
    server = socketserver.ThreadingTCPServer(('0.0.0.0', port), StreamingHandler)
    server.serve_forever()

t_stream = threading.Thread(target=stream_thread_func, daemon=True)
t_stream.start()

def main():
    parser = argparse.ArgumentParser(description='Sentinel Drone Companion Computer')
    parser.add_argument('--sim',      action='store_true',
                        help='Simulation mode — no real FC connection')
    parser.add_argument('--no-xbee', action='store_true',
                        help='Disable XBee (solo mode, no swarm comms)')
    parser.add_argument('--alt',     type=float, default=CRUISE_ALT,
                        help=f'Cruise altitude in metres (default: {CRUISE_ALT})')
    parser.add_argument('--verbose', action='store_true',
                        help='Enable debug logging')
    args = parser.parse_args()

    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    controller = DroneController(args)
    try:
        controller.run()
    except KeyboardInterrupt:
        log.info("\n[Main] Ctrl+C — initiating safe RTL...")
        controller.fc.rtl()
        time.sleep(2.0)
        log.info("[Main] Shutdown complete.")













if __name__ == '__main__':
    main()


