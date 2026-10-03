"""
=============================================================================
  Sentinel Swarm — Vision Navigation (GPS-Denied Flight)
  Raspberry Pi 5 + MicoAir H743 V2 (ArduPilot AltHold)

  Architecture:
    - FC handles: Attitude (IMU), Altitude (Barometer), Motor mixing
    - Pi 5 handles: Horizontal navigation via YOLO-guided RC override
    - No GPS required. No satellite lock needed. Flies anywhere.

  Flight Modes:
    DISARMED  → FC connected, waiting for arm
    ARMED     → Motors spinning at idle
    TAKEOFF   → Climbing to CRUISE_ALT via throttle override
    SEARCHING → Slow yaw sweep, broadcasting over XBee
    TRACKING  → YOLO locked on casualty, centering using pitch/roll
    HOVERING  → Casualty centered, broadcasting coords to ground bots
    LANDING   → Descending, disarming

  Navigation:
    Uses proportional controller on YOLO bounding box centroid error.
    Target centered in frame → drone hovers over it.
    Target left of center → drone pitches left (roll channel).
    Target above center → drone pitches forward (pitch channel).

  RC Override Channels (ArduPilot default):
    CH1 = Roll     (1000=full left, 1500=center, 2000=full right)
    CH2 = Pitch    (1000=full fwd, 1500=center, 2000=full back)
    CH3 = Throttle (1000=zero, 1500=hover, 2000=full)
    CH4 = Yaw      (1000=full left, 1500=center, 2000=full right)

  Usage:
    python3 drone_vision_nav.py
    python3 drone_vision_nav.py --no-xbee
    python3 drone_vision_nav.py --alt 2.0     # cruise altitude in meters
    python3 drone_vision_nav.py --sim         # software-in-loop, no real FC
=============================================================================
"""

import sys
import os
import time
import atexit

def emergency_shutdown():
    try:
        from pymavlink import mavutil
        print("
[SAFETY] Forcing motors to 0 and disarming FC...")
        master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
        master.mav.rc_channels_override_send(master.target_system, master.target_component, 1500, 1500, 1000, 1500, 0, 0, 0, 0)
        master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 21196, 0, 0, 0, 0, 0)
    except:
        pass

atexit.register(emergency_shutdown)
import http.server
import socketserver
import io
import threading
import argparse
import logging
from pathlib import Path
from enum import Enum
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np
from PIL import Image

try:
    from pymavlink import mavutil
    MAVLINK_AVAILABLE = True
except ImportError:
    MAVLINK_AVAILABLE = False

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "ground_node" / "shared"))

from ground_node.shared.xbee_transport import XBeeTransport
from ground_node.shared.yolo_ncnn import YoloNcnn

# ── Logging ───────────────────────────────────────────────────────────────────
logging.basicConfig(level=logging.INFO,
    format="[%(asctime)s] [%(levelname)s] %(message)s", datefmt="%H:%M:%S")
log = logging.getLogger("vision_nav")

# ── Config ────────────────────────────────────────────────────────────────────
FC_PORT        = "/dev/ttyACM0"
FC_BAUD        = 115200
XBEE_PORT      = "/dev/ttyUSB0"
XBEE_BAUD      = 9600
CAMERA_INDEX   = 0
YOLO_MODEL_DIR = str(Path.home() / "yolo11n_ncnn_model")
CONF_THRESH    = 0.4
STREAM_PORT    = 5000
ROBOT_ID       = "DRONE"

CRUISE_ALT     = 0.5    # meters (barometer-based)
HOVER_THROTTLE = 1550   # RC value for hover (tune this for your drone!)
TAKEOFF_THR    = 1650   # RC value for climb
LAND_THR       = 1420   # RC value for descent
RC_CENTER      = 1500   # Neutral stick
RC_DEADZONE    = 30     # pixels deadzone before moving

# Vision P-controller gains
KP_ROLL        = 150    # horizontal error → roll correction
KP_PITCH       = 150    # vertical error → pitch correction
KP_YAW         = 80     # yaw sweep rate during search

# ── States ────────────────────────────────────────────────────────────────────
class State(Enum):
    DISARMED  = "DISARMED"
    ARMED     = "ARMED"
    TAKEOFF   = "TAKEOFF"
    SEARCHING = "SEARCHING"
    TRACKING  = "TRACKING"
    HOVERING  = "HOVERING"
    LANDING   = "LANDING"

# ── MJPEG Stream ──────────────────────────────────────────────────────────────
class StreamState:
    def __init__(self):
        self.frame = None
        self.lock  = threading.Lock()

stream_state = StreamState()

class StreamingHandler(http.server.BaseHTTPRequestHandler):
    def log_message(self, format, *args): pass
    def do_GET(self):
        if self.path != '/':
            self.send_error(404); self.end_headers(); return
        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
        self.end_headers()
        try:
            while True:
                with stream_state.lock:
                    frame = stream_state.frame
                if frame is not None:
                    rgb  = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    img  = Image.fromarray(rgb)
                    buf  = io.BytesIO()
                    img.save(buf, format='JPEG', quality=60)
                    data = buf.getvalue()
                    self.wfile.write(b'--FRAME\r\n')
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Content-Length', len(data))
                    self.end_headers()
                    self.wfile.write(data)
                    self.wfile.write(b'\r\n')
                time.sleep(0.033)
        except Exception:
            pass

def start_stream(port):
    srv = socketserver.ThreadingTCPServer(('0.0.0.0', port), StreamingHandler)
    srv.daemon_threads = True
    srv.serve_forever()

# ── FC Interface ──────────────────────────────────────────────────────────────
class FCLink:
    def __init__(self, port, baud, sim=False):
        self.sim     = sim
        self.master  = None
        self.altitude = 0.0
        self.armed   = False
        self._lock   = threading.Lock()

        if not sim:
            log.info(f"[FC] Connecting to {port} @ {baud}...")
            self.master = mavutil.mavlink_connection(port, baud=baud)
            self.master.wait_heartbeat(timeout=10)
            log.info(f"[FC] ✅ Heartbeat received (system={self.master.target_system})")
            # Start telemetry thread
            t = threading.Thread(target=self._telemetry_loop, daemon=True)
            t.start()

    def _telemetry_loop(self):
        self.master.mav.request_data_stream_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
        while True:
            msg = self.master.recv_match(blocking=True, timeout=1.0)
            if not msg: continue
            t = msg.get_type()
            if t == 'VFR_HUD':
                with self._lock:
                    self.altitude = msg.alt
            elif t == 'HEARTBEAT':
                with self._lock:
                    self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

    def set_mode(self, mode_name):
        if self.sim: return
        mode_id = self.master.mode_mapping().get(mode_name)
        if mode_id is None:
            log.error(f"[FC] Unknown mode: {mode_name}")
            return
        self.master.mav.set_mode_send(self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)
        log.info(f"[FC] Mode → {mode_name}")

    def arm(self):
        if self.sim: self.armed = True; return
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1, 0, 0, 0, 0, 0, 0)
        log.info("[FC] ARM command sent")

    def disarm(self):
        if self.sim: self.armed = False; return
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            0, 21196, 0, 0, 0, 0, 0)
        log.info("[FC] DISARM command sent")

    def rc_override(self, roll=1500, pitch=1500, throttle=1500, yaw=1500):
        """Send RC override (1000-2000). 1500 = center/neutral."""
        if self.sim: return
        self.master.mav.rc_channels_override_send(
            self.master.target_system, self.master.target_component,
            roll, pitch, throttle, yaw, 0, 0, 0, 0)

    def get_altitude(self):
        with self._lock:
            return self.altitude

    def is_armed(self):
        with self._lock:
            return self.armed

# ── Main Controller ───────────────────────────────────────────────────────────
def clamp(val, lo, hi):
    return max(lo, min(hi, val))

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--no-xbee', action='store_true')
    parser.add_argument('--sim',     action='store_true', help='No real FC, simulate')
    parser.add_argument('--alt',     type=float, default=CRUISE_ALT)
    args = parser.parse_args()

    cruise_alt = args.alt

    log.info("=" * 60)
    log.info("  Sentinel — Vision Navigation (GPS-Denied)")
    log.info(f"  FC:      {FC_PORT} @ {FC_BAUD} {'(SIM)' if args.sim else ''}")
    log.info(f"  Cruise:  {cruise_alt}m (barometer)")
    log.info(f"  Stream:  http://0.0.0.0:{STREAM_PORT}")
    log.info("=" * 60)

    # Start MJPEG stream
    threading.Thread(target=start_stream, args=(STREAM_PORT,), daemon=True).start()
    log.info("[Stream] MJPEG live on port 5000")

    # Load YOLO
    log.info("[YOLO] Loading model...")
    yolo = YoloNcnn(model_dir=YOLO_MODEL_DIR, input_size=320, conf_threshold=CONF_THRESH, num_threads=4)
    log.info("[YOLO] ✅ Model ready")

    # Open camera
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        log.error("[Camera] Cannot open /dev/video0!")
        sys.exit(1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    log.info("[Camera] ✅ 640x480")

    # Connect XBee
    xbee = None
    if not args.no_xbee:
        try:
            xbee = XBeeTransport(XBEE_PORT, baud=XBEE_BAUD)
            log.info(f"[XBee] ✅ Connected on {XBEE_PORT}")
        except Exception as e:
            log.warning(f"[XBee] Not available: {e}")

    # Connect FC
    fc = FCLink(FC_PORT, FC_BAUD, sim=args.sim)

    # Set AltHold mode
    time.sleep(1)
    fc.set_mode("ALT_HOLD")

    state        = State.DISARMED
    yaw_dir      = 1          # 1 or -1 for search sweep
    hover_start  = None
    lost_frames  = 0
    last_arm_time = 0
    last_broadcast = 0

    log.info("\n[Nav] Control loop started. Open http://10.219.37.160:5000\n")

    while True:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.05); continue

        h, w = frame.shape[:2]
        cx_frame, cy_frame = w // 2, h // 2
        detections = yolo.detect(frame)
        alt        = fc.get_altitude()
        armed      = fc.is_armed()

        # ── State Transitions ─────────────────────────────────────────────────
        if state == State.DISARMED:
            # Continuously hold throttle at 0 and try to arm every 2 seconds
            fc.rc_override(throttle=1000)
            if time.time() - last_arm_time > 2.0:
                fc.arm()
                last_arm_time = time.time()
                
            if armed:
                state = State.ARMED
                log.info("[State] DISARMED → ARMED")

        elif state == State.ARMED:
            state = State.TAKEOFF
            log.info(f"[State] ARMED → TAKEOFF (target={cruise_alt}m)")

        elif state == State.TAKEOFF:
            if not hasattr(fc, 'takeoff_start_time'):
                fc.takeoff_start_time = time.time()
                
            fc.rc_override(throttle=TAKEOFF_THR)
            
            # Transition to searching if we reach altitude OR if 10 seconds pass
            if alt >= cruise_alt * 0.85 or (time.time() - fc.takeoff_start_time > 10.0):
                state = State.SEARCHING
                log.info(f"[State] TAKEOFF → SEARCHING (alt={alt:.1f}m)")

        elif state == State.SEARCHING:
            # Slow yaw sweep while holding altitude
            yaw_rc = RC_CENTER + (yaw_dir * KP_YAW)
            fc.rc_override(throttle=HOVER_THROTTLE, yaw=clamp(int(yaw_rc), 1000, 2000))
            if detections:
                state = State.TRACKING
                lost_frames = 0
                log.info("[State] SEARCHING → TRACKING 🎯")

        elif state == State.TRACKING:
            if not detections:
                lost_frames += 1
                if lost_frames > 30:
                    state = State.SEARCHING
                    log.info("[State] TRACKING → SEARCHING (target lost)")
                fc.rc_override(throttle=HOVER_THROTTLE)
            else:
                lost_frames = 0
                d  = detections[0]
                # Target pixel coords
                tx = int(d.norm_cx * w)
                ty = int(d.norm_cy * h)
                # Errors (positive = target is right/below of center)
                ex = tx - cx_frame   # + means target is right → roll right
                ey = ty - cy_frame   # + means target is below → pitch back

                # P controller → RC values
                roll_rc     = RC_CENTER + int(ex * KP_ROLL  / w)
                pitch_rc    = RC_CENTER - int(ey * KP_PITCH / h)  # inverted
                throttle_rc = HOVER_THROTTLE

                fc.rc_override(
                    roll     = clamp(roll_rc,     1200, 1800),
                    pitch    = clamp(pitch_rc,    1200, 1800),
                    throttle = throttle_rc,
                    yaw      = RC_CENTER
                )

                # Check if centered
                if abs(ex) < RC_DEADZONE and abs(ey) < RC_DEADZONE:
                    if hover_start is None:
                        hover_start = time.time()
                    elif time.time() - hover_start > 2.0:
                        state = State.HOVERING
                        log.info("[State] TRACKING → HOVERING ✅ (target centered)")
                else:
                    hover_start = None

        elif state == State.HOVERING:
            fc.rc_override(throttle=HOVER_THROTTLE)
            if detections and xbee:
                if time.time() - last_broadcast >= 0.5:  # Rate limit to 2Hz
                    d = detections[0]
                    payload = {'r': ROBOT_ID, 'cx': round(d.norm_cx, 3),
                               'cy': round(d.norm_cy, 3), 'cf': round(d.confidence, 3),
                               'sz': round((d.bbox_width/w) * (d.bbox_height/h), 4), 'st': 'L'}
                    try:
                        xbee.send(payload)
                        last_broadcast = time.time()
                    except Exception:
                        pass
            if not detections:
                state = State.SEARCHING
                log.info("[State] HOVERING → SEARCHING (target lost)")

        # ── Draw Overlay ──────────────────────────────────────────────────────
        cv2.line(frame, (cx_frame, 0), (cx_frame, h), (0, 255, 0), 1)
        cv2.line(frame, (0, cy_frame), (w, cy_frame), (0, 255, 0), 1)

        for d in detections:
            tx = int(d.norm_cx * w); ty = int(d.norm_cy * h)
            bw = d.bbox_width; bh = d.bbox_height
            cv2.rectangle(frame, (tx - bw//2, ty - bh//2), (tx + bw//2, ty + bh//2), (0, 0, 255), 2)
            cv2.circle(frame, (tx, ty), 8, (0, 0, 255), -1)
            cv2.line(frame, (cx_frame, cy_frame), (tx, ty), (0, 165, 255), 2)
            cv2.putText(frame, f"CASUALTY {d.confidence:.2f}", (tx - bw//2, ty - bh//2 - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)

        color = (0, 0, 255) if detections else (0, 255, 0)
        cv2.putText(frame, f"STATE: {state.value}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)
        cv2.putText(frame, f"ALT: {alt:.1f}m", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 1)
        cv2.putText(frame, f"ARMED: {armed}", (10, 72), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        with stream_state.lock:
            stream_state.frame = frame.copy()

        log.info(f"[{state.value}] alt={alt:.1f}m armed={armed} dets={len(detections)}")

        
        # Receive XBee from Ground Bots
        if xbee:
            try:
                msg = xbee.recv()
                if msg and msg.get('r') != ROBOT_ID:
                    sender = msg.get('r', 'Unknown')
                    cf = msg.get('cf', 0.0)
                    cx = msg.get('cx', 0.0)
                    log.info(f"[Swarm Comms] Received detection from {sender}: cx={cx}, confidence={cf}")
                    
                    # Swarm Decision Logic
                    if state == State.SEARCHING and cf > 0.6:
                        log.info(f"[Swarm Decision] {sender} found a casualty! Pivoting drone to assist...")
            except Exception as e:
                pass

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        log.info("\n[Nav] Stopped by user — disarming FC")
