"""
Sentinel_Ground.py — Symmetric Swarm Peer
=====================================
Runs identically on BOTH robots. Each robot:

  1. Captures camera frames and runs YOLO person detection.
  2. Broadcasts its best detection to its partner over XBee.
  3. Receives its partner's detection over XBee.
  4. Picks whichever detection (own or partner's) is more confident.
  5. Steers its motors toward the best known target position.
  6. Stops safely if both detections go stale (watchdog).
  7. Optionally streams annotated frames over HTTP (--stream flag).

Usage:
    ~/swarm_venv/bin/python ~/Sentinel_Ground.py --id A           # Robot A, no stream
    ~/swarm_venv/bin/python ~/Sentinel_Ground.py --id B           # Robot B, no stream
    ~/swarm_venv/bin/python ~/Sentinel_Ground.py --id A --stream  # Robot A + live stream on :5000

Stream is a background thread — it NEVER blocks YOLO inference or motor commands.
YOLO writes annotated frames to a shared buffer; the stream thread reads from it lazily.

No master. No slave. Both robots are equal peers.
"""

import sys
import os
import time
import json
import threading
import argparse
import logging
from pathlib import Path
from dataclasses import dataclass, field

import cv2
import serial
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn

# ── Resolve shared/ module path ───────────────────────────────────────────────
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "shared"))
try:
    from yolo_ncnn import YoloNcnn
    from xbee_transport import XBeeTransport
except ImportError:
    # Running directly from home dir on Pi — both files are in ~/
    sys.path.insert(0, str(Path.home()))
    from yolo_ncnn import YoloNcnn
    from xbee_transport import XBeeTransport

logging.basicConfig(level=logging.INFO, format='[%(asctime)s] %(message)s',
                    datefmt='%H:%M:%S')
log = logging.getLogger(__name__)

# ─── Configuration ────────────────────────────────────────────────────────────
ARDUINO_PORT   = '/dev/ttyUSB1'
XBEE_PORT      = '/dev/ttyUSB0'
CAMERA_DEVICE  = '/dev/video0'
BAUD_RATE      = 115200

# Motor tuning
KP_TURN        = 250
MAX_SPEED      = 120
MIN_SPEED      = 55
DEADZONE_STOP  = 0.12   # Stop turning when error < this
DEADZONE_START = 0.20   # Start turning when error > this

# Inversion (one motor is mounted mirrored on Custom Motor Driver)
INVERT_RIGHT   = True
INVERT_LEFT    = False

# How old (seconds) a detection can be before we ignore it
MY_STALE_S      = 1.0   # My own camera detection staleness limit
PARTNER_STALE_S = 2.0   # Partner's XBee message staleness limit

# ─── Shared State (thread-safe) ───────────────────────────────────────────────
@dataclass
class Target:
    """Represents a person detection from any source."""
    cx: float       = 0.5    # Normalised horizontal centre (0=left, 1=right)
    cy: float       = 0.5    # Normalised vertical centre
    confidence: float = 0.0  # YOLO confidence score
    source: str     = '?'    # 'self' or 'partner'
    timestamp: float = field(default_factory=time.time)

    @property
    def error_x(self):
        """Signed horizontal error from centre (-0.5 to +0.5)."""
        return self.cx - 0.5

    @property
    def age(self):
        return time.time() - self.timestamp

    def is_fresh(self, max_age):
        return self.age < max_age


class SwarmState:
    """Thread-safe container for shared swarm state."""
    def __init__(self):
        self._lock = threading.Lock()
        self.my_target      = None   # Latest detection from own camera
        self.partner_target = None   # Latest detection from XBee
        self.is_turning     = False  # Hysteresis state
        # Stream buffer — camera thread writes, stream thread reads lazily
        self._frame_lock    = threading.Lock()
        self.stream_frame   = None   # Latest annotated BGR frame (numpy array)

    def push_frame(self, frame):
        """Non-blocking frame push for stream. Drops old frame if not consumed."""
        with self._frame_lock:
            self.stream_frame = frame

    def pop_frame(self):
        """Non-blocking frame read for stream thread."""
        with self._frame_lock:
            return self.stream_frame

    def update_my(self, t: Target):
        with self._lock:
            self.my_target = t

    def update_partner(self, t: Target):
        with self._lock:
            self.partner_target = t

    def best_target(self) -> Target | None:
        """
        Returns the freshest, most confident target from either source.
        Priority:
          1. If only one source has a fresh target, use that.
          2. If both are fresh, use whichever has higher confidence.
          3. If neither is fresh, return None (triggers stop).
        """
        with self._lock:
            my = self.my_target if (self.my_target and
                                     self.my_target.is_fresh(MY_STALE_S)) else None
            pt = self.partner_target if (self.partner_target and
                                          self.partner_target.is_fresh(PARTNER_STALE_S)) else None
            if my and pt:
                return my if my.confidence >= pt.confidence else pt
            return my or pt


# ─── Motor Control ────────────────────────────────────────────────────────────
def init_serial(port, baud):
    try:
        ser = serial.Serial(port, baud, timeout=1)
        time.sleep(2)
        log.info(f"Arduino connected on {port}")
        return ser
    except Exception as e:
        log.warning(f"Arduino not available: {e}")
        return None


def send_motor(ser, left_spd, left_dir, right_spd, right_dir):
    if INVERT_LEFT:
        left_dir  = 'B' if left_dir  == 'F' else 'F'
    if INVERT_RIGHT:
        right_dir = 'B' if right_dir == 'F' else 'F'
    cmd = f"M,{int(left_spd)},{left_dir},{int(right_spd)},{right_dir}\n"
    if ser:
        ser.write(cmd.encode('ascii'))
    return cmd.strip()


def stop_motors(ser):
    return send_motor(ser, 0, 'F', 0, 'F')


# ─── Camera + YOLO Thread ─────────────────────────────────────────────────────
def camera_thread(state: SwarmState, robot_id: str, xbee: XBeeTransport):
    log.info("Loading YOLO model (1 thread)...")
    yolo = YoloNcnn(
        model_dir=os.path.expanduser('~/yolo11n_ncnn_model'),
        input_size=320,
        num_threads=1
    )

    cap = cv2.VideoCapture(CAMERA_DEVICE)
    if not cap.isOpened():
        cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

    log.info("Camera + YOLO ready. Entering detection loop...")

    while True:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.05)
            continue

        detections = yolo.detect(frame)
        persons    = [d for d in detections if d.class_id == 0]

        # ── Annotate frame for stream (cheap, always done) ────────────────
        h, w = frame.shape[:2]
        cv2.line(frame, (w//2, 0), (w//2, h), (100, 100, 255), 1)
        for d in detections:
            color = (0, 255, 0) if d.class_id == 0 else (180, 180, 180)
            cv2.rectangle(frame,
                          (int(d.bbox_x1), int(d.bbox_y1)),
                          (int(d.bbox_x2), int(d.bbox_y2)), color, 2)
            cv2.putText(frame, f"{d.class_name} {d.confidence:.2f}",
                        (int(d.bbox_x1), int(d.bbox_y1) - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

        if persons:
            best = max(persons, key=lambda p: p.bbox_width * p.bbox_height)
            # Highlight tracked target in bright green
            cv2.rectangle(frame,
                          (int(best.bbox_x1), int(best.bbox_y1)),
                          (int(best.bbox_x2), int(best.bbox_y2)),
                          (0, 255, 128), 3)
            t = Target(cx=best.norm_cx, cy=best.norm_cy,
                       confidence=best.confidence, source='self')
            state.update_my(t)

            # Label which robot this is and its confidence
            cv2.putText(frame, f"Bot {robot_id} | conf={best.confidence:.2f}",
                        (4, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 128), 1)

            xbee.send({'r': robot_id, 'cx': round(t.cx, 3),
                       'cy': round(t.cy, 3), 'cf': round(t.confidence, 2),
                       'st': 'T'})
        else:
            cv2.putText(frame, f"Bot {robot_id} | NO TARGET",
                        (4, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)
            xbee.send({'r': robot_id, 'st': 'L'})

        # Push annotated frame to stream buffer (non-blocking, O(1))
        state.push_frame(frame)


# ─── XBee Receive Thread ──────────────────────────────────────────────────────
def xbee_recv_thread(state: SwarmState, robot_id: str, xbee: XBeeTransport):
    log.info("XBee receive thread running...")
    while True:
        msg = xbee.recv()
        if msg and msg.get('r') != robot_id:   # Ignore own echoes
            if msg.get('st') == 'T':
                t = Target(
                    cx=msg.get('cx', 0.5),
                    cy=msg.get('cy', 0.5),
                    confidence=msg.get('cf', 0.0),
                    source='partner'
                )
                state.update_partner(t)
            elif msg.get('st') == 'L':
                state.update_partner(None)
        time.sleep(0.02)


# ─── MJPEG Stream Thread (optional, --stream flag) ────────────────────────────
STREAM_PORT = 5000

def make_stream_handler(state: SwarmState):
    """Factory so the handler can access shared state."""
    class StreamHandler(BaseHTTPRequestHandler):
        def log_message(self, *args): pass  # Silence HTTP access logs

        def do_GET(self):
            if self.path != '/':
                self.send_response(404); self.end_headers(); return
            self.send_response(200)
            self.send_header('Content-Type',
                             'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            while True:
                frame = state.pop_frame()
                if frame is None:
                    time.sleep(0.05)
                    continue
                ret, jpg = cv2.imencode('.jpg', frame,
                                        [cv2.IMWRITE_JPEG_QUALITY, 60])
                if not ret:
                    continue
                try:
                    self.wfile.write(
                        b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'
                        + jpg.tobytes() + b'\r\n')
                except Exception:
                    break  # Client disconnected
    return StreamHandler


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True


def stream_thread(state: SwarmState):
    handler = make_stream_handler(state)
    server  = ThreadedHTTPServer(('0.0.0.0', STREAM_PORT), handler)
    log.info(f"[Stream] Live feed at http://0.0.0.0:{STREAM_PORT}")
    server.serve_forever()


# ─── Main Control Loop ────────────────────────────────────────────────────────
def control_loop(state: SwarmState, ser):
    log.info("Motor control loop active.")
    last_log = 0

    while True:
        target = state.best_target()

        if target is None:
            # No fresh target from either source → stop
            if state.is_turning:
                stop_motors(ser)
                state.is_turning = False
            if time.time() - last_log > 1.0:
                log.info("No target (own or partner) — stopped.")
                last_log = time.time()
            time.sleep(0.05)
            continue

        error_x    = target.error_x
        turn_effort = error_x * KP_TURN
        speed      = min(max(abs(turn_effort), MIN_SPEED), MAX_SPEED)
        left_dir   = 'F' if turn_effort >= 0 else 'B'
        right_dir  = 'B' if turn_effort >= 0 else 'F'

        # Hysteresis deadzone
        if state.is_turning:
            if abs(error_x) < DEADZONE_STOP:
                state.is_turning = False
                cmd = stop_motors(ser)
            else:
                cmd = send_motor(ser, speed, left_dir, speed, right_dir)
        else:
            if abs(error_x) > DEADZONE_START:
                state.is_turning = True
                cmd = send_motor(ser, speed, left_dir, speed, right_dir)
            else:
                cmd = stop_motors(ser)

        if time.time() - last_log > 0.5:
            direction = "RIGHT" if error_x > 0 else "LEFT " if error_x < 0 else "CNTR "
            log.info(f"[{target.source:^7}] conf={target.confidence:.2f} "
                     f"err={error_x:+.2f} → {direction} | {cmd}")
            last_log = time.time()

        time.sleep(0.05)


# ─── Entry Point ─────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description='Swarm Bot — Symmetric Peer')
    parser.add_argument('--id', required=True, choices=['A', 'B'],
                        help='Robot identity: A or B')
    parser.add_argument('--stream', action='store_true',
                        help='Enable MJPEG stream on port 5000 (for debugging)')
    args = parser.parse_args()

    log.info(f"=== SWARM BOT — Robot {args.id} ===")
    log.info(f"XBee: {XBEE_PORT} | Arduino: {ARDUINO_PORT} | Camera: {CAMERA_DEVICE}")
    if args.stream:
        log.info(f"[Stream] Enabled — open http://<this-robot-ip>:{STREAM_PORT} in browser")

    # Initialise hardware
    ser  = init_serial(ARDUINO_PORT, BAUD_RATE)
    xbee = XBeeTransport(XBEE_PORT)
    state = SwarmState()

    if not xbee.is_connected:
        log.warning("XBee not connected — running in solo mode (own camera only).")

    try:
        # Spawn background threads
        threads = [
            threading.Thread(target=camera_thread,
                             args=(state, args.id, xbee), daemon=True),
            threading.Thread(target=xbee_recv_thread,
                             args=(state, args.id, xbee), daemon=True),
        ]
        # Optional stream thread — completely decoupled from YOLO and motors
        if args.stream:
            threads.append(
                threading.Thread(target=stream_thread,
                                 args=(state,), daemon=True)
            )
        for t in threads:
            t.start()

        # Main control loop (blocks until Ctrl+C)
        control_loop(state, ser)

    except KeyboardInterrupt:
        log.info("Ctrl+C — shutting down...")
    finally:
        stop_motors(ser)
        if ser:
            ser.close()
        xbee.close()
        log.info("Safe shutdown complete.")


if __name__ == '__main__':
    main()
