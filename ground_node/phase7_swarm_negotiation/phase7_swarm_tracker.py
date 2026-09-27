"""
=============================================================================
  Sentinel Swarm — Phase 7: Full Cooperative Swarm Test Entry Point
  Replaces phase4_tracking/ground_tracker.py for Phase 7 testing
=============================================================================

Run on BOTH robots simultaneously:
  Robot A: python phase7_swarm_tracker.py --id A
  Robot B: python phase7_swarm_tracker.py --id B [--stream]

Changes from Phase 4:
  - SwarmNegotiator replaces the simple SwarmState confidence comparison.
  - Leader election happens every 100ms via weighted scoring.
  - The HUD now shows role (LEADER / FOLLOWER / SCANNING / SOLO) + score.
  - Target size (bbox area) is now transmitted so both robots know if the
    other has a bigger / closer view.
  - Scan sweep: if no robot sees the target for 3+ seconds, both robots
    slowly rotate in opposite directions to re-acquire (split search).
"""

import sys
import os
import time
import threading
import argparse
import logging
import serial
import cv2

# ── Path setup ───────────────────────────────────────────────────────────────
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from shared.xbee_transport import XBeeTransport
from shared.yolo_ncnn      import YoloNcnn
from phase7_swarm_negotiation.swarm_negotiator import (
    SwarmNegotiator, RobotRole, annotate_frame_phase7
)

# ── Logging ──────────────────────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    datefmt='%H:%M:%S'
)
log = logging.getLogger(__name__)

# ── Hardware Config ───────────────────────────────────────────────────────────
XBEE_PORT     = '/dev/ttyUSB0'
ARDUINO_PORT  = '/dev/ttyUSB1'
BAUD_RATE     = 115200
CAMERA_DEVICE = 0

YOLO_MODEL_DIR = os.path.expanduser('~/yolo11n_ncnn_model')

# ── Motor Control ─────────────────────────────────────────────────────────────
MIN_SPEED    = 80
MAX_SPEED    = 200
KP_TURN      = 220.0
DEADZONE_START = 0.08
DEADZONE_STOP  = 0.04

# Scan sweep speed (used when no target in entire swarm)
SCAN_SPEED = 70


# ─────────────────────────────────────────────────────────────────────────────
# Serial Motor Helpers (identical to phase4 — backward compatible)
# ─────────────────────────────────────────────────────────────────────────────

def init_serial(port, baud):
    try:
        ser = serial.Serial(port, baud, timeout=1)
        time.sleep(2)
        log.info(f"[Arduino] Serial open on {port}")
        return ser
    except Exception as e:
        log.warning(f"[Arduino] Cannot open {port}: {e}. Motor commands disabled.")
        return None


def send_motor(ser, lspeed, ldir, rspeed, rdir) -> str:
    cmd = f"M,{lspeed},{ldir},{rspeed},{rdir}\n"
    if ser:
        try:
            ser.write(cmd.encode('ascii'))
        except Exception as e:
            log.error(f"[Motor] Write error: {e}")
    return cmd.strip()


def stop_motors(ser) -> str:
    return send_motor(ser, 0, 'F', 0, 'F')


def scan_rotate(ser, robot_id: str):
    """
    Robot A rotates right, Robot B rotates left to cover both directions.
    Called when SCANNING role is active.
    """
    if robot_id == 'A':
        send_motor(ser, SCAN_SPEED, 'F', SCAN_SPEED, 'B')
    else:
        send_motor(ser, SCAN_SPEED, 'B', SCAN_SPEED, 'F')


# ─────────────────────────────────────────────────────────────────────────────
# Camera Thread
# ─────────────────────────────────────────────────────────────────────────────

def camera_thread(negotiator: SwarmNegotiator, robot_id: str, stream_state: dict):
    """
    Runs YOLO inference and feeds detections into the SwarmNegotiator.
    Also renders the Phase 7 HUD and pushes annotated frames to stream buffer.
    """
    log.info("[Camera] Initializing YOLO NCNN model...")
    detector = YoloNcnn(YOLO_MODEL_DIR, num_threads=1, conf_threshold=0.40)

    cap = cv2.VideoCapture(CAMERA_DEVICE)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    if not cap.isOpened():
        log.error("[Camera] Cannot open camera!")
        return

    log.info("[Camera] YOLO running.")
    frame_w, frame_h = 640, 480
    frame_area = frame_w * frame_h

    while True:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.05)
            continue

        detections = detector.detect(frame)

        if detections:
            # Pick highest-confidence detection
            best = max(detections, key=lambda d: d.confidence)

            # Compute normalized target size (bbox area / frame area)
            bbox_area    = (best.bbox_x2 - best.bbox_x1) * (best.bbox_y2 - best.bbox_y1)
            target_size  = min(1.0, bbox_area / frame_area)

            # Register with negotiator (also XBee-broadcasts it)
            negotiator.register_self_detection(
                cx          = best.norm_cx,
                cy          = best.norm_cy,
                confidence  = best.confidence,
                target_size = target_size,
            )

            # Draw local detection box
            cv2.rectangle(frame,
                          (int(best.bbox_x1), int(best.bbox_y1)),
                          (int(best.bbox_x2), int(best.bbox_y2)),
                          (200, 200, 200), 1)

            self_conf = best.confidence
        else:
            negotiator.register_self_lost()
            self_conf = 0.0

        # Annotate with Phase 7 HUD
        decision = negotiator.get_steering_target()
        frame    = annotate_frame_phase7(frame, robot_id, decision, self_conf)

        # Push to stream buffer (non-blocking)
        stream_state['frame'] = frame

    cap.release()


# ─────────────────────────────────────────────────────────────────────────────
# XBee Receive Thread
# ─────────────────────────────────────────────────────────────────────────────

def xbee_recv_thread(negotiator: SwarmNegotiator, xbee: XBeeTransport):
    """Receives XBee packets and feeds them into the negotiator."""
    log.info("[XBee] Receive thread running.")
    while True:
        msg = xbee.recv()
        if msg:
            negotiator.ingest_xbee_packet(msg)
        time.sleep(0.02)


# ─────────────────────────────────────────────────────────────────────────────
# Main Control Loop
# ─────────────────────────────────────────────────────────────────────────────

def control_loop(negotiator: SwarmNegotiator, ser, robot_id: str):
    """
    Uses the Phase 7 negotiator to get the swarm-wide best target.
    Drives motors based on the elected leader's target coordinates.
    """
    log.info("[Control] Phase 7 motor control loop active.")
    is_turning = False
    last_log   = 0.0

    while True:
        decision = negotiator.get_steering_target()

        # ── No target anywhere in the swarm ──────────────────────────────────
        if decision is None:
            role = negotiator.current_role

            if role == RobotRole.SCANNING:
                # Active scan sweep — rotate to re-acquire
                scan_rotate(ser, robot_id)
                is_turning = True
            else:
                if is_turning:
                    stop_motors(ser)
                    is_turning = False

            if time.time() - last_log > 1.0:
                log.info(f"[Control] {role.value} — no target.")
                last_log = time.time()

            time.sleep(0.05)
            continue

        # ── Target found (from self or peer via negotiation) ─────────────────
        error_x      = decision.source_cx - 0.5
        turn_effort  = error_x * KP_TURN
        speed        = min(max(abs(turn_effort), MIN_SPEED), MAX_SPEED)
        left_dir     = 'F' if turn_effort >= 0 else 'B'
        right_dir    = 'B' if turn_effort >= 0 else 'F'

        if is_turning:
            if abs(error_x) < DEADZONE_STOP:
                is_turning = False
                stop_motors(ser)
            else:
                send_motor(ser, speed, left_dir, speed, right_dir)
        else:
            if abs(error_x) > DEADZONE_START:
                is_turning = True
                send_motor(ser, speed, left_dir, speed, right_dir)
            else:
                stop_motors(ser)

        if time.time() - last_log > 0.5:
            direction = "RIGHT" if error_x > 0 else "LEFT " if error_x < 0 else "CNTR "
            log.info(
                f"[{decision.role.value:^8}] leader={decision.leader_id} "
                f"score={decision.score:.3f} conf={decision.confidence:.2f} "
                f"err={error_x:+.2f} → {direction}"
            )
            last_log = time.time()

        time.sleep(0.05)


# ─────────────────────────────────────────────────────────────────────────────
# Optional MJPEG Stream
# ─────────────────────────────────────────────────────────────────────────────

def stream_thread(stream_state: dict, port: int = 5000):
    """Simple MJPEG server for debug streaming."""
    from http.server import BaseHTTPRequestHandler, HTTPServer
    from socketserver import ThreadingMixIn

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args): pass
        def do_GET(self):
            if self.path != '/':
                self.send_response(404); self.end_headers(); return
            self.send_response(200)
            self.send_header('Content-Type',
                             'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            while True:
                frame = stream_state.get('frame')
                if frame is None:
                    time.sleep(0.05); continue
                ret, jpg = cv2.imencode('.jpg', frame,
                                        [cv2.IMWRITE_JPEG_QUALITY, 60])
                if not ret: continue
                try:
                    self.wfile.write(
                        b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'
                        + jpg.tobytes() + b'\r\n')
                except Exception:
                    break

    class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
        daemon_threads = True

    server = ThreadedHTTPServer(('0.0.0.0', port), Handler)
    log.info(f"[Stream] Live feed at http://0.0.0.0:{port}")
    server.serve_forever()


# ─────────────────────────────────────────────────────────────────────────────
# Entry Point
# ─────────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description='Sentinel Swarm — Phase 7')
    parser.add_argument('--id', required=True, choices=['A', 'B', 'C', 'D'],
                        help='Robot identity (A / B / C / D for multi-node)')
    parser.add_argument('--stream', action='store_true',
                        help='Enable MJPEG debug stream on port 5000')
    parser.add_argument('--verbose', action='store_true',
                        help='Enable debug-level logging')
    args = parser.parse_args()

    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    log.info(f"=== SENTINEL SWARM — Phase 7 | Robot {args.id} ===")

    # ── Hardware init ─────────────────────────────────────────────────────────
    ser  = init_serial(ARDUINO_PORT, BAUD_RATE)
    xbee = XBeeTransport(XBEE_PORT)

    if not xbee.is_connected:
        log.warning("[XBee] Not connected — running in SOLO mode (own camera only).")

    # ── Phase 7 Negotiator ────────────────────────────────────────────────────
    negotiator = SwarmNegotiator(
        robot_id       = args.id,
        xbee           = xbee,
        xbee_link_alive_fn = xbee.link_alive,
    )

    # ── Shared stream buffer ──────────────────────────────────────────────────
    stream_state = {'frame': None}

    # ── Threads ───────────────────────────────────────────────────────────────
    threads = [
        threading.Thread(target=camera_thread,
                         args=(negotiator, args.id, stream_state), daemon=True),
        threading.Thread(target=xbee_recv_thread,
                         args=(negotiator, xbee), daemon=True),
    ]
    if args.stream:
        threads.append(
            threading.Thread(target=stream_thread,
                             args=(stream_state,), daemon=True)
        )
    for t in threads:
        t.start()

    try:
        control_loop(negotiator, ser, args.id)
    except KeyboardInterrupt:
        log.info("Ctrl+C — shutting down.")
    finally:
        stop_motors(ser)
        if ser: ser.close()
        xbee.close()
        log.info("Safe shutdown complete.")


if __name__ == '__main__':
    main()
