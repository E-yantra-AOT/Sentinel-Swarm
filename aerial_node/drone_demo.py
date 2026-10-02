"""
=============================================================================
  Sentinel Swarm — Drone DEMO Mode (GPS-Free)
  Raspberry Pi 5 (8GB)

  This script is a standalone demo that runs the full Sentinel Drone
  AI + Swarm stack WITHOUT requiring a GPS lock or Flight Controller.

  What it does:
    - Opens the USB webcam (/dev/video0)
    - Runs YOLO11n NCNN casualty detection in real-time
    - Streams annotated live feed via HTTP MJPEG on port 5000
    - Broadcasts any detections to Ground Bots over XBee mesh
    - Receives detections from Ground Bots and prints them

  What it does NOT do:
    - Connect to Flight Controller
    - Require GPS
    - Arm motors or fly

  Usage:
    python3 drone_demo.py
    python3 drone_demo.py --no-xbee     # Run without XBee (camera + YOLO only)
    python3 drone_demo.py --port 5001   # Custom stream port

  Stream URL: http://<pi5-ip>:5000
=============================================================================
"""

import sys
import os
import time
import http.server
import socketserver
import io
import threading
import argparse
import logging
from pathlib import Path
from dataclasses import dataclass

import cv2
import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "ground_node" / "shared"))

from ground_node.shared.xbee_transport import XBeeTransport
from ground_node.shared.yolo_ncnn import YoloNcnn

# ── Logging ───────────────────────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S"
)
log = logging.getLogger("drone_demo")

# ── Config ────────────────────────────────────────────────────────────────────
ROBOT_ID       = "DRONE"
XBEE_PORT      = "/dev/ttyUSB0"
XBEE_BAUD      = 9600
CAMERA_INDEX   = 0
YOLO_MODEL_DIR = str(Path.home() / "yolo11n_ncnn_model")
CONF_THRESH    = 0.4
STREAM_PORT    = 5000

# ── MJPEG Stream ──────────────────────────────────────────────────────────────
class StreamState:
    def __init__(self):
        self.frame = None
        self.lock  = threading.Lock()

stream_state = StreamState()

class StreamingHandler(http.server.BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass  # Silence request logs

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
                        rgb  = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                        img  = Image.fromarray(rgb)
                        buf  = io.BytesIO()
                        img.save(buf, format='JPEG', quality=65)
                        data = buf.getvalue()
                        self.wfile.write(b'--FRAME\r\n')
                        self.send_header('Content-Type', 'image/jpeg')
                        self.send_header('Content-Length', len(data))
                        self.end_headers()
                        self.wfile.write(data)
                        self.wfile.write(b'\r\n')
                    time.sleep(0.033)  # ~30fps cap
            except Exception:
                pass
        else:
            self.send_error(404)
            self.end_headers()

def start_stream_server(port):
    server = socketserver.ThreadingTCPServer(('0.0.0.0', port), StreamingHandler)
    server.daemon_threads = True
    server.serve_forever()

# ── Main Demo Loop ────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="Sentinel Drone Demo (GPS-free)")
    parser.add_argument('--no-xbee', action='store_true', help='Disable XBee')
    parser.add_argument('--port',    type=int, default=STREAM_PORT, help='MJPEG stream port')
    args = parser.parse_args()

    log.info("=" * 60)
    log.info("  Sentinel Swarm — DRONE DEMO MODE (GPS-Free)")
    log.info(f"  Camera:  /dev/video{CAMERA_INDEX}")
    log.info(f"  YOLO:    {YOLO_MODEL_DIR}")
    log.info(f"  XBee:    {'DISABLED' if args.no_xbee else XBEE_PORT}")
    log.info(f"  Stream:  http://0.0.0.0:{args.port}")
    log.info("=" * 60)

    # Start MJPEG stream server
    t = threading.Thread(target=start_stream_server, args=(args.port,), daemon=True)
    t.start()
    log.info(f"[Stream] MJPEG server started on port {args.port}")

    # Load YOLO
    log.info("[YOLO] Loading YOLO11n NCNN model...")
    yolo = YoloNcnn(model_dir=YOLO_MODEL_DIR, input_size=320, conf_threshold=CONF_THRESH, num_threads=4)
    log.info("[YOLO] Model loaded successfully!")

    # Open camera
    log.info("[Camera] Opening /dev/video0...")
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        log.error("[Camera] Could not open /dev/video0! Is the webcam plugged in?")
        sys.exit(1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    log.info(f"[Camera] Opened at 640x480")

    # Connect XBee
    xbee = None
    if not args.no_xbee:
        try:
            xbee = XBeeTransport(XBEE_PORT, baud=XBEE_BAUD)
            log.info(f"[XBee] Connected on {XBEE_PORT}")
        except Exception as e:
            log.warning(f"[XBee] Could not connect: {e} — continuing without XBee")

    log.info("\n[Demo] Running! Open your browser to http://10.219.37.160:5000\n")

    frame_count = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            log.warning("[Camera] Failed to read frame, retrying...")
            time.sleep(0.1)
            continue

        h, w = frame.shape[:2]
        detections = yolo.detect(frame)

        # Draw crosshair
        cv2.line(frame, (w // 2, 0),     (w // 2, h),     (0, 255, 0), 1)
        cv2.line(frame, (0,     h // 2), (w,      h // 2), (0, 255, 0), 1)

        # Draw detections
        for d in detections:
            cx = int(d.norm_cx * w)
            cy = int(d.norm_cy * h)
            bw = int(d.norm_w  * w)
            bh = int(d.norm_h  * h)
            x1, y1 = cx - bw // 2, cy - bh // 2
            x2, y2 = cx + bw // 2, cy + bh // 2
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
            cv2.circle(frame, (cx, cy), 8, (0, 0, 255), -1)
            cv2.putText(frame, f"CASUALTY {d.confidence:.2f}",
                        (x1, y1 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
            log.info(f"[YOLO] 🎯 Casualty detected! conf={d.confidence:.2f} cx={d.norm_cx:.2f} cy={d.norm_cy:.2f}")

            # Broadcast over XBee
            if xbee:
                payload = {
                    'r':  ROBOT_ID,
                    'cx': round(d.norm_cx, 3),
                    'cy': round(d.norm_cy, 3),
                    'cf': round(d.confidence, 3),
                    'sz': round(d.norm_w * d.norm_h, 4),
                    'st': 'T'
                }
                try:
                    xbee.send(payload)
                    log.info(f"[XBee] Broadcast: {payload}")
                except Exception as e:
                    log.warning(f"[XBee] Send failed: {e}")

        # Draw status overlay
        status = f"SENTINEL DEMO | {'CASUALTY DETECTED' if detections else 'SEARCHING...'}"
        color  = (0, 0, 255) if detections else (0, 255, 0)
        cv2.putText(frame, status, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        cv2.putText(frame, f"Frame: {frame_count}", (10, h - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

        # Push to stream
        with stream_state.lock:
            stream_state.frame = frame.copy()

        # Receive XBee messages from ground bots
        if xbee:
            msg = xbee.recv_nonblocking()
            if msg and msg.get('r') != ROBOT_ID:
                log.info(f"[Swarm] Ground bot {msg.get('r')} reports target at cx={msg.get('cx')} cy={msg.get('cy')} conf={msg.get('cf')}")

        frame_count += 1

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        log.info("\n[Demo] Stopped by user.")
