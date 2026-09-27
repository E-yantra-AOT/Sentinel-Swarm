import cv2
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
import threading
import sys
from pathlib import Path

# Add the shared directory to the Python path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "shared"))
from yolo_ncnn import YoloNcnn

import os

# Load the NCNN model
print("Loading YOLO NCNN Model...")
yolo = YoloNcnn(
    model_dir=os.path.expanduser('~/yolo11n_ncnn_model'),
    input_size=320,
    num_threads=1
)
print("Model loaded!")

# Global frame buffer for the stream
current_frame = None
frame_lock = threading.Lock()

def camera_loop():
    global current_frame
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    print("Camera started. Running inference...")
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            time.sleep(1)
            continue
            
        # Run YOLO inference
        t0 = time.time()
        results = yolo.detect(frame)
        t1 = time.time()
        
        # Draw boxes
        fps = 1.0 / (t1 - t0 + 0.001)
        cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        for d in results:
            x1, y1, x2, y2 = map(int, [d.bbox_x1, d.bbox_y1, d.bbox_x2, d.bbox_y2])
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
            label = f"Obj {d.class_id}: {d.confidence:.2f}"
            cv2.putText(frame, label, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            
        # Encode to JPEG
        ret, jpg = cv2.imencode('.jpg', frame)
        if ret:
            with frame_lock:
                current_frame = jpg.tobytes()

class CamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=--jpgboundary')
            self.end_headers()
            
            while True:
                with frame_lock:
                    frame_data = current_frame
                
                if frame_data:
                    try:
                        self.wfile.write(b'--jpgboundary\r\n')
                        self.send_header('Content-type', 'image/jpeg')
                        self.send_header('Content-length', len(frame_data))
                        self.end_headers()
                        self.wfile.write(frame_data)
                        self.wfile.write(b'\r\n')
                    except Exception as e:
                        break # Client disconnected
                time.sleep(0.05) # Limit stream to ~20fps

class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    pass

if __name__ == '__main__':
    # Start camera in a background thread so the web server doesn't block it
    threading.Thread(target=camera_loop, daemon=True).start()
    
    server = ThreadedHTTPServer(('0.0.0.0', 5000), CamHandler)
    print("\n" + "="*50)
    print("LIVE YOLO STREAM READY!")
    print("Open a web browser on your Windows PC and go to:")
    print("http://pi.local:5000")
    print("="*50 + "\n")
    server.serve_forever()
