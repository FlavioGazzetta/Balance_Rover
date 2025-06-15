#!/usr/bin/env python3
"""
gpu_server.py – laptop “brain”
• pulls JPEG thumbnails via ZMQ 5555
• runs YOLOv8 (person-only, conf 0.35)
• drains queue to get latest frame only
• publishes bounding-box JSON on ZMQ 5556
"""

import zmq, struct, json, cv2, numpy as np, signal, sys, pathlib, time
from ultralytics import YOLO

# --- fast JPEG decoder (TurboJPEG) -----------------------------------
try:
    from turbojpeg import TurboJPEG
    J = TurboJPEG()
    def jpeg_decode(buf: bytes) -> np.ndarray:
        return J.decode(buf)
except Exception:
    def jpeg_decode(buf: bytes) -> np.ndarray:
        return cv2.imdecode(np.frombuffer(buf, np.uint8), cv2.IMREAD_COLOR)
# ---------------------------------------------------------------------

MODEL_PATH = pathlib.Path("models/yolov8n.pt")
model = YOLO(str(MODEL_PATH))

ctx  = zmq.Context()
pull = ctx.socket(zmq.PULL); pull.bind("tcp://*:5555")
pub  = ctx.socket(zmq.PUB);  pub.bind("tcp://*:5556")

print("🖥️  YOLO server ready  (PULL 5555, PUB 5556)   Ctrl-C to quit", flush=True)
signal.signal(signal.SIGINT, lambda *_: sys.exit(0))

while True:
    buf = pull.recv()  # receive at least one frame

    # Drain the ZMQ queue to get the most recent frame only
    while True:
        try:
            buf = pull.recv(flags=zmq.DONTWAIT)
        except zmq.Again:
            break

    ts  = struct.unpack("<d", buf[:8])[0]  # first 8 bytes = timestamp
    img = jpeg_decode(buf[8:])            # decode JPEG to numpy array (BGR)

    # --- inference ---------------------------------------------------
    res   = model(img, imgsz=320, conf=0.35, classes=[0], verbose=False)[0]  # 0 = person
    boxes = res.boxes.xyxy.cpu().int().tolist()
    confs = res.boxes.conf.cpu().tolist()
    payload = [[x1, y1, x2, y2, float(c)] for (x1, y1, x2, y2), c in zip(boxes, confs)]

    # Clamp negative latencies in case of clock drift
    latency_ms = max((time.time() - ts) * 1000, 0)
    print(f"boxes {len(payload)}   latency {latency_ms:.0f} ms")

    pub.send_json({"t": ts, "boxes": payload})
