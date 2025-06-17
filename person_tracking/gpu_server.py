#!/usr/bin/env python3
"""
gpu_server.py – laptop “brain” with per‐stage profiling, including Pi→server latency

• pulls JPEG thumbs via ZMQ 5555 (HWM=1)
• runs YOLOv8 (person‐only, conf 0.35)
• profiles network/recv/decode/infer/publish/server_total/end2end latencies
• publishes bounding‐box JSON on ZMQ 5556
"""

import zmq
import struct
import cv2
import numpy as np
import signal
import sys
import pathlib
import time
from ultralytics import YOLO
import torch
import torch.backends.cudnn as cudnn

# 1) Detect device
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print("CUDA available:", torch.cuda.is_available())
print("Using device:", device)

# 2) Fast JPEG decoder (TurboJPEG fallback)
try:
    from turbojpeg import TurboJPEG
    J = TurboJPEG()
    def jpeg_decode(buf: bytes) -> np.ndarray:
        return J.decode(buf)
except ImportError:
    def jpeg_decode(buf: bytes) -> np.ndarray:
        arr = np.frombuffer(buf, np.uint8)
        return cv2.imdecode(arr, cv2.IMREAD_COLOR)

# 3) Load YOLOv8 model (TensorRT engine if available)
MODEL_PATH = pathlib.Path("models/yolov8n.engine"
                          if pathlib.Path("models/yolov8n.engine").exists()
                          else "models/yolov8n.pt")
model = YOLO(str(MODEL_PATH))

# 4) Optimize for inference on CUDA
if device.type == "cuda":
    cudnn.benchmark = True
    model.model.to(device).eval().fuse()

# 5) Warm up GPU
with torch.no_grad():
    dummy = np.zeros((240, 320, 3), dtype=np.uint8)
    for _ in range(5):
        _ = model(dummy, imgsz=320, conf=0.35, classes=[0], verbose=False)

# 6) ZeroMQ setup
ctx = zmq.Context()

# PULL socket receives thumbnails; buffer only one
pull = ctx.socket(zmq.PULL)
pull.bind("tcp://*:5555")
pull.setsockopt(zmq.RCVHWM, 1)

# PUB socket sends detection results
pub = ctx.socket(zmq.PUB)
pub.bind("tcp://*:5556")

print("🖥️  YOLO server ready (PULL 5555 [HWM=1], PUB 5556)  Ctrl-C to quit", flush=True)
signal.signal(signal.SIGINT, lambda *_: sys.exit(0))

while True:
    # — Stage 1: Recv (and capture Pi timestamp)
    t0 = time.time()
    buf = pull.recv()
    t1 = time.time()

    # Unpack Pi‐side timestamp (seconds since epoch, float)
    ts = struct.unpack("<d", buf[:8])[0]

    # — Pi→Server network latency
    network_ms = max((t0 - ts) * 1000, 0.0)

    # — Stage 2: Decode JPEG
    t2 = time.time()
    img = jpeg_decode(buf[8:])
    t3 = time.time()

    # — Stage 3: Inference
    t4 = time.time()
    with torch.no_grad():
        res = model(img, imgsz=320, conf=0.35, classes=[0], verbose=False)[0]
    t5 = time.time()

    # Extract boxes + confidences
    boxes = res.boxes.xyxy.cpu().int().tolist()
    confs = res.boxes.conf.cpu().tolist()
    payload = [[x1, y1, x2, y2, float(c)]
               for (x1, y1, x2, y2), c in zip(boxes, confs)]

    # — Stage 4: Publish
    t6 = time.time()
    pub.send_json({"t": ts, "boxes": payload})
    t7 = time.time()

    # Compute per‐stage durations
    recv_ms    = (t1 - t0) * 1000
    decode_ms  = (t3 - t2) * 1000
    infer_ms   = (t5 - t4) * 1000
    publish_ms = (t7 - t6) * 1000
    server_total_ms = recv_ms + decode_ms + infer_ms + publish_ms

    # End‐to‐end: Pi timestamp → after publish
    end2end_ms = max((t7 - ts) * 1000, 0.0)

    # Print the profiling line
    print(
        f"boxes={len(payload):2d}  "
        f"net={network_ms:5.1f} ms  "
        f"recv={recv_ms:5.1f} ms  "
        f"decode={decode_ms:5.1f} ms  "
        f"infer={infer_ms:5.1f} ms  "
        f"pub={publish_ms:5.1f} ms  "
        f"srv_total={server_total_ms:5.1f} ms  "
        f"end2end={end2end_ms:5.1f} ms"
    )
