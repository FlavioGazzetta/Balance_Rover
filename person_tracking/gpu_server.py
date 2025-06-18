#!/usr/bin/env python3
"""
gpu_server_debug.py – ultraverbose debug on every stage:
  • logs each ZMQ pull (size, timestamp)
  • logs decode time & any failures
  • logs inference boxes + confidences
  • logs each ZMQ pub (payload size)
  • logs end2end, per‐stage latencies
"""

import zmq
import struct
import cv2
import numpy as np
import signal
import sys
import time
import pathlib
import traceback
from ultralytics import YOLO
import torch
import torch.backends.cudnn as cudnn

# 1) Device
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(f"[INIT ] CUDA available: {torch.cuda.is_available()}, device={device}")

# 2) JPEG decode
try:
    from turbojpeg import TurboJPEG
    J = TurboJPEG()
    def jpeg_decode(buf):
        return J.decode(buf)
    print("[INIT ] Using TurboJPEG decoder")
except ImportError:
    def jpeg_decode(buf):
        arr = np.frombuffer(buf, np.uint8)
        return cv2.imdecode(arr, cv2.IMREAD_COLOR)
    print("[INIT ] Using OpenCV JPEG decoder fallback")

# 3) Load model
MODEL_PATH = pathlib.Path("models/yolov8n.engine" if pathlib.Path("models/yolov8n.engine").exists() else "models/yolov8n.pt")
print(f"[INIT ] Loading YOLOv8 model from {MODEL_PATH}")
model = YOLO(str(MODEL_PATH))

# 4) Optimize
if device.type == "cuda":
    cudnn.benchmark = True
    model.model.to(device).eval().fuse()
    print("[INIT ] Model fused & moved to CUDA")

# 5) Warmup
with torch.no_grad():
    dummy = np.zeros((240, 320, 3), dtype=np.uint8)
    for i in range(5):
        _ = model(dummy, imgsz=320, conf=0.35, classes=[0], verbose=False)
    print("[INIT ] Warmup complete")

# 6) ZeroMQ setup
ctx = zmq.Context()

# PULL socket receives thumbnails; buffer only one
pull = ctx.socket(zmq.PULL)
pull.bind("tcp://*:5555")
pull.setsockopt(zmq.RCVHWM, 1)

# PUB socket sends detection results
pub = ctx.socket(zmq.PUB)
pub.bind("tcp://*:5556")

print("🖥️  YOLO server ready (PULL 5555 [HWM=1], PUB 5556)  Ctrl-C to quit")

signal.signal(signal.SIGINT, lambda *_: sys.exit(0))

while True:
    try:
        # — RECV —
        t0 = time.time()
        msg = pull.recv()
        t1 = time.time()
        size = len(msg)
        print(f"\n[RECV ] {size} bytes, recv_latency={(t1-t0)*1000:.1f} ms")

        # Unpack Pi timestamp
        ts = struct.unpack("<d", msg[:8])[0]
        ts_local = time.localtime(ts)
        print(f"[TS   ] Pi timestamp={ts:.6f} ({time.strftime('%H:%M:%S', ts_local)})")

        # — DECODE —
        d0 = time.time()
        img = jpeg_decode(msg[8:])
        d1 = time.time()
        if img is None:
            print("[DECODE][ERROR] decoder returned None")
            continue
        h, w = img.shape[:2]
        print(f"[DECODE] {w}×{h} decoded in {(d1-d0)*1000:.1f} ms")

        # — INFERENCE —
        i0 = time.time()
        with torch.no_grad():
            res = model(img, imgsz=320, conf=0.35, classes=[0], verbose=False)[0]
        i1 = time.time()
        boxes = res.boxes.xyxy.cpu().int().tolist()
        confs = res.boxes.conf.cpu().tolist()
        print(f"[INFER ] {len(boxes)} boxes in {(i1-i0)*1000:.1f} ms")
        for idx, ((x1, y1, x2, y2), c) in enumerate(zip(boxes, confs)):
            print(f"         box#{idx}: [{x1},{y1},{x2},{y2}] conf={c:.2f}")

        payload = [[x1, y1, x2, y2, float(c)] for (x1, y1, x2, y2), c in zip(boxes, confs)]

        # — PUBLISH —
        p0 = time.time()
        pub.send_json({"t": ts, "boxes": payload})
        p1 = time.time()
        print(f"[PUBLISH] {len(payload)} boxes, json_payload_size≈{sys.getsizeof(payload)} bytes in {(p1-p0)*1000:.1f} ms")

        # — PROFILING —
        recv_ms   = (t1 - t0) * 1000
        decode_ms = (d1 - d0) * 1000
        infer_ms  = (i1 - i0) * 1000
        pub_ms    = (p1 - p0) * 1000
        total_srv = recv_ms + decode_ms + infer_ms + pub_ms
        end2end   = max((p1 - ts) * 1000, 0.0)
        print(f"[PROF  ] net={recv_ms:.1f} ms  dec={decode_ms:.1f} ms  inf={infer_ms:.1f} ms  pub={pub_ms:.1f} ms  srv_total={total_srv:.1f} ms  end2end={end2end:.1f} ms")

    except Exception:
        print("[ERROR ] Exception in main loop:\n" + traceback.format_exc(), file=sys.stderr)
