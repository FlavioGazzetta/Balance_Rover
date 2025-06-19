#!/usr/bin/env python3
"""
gpu_server.py – YOLO v8 + ultra-verbose diagnostics + very-light tracker
────────────────────────────────────────────────────────────────────────
• PULL 5555   receives   320×240 JPEG thumbnails from Pi
• YOLOv8n     (person-only) runs on CUDA if available
• A tiny centroid tracker gives each person a persistent ID
• PUB  5556   sends  [x1,y1,x2,y2,id,conf] for every box
• Logs per-stage latencies & end-to-end timing

2025-06-18 m  (weights-only patch + tracker-bugfix)
"""

import time, struct, signal, sys, pathlib, traceback
import cv2, numpy as np, zmq, torch
from ultralytics import YOLO

# ───────────────── 0)  Torch “weights-only” monkey-patch  ────────────
# PyTorch ≥2.6 defaults to weights_only=True which breaks Ultralytics
_orig_torch_load = torch.load
def _patched_load(*args, **kw):
    kw.setdefault('weights_only', False)
    return _orig_torch_load(*args, **kw)
torch.load = _patched_load                                       # ← boom, fixed
# ─────────────────────────────────────────────────────────────────────

# 1)   Device
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
print(f"[INIT ] device={device}, CUDA={torch.cuda.is_available()}")

# 2)   JPEG decode
try:
    from turbojpeg import TurboJPEG
    TJ = TurboJPEG()
    def jpeg_decode(buf):  return TJ.decode(buf)
    print("[INIT ] TurboJPEG decoder")
except Exception as e:
    print(f"[INIT ] TurboJPEG not available ({e}) → OpenCV fallback")
    def jpeg_decode(buf):
        arr = np.frombuffer(buf, np.uint8)
        return cv2.imdecode(arr, cv2.IMREAD_COLOR)

# 3)   YOLO model
MODEL_PATH = pathlib.Path("models/yolov8n.pt")
print(f"[INIT ] Loading model: {MODEL_PATH}")
model = YOLO(str(MODEL_PATH))
model.fuse()
if device.type == 'cuda':
    model.model.to(device).eval()
    torch.backends.cudnn.benchmark = True
print("[INIT ] Model ready")

# 4)   Warm-up
dummy = np.zeros((240, 320, 3), np.uint8)
for _ in range(5):
    _ = model(dummy, imgsz=320, conf=0.35, classes=[0], verbose=False)
print("[INIT ] Warm-up complete")

# 5)   ZMQ sockets
ctx   = zmq.Context()
pull  = ctx.socket(zmq.PULL); pull.bind("tcp://*:5555"); pull.setsockopt(zmq.RCVHWM, 1)
pub   = ctx.socket(zmq.PUB ); pub .bind("tcp://*:5556")
print("🖥️  GPU server ready  (PULL 5555, PUB 5556) — Ctrl-C to quit")

# ─────────────────── 6)  Minimal centroid tracker ────────────────────
class Track:
    _next_id = 0
    def __init__(self, cx, cy, w, h):
        self.id   = Track._next_id; Track._next_id += 1
        self.cx   = cx; self.cy = cy; self.w = w; self.h = h
        self.hits = 1   # consecutive matches
        self.miss = 0   # consecutive misses
    def update(self, cx, cy, w, h):
        self.cx, self.cy, self.w, self.h = cx, cy, w, h
        self.hits += 1; self.miss = 0
    def mark_missed(self):
        self.miss += 1

TRACK_TOLERANCE = 90          # pixels – max centroid distance to match
MAX_MISSES      = 8           # frames before a track is dropped
tracks: list['Track'] = []    # global list

def update_tracker(dets_xyxy):
    global tracks
    # compute centroids of detections
    det_centroids = [
        ((x1+x2)//2, (y1+y2)//2, x2-x1, y2-y1, (x1,y1,x2,y2))
        for x1,y1,x2,y2 in dets_xyxy
    ]

    # assignment: greedy close-centroid
    unused_tracks = set(range(len(tracks)))
    for cx,cy,w,h,xyxy in det_centroids:
        best_t, best_d = None, TRACK_TOLERANCE+1
        for ti in unused_tracks:
            dx = tracks[ti].cx - cx; dy = tracks[ti].cy - cy
            d  = abs(dx) + abs(dy)
            if d < best_d:
                best_d, best_t = d, ti
        if best_t is not None:
            tracks[best_t].update(cx, cy, w, h)
            tracks[best_t].last_xyxy = xyxy
            unused_tracks.remove(best_t)
        else:
            t = Track(cx, cy, w, h)
            t.last_xyxy = xyxy
            tracks.append(t)

    # mark unmatched tracks
    for ti in unused_tracks:
        tracks[ti].mark_missed()

    # cull dead tracks
    tracks = [t for t in tracks if t.miss < MAX_MISSES]

    # output: [x1,y1,x2,y2,id]
    return [list(t.last_xyxy)+[t.id] for t in tracks]
# ─────────────────────────────────────────────────────────────────────

signal.signal(signal.SIGINT, lambda *_: sys.exit(0))

# 7)   Main loop
while True:
    try:
        # — recv
        t0  = time.time()
        msg = pull.recv()
        recv_ms = (time.time()-t0)*1000
        ts_pi  = struct.unpack('<d', msg[:8])[0]
        img    = jpeg_decode(msg[8:])
        if img is None:
            print("[DECODE] failed"); continue

        # — inference
        ti0 = time.time()
        with torch.no_grad():
            res = model(img, imgsz=320, conf=0.35, classes=[0], verbose=False)[0]
        infer_ms = (time.time()-ti0)*1000

        dets = res.boxes.xyxy.cpu().int().tolist()
        conf = res.boxes.conf.cpu().tolist()

        # — tracker
        trk0 = time.time()
        tracks_out = update_tracker([d[:4] for d in dets])
        track_ms   = (time.time()-trk0)*1000

        # compose payload [x1,y1,x2,y2,id,conf]  (conf matched by index)
        payload = []
        for x1,y1,x2,y2,tid in tracks_out:
            # find original detection for conf (nearest box)
            best_i, best_d = None, 1e9
            for i,(dx1,dy1,dx2,dy2) in enumerate(dets):
                d = abs(dx1-x1)+abs(dy1-y1)
                if d < best_d:
                    best_d, best_i = d, i
            payload.append([x1,y1,x2,y2,tid,float(conf[best_i] if best_i is not None else 0)])

        # — publish
        pub.send_json({"t": ts_pi, "boxes": payload})

        # — logs
        total_ms = recv_ms + infer_ms + track_ms
        print(f"[{len(dets):2d} det → {len(tracks_out):2d} trk] "
              f"net={recv_ms:5.1f} ms  inf={infer_ms:5.1f} ms "
              f"trk={track_ms:5.1f} ms  total={total_ms:5.1f} ms")

    except Exception:
        print("[ERROR]", traceback.format_exc(), file=sys.stderr)