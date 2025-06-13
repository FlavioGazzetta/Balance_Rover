#!/usr/bin/env python3
"""
pi_client.py – Pi 3 A+ thin client
• receives raw H.264 on UDP 1234 from the laptop
• sends 320×240 JPEG thumbs to the laptop (tcp://SERVER_IP:5555)
• receives bounding-box JSON (tcp://SERVER_IP:5556)
• draws rectangles (scaled to full 640×480)
• optional: restreams annotated video on UDP 1236
"""

import cv2, zmq, struct, time, threading, subprocess, atexit, sys
from turbojpeg import TurboJPEG
J = TurboJPEG()

# ---------- edit these two lines only -----------------
SERVER_IP = "172.20.10.4"               # laptop address
SRC_URL   = "udp://0.0.0.0:1234?fifo_size=1000000&overrun_nonfatal=1&loglevel=error"
# ------------------------------------------------------

JPEG_W, JPEG_H, FPS = 320, 240, 15
OUT_PORT           = 1236               # Pi restreams annotated video here
OUT_SIZE           = (640, 480)         # matches the incoming webcam

ctx  = zmq.Context()
push = ctx.socket(zmq.PUSH); push.connect(f"tcp://{SERVER_IP}:5555")
sub  = ctx.socket(zmq.SUB);  sub.connect(f"tcp://{SERVER_IP}:5556")
sub.setsockopt_string(zmq.SUBSCRIBE, "")

boxes = []                               # shared list updated in background
def recv_loop():
    global boxes
    while True:
        try:
            boxes = sub.recv_json()["boxes"]
        except Exception:
            pass
threading.Thread(target=recv_loop, daemon=True).start()

cap = cv2.VideoCapture(SRC_URL, cv2.CAP_FFMPEG)
if not cap.isOpened():
    sys.exit(f"❌ cannot open stream {SRC_URL}")

# --- FFmpeg restream for viewing elsewhere ---------------------------
ff = subprocess.Popen([
    "ffmpeg","-loglevel","error","-f","rawvideo","-pixel_format","bgr24",
    "-video_size",f"{OUT_SIZE[0]}x{OUT_SIZE[1]}","-framerate","30","-i","-",
    "-c:v","libx264","-preset","ultrafast","-tune","zerolatency",
    "-g","30","-pix_fmt","yuv420p","-f","mpegts",
    f"udp://0.0.0.0:{OUT_PORT}?pkt_size=1316"
], stdin=subprocess.PIPE)
atexit.register(lambda: ff.stdin.close() or ff.wait())
# ---------------------------------------------------------------------

last = 0.0
scale_x = OUT_SIZE[0] / JPEG_W          # 640 / 320 = 2
scale_y = OUT_SIZE[1] / JPEG_H          # 480 / 240 = 2

print("🎥  Pi client running – forwarding frames to server …", flush=True)

while True:
    ok, frame = cap.read()
    if not ok:
        continue

    t = time.time()
    if t - last >= 1 / FPS:
        thumb = cv2.resize(frame, (JPEG_W, JPEG_H))
        push.send(struct.pack("<d", t) + J.encode(thumb, quality=70))
        last = t

    # draw latest boxes (scaled to full 640×480)
    for x1, y1, x2, y2, sc in boxes:
        x1, x2 = int(x1 * scale_x), int(x2 * scale_x)
        y1, y2 = int(y1 * scale_y), int(y2 * scale_y)
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(frame, f"{sc:.2f}", (x1, y1 - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    ff.stdin.write(frame.tobytes())      # ship to UDP 1236