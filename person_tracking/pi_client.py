"""
pi_client.py – Pi Camera v2.1 client via Picamera2
• Captures 640×480@30fps with Picamera2/libcamera
• Sends 320×240 JPEG thumbs → server (tcp://SERVER_IP:5555)
• Receives boxes ← server (tcp://SERVER_IP:5556)
• Draws and displays locally with OpenCV
"""

import sys, time, struct, threading, zmq
from turbojpeg import TurboJPEG
from picamera2 import Picamera2, Preview
import cv2, atexit, subprocess

# --- UPDATE THIS ONLY ---
SERVER_IP     = "172.20.10.4"
JPEG_W, JPEG_H = 320, 240
CAP_W,  CAP_H  = 640, 480
FPS            = 15
# ------------------------

# fast JPEG encoder
J = TurboJPEG()

# ZMQ sockets
ctx  = zmq.Context()
push = ctx.socket(zmq.PUSH); push.connect(f"tcp://{SERVER_IP}:5555")
sub  = ctx.socket(zmq.SUB);  sub.connect(f"tcp://{SERVER_IP}:5556")
sub.setsockopt_string(zmq.SUBSCRIBE, "")

boxes = []
def recv_loop():
    global boxes
    while True:
        try:
            boxes = sub.recv_json()["boxes"]
        except:
            pass

threading.Thread(target=recv_loop, daemon=True).start()

# ---- Picamera2 setup ----
picam2 = Picamera2()
preview_config = picam2.create_preview_configuration(
    main={"size": (CAP_W, CAP_H), "format":"RGB888"})
picam2.configure(preview_config)
picam2.start()
atexit.register(picam2.stop)

# ---- optional: display window ----
cv2.namedWindow("Pi-3 live", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Pi-3 live", CAP_W, CAP_H)

last = 0.0
scale_x = CAP_W / JPEG_W
scale_y = CAP_H / JPEG_H

print("🎥  PiCam2 client started – sending thumbs to server …", flush=True)

while True:
    frame = picam2.capture_array()             # RGB888
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    now = time.time()
    if now - last >= 1 / FPS:
        thumb = cv2.resize(frame, (JPEG_W, JPEG_H))
        push.send(struct.pack("<d", now) + J.encode(thumb, quality=70))
        last = now

    # draw boxes
    for x1,y1,x2,y2,conf in boxes:
        x1,x2 = int(x1*scale_x), int(x2*scale_x)
        y1,y2 = int(y1*scale_y), int(y2*scale_y)
        cv2.rectangle(frame,(x1,y1),(x2,y2),(0,255,0),2)
        cv2.putText(frame,f"{conf:.2f}",(x1,y1-4),
                    cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,255,0),2)

    # display locally
    cv2.imshow("Pi-3 live", frame)
    if cv2.waitKey(1)&0xFF==27:
        break