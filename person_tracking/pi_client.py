#!/usr/bin/env python3
"""
pi_client.py – Pi Camera v2.1 client via Picamera2
• Captures full‐FoV 1280×960@15fps (video mode, no crop)
• Sends 320×240 JPEG thumbs → YOLO server (tcp://SERVER_IP:5555)
• Receives boxes ← YOLO server (tcp://SERVER_IP:5556)
• Computes person center-x & box area → UDP → ESP32
• Draws boxes on the 1280×960 frame
• Resizes to 640×480 for display (so it looks “zoomed out”)
• Correct RGB→BGR handling
"""

import time, struct, threading, socket, zmq
from turbojpeg import TurboJPEG, TJPF_RGB
from picamera2 import Picamera2
import cv2, atexit

# ——— CONFIG —————————————————————————————————————————————
SERVER_IP      = "172.20.10.4"   # YOLO server IP
ESP32_IP       = "172.20.10.5"   # ESP32 IP
ESP32_PORT     = 8888            # must match ESP32’s UDP_PORT

# Capture at double the original 640×480 → 1280×960 (full FoV)
CAP_W, CAP_H   = 1280, 960

# Inference thumbnail size
JPEG_W, JPEG_H = 320, 240

# On-screen display size (same as original 640×480)
DISPLAY_W, DISPLAY_H = 640, 480

FPS            = 15
JPEG_QUALITY   = 70
# ————————————————————————————————————————————————————————

# JPEG encoder (we’ll encode the lo-res RGB correctly)
J = TurboJPEG()

# ZeroMQ PUSH/PULL
ctx  = zmq.Context()
push = ctx.socket(zmq.PUSH); push.connect(f"tcp://{SERVER_IP}:5555"); push.setsockopt(zmq.SNDHWM, 1)
sub  = ctx.socket(zmq.SUB);  sub.connect(f"tcp://{SERVER_IP}:5556"); sub.setsockopt_string(zmq.SUBSCRIBE, "")

# UDP socket to ESP32
udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# receive thread
boxes = []
def recv_loop():
    global boxes
    while True:
        try:
            boxes = sub.recv_json(flags=zmq.NOBLOCK).get("boxes", [])
        except zmq.Again:
            time.sleep(0.001)

threading.Thread(target=recv_loop, daemon=True).start()

# Picamera2: video mode @ 1280×960 for full sensor FoV
picam2 = Picamera2()
video_config = picam2.create_video_configuration(
    main={"size": (CAP_W, CAP_H), "format": "RGB888"}
)
picam2.configure(video_config)
picam2.start()
atexit.register(picam2.stop)

# Create a fixed-size window for display
cv2.namedWindow("Pi-3 live", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Pi-3 live", DISPLAY_W, DISPLAY_H)

# Precompute scales from thumbnail coords → full capture
scale_x = CAP_W / JPEG_W
scale_y = CAP_H / JPEG_H

last_send = time.time()
print("🎥 PiCam2 client started – 1280×960 capture, 640×480 display.")

try:
    while True:
        # 1) Capture full-res RGB
        frame_rgb = picam2.capture_array()

        # 2) Convert once to BGR for OpenCV display
        frame = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

        now = time.time()
        # 3) Every 1/FPS seconds, send a 320×240 RGB thumb to YOLO
        if now - last_send >= 1.0 / FPS:
            thumb_rgb = cv2.resize(frame_rgb, (JPEG_W, JPEG_H))
            packet    = struct.pack("<d", now) + \
                        J.encode(thumb_rgb, quality=JPEG_QUALITY, pixel_format=TJPF_RGB)
            try:
                push.send(packet, flags=zmq.NOBLOCK)
            except zmq.Again:
                pass  # queue full → drop
            last_send = now

        # 4) Draw each detection and send x/area to ESP32
        for x1, y1, x2, y2, conf in boxes:
            sx1, sy1 = int(x1 * scale_x), int(y1 * scale_y)
            sx2, sy2 = int(x2 * scale_x), int(y2 * scale_y)
            cv2.rectangle(frame, (sx1, sy1), (sx2, sy2), (0, 255, 0), 2)
            cv2.putText(frame, f"{conf:.2f}", (sx1, sy1 - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            x_center = (sx1 + sx2) // 2
            area     = (sx2 - sx1) * (sy2 - sy1)
            udp_sock.sendto(f"{x_center} {area}".encode("ascii"),
                            (ESP32_IP, ESP32_PORT))

        # 5) Downscale to 640×480 so it looks “zoomed out”
        disp = cv2.resize(frame, (DISPLAY_W, DISPLAY_H))
        cv2.imshow("Pi-3 live", disp)

        if cv2.waitKey(1) & 0xFF == 27:
            break

except KeyboardInterrupt:
    pass

finally:
    picam2.stop()
    cv2.destroyAllWindows()
    push.close(); sub.close(); ctx.term(); udp_sock.close()
    print("Clean exit.")
