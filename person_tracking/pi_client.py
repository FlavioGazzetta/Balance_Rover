#!/usr/bin/env python3
"""
pi_client_headless.py – Pi Camera v2.1 client via Picamera2, headless mode

• Captures full‐FoV 1280×960@15fps
• Sends 320×240 JPEG thumbs → YOLO server (tcp://SERVER_IP:5555)
• Receives boxes ← YOLO server (tcp://SERVER_IP:5556)
• Computes person center-x & box area → UDP → ESP32
• Draws boxes on the 1280×960 frame (for streaming)
• If run with --stream PORT, serves MJPEG @ http://<pi-ip>:PORT/stream
• Otherwise no GUI at all
"""

import argparse, time, struct, threading, socket, zmq, sys
from turbojpeg import TurboJPEG, TJPF_RGB
from picamera2 import Picamera2
import cv2
from flask import Flask, Response

# ——— CONFIG —————————————————————————————————————————————
SERVER_IP      = "172.20.10.4"   # YOLO server IP
ESP32_IP       = "172.20.10.5"   # ESP32 IP
ESP32_PORT     = 8888            # must match ESP32’s UDP_PORT

CAP_W, CAP_H   = 1280, 960
JPEG_W, JPEG_H = 320, 240
FPS            = 15
JPEG_QUALITY   = 70
# ————————————————————————————————————————————————————————

J = TurboJPEG()
ctx  = zmq.Context()
push = ctx.socket(zmq.PUSH)
push.connect(f"tcp://{SERVER_IP}:5555")
push.setsockopt(zmq.SNDHWM, 1)
sub  = ctx.socket(zmq.SUB)
sub.connect(f"tcp://{SERVER_IP}:5556")
sub.setsockopt_string(zmq.SUBSCRIBE, "")

udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

boxes = []
def recv_loop():
    global boxes
    while True:
        try:
            boxes = sub.recv_json(flags=zmq.NOBLOCK).get("boxes", [])
        except zmq.Again:
            time.sleep(0.001)

def build_arg_parser():
    p = argparse.ArgumentParser(
        description="PiCam2 headless client: send to YOLO+ESP32; optional HTTP MJPEG stream"
    )
    p.add_argument(
        "--stream", "-s", type=int, metavar="PORT",
        help="run MJPEG HTTP server on this port"
    )
    return p

### HTTP MJPEG STREAMING ###

app = Flask(__name__)
current_frame = None
frame_lock = threading.Lock()

def gen_mjpeg():
    """MJPEG generator for Flask."""
    global current_frame
    while True:
        with frame_lock:
            f = current_frame.copy() if current_frame is not None else None
        if f is None:
            time.sleep(0.01)
            continue
        ret, jpg = cv2.imencode('.jpg', f)
        if not ret:
            continue
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + jpg.tobytes() + b'\r\n')

@app.route("/stream")
def stream():
    return Response(gen_mjpeg(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


def start_flask(port):
    threading.Thread(
        target=app.run,
        kwargs={"host": "0.0.0.0", "port": port, "threaded": True, "debug": False},
        daemon=True
    ).start()

### MAIN ###

def main():
    args = build_arg_parser().parse_args()

    if args.stream:
        print(f"🚀 Starting MJPEG HTTP server on port {args.stream} …")
        start_flask(args.stream)

    threading.Thread(target=recv_loop, daemon=True).start()

    picam2 = Picamera2()
    video_config = picam2.create_video_configuration(
        main={"size": (CAP_W, CAP_H), "format": "RGB888"}
    )
    picam2.configure(video_config)
    picam2.start()

    scale_x = CAP_W / JPEG_W
    scale_y = CAP_H / JPEG_H

    last_send = time.time()
    print("🎥 PiCam2 headless client started.")

    try:
        while True:
            # 1) Capture full-res RGB
            frame_rgb = picam2.capture_array()
            # 2) Swap R<->B (to BGR), and convert for OpenCV
            frame_rgb[:, :, [0, 2]] = frame_rgb[:, :, [2, 0]]
            frame = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

            # 3) Send thumbnail to YOLO server at FPS rate
            now = time.time()
            if now - last_send >= 1.0 / FPS:
                thumb = cv2.resize(frame_rgb, (JPEG_W, JPEG_H))
                packet = struct.pack("<d", now) + \
                         J.encode(thumb, quality=JPEG_QUALITY, pixel_format=TJPF_RGB)
                try:
                    push.send(packet, flags=zmq.NOBLOCK)
                except zmq.Again:
                    pass
                last_send = now

            # 4) Draw boxes & send UDP to ESP32
            for x1, y1, x2, y2, conf in boxes:
                sx1, sy1 = int(x1 * scale_x), int(y1 * scale_y)
                sx2, sy2 = int(x2 * scale_x), int(y2 * scale_y)
                cv2.rectangle(frame, (sx1, sy1), (sx2, sy2), (0,255,0), 2)
                x_center = (sx1 + sx2)//2
                area     = (sx2 - sx1)*(sy2 - sy1)
                udp_sock.sendto(f"{x_center} {area}".encode(),
                                (ESP32_IP, ESP32_PORT))

            # 5) Update shared frame for HTTP streaming
            if args.stream:
                with frame_lock:
                    current_frame = cv2.resize(frame, (CAP_W//2, CAP_H//2))

            # 6) Headless → no imshow, no waitKey
    except KeyboardInterrupt:
        pass
    finally:
        picam2.stop()
        push.close(); sub.close(); ctx.term(); udp_sock.close()
        print("Clean exit.")

if __name__ == "__main__":
    main()
