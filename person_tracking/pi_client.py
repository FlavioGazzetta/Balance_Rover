#!/usr/bin/env python3
"""
pi_client.py — Raspberry Pi capture/transmit side (hybrid debug & resilient edition)
──────────────────────────────────────────────────────────────────────────────
• Captures frames from the Pi camera (Picamera2)
• Publishes **320 × 240 JPEG thumbnails** over ZMQ PUSH→PULL to the GPU/YOLO server
• Receives detection boxes over ZMQ SUB and draws them for a preview overlay
• Optional MJPEG /stream HTTP endpoint — **now streams the full 1280 × 960 view**
• Smart one‑slot FRAME_Q so the preview never piles up frames in RAM
• Auto‑restart of the camera on *any* capture exception
• Clean shutdown of sockets & camera on Ctrl‑C

🔄 **2025‑06‑18 i**  (full‑res preview)
──────────────────────────────────────────────────────────────────────────────
```bash
# Full‑res preview at 8000, thumbnails @15 fps, JPEG 60, queue depth 100
python pi_client.py --server-ip 172.20.10.4 \
                    --stream 8000           \
                    --rate   15             \
                    --quality 60            \
                    --hwm    100
```
"""

import os, sys, signal, time, struct, threading, socket, queue, argparse, logging
from datetime import datetime

# ---------- 3rd‑party ----------
import cv2, zmq, numpy as np
from flask import Flask, Response, jsonify, request, make_response

# ---------- CLI ------------------------------------------------------
parser = argparse.ArgumentParser(description="PiCam2 client with preview & resilience")
parser.add_argument('--stream',  type=int,   default=0,  metavar='PORT', help='start MJPEG preview on this port (0 = disable)')
parser.add_argument('--quality', type=int,   default=80, metavar='Q',    help='JPEG quality 1‑100 (both stream & thumb)')
parser.add_argument('--rate',    type=float, default=0,  metavar='FPS',  help='send FPS cap for thumbnails (0 = unlimited)')
parser.add_argument('--hwm',     type=int,   default=50, metavar='N',    help='ZMQ SNDHWM (queue size before drops)')
parser.add_argument('--esp-ip',  dest='esp_ip',   type=str,   default=None, help='ESP32 telemetry IP (optional)')
parser.add_argument('--esp-port',dest='esp_port', type=int,   default=8888, help='ESP32 UDP port')
parser.add_argument('--server-ip',dest='server_ip',type=str,  default='127.0.0.1', help='YOLO server IP')
conflate = parser.add_mutually_exclusive_group()
conflate.add_argument('--conflate',    dest='conflate', action='store_true',  help='keep only newest unsent frame (default)')
conflate.add_argument('--no-conflate', dest='conflate', action='store_false', help='use regular FIFO ZMQ queue')
parser.set_defaults(conflate=True)
args = parser.parse_args()

JPEG_QUALITY = int(max(1, min(args.quality, 100)))
RATE_LIMIT   = args.rate if args.rate > 0 else None
SNDHWM       = max(1, args.hwm)

# ---------- JPEG (Turbo or OpenCV fallback) --------------------------
try:
    from turbojpeg import TurboJPEG, TJPF_RGB
    TJ = TurboJPEG()
    def jpeg_encode(img, q):
        return TJ.encode(img, quality=q, pixel_format=TJPF_RGB)
    print("[INIT] Using TurboJPEG encoder/decoder")
except ImportError:
    def jpeg_encode(img, q):
        return cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, q])[1].tobytes()
    print("[INIT] TurboJPEG unavailable → falling back to OpenCV JPEG")

# ---------- Helpers --------------------------------------------------

def _get_ip():
    try:
        import netifaces as ni
        for iface in ni.interfaces():
            if iface == 'lo':
                continue
            addrs = ni.ifaddresses(iface).get(ni.AF_INET)
            if addrs:
                return addrs[0]['addr']
    except Exception:
        pass
    return "0.0.0.0"

# ---------- Constants -----------------------------------------------
PREV_W, PREV_H   = 1280, 960            # full‑resolution preview size
THUMB_W, THUMB_H = 320, 240             # thumbnail sent to YOLO server
PUB_PORT, SUB_PORT = 5555, 5556
ESP32_ADDR         = (args.esp_ip, args.esp_port) if args.esp_ip else None
UDP_SOCK           = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if ESP32_ADDR else None

# ---------------- ZMQ ------------------------------------------------
ctx  = zmq.Context.instance()
push = ctx.socket(zmq.PUSH)
push.setsockopt(zmq.SNDHWM, SNDHWM)
if args.conflate:
    push.setsockopt(zmq.CONFLATE, 1)
    print("[INIT] ZMQ CONFLATE enabled — queue overwrites old frames")
push.connect(f"tcp://{args.server_ip}:{PUB_PORT}")

sub  = ctx.socket(zmq.SUB)
sub.setsockopt(zmq.SUBSCRIBE, b"")
sub.connect(f"tcp://{args.server_ip}:{SUB_PORT}")
print(f"[INIT] ZMQ PUSH→{PUB_PORT}, SUB→{SUB_PORT}  HWM={SNDHWM}")

# ---------------- Flask preview -------------------------------------
app = Flask(__name__)
FRAME_Q = queue.Queue(maxsize=1)  # flush‑slot for preview JPEGs

@app.before_request
def _log_req():
    print(f"[HTTP] {request.method} {request.path} from {request.remote_addr}")

@app.route('/')
def index():
    return ("<html><body><h1>Pi Client</h1>"
            "<p><a href='/health'>health</a> · <a href='/test'>test</a> · <a href='/stream'>stream</a></p>"
            "<img src='/stream' width='640'></body></html>")

@app.route('/health')
def health():
    return jsonify(status='ok', time=time.time())

@app.route('/test')
def test():
    deadline = time.time() + 2
    while FRAME_Q.empty() and time.time() < deadline:
        time.sleep(0.05)
    if FRAME_Q.empty():
        return "no frame yet", 503
    img = FRAME_Q.get(); FRAME_Q.put(img)  # put back for stream
    resp = make_response(img)
    resp.headers['Content-Type'] = 'image/jpeg'
    resp.headers['Content-Length'] = str(len(img))
    return resp

@app.route('/stream')
def stream():
    def gen():
        while True:
            img = FRAME_Q.get()
            yield (b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                   + str(len(img)).encode() + b"\r\n\r\n" + img + b"\r\n")
    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')

if args.stream:
    threading.Thread(target=lambda: app.run(host='0.0.0.0', port=args.stream,
                                            debug=False, use_reloader=False, threaded=True), daemon=True).start()
    print(f"[HTTP] Preview: http://{_get_ip()}:{args.stream}/")
else:
    print("[HTTP] preview disabled")

# ---------------- Camera (auto‑restart) -----------------------------
import picamera2

def open_camera():
    pc = picamera2.Picamera2()
    conf = pc.create_preview_configuration(main={'format':'RGB888', 'size':(PREV_W, PREV_H)})
    pc.configure(conf)
    pc.start()
    print("[INIT] PiCamera2 started")
    return pc

picam = open_camera()

# ---------------- Shared state --------------------------------------
cur_boxes      = []
cur_boxes_lock = threading.Lock()

# ---------------- ZMQ recv thread -----------------------------------

def recv_loop():
    while True:
        try:
            data = sub.recv_json(flags=zmq.NOBLOCK)
            with cur_boxes_lock:
                cur_boxes[:] = data.get('boxes', [])
        except zmq.Again:
            time.sleep(0.003)

t_recv = threading.Thread(target=recv_loop, daemon=True); t_recv.start()

# ---------------- Main loop -----------------------------------------
last_stat = time.time(); sent = dropped = skipped = 0
running   = True

def sigint_handler(sig, frm):
    global running
    running = False
signal.signal(signal.SIGINT, sigint_handler)

while running:
    loop_t0 = time.time()
    # ---- capture with auto‑restart ----
    try:
        frame = picam.capture_array('main')  # 1280×960 RGB888
    except Exception as e:
        logging.warning("[WARN] camera error: %s — re‑opening", e)
        try:
            picam.close()
        except Exception:
            pass
        time.sleep(0.2)
        picam = open_camera()
        skipped += 1
        continue

    # ---- draw detections ----
    with cur_boxes_lock:
        for x1, y1, x2, y2, conf in cur_boxes:
            sx = PREV_W / THUMB_W
            sy = PREV_H / THUMB_H
            sx1, sy1 = int(x1 * sx), int(y1 * sy)
            sx2, sy2 = int(x2 * sx), int(y2 * sy)
            cv2.rectangle(frame, (sx1, sy1), (sx2, sy2), (0, 255, 0), 2)
            if ESP32_ADDR:
                cx   = (sx1 + sx2) // 2
                area = (sx2 - sx1) * (sy2 - sy1)
                UDP_SOCK.sendto(f"{cx} {area}".encode(), ESP32_ADDR)

    # ---- thumbnail for YOLO server ----
    thumb_small = cv2.resize(frame, (THUMB_W, THUMB_H))
    jpg_small   = jpeg_encode(thumb_small, JPEG_QUALITY)

    if push.poll(0, zmq.POLLOUT):
        payload = struct.pack('<d', time.time()) + jpg_small
        push.send(payload)
        sent += 1
    else:
        dropped += 1

    # ---- full‑res preview JPEG ----
    if args.stream:
        jpg_full = jpeg_encode(frame, JPEG_QUALITY)
        try:
            FRAME_Q.put_nowait(jpg_full)
        except queue.Full:
            FRAME_Q.get_nowait(); FRAME_Q.put_nowait(jpg_full)

    # ---- rate limit thumbnails ----
    if RATE_LIMIT:
        dt = time.time() - loop_t0
        time.sleep(max(0, (1.0 / RATE_LIMIT) - dt))

    # ---- periodic stats ----
    if time.time() - last_stat > 5:
        print(f"[STAT] sent={sent} drop={dropped} skip={skipped} q={JPEG_QUALITY} fps={RATE_LIMIT or 0:.1f} HWM={SNDHWM}")
        sent = dropped = skipped = 0
        last_stat = time.time()

# ---------------- Cleanup -------------------------------------------
print("[BYE] Shutting down …")
try:
    picam.stop(); picam.close()
except Exception:
    pass
push.close(); sub.close(); ctx.term()
if UDP_SOCK:
    UDP_SOCK.close()
