#!/usr/bin/env python3
"""
pi_client.py – Raspberry Pi capture/transmit side (legacy xCam/area **plus** new JSON IDS stream)
────────────────────────────────────────────────────────────────────────
Run **with no arguments at all**:

    python3 pi_client.py

What it does now (only the IDS part is new; the rest is unchanged):

* **xCam area** → ASCII string `<xCam> <area>` on **UDP port 8888** (exactly as before).
* **People‑ID list** → JSON array (e.g. `[3,7,15]` or `[]`) on **UDP port 9999** so your Flutter side can `jsonDecode()` it directly.
* All other behaviour (MJPEG preview, thumbnail ZMQ, CLI flags, defaults) is identical to your original script.
"""

import sys, time, struct, signal, queue, threading, socket, argparse, json
import cv2, zmq, numpy as np
from flask import Flask, Response
from picamera2 import Picamera2

# ---------- CLI defaults (match user’s wish) ------------------------
parser = argparse.ArgumentParser()
parser.add_argument('--stream',  type=int,   default=8000,
                    help='MJPEG preview port (0 = disabled)')
parser.add_argument('--quality', type=int,   default=60,
                    help='JPEG quality 1‑100 (thumb + preview)')
parser.add_argument('--rate',    type=float, default=15,
                    help='Thumbnail FPS cap (0 = unlimited)')
parser.add_argument('--hwm',     type=int,   default=100,
                    help='ZMQ SNDHWM (queue size before drops)')
parser.add_argument('--esp-ip',  type=str,   default='172.20.10.1',
                    help='Cole IP address (UDP enabled if set)')
parser.add_argument('--cole-ip',  type=str,   default='172.20.10.9',
                    help='ESP32 IP address (UDP enabled if set)')
parser.add_argument('--esp-port', type=int,  default=8888,
                    help='ESP32 UDP port for xCam/area')
parser.add_argument('--ids-port', type=int,  default=9999,
                    help='UDP port for JSON IDS list')
parser.add_argument('--server-ip', type=str, default='172.20.10.3',
                    help='YOLO‑tracker server IP')
parser.add_argument('--no-udp', action='store_true',
                    help='Disable UDP even if esp-ip default is present')
conflate = parser.add_mutually_exclusive_group()
conflate.add_argument('--conflate', dest='conflate', action='store_true',
                      help='Enable ZMQ CONFLATE (overwrite old frames)')
conflate.add_argument('--fifo', dest='conflate', action='store_false',
                      help='Regular FIFO ZMQ queue (default)')
parser.set_defaults(conflate=False)
args = parser.parse_args()

JPEG_Q     = max(1, min(args.quality, 100))
RATE_LIMIT = args.rate if args.rate > 0 else None
SNDHWM     = max(1, args.hwm)

PREV_W, PREV_H   = 1280, 960
THUMB_W, THUMB_H = 320, 240
SX, SY           = PREV_W/THUMB_W, PREV_H/THUMB_H

# ---------- UDP ------------------------------------------------------
if args.no_udp or not args.esp_ip:
    UDP_SOCK = None
    print('[INIT] UDP disabled')
else:
    XCAM_ADDR = (args.esp_ip, args.esp_port)      # legacy packet
    IDS_ADDR  = (args.esp_ip, args.ids_port)      # new JSON IDS packet
    UDP_SOCK  = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(f"[INIT] UDP  xCam/area → {XCAM_ADDR[0]}:{XCAM_ADDR[1]}   |   IDS(JSON) → {IDS_ADDR[0]}:{IDS_ADDR[1]}")

# ---------- JPEG encoder --------------------------------------------
try:
    from turbojpeg import TurboJPEG, TJPF_RGB
    TJ = TurboJPEG()
    def jpeg_encode(img):
        return TJ.encode(img, quality=JPEG_Q, pixel_format=TJPF_RGB)
except ImportError:
    def jpeg_encode(img):
        return cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, JPEG_Q])[1].tobytes()

# ---------- ZMQ ------------------------------------------------------
ctx  = zmq.Context.instance()
push = ctx.socket(zmq.PUSH)
push.setsockopt(zmq.SNDHWM, SNDHWM)
if args.conflate:
    push.setsockopt(zmq.CONFLATE, 1)
    print('[INIT] ZMQ CONFLATE on')
push.connect(f"tcp://{args.server_ip}:5555")

sub  = ctx.socket(zmq.SUB)
sub.setsockopt(zmq.SUBSCRIBE, b"")
sub.connect(f"tcp://{args.server_ip}:5556")
print('[INIT] ZMQ ready  (PUSH 5555, SUB 5556)')

# ---------- Preview server ------------------------------------------
app = Flask(__name__)
FRAME_Q = queue.Queue(maxsize=1)

@app.route('/')
def index():
    return "<img src='/stream'>"

@app.route('/stream')
def stream():
    def gen():
        while True:
            img = FRAME_Q.get()
            yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + img + b"\r\n"
    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')

if args.stream:
    threading.Thread(target=lambda: app.run(host='0.0.0.0', port=args.stream, debug=False, use_reloader=False),
                     daemon=True).start()
    print(f"[HTTP] Preview :{args.stream}")

# ---------- Camera ---------------------------------------------------
picam = Picamera2()
picam.configure(picam.create_preview_configuration(main={'format':'RGB888','size':(PREV_W,PREV_H)}))
picam.start(); print('[INIT] Picamera2 started')

# ---------- ZMQ receive thread --------------------------------------
cur_boxes = []
cur_boxes_lock = threading.Lock()

def recv_loop():
    while True:
        try:
            data = sub.recv_json(flags=zmq.NOBLOCK)
            with cur_boxes_lock:
                cur_boxes[:] = data.get('boxes', [])
        except zmq.Again:
            time.sleep(0.002)
threading.Thread(target=recv_loop, daemon=True).start()

# ---------- Helper: send people‑ID JSON list ------------------------

def send_ids(ids_set):
    if not UDP_SOCK:
        return
    payload = json.dumps(sorted(ids_set))  # e.g. "[3,7,15]" or "[]"
    UDP_SOCK.sendto(payload.encode(), IDS_ADDR)

# ---------- Main loop -----------------------------------------------
last_thumb = last_dbg = time.time()
signal.signal(signal.SIGINT, lambda *_: sys.exit(0))

while True:
    frame = picam.capture_array('main')
    frame[:, :, [0,2]] = frame[:, :, [2,0]]  # RGB → BGR

    # -------- choose selected box -----------------------------------
    with cur_boxes_lock:
        ids = {b[4] for b in cur_boxes}      # ← collect IDs first
        box = None
        if cur_boxes:
            box = cur_boxes[0] if len(ids)==1 else max(cur_boxes, key=lambda b: b[-1])

        for x1,y1,x2,y2,tid,_ in cur_boxes:
            cv2.rectangle(frame,(int(x1*SX),int(y1*SY)),(int(x2*SX),int(y2*SY)),(0,255,0),2)
            cx,cy = int((x1+x2)*SX*0.5), int((y1+y2)*SY*0.5)
            cv2.putText(frame,str(tid),(cx,cy),cv2.FONT_HERSHEY_SIMPLEX,0.55,(0,0,0),1)

    # ---- legacy xCam/area (unchanged) ------------------------------
    if box and UDP_SOCK:
        x1,y1,x2,y2,_,_ = box
        xCam = int((x1+x2)*0.5 * SX)
        area = int((x2-x1)*SX * (y2-y1)*SY)
        UDP_SOCK.sendto(f"{xCam} {area}".encode(), XCAM_ADDR)
        print(f"[UDP] sent xCam={xCam} area={area}")
    else:
        if time.time()-last_dbg>1.0:
            status = 'Waiting for detections …' if not cur_boxes else 'Detections present but none selected'
            print(f'[DBG] {status}')
            last_dbg = time.time()

    # ---- NEW: JSON IDS list ---------------------------------------
    send_ids(ids)

    # -------- ZMQ thumbnail (rate‑limited) ---------------------------
    now = time.time()
    if RATE_LIMIT is None or (now - last_thumb) >= 1.0/max(RATE_LIMIT,1e-6):
        push.send(struct.pack('<d', now) + jpeg_encode(cv2.resize(frame,(THUMB_W,THUMB_H))))
        last_thumb = now

    # -------- preview MJPEG -----------------------------------------
    if args.stream:
        try:
            jpg = jpeg_encode(frame)
            if FRAME_Q.full(): FRAME_Q.get_nowait()
            FRAME_Q.put_nowait(jpg)
        except Exception:
            pass
