#!/usr/bin/env python3
"""
pi_client.py – Raspberry Pi capture/transmit side (WebSocket + overlay)
────────────────────────────────────────────────────────────────────────
Run with **no arguments**:

    python3 pi_client.py

• tracking_id  ← UDP 7777 (ASCII)
• IDS list     → UDP 9999 (JSON array)
• xCam,area    → WebSocket ws://172.20.10.11:80/ws_data   (text: "x,area")
• MJPEG preview (boxes + large white ID labels) on http://<Pi-IP>:8000/stream
"""

import sys, time, struct, signal, queue, threading, socket, json
import cv2, zmq, websocket               # pip install websocket-client
from flask import Flask, Response
from picamera2 import Picamera2

# ───────────── Network constants ────────────────────────────────────
ESP_IP      = "172.20.10.11"
WS_URL      = f"ws://{ESP_IP}:80/ws_data"     # WebSocket target
UI_IP       = "172.20.10.1"
YOLO_IP     = "172.20.10.3"
PORT_IDS    = 9999
PORT_TID    = 7777
STREAM_PORT = 8000

# ───────────── Image / preview settings ─────────────────────────────
JPEG_Q    = 60
THUMB_FPS = 15.0
PREV_W, PREV_H   = 1280, 960
THUMB_W, THUMB_H = 320,  240
SX, SY = PREV_W / THUMB_W, PREV_H / THUMB_H   # scale tracker→preview

# ───────────── UDP sockets ──────────────────────────────────────────
sock_ids = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
IDS_ADDR = (UI_IP, PORT_IDS)

sock_tid = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_tid.bind(("0.0.0.0", PORT_TID))
sock_tid.setblocking(False)

print(f"[INIT] IDS→{IDS_ADDR}   |  listen TID *:{PORT_TID}")

# ───────────── WebSocket helper ─────────────────────────────────────
ws_lock = threading.Lock()
ws_conn = None
def ws_send(txt: str):
    global ws_conn
    with ws_lock:
        try:
            if not ws_conn or not ws_conn.connected:
                ws_conn = websocket.create_connection(WS_URL, timeout=2)
                print(f"[WS] connected → {WS_URL}")
            ws_conn.send(txt)
        except Exception as e:
            print(f"[WS] error: {e}")
            try: ws_conn.close()
            except: pass
            ws_conn = None

# ───────────── JPEG encoder (TurboJPEG if available) ────────────────
try:
    from turbojpeg import TurboJPEG, TJPF_RGB
    TJ = TurboJPEG()
    def jpeg_encode(img):
        return TJ.encode(img, quality=JPEG_Q, pixel_format=TJPF_RGB)
except Exception:
    def jpeg_encode(img):
        return cv2.imencode('.jpg', img,
                            [cv2.IMWRITE_JPEG_QUALITY, JPEG_Q])[1].tobytes()

# ───────────── ZMQ sockets (tracker) ────────────────────────────────
ctx  = zmq.Context.instance()
push = ctx.socket(zmq.PUSH); push.connect(f"tcp://{YOLO_IP}:5555")
sub  = ctx.socket(zmq.SUB ); sub.setsockopt(zmq.SUBSCRIBE, b"")
sub.connect(f"tcp://{YOLO_IP}:5556")

cur_boxes, cur_lock = [], threading.Lock()
def recv_loop():
    while True:
        try:
            data = sub.recv_json(flags=zmq.NOBLOCK)
            with cur_lock:
                cur_boxes[:] = data.get("boxes", [])
        except zmq.Again:
            time.sleep(0.002)
threading.Thread(target=recv_loop, daemon=True).start()

# ───────────── Flask MJPEG preview ──────────────────────────────────
app = Flask(__name__)
FRAME_Q = queue.Queue(maxsize=1)
@app.route("/stream")
def stream():
    def gen():
        while True:
            jpg = FRAME_Q.get()
            yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                   + jpg + b"\r\n")
    return Response(gen(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")

threading.Thread(target=lambda:
    app.run(host="0.0.0.0", port=STREAM_PORT,
            debug=False, use_reloader=False),
    daemon=True).start()
print(f"[HTTP] Preview :{STREAM_PORT}/stream")

# ───────────── Camera ------------------------------------------------
picam = Picamera2()
picam.configure(picam.create_preview_configuration(
        main={'format':'RGB888','size':(PREV_W, PREV_H)}))
picam.start(); print("[INIT] Picamera2 started")

# ───────────── Globals ----------------------------------------------
tracking_id = None   # chosen via UDP 7777

# ───────────── Helpers ----------------------------------------------
def send_ids(ids_set):
    sock_ids.sendto(json.dumps(sorted(ids_set)).encode(), IDS_ADDR)

# ───────────── Main loop ────────────────────────────────────────────
last_thumb = time.time()
signal.signal(signal.SIGINT, lambda *_: sys.exit(0))

FONT        = cv2.FONT_HERSHEY_SIMPLEX
FONT_SCALE  = 1.0        # ↑ bigger
FONT_THICK  = 2
BOX_PAD     = 4          # padding around text for white box

while True:
    # 0) new tracking_id?
    try:
        pkt, _ = sock_tid.recvfrom(32)
        tracking_id = int(pkt.decode().strip())
        print(f"[TID] set tracking_id → {tracking_id}")
    except BlockingIOError:
        pass
    except ValueError:
        print("[TID] malformed packet")

    # 1) capture frame
    frame = picam.capture_array('main')
    frame[:, :, [0,2]] = frame[:, :, [2,0]]  # RGB→BGR

    # 2) snapshot detections
    with cur_lock:
        boxes = list(cur_boxes)
        ids_set = {b[4] for b in boxes}

    # 3) find target box (if present)
    box = None
    if tracking_id in ids_set:
        box = next(b for b in boxes if b[4] == tracking_id)

    # 4) draw all boxes + large white ID
    for x1,y1,x2,y2,tid,_ in boxes:
        # rectangle
        cv2.rectangle(frame,(int(x1*SX),int(y1*SY)),
                             (int(x2*SX),int(y2*SY)),(0,255,0),2)
        # text background
        label   = str(tid)
        (tw,th), _ = cv2.getTextSize(label, FONT, FONT_SCALE, FONT_THICK)
        px = int((x1+x2)*SX*0.5 - tw/2) - BOX_PAD
        py = int((y1+y2)*SY*0.5 + th/2) + BOX_PAD
        cv2.rectangle(frame,(px,py-th-2*BOX_PAD),
                             (px+tw+2*BOX_PAD, py),
                             (255,255,255), thickness=-1)
        # text
        cv2.putText(frame,label,(px+BOX_PAD,py-BOX_PAD),
                    FONT,FONT_SCALE,(0,0,0),FONT_THICK)

    # 5) send xCam,area via WebSocket for target
    if box:
        x1,y1,x2,y2,_,_ = box
        xCam = int((x1+x2)*0.5*SX)
        area = int((x2-x1)*SX * (y2-y1)*SY)
        ws_send(f"{xCam},{area}")

    # 6) IDS list broadcast
    send_ids(ids_set)

    # 7) thumbnail PUSH (fps-capped)
    now = time.time()
    if (now-last_thumb) >= 1.0/max(THUMB_FPS,1e-6):
        thumb = cv2.resize(frame,(THUMB_W,THUMB_H))
        push.send(struct.pack('<d',now) + jpeg_encode(thumb))
        last_thumb = now

    # 8) preview
    try:
        jpg = jpeg_encode(frame)
        if FRAME_Q.full(): FRAME_Q.get_nowait()
        FRAME_Q.put_nowait(jpg)
    except Exception:
        pass
