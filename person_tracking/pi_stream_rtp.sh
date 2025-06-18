#!/usr/bin/env bash
#
# pi_stream_rtp.sh – hardware H.264 encode + RTP over UDP

# Change this to your laptop’s IBSS IP:
LAPTOP_IP="192.168.50.2"
PORT=5000

# No preview (-n), unlimited time (-t 0), 1280×960 @ 15 fps
raspivid -n -t 0 -w 1280 -h 960 -fps 15 -o - \
  | gst-launch-1.0 -v \
      fdsrc ! h264parse ! rtph264pay config-interval=1 pt=96 \
      ! udpsink host=${LAPTOP_IP} port=${PORT}
