#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"

python3 -u pc/arduino_topst_bridge.py \
  --serial /dev/ttyACM0 \
  --baud 115200 \
  --topst-ip 192.168.50.20 \
  --topst-port 5005
