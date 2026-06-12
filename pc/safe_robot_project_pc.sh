#!/bin/bash
cd ~/safe_robot_project

echo "[PC] safe_robot_project PC Gateway start"

./safe_robot_project_net_pc.sh

echo "[PC] TOPST = 192.168.50.20:5005"
echo "[PC] PI    = 192.168.0.12:5006"

sudo fuser -k 5006/udp 6002/udp 2>/dev/null || true
pkill -f "safe_robot_project.py --role pc_gateway" 2>/dev/null || true

python3 -u safe_robot_project.py \
  --role pc_gateway \
  --perception-port 6002 \
  --cmd-port 5006 \
  --topst-ip 192.168.50.20 \
  --topst-port 5005 \
  --pi-ip 192.168.0.12 \
  --pi-cmd-port 5006 \
  --target-mask 4
