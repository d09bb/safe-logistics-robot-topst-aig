#!/bin/sh
cd /home/root

echo "[AI-G] set eth0 = 192.168.60.2"
ifconfig eth0 192.168.60.2 netmask 255.255.255.0 up

echo "[AI-G] start perception receiver - obstacle tuned"
./safe_robot_project \
  --listen-port 7000 \
  --out-ip 192.168.60.1 \
  --out-port 6002 \
  --threshold 70 \
  --min-area 800
