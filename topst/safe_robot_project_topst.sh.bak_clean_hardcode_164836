#!/bin/sh

cd /home/root/safe_robot_project || exit 1

echo "[TOPST_NET] setup"
if [ -x ./safe_robot_project_net_topst_fixed.sh ]; then
    ./safe_robot_project_net_topst_fixed.sh
else
    echo "[TOPST_NET] WARN: network setup script not found"
fi

echo "[TOPST] stop old TOPST"
pkill -f "safe_robot_topst_clean.py" 2>/dev/null || true
pkill -f "safe_robot_project.py --role topst" 2>/dev/null || true

sleep 1

echo "[TOPST] safe_robot_project TOPST Supervisor start"

exec python3 -u safe_robot_project.py \
  --role topst \
  --listen-port 5005 \
  --pc-ip 192.168.50.10 \
  --pc-port 5006 \
  --targets "" \
  --send-hz 10 \
  --ttl-ms 2000 \
  --center 320 \
  --deadband 10 \
  --speed 40 \
  --reach-area 100000 \
  --reach-count 3 \
  --perception-timeout-ms 2000 \
  --target-lost-hold-ms 1200
