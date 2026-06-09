#!/bin/bash
set -e

BASE=${SAFE_ROBOT_DIR:-/home/a/safe_robot_project}
cd "$BASE"

if [ -f ./robot_env.sh ]; then
  source ./robot_env.sh
fi

PI_AIG_IP=${PI_AIG_IP:-192.168.60.1}
AIG_IP=${AIG_IP:-192.168.60.2}
PC_IP=${PC_IP:-192.168.0.8}

PERCEPTION_PORT=${PERCEPTION_PORT:-6002}
CMD_PORT=${CMD_PORT:-5006}
AIG_FRAME_PORT=${AIG_FRAME_PORT:-7000}

ULTRASONIC_TRIG=${ULTRASONIC_TRIG:-22}
ULTRASONIC_ECHO=${ULTRASONIC_ECHO:-25}
ULTRA_OBSTACLE_MM=${ULTRA_OBSTACLE_MM:-230}
ULTRA_STALE_MS=${ULTRA_STALE_MS:-500}

echo "[PI] set eth0 = ${PI_AIG_IP} for AI-G"
sudo ifconfig eth0 "${PI_AIG_IP}" netmask 255.255.255.0 up

echo "[PI] stop old Pi-side processes"

safe_kill() {
  PATTERN="$1"
  PIDS=$(pgrep -f "$PATTERN" 2>/dev/null || true)

  for PID in $PIDS; do
    if [ "$PID" != "$$" ]; then
      sudo kill -9 "$PID" 2>/dev/null || true
    fi
  done
}

safe_kill "safe_robot_project.py --role pi_perception"
safe_kill "safe_robot_project.py --role pi_vehicle"
safe_kill "safe_robot_project.py --role pi_frame"
safe_kill "safe_robot_project_pi_frame_stable.py"
safe_kill "safe_robot_project_vehicle_stable.py"
safe_kill "safe_robot_project_pi_perception_tof.py"
safe_kill "safe_robot_project_pi_perception_ultrasonic.py"

sudo fuser -k ${CMD_PORT}/udp ${PERCEPTION_PORT}/udp 2>/dev/null || true
sudo fuser -k /dev/video0 /dev/video1 /dev/video2 /dev/video3 2>/dev/null || true

echo "[PI] start ultrasonic perception relay"
echo "[PI] AI-G perception UDP ${PERCEPTION_PORT} -> PC ${PC_IP}:${PERCEPTION_PORT}"
echo "[PI] ultrasonic trig=${ULTRASONIC_TRIG} echo=${ULTRASONIC_ECHO} obstacle<=${ULTRA_OBSTACLE_MM}mm"

PC_IP="${PC_IP}" \
PC_PORT="${PERCEPTION_PORT}" \
LISTEN_PORT="${PERCEPTION_PORT}" \
ULTRASONIC_TRIG="${ULTRASONIC_TRIG}" \
ULTRASONIC_ECHO="${ULTRASONIC_ECHO}" \
ULTRA_OBSTACLE_MM="${ULTRA_OBSTACLE_MM}" \
ULTRA_STALE_MS="${ULTRA_STALE_MS}" \
python3 -u safe_robot_project_pi_perception_ultrasonic.py \
  > /tmp/safe_robot_project_pi_perception.log 2>&1 &

echo "[PI] start vehicle motor receiver: UDP ${CMD_PORT}"
sudo python3 -u safe_robot_project_vehicle_stable.py \
  > /tmp/safe_robot_project_pi_vehicle.log 2>&1 &

echo "[PI] start frame sender: /dev/video0 -> AI-G ${AIG_IP}:${AIG_FRAME_PORT}"
sudo python3 -u safe_robot_project_pi_frame_stable.py \
  --aig-ip "${AIG_IP}" \
  --aig-port "${AIG_FRAME_PORT}" \
  --dev /dev/video0 \
  --width 640 \
  --height 360 \
  --fps 8 \
  > /tmp/safe_robot_project_pi_frame.log 2>&1 &

echo "[PI] started."
echo "  tail -f /tmp/safe_robot_project_pi_perception.log"
echo "  tail -f /tmp/safe_robot_project_pi_vehicle.log"
echo "  tail -f /tmp/safe_robot_project_pi_frame.log"
