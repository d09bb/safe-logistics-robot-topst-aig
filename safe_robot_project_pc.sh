#!/usr/bin/env bash
set -e

cd "$(dirname "$0")"

echo "[PC] safe_robot_project PC gateway start"

# 기존 PC gateway 종료
pkill -f "safe_robot_project.py --role pc_gateway" 2>/dev/null || true

# 이 repo 기준 PC gateway 파일
MAIN="./pc/safe_robot_project.py"

if [ ! -f "$MAIN" ]; then
  echo "[PC][ERROR] $MAIN 파일이 없음"
  echo "[PC] 현재 파일 목록:"
  find . -maxdepth 3 -type f | sort
  exit 1
fi

echo "[PC] MAIN=$MAIN"

python3 "$MAIN" --role pc_gateway \
  --perception-port 6002 \
  --cmd-port 5006 \
  --topst-ip 192.168.50.20 \
  --topst-port 5005 \
  --pi-ip 192.168.0.24 \
  --pi-cmd-port 5006
