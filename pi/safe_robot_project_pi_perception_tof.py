#!/usr/bin/env python3
import socket
import time
import threading
import os
from collections import deque

import RPi.GPIO as GPIO

# ============================================================
# Network
# ============================================================
PC_IP = "192.168.0.8"
PC_PORT = 6002
LISTEN_PORT = 6002

# ============================================================
# Ultrasonic sensor pins
# HC-SR04:
# TRIG -> GPIO22
# ECHO -> GPIO25 through voltage divider
# ============================================================
TRIG = 22
ECHO = 25

# ============================================================
# Obstacle threshold
# ============================================================
ULTRA_OBSTACLE_CM = 15.0
ULTRA_CLEAR_CM = 22.0
ULTRA_TIMEOUT_SEC = 0.03
ULTRA_PERIOD_SEC = 0.08

raw_dist_cm = -1.0
dist_cm_latest = -1.0
dist_mm_latest = -1
dist_state = "INIT"
dist_last_update_ms = 0

dist_buf = deque(maxlen=3)
obstacle_latched = False


def now_ms():
    return int(time.time() * 1000)


def parse_kv(line):
    parts = line.strip().split()
    if not parts:
        return "", {}

    msg_type = parts[0]
    d = {}

    for tok in parts[1:]:
        if "=" in tok:
            k, v = tok.split("=", 1)
            d[k] = v

    return msg_type, d


def build_kv(msg_type, d):
    return " ".join([msg_type] + [f"{k}={v}" for k, v in d.items()])


def _to_int(v, default=-1):
    try:
        return int(float(v))
    except Exception:
        return default


def setup_ultrasonic():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(TRIG, GPIO.OUT)
    GPIO.setup(ECHO, GPIO.IN)
    GPIO.output(TRIG, False)
    time.sleep(0.3)


def measure_cm():
    GPIO.output(TRIG, False)
    time.sleep(0.000002)

    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    t0 = time.time()

    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()
        if pulse_start - t0 > ULTRA_TIMEOUT_SEC:
            return -1.0

    t1 = time.time()

    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()
        if pulse_end - t1 > ULTRA_TIMEOUT_SEC:
            return -1.0

    duration = pulse_end - pulse_start
    return duration * 34300.0 / 2.0


def ultrasonic_loop():
    global raw_dist_cm, dist_cm_latest, dist_mm_latest
    global dist_state, dist_last_update_ms, obstacle_latched

    try:
        setup_ultrasonic()
        print(f"[ULTRA] HC-SR04 start TRIG=GPIO{TRIG} ECHO=GPIO{ECHO}", flush=True)
        print(
            f"[ULTRA] obstacle<={ULTRA_OBSTACLE_CM:.1f}cm clear>={ULTRA_CLEAR_CM:.1f}cm",
            flush=True,
        )
    except Exception as e:
        print("[ULTRA] GPIO setup failed:", e, flush=True)
        return

    while True:
        try:
            d_cm = measure_cm()
            raw_dist_cm = d_cm
            dist_last_update_ms = now_ms()

            if d_cm > 0:
                dist_buf.append(d_cm)
                b = sorted(dist_buf)
                dist_cm_latest = b[len(b) // 2]
                dist_mm_latest = int(dist_cm_latest * 10.0)

                if dist_cm_latest <= ULTRA_OBSTACLE_CM:
                    obstacle_latched = True
                    dist_state = "BLOCKED"
                elif dist_cm_latest >= ULTRA_CLEAR_CM:
                    obstacle_latched = False
                    dist_state = "CLEAR"
                else:
                    dist_state = "BLOCKED" if obstacle_latched else "CLEAR"
            else:
                # No echo usually means no close object or out of range.
                # Keep this as CLEAR for demo stability.
                dist_buf.clear()
                dist_cm_latest = -1.0
                dist_mm_latest = -1
                obstacle_latched = False
                dist_state = "ULTRA_NO_ECHO_CLEAR"

        except Exception as e:
            print("[ULTRA] read error:", e, flush=True)
            raw_dist_cm = -1.0
            dist_cm_latest = -1.0
            dist_mm_latest = -1
            dist_state = "ULTRA_ERROR_CLEAR"
            obstacle_latched = False

        time.sleep(ULTRA_PERIOD_SEC)


def normalize_aruco(d):
    _aruco = _to_int(d.get("aruco", 0), 0)
    _id = _to_int(d.get("id", -1), -1)

    if _aruco != 1 or _id not in (0, 1, 2):
        d["aruco"] = 0
        d["id"] = -1
        d["cx"] = 320
        d["area"] = 0
    else:
        d["aruco"] = 1
        d["id"] = _id
        d["cx"] = _to_int(d.get("cx", 320), 320)
        d["area"] = _to_int(d.get("area", 0), 0)

    return d


def main():
    threading.Thread(target=ultrasonic_loop, daemon=True).start()

    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rx.bind(("0.0.0.0", LISTEN_PORT))

    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print(f"[PI_ULTRA_RELAY] listen :{LISTEN_PORT} -> PC {PC_IP}:{PC_PORT}", flush=True)

    while True:
        data, addr = rx.recvfrom(4096)
        line = data.decode(errors="ignore").strip()
        msg_type, d = parse_kv(line)

        if msg_type != "PERCEPTION":
            print(f"[PI_ULTRA_RELAY IGNORE] {addr} {line}", flush=True)
            continue

        # Preserve AI-G raw obstacle fields separately, then ignore them.
        d["aig_obstacle"] = d.get("obstacle", "0")
        d["aig_obs_left"] = d.get("obs_left", "0")
        d["aig_obs_center"] = d.get("obs_center", "0")
        d["aig_obs_right"] = d.get("obs_right", "0")

        # Normalize ArUco: valid IDs are only 0, 1, 2.
        d = normalize_aruco(d)

        # Remove AI-G side obstacle/person/fault fields.
        for k in (
            "worker", "fault",
            "obs_left", "obs_center", "obs_right",
            "aig_obstacle", "aig_obs_left", "aig_obs_center", "aig_obs_right",
        ):
            d.pop(k, None)

        # Final obstacle decision from ultrasonic only.
        d["raw_dist_cm"] = f"{raw_dist_cm:.1f}" if raw_dist_cm > 0 else "-1"
        d["dist_cm"] = f"{dist_cm_latest:.1f}" if dist_cm_latest > 0 else "-1"
        d["dist_mm"] = dist_mm_latest
        d["tof_state"] = dist_state  # keep field name for TOPST compatibility

        if obstacle_latched:
            d["obstacle"] = 1
            d["obs_left"] = 0
            d["obs_center"] = 1
            d["obs_right"] = 0
        else:
            d["obstacle"] = 0
            d["obs_left"] = 0
            d["obs_center"] = 0
            d["obs_right"] = 0

        out = build_kv("PERCEPTION", d)
        tx.sendto(out.encode(), (PC_IP, PC_PORT))

        print(
            f"[PI_ULTRA_RELAY TX] raw_cm={d['raw_dist_cm']} dist_cm={d['dist_cm']} "
            f"state={d['tof_state']} obstacle={d['obstacle']} "
            f"aruco={d.get('aruco')} id={d.get('id')} cx={d.get('cx')} area={d.get('area')}",
            flush=True,
        )


if __name__ == "__main__":
    main()
