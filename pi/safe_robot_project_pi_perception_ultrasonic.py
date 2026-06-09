#!/usr/bin/env python3
import os
import socket
import time
import threading
from collections import deque

PC_IP = os.environ.get("PC_IP", "192.168.0.8")
PC_PORT = int(os.environ.get("PC_PORT", "6002"))
LISTEN_PORT = int(os.environ.get("LISTEN_PORT", "6002"))

# HC-SR04, BCM numbering
ULTRASONIC_TRIG = int(os.environ.get("ULTRASONIC_TRIG", "22"))
ULTRASONIC_ECHO = int(os.environ.get("ULTRASONIC_ECHO", "25"))

MIN_VALID_MM = int(os.environ.get("ULTRA_MIN_VALID_MM", "20"))
DANGER_MM = int(os.environ.get("ULTRA_DANGER_MM", "120"))
OBSTACLE_MM = int(os.environ.get("ULTRA_OBSTACLE_MM", "230"))
MAX_VALID_MM = int(os.environ.get("ULTRA_MAX_VALID_MM", "2000"))
STALE_MS = int(os.environ.get("ULTRA_STALE_MS", "500"))
MIN_ARUCO_AREA = int(os.environ.get("MIN_ARUCO_AREA", "8000"))

raw_dist_mm = -1
dist_mm_latest = -1
dist_valid = False
dist_state = "INIT"
dist_last_update_ms = 0
dist_buf = deque(maxlen=5)

# ArUco false-positive / wrong-ID filter.
# A marker ID is accepted only after the same ID appears repeatedly.
ARUCO_STABLE_COUNT = int(os.environ.get("ARUCO_STABLE_COUNT", "3"))
MIN_ARUCO_AREA = int(os.environ.get("MIN_ARUCO_AREA", "8000"))
MIN_ARUCO_AREA_ID0 = int(os.environ.get("MIN_ARUCO_AREA_ID0", "1200"))
MIN_ARUCO_AREA_ID1 = int(os.environ.get("MIN_ARUCO_AREA_ID1", "1800"))
MIN_ARUCO_AREA_ID2 = int(os.environ.get("MIN_ARUCO_AREA_ID2", "2500"))
aruco_candidate_id = -1
aruco_candidate_count = 0


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


def to_int(v, default=-1):
    try:
        return int(float(v))
    except Exception:
        return default


def update_ultrasonic(raw):
    global raw_dist_mm, dist_mm_latest, dist_valid, dist_state, dist_last_update_ms

    raw_dist_mm = int(raw) if raw is not None else -1
    dist_last_update_ms = now_ms()

    if raw_dist_mm <= 0 or raw_dist_mm <= MIN_VALID_MM:
        dist_buf.clear()
        dist_mm_latest = -1
        dist_valid = False
        dist_state = "INVALID_LOW"
        return

    if raw_dist_mm >= MAX_VALID_MM:
        dist_buf.clear()
        dist_mm_latest = raw_dist_mm
        dist_valid = False
        dist_state = "OUT_OF_RANGE"
        return

    dist_buf.append(raw_dist_mm)
    b = sorted(dist_buf)
    dist_mm_latest = b[len(b) // 2]
    dist_valid = True

    if dist_mm_latest < DANGER_MM:
        dist_state = "DANGER"
    elif dist_mm_latest < OBSTACLE_MM:
        dist_state = "BLOCKED"
    else:
        dist_state = "CLEAR"


def ultrasonic_loop():
    try:
        from gpiozero import DistanceSensor
    except Exception as e:
        print(f"[ULTRA] gpiozero import failed: {e}", flush=True)
        update_ultrasonic(-1)
        return

    try:
        sensor = DistanceSensor(
            echo=ULTRASONIC_ECHO,
            trigger=ULTRASONIC_TRIG,
            max_distance=MAX_VALID_MM / 1000.0,
            queue_len=3,
        )
        print(
            f"[ULTRA] HC-SR04 ready trig={ULTRASONIC_TRIG} echo={ULTRASONIC_ECHO}",
            flush=True,
        )
    except Exception as e:
        print(f"[ULTRA] sensor init failed: {e}", flush=True)
        update_ultrasonic(-1)
        return

    while True:
        try:
            distance_m = sensor.distance
            raw_mm = int(distance_m * 1000) if distance_m and distance_m > 0 else -1
            update_ultrasonic(raw_mm)
        except Exception as e:
            print(f"[ULTRA] read error: {e}", flush=True)
            update_ultrasonic(-1)

        time.sleep(0.05)


def normalize_aruco(d):
    global aruco_candidate_id, aruco_candidate_count

    aruco = to_int(d.get("aruco", 0), 0)
    marker_id = to_int(d.get("id", -1), -1)
    area = to_int(d.get("area", 0), 0)
    cx = to_int(d.get("cx", 320), 320)

    # Final rule: valid ArUco IDs are only 0, 1, 2.
    # Use ID-specific area thresholds:
    # - ID 0 is the first target and should be easier to acquire.
    # - ID 2 has shown false positives, so keep it stricter.
    area_min_by_id = {
        0: MIN_ARUCO_AREA_ID0,
        1: MIN_ARUCO_AREA_ID1,
        2: MIN_ARUCO_AREA_ID2,
    }
    required_area = area_min_by_id.get(marker_id, MIN_ARUCO_AREA)
    valid_now = (aruco == 1 and marker_id in (0, 1, 2) and area >= required_area)

    if not valid_now:
        aruco_candidate_id = -1
        aruco_candidate_count = 0
        d["aruco"] = 0
        d["id"] = -1
        d["cx"] = 320
        d["area"] = 0
        return

    if marker_id == aruco_candidate_id:
        aruco_candidate_count += 1
    else:
        aruco_candidate_id = marker_id
        aruco_candidate_count = 1

    # Reject unstable one-frame ID changes.
    if aruco_candidate_count < ARUCO_STABLE_COUNT:
        d["aruco"] = 0
        d["id"] = -1
        d["cx"] = 320
        d["area"] = 0
        return

    d["aruco"] = 1
    d["id"] = marker_id
    d["cx"] = cx
    d["area"] = area


def apply_ultrasonic_result(d):
    age_ms = now_ms() - dist_last_update_ms if dist_last_update_ms > 0 else 999999

    d["raw_dist_mm"] = raw_dist_mm
    d["dist_mm"] = dist_mm_latest
    d["tof_age_ms"] = age_ms
    d["sensor"] = "ULTRASONIC"

    # Fail-safe: if ultrasonic data is stale, stop the robot.
    if age_ms > STALE_MS:
        d["obstacle"] = 1
        d["fault"] = 1
        d["tof_state"] = "ULTRASONIC_STALE_FAULT"
        return

    if not dist_valid:
        d["obstacle"] = 0
        d["fault"] = 0
        d["tof_state"] = dist_state
        return

    if dist_mm_latest > 0 and dist_mm_latest <= OBSTACLE_MM:
        d["obstacle"] = 1
        d["fault"] = 0
        d["tof_state"] = "BLOCKED"
        return

    d["obstacle"] = 0
    d["fault"] = 0
    d["tof_state"] = "CLEAR"


def main():
    threading.Thread(target=ultrasonic_loop, daemon=True).start()

    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rx.bind(("0.0.0.0", LISTEN_PORT))

    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print(f"[PI_ULTRA_RELAY] listen :{LISTEN_PORT} -> PC {PC_IP}:{PC_PORT}", flush=True)
    print(
        f"[PI_ULTRA_RELAY] obstacle<={OBSTACLE_MM}mm stale>{STALE_MS}ms",
        flush=True,
    )

    while True:
        data, addr = rx.recvfrom(4096)
        line = data.decode(errors="ignore").strip()
        msg_type, d = parse_kv(line)

        if msg_type != "PERCEPTION":
            print(f"[PI_ULTRA_RELAY IGNORE] {addr} {line}", flush=True)
            continue

        raw_aruco = d.get("aruco")
        raw_id = d.get("id")
        raw_cx = d.get("cx")
        raw_area = d.get("area")

        normalize_aruco(d)

        print(
            f"[PI_ARUCO_FILTER] raw_aruco={raw_aruco} raw_id={raw_id} "
            f"raw_cx={raw_cx} raw_area={raw_area} -> "
            f"aruco={d.get('aruco')} id={d.get('id')} "
            f"cx={d.get('cx')} area={d.get('area')} "
            f"min_area={MIN_ARUCO_AREA} stable_count={ARUCO_STABLE_COUNT}",
            flush=True,
        )

        # Ignore AI-G-side obstacle/person fields.
        d["worker"] = 0
        d["obs_left"] = 0
        d["obs_center"] = 0
        d["obs_right"] = 0

        apply_ultrasonic_result(d)

        out = build_kv("PERCEPTION", d)
        tx.sendto(out.encode(), (PC_IP, PC_PORT))

        print(
            f"[PI_ULTRA_RELAY TX] aruco={d.get('aruco')} id={d.get('id')} "
            f"cx={d.get('cx')} area={d.get('area')} "
            f"dist={d.get('dist_mm')} state={d.get('tof_state')} "
            f"obstacle={d.get('obstacle')} fault={d.get('fault')}",
            flush=True,
        )


if __name__ == "__main__":
    main()
