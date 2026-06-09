#!/usr/bin/env python3
import argparse
import socket
import time

try:
    import serial
except ImportError:
    raise SystemExit("pyserial is missing. Install with: sudo apt install -y python3-serial")


def parse_kv_tokens(text):
    d = {}
    for tok in text.replace(",", " ").split():
        if "=" in tok:
            k, v = tok.split("=", 1)
            d[k.strip()] = v.strip()
    return d


def to_int(v, default=0):
    try:
        return int(float(v))
    except Exception:
        return default


def normalize_controller_line(raw, seq_fallback):
    line = raw.strip()
    if not line:
        return None

    if "READY" in line:
        return None

    d = parse_kv_tokens(line)

    # Friend Arduino format:
    # CTRL seq=... start=... estop=... deadman=... target_mask=... joyx=... joyy=...
    if line.startswith("CTRL") or "target_mask" in d or "start" in d or "estop" in d or "deadman" in d:
        seq = to_int(d.get("seq", seq_fallback), seq_fallback)

        # Final project uses ArUco IDs 0, 1, 2 only.
        # Therefore bit 3, target_mask=8, is ignored.
        target_mask = to_int(d.get("target_mask", 0), 0) & 0b111

        start = to_int(d.get("start", 0), 0)
        estop = to_int(d.get("estop", 0), 0)
        deadman_raw = to_int(d.get("deadman", 0), 0)

        # Arduino code does not send manual explicitly.
        manual = to_int(d.get("manual", 0), 0)

        # Final policy:
        # - E-STOP locked: deadman must be 0
        # - Manual mode: use Arduino deadman
        # - Auto mode: allow motion when E-STOP is released
        if estop == 1:
            deadman = 0
        elif manual == 1:
            deadman = deadman_raw
        else:
            deadman = 1

        joy_x = to_int(d.get("joy_x", d.get("joyx", d.get("joyX", 512))), 512)
        joy_y = to_int(d.get("joy_y", d.get("joyy", d.get("joyY", 512))), 512)

        return (
            f"CONTROLLER seq={seq} "
            f"target_mask={target_mask} "
            f"start={start} "
            f"estop={estop} "
            f"deadman={deadman} "
            f"manual={manual} "
            f"joy_x={joy_x} "
            f"joy_y={joy_y}"
        )

    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--serial", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--topst-ip", default="192.168.50.20")
    ap.add_argument("--topst-port", type=int, default=5005)
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print(f"[ARDUINO_BRIDGE] serial={args.serial} baud={args.baud}", flush=True)
    print(f"[ARDUINO_BRIDGE] TOPST={args.topst_ip}:{args.topst_port}", flush=True)

    ser = serial.Serial(args.serial, args.baud, timeout=1)
    time.sleep(2)

    seq_fallback = 1

    while True:
        raw = ser.readline().decode(errors="ignore").strip()
        if not raw:
            continue

        print(f"[ARDUINO_RX] {raw}", flush=True)

        msg = normalize_controller_line(raw, seq_fallback)
        if msg is None:
            print("[ARDUINO_BRIDGE] ignored", flush=True)
            continue

        sock.sendto(msg.encode(), (args.topst_ip, args.topst_port))
        print(f"[ARDUINO_TX] {msg}", flush=True)

        seq_fallback += 1


if __name__ == "__main__":
    main()
