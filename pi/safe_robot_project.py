#!/usr/bin/env python3
import argparse
import socket
import time
import threading
import sys
import os

# ============================================================
# Common utilities
# ============================================================

def now_ms():
    return int(time.time() * 1000)

def safe_int(v, default=0):
    try:
        return int(v)
    except Exception:
        return default

def parse_kv(line):
    parts = line.strip().split()
    if not parts:
        return "", {}

    msg_type = parts[0]
    data = {}

    for tok in parts[1:]:
        if "=" not in tok:
            continue
        k, v = tok.split("=", 1)
        data[k] = v

    return msg_type, data

def build_kv(msg_type, data):
    parts = [msg_type]
    for k, v in data.items():
        parts.append(f"{k}={v}")
    return " ".join(parts)

def udp_send(sock, ip, port, text):
    sock.sendto(text.encode(), (ip, port))

def bind_udp(port, ip="0.0.0.0", blocking=True):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((ip, port))
    s.setblocking(blocking)
    return s

# ============================================================
# Role 1: PC Gateway
# - Pi perception UDP 6002 -> TOPST UDP 5005
# - TOPST CMD UDP 5006 -> Pi UDP 5006
# - Optional controller keepalive -> TOPST UDP 5005
# ============================================================

def patch_perception_for_demo(text, force_obstacle0=False, force_id=None, force_area=None):
    msg_type, data = parse_kv(text)
    if msg_type != "PERCEPTION":
        return text

    if force_obstacle0:
        data["obstacle"] = "0"
        data["obs_left"] = "0"
        data["obs_center"] = "0"
        data["obs_right"] = "0"

    if force_id is not None:
        data["aruco"] = "1"
        data["id"] = str(force_id)
        if "cx" not in data:
            data["cx"] = "160"
        if force_area is not None:
            data["area"] = str(force_area)
        elif "area" not in data or safe_int(data.get("area"), 0) <= 0:
            data["area"] = "1500"

    if "worker" not in data:
        data["worker"] = "0"
    if "fault" not in data:
        data["fault"] = "0"

    return build_kv("PERCEPTION", data)

def role_pc_gateway(args):
    print("[SAFE_PC] start pc_gateway")
    print(f"[SAFE_PC] perception listen UDP :{args.perception_port} -> TOPST {args.topst_ip}:{args.topst_port}")
    print(f"[SAFE_PC] cmd listen UDP :{args.cmd_port} -> PI {args.pi_ip}:{args.pi_cmd_port}")
    print(f"[SAFE_PC] force_obstacle0={args.force_obstacle0}, force_id={args.force_id}")

    p_rx = bind_udp(args.perception_port)
    c_rx = bind_udp(args.cmd_port)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    running = True

    def perception_loop():
        while running:
            data, addr = p_rx.recvfrom(4096)
            text = data.decode(errors="ignore").strip()

            if not text.startswith("PERCEPTION "):
                print(f"[SAFE_PC P_IGNORE] {addr} {text}", flush=True)
                continue

            patched = patch_perception_for_demo(
                text,
                force_obstacle0=args.force_obstacle0,
                force_id=args.force_id,
                force_area=args.force_area,
            )

            print(f"[SAFE_PC P_RX] {addr} {text}", flush=True)
            if patched != text:
                print(f"[SAFE_PC P_PATCH] {patched}", flush=True)

            udp_send(tx, args.topst_ip, args.topst_port, patched)
            print("[SAFE_PC P_TX] -> TOPST", flush=True)

    def cmd_loop():
        while running:
            data, addr = c_rx.recvfrom(4096)
            text = data.decode(errors="ignore").strip()

            if not text.startswith("CMD "):
                print(f"[SAFE_PC C_IGNORE] {addr} {text}", flush=True)
                continue

            print(f"[SAFE_PC C_RX] {addr} {text}", flush=True)
            udp_send(tx, args.pi_ip, args.pi_cmd_port, text)
            print("[SAFE_PC C_TX] -> PI", flush=True)

    def controller_keepalive_loop():
        seq = 1
        while running:
            if args.controller_keepalive:
                msg = (
                    f"CONTROLLER seq={seq} start=1 estop=0 deadman=1 manual=0 "
                    f"joy_x=512 joy_y=512 target_mask={args.target_mask}"
                )
                udp_send(tx, args.topst_ip, args.topst_port, msg)
                print(f"[SAFE_PC CTRL_TX] {msg}", flush=True)
                seq += 1
            time.sleep(args.controller_period)

    threading.Thread(target=perception_loop, daemon=True).start()
    threading.Thread(target=cmd_loop, daemon=True).start()
    threading.Thread(target=controller_keepalive_loop, daemon=True).start()

    while True:
        time.sleep(1)

# ============================================================
# Role 2: Pi perception relay
# - AI-G UDP 6002 -> PC UDP 6002
# ============================================================

def role_pi_perception(args):
    print("[SAFE_PI_PERCEPTION] start")
    print(f"[SAFE_PI_PERCEPTION] listen UDP :{args.listen_port} -> PC {args.pc_ip}:{args.pc_port}")

    rx = bind_udp(args.listen_port)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    while True:
        data, addr = rx.recvfrom(4096)
        text = data.decode(errors="ignore").strip()

        if not text.startswith("PERCEPTION "):
            print(f"[SAFE_PI_PERCEPTION IGNORE] {addr} {text}", flush=True)
            continue

        print(f"[SAFE_PI_PERCEPTION RX] {addr} {text}", flush=True)
        udp_send(tx, args.pc_ip, args.pc_port, text)
        print("[SAFE_PI_PERCEPTION TX] -> PC", flush=True)

# ============================================================
# Role 3: Pi frame sender
# - Pi Arducam /dev/videoX -> AI-G TCP 7000
# Protocol: FRAM + seq + w + h + len + grayscale bytes
# ============================================================

def role_pi_frame(args):
    try:
        import cv2
    except Exception as e:
        print(f"[SAFE_PI_FRAME] cv2 import failed: {e}")
        sys.exit(1)

    import struct

    def connect_loop():
        while True:
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.connect((args.aig_ip, args.aig_port))
                print(f"[SAFE_PI_FRAME] connected to AI-G {args.aig_ip}:{args.aig_port}", flush=True)
                return s
            except Exception as e:
                print(f"[SAFE_PI_FRAME] connect retry: {e}", flush=True)
                time.sleep(1)

    print("[SAFE_PI_FRAME] start")
    print(f"[SAFE_PI_FRAME] dev={args.dev}, send={args.width}x{args.height}, fps={args.fps}")

    cap = cv2.VideoCapture(args.dev, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.cam_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.cam_height)
    cap.set(cv2.CAP_PROP_FPS, args.cam_fps)

    if not cap.isOpened():
        print(f"[SAFE_PI_FRAME] camera open failed: {args.dev}")
        sys.exit(1)

    ready = False
    for _ in range(20):
        ok, frame = cap.read()
        if ok and frame is not None:
            print(f"[SAFE_PI_FRAME] camera ready shape={frame.shape}", flush=True)
            ready = True
            break
        time.sleep(0.1)

    if not ready:
        print("[SAFE_PI_FRAME] camera read failed")
        sys.exit(1)

    sock = connect_loop()
    seq = 1
    interval = 1.0 / args.fps

    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            print("[SAFE_PI_FRAME] frame read failed", flush=True)
            time.sleep(0.2)
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.resize(gray, (args.width, args.height))
        payload = gray.tobytes()

        header = struct.pack("!4sIHHI", b"FRAM", seq, args.width, args.height, len(payload))

        try:
            sock.sendall(header + payload)
            print(f"[SAFE_PI_FRAME] sent seq={seq} {args.width}x{args.height}", flush=True)
        except Exception as e:
            print(f"[SAFE_PI_FRAME] send failed: {e}", flush=True)
            try:
                sock.close()
            except Exception:
                pass
            sock = connect_loop()

        seq += 1
        time.sleep(interval)

# ============================================================
# Role 4: TOPST Supervisor
# - Receives PERCEPTION / CONTROLLER at UDP 5005
# - Maintains target_list, visited, current_target, GO_FINAL
# - Sends CMD lease to PC Gateway UDP 5006
# ============================================================

class SafeTopstState:
    def __init__(self, targets):
        self.latest_p = {
            "seq": 0,
            "aruco": 0,
            "id": -1,
            "cx": 160,
            "area": 0,
            "obstacle": 0,
            "obs_left": 0,
            "obs_center": 0,
            "obs_right": 0,
            "worker": 0,
            "fault": 0,
        }
        self.latest_c = {
            "seq": 0,
            "start": 1,
            "estop": 0,
            "deadman": 1,
            "manual": 0,
            "joy_x": 512,
            "joy_y": 512,
            "target_mask": 0,
        }

        self.last_p_time = 0
        self.last_c_time = 0

        self.started = False
        self.target_locked = False
        self.target_list = list(targets)
        self.default_targets = list(targets)
        self.visited = set()
        self.current_target = self.target_list[0] if self.target_list else 4
        self.final_target = 4
        self.going_final = False
        self.finished = False
        self.reached_count = 0

    def targets_from_mask(self, mask):
        out = []
        for i in range(4):
            if mask & (1 << i):
                out.append(i)
        return out

    def start_and_lock(self):
        if self.target_locked:
            return

        mask = safe_int(self.latest_c.get("target_mask", 0), 0)
        selected = self.targets_from_mask(mask)

        if selected:
            self.target_list = selected
        else:
            self.target_list = list(self.default_targets)

        if not self.target_list:
            self.target_list = [0]

        self.current_target = self.target_list[0]
        self.target_locked = True
        self.started = True

        print(f"[SAFE_TOPST LOCK] target_list={self.target_list}, current_target={self.current_target}", flush=True)

    def select_next_target(self):
        remaining = [t for t in self.target_list if t not in self.visited]

        if remaining:
            self.current_target = remaining[0]
            self.going_final = False
            print(f"[SAFE_TOPST NEXT] current_target={self.current_target}, visited={sorted(self.visited)}", flush=True)
        else:
            self.current_target = self.final_target
            self.going_final = True
            print(f"[SAFE_TOPST FINAL] current_target=4", flush=True)

def make_cmd(seq, ttl, mode, target, drive, speed, steer="CENTER", servo=90, buzzer="OFF", fault="NONE"):
    return (
        f"CMD seq={seq} ttl={ttl} mode={mode} target={target} "
        f"drive={drive} speed={speed} steer={steer} servo={servo} buzzer={buzzer} fault={fault}"
    )

def decide_follow(cx, center, deadband, speed):
    err = cx - center

    if err < -deadband:
        return "TURN_LEFT", speed, "LEFT"

    if err > deadband:
        return "TURN_RIGHT", speed, "RIGHT"

    return "FORWARD", speed, "CENTER"

def decide_avoid(p, speed):
    obs_left = safe_int(p.get("obs_left", 0), 0)
    obs_center = safe_int(p.get("obs_center", 0), 0)
    obs_right = safe_int(p.get("obs_right", 0), 0)

    if obs_left:
        return "AUTO_AVOID", "TURN_RIGHT", speed, "RIGHT", "WARN", "OBSTACLE_LEFT"
    if obs_right:
        return "AUTO_AVOID", "TURN_LEFT", speed, "LEFT", "WARN", "OBSTACLE_RIGHT"
    if obs_center:
        return "AUTO_AVOID", "TURN_LEFT", speed, "LEFT", "WARN", "OBSTACLE_CENTER"

    return "AUTO_AVOID", "TURN_LEFT", speed, "LEFT", "WARN", "OBSTACLE"

def role_topst(args):
    state = SafeTopstState(parse_targets(args.targets))

    rx = bind_udp(args.listen_port, blocking=False)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print("[SAFE_TOPST] start")
    print(f"[SAFE_TOPST] listen UDP :{args.listen_port}")
    print(f"[SAFE_TOPST] CMD -> PC {args.pc_ip}:{args.pc_port}")
    print(f"[SAFE_TOPST] default_targets={state.default_targets}, final=4")
    print(f"[SAFE_TOPST] auto_start={args.auto_start}, ignore_obstacle={args.ignore_obstacle}")

    if args.auto_start:
        state.started = True
        state.target_locked = True
        if state.target_list:
            state.current_target = state.target_list[0]
        print(f"[SAFE_TOPST AUTO_START] target_list={state.target_list}, current_target={state.current_target}", flush=True)

    seq = 1
    interval = 1.0 / args.send_hz
    last_send = 0

    while True:
        now = now_ms()

        while True:
            try:
                data, addr = rx.recvfrom(4096)
            except BlockingIOError:
                break

            line = data.decode(errors="ignore").strip()
            msg_type, kv = parse_kv(line)

            if msg_type == "PERCEPTION":
                for k in state.latest_p:
                    if k in kv:
                        state.latest_p[k] = safe_int(kv[k], state.latest_p[k])
                state.last_p_time = now
                print(f"[SAFE_TOPST P_RX] {state.latest_p}", flush=True)

            elif msg_type == "CONTROLLER":
                for k in state.latest_c:
                    if k in kv:
                        state.latest_c[k] = safe_int(kv[k], state.latest_c[k])
                state.last_c_time = now
                print(f"[SAFE_TOPST C_RX] {state.latest_c}", flush=True)

                if safe_int(state.latest_c.get("start", 0), 0) == 1:
                    state.start_and_lock()

            else:
                if line:
                    print(f"[SAFE_TOPST IGNORE] {addr} {line}", flush=True)

        if now - last_send < interval * 1000:
            time.sleep(0.01)
            continue

        last_send = now

        p = state.latest_p
        c = state.latest_c

        p_age = now - state.last_p_time if state.last_p_time else 999999

        aruco = safe_int(p.get("aruco", 0), 0)
        marker_id = safe_int(p.get("id", -1), -1)
        cx = safe_int(p.get("cx", args.center), args.center)
        area = safe_int(p.get("area", 0), 0)

        estop = safe_int(c.get("estop", 0), 0)
        deadman = safe_int(c.get("deadman", 1), 1)
        manual = safe_int(c.get("manual", 0), 0)

        obstacle = safe_int(p.get("obstacle", 0), 0)
        worker = safe_int(p.get("worker", 0), 0)
        fault = safe_int(p.get("fault", 0), 0)

        mode = "SEARCH_TARGET"
        drive = "STOP"
        speed = 0
        steer = "CENTER"
        buzzer = "OFF"
        fault_text = "NONE"
        servo = 90

        if not state.started:
            mode = "IDLE"
            fault_text = "WAIT_START"

        elif estop == 1:
            mode = "SAFETY_STOP"
            fault_text = "ESTOP"

        elif deadman == 0:
            mode = "SAFETY_STOP"
            fault_text = "DEADMAN"

        elif worker == 1:
            mode = "SAFETY_STOP"
            fault_text = "WORKER"

        elif fault == 1:
            mode = "SAFETY_STOP"
            fault_text = "AIG_FAULT"

        elif p_age > args.perception_timeout_ms:
            mode = "SEARCH_TARGET"
            fault_text = "PERCEPTION_TIMEOUT"

        elif state.finished:
            mode = "FINISH"
            fault_text = "NONE"
            buzzer = "FINISH"

        elif (not args.ignore_obstacle) and obstacle == 1:
            mode, drive, speed, steer, buzzer, fault_text = decide_avoid(p, args.speed)

        elif aruco == 1 and marker_id == state.current_target:
            if area >= args.reach_area:
                state.reached_count += 1
            else:
                state.reached_count = 0

            if state.reached_count >= args.reach_count:
                if state.current_target == state.final_target:
                    state.finished = True
                    mode = "FINISH"
                    drive = "STOP"
                    speed = 0
                    buzzer = "FINISH"
                    fault_text = "NONE"
                else:
                    state.visited.add(state.current_target)
                    state.select_next_target()
                    mode = "TARGET_REACHED"
                    drive = "STOP"
                    speed = 0
                    buzzer = "GOAL"
                    fault_text = "NONE"
            else:
                mode = "GO_TO_TARGET"
                drive, speed, steer = decide_follow(cx, args.center, args.deadband, args.speed)
                fault_text = "NONE"

        elif state.going_final and aruco == 1 and marker_id >= 0:
            # Final stage fallback: if ID4 not seen, move toward the highest visible marker.
            mode = "SEARCH_FINAL"
            drive, speed, steer = decide_follow(cx, args.center, args.deadband, max(25, args.speed - 10))
            fault_text = "FINAL_MARKER_FALLBACK"

        else:
            mode = "SEARCH_TARGET"
            drive = "STOP"
            speed = 0
            fault_text = "TARGET_NOT_FOUND"

        cmd = make_cmd(
            seq=seq,
            ttl=args.ttl_ms,
            mode=mode,
            target=state.current_target,
            drive=drive,
            speed=speed,
            steer=steer,
            servo=servo,
            buzzer=buzzer,
            fault=fault_text,
        )

        try:
            udp_send(tx, args.pc_ip, args.pc_port, cmd)
            print(
                f"[SAFE_TOPST CMD_TX] {cmd} | "
                f"p_age={p_age} aruco={aruco} id={marker_id} cx={cx} area={area} obs={obstacle}",
                flush=True,
            )
        except Exception as e:
            print(f"[SAFE_TOPST CMD_ERR] {e} | {cmd}", flush=True)

        seq += 1

def parse_targets(s):
    out = []
    for x in str(s).split(","):
        x = x.strip()
        if not x:
            continue
        try:
            v = int(x)
            if v not in out:
                out.append(v)
        except Exception:
            pass
    return out

# ============================================================
# Role 5: Pi UDP vehicle
# - Receives CMD UDP 5006
# - Executes motor
# ============================================================


class MotorDriver:
    def __init__(self, dry_run=False, pwm_freq=1000):
        self.dry_run = dry_run
        self.pwm_freq = pwm_freq
        self.gpio = None

        # motor_test.py 기준 BCM 핀맵
        self.LF_IN1, self.LF_IN2, self.LF_PWM = 5, 6, 12
        self.LR_IN1, self.LR_IN2, self.LR_PWM = 16, 20, 13
        self.RF_IN1, self.RF_IN2, self.RF_PWM = 23, 24, 18
        self.RR_IN1, self.RR_IN2, self.RR_PWM = 21, 26, 19

        self.pwm_lf = None
        self.pwm_lr = None
        self.pwm_rf = None
        self.pwm_rr = None

        if self.dry_run:
            print("[SAFE_PI_VEHICLE] dry-run motor initialized", flush=True)
            return

        try:
            import RPi.GPIO as GPIO
            self.gpio = GPIO

            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BCM)

            pins = [
                self.LF_IN1, self.LF_IN2, self.LF_PWM,
                self.LR_IN1, self.LR_IN2, self.LR_PWM,
                self.RF_IN1, self.RF_IN2, self.RF_PWM,
                self.RR_IN1, self.RR_IN2, self.RR_PWM,
            ]

            for pin in pins:
                GPIO.setup(pin, GPIO.OUT)
                GPIO.output(pin, GPIO.LOW)

            self.pwm_lf = GPIO.PWM(self.LF_PWM, pwm_freq)
            self.pwm_lr = GPIO.PWM(self.LR_PWM, pwm_freq)
            self.pwm_rf = GPIO.PWM(self.RF_PWM, pwm_freq)
            self.pwm_rr = GPIO.PWM(self.RR_PWM, pwm_freq)

            self.pwm_lf.start(0)
            self.pwm_lr.start(0)
            self.pwm_rf.start(0)
            self.pwm_rr.start(0)

            self.stop()
            print("[SAFE_PI_VEHICLE] motor_test style GPIO initialized", flush=True)

        except Exception as e:
            print(f"[SAFE_PI_VEHICLE] GPIO init failed: {e}", flush=True)
            self.dry_run = True

    def _motor(self, in1, in2, pwm, direction, speed):
        speed = max(0, min(100, int(speed)))

        if self.dry_run:
            return

        GPIO = self.gpio

        if direction == "F":
            GPIO.output(in1, GPIO.HIGH)
            GPIO.output(in2, GPIO.LOW)
            pwm.ChangeDutyCycle(speed)
        elif direction == "B":
            GPIO.output(in1, GPIO.LOW)
            GPIO.output(in2, GPIO.HIGH)
            pwm.ChangeDutyCycle(speed)
        else:
            GPIO.output(in1, GPIO.LOW)
            GPIO.output(in2, GPIO.LOW)
            pwm.ChangeDutyCycle(0)

    def stop(self):
        self._motor(self.LF_IN1, self.LF_IN2, self.pwm_lf, "S", 0)
        self._motor(self.LR_IN1, self.LR_IN2, self.pwm_lr, "S", 0)
        self._motor(self.RF_IN1, self.RF_IN2, self.pwm_rf, "S", 0)
        self._motor(self.RR_IN1, self.RR_IN2, self.pwm_rr, "S", 0)

    def forward(self, speed):
        # Direction fix:
        # Raw F made the vehicle move backward, so logical FORWARD uses raw B.
        self._motor(self.LF_IN1, self.LF_IN2, self.pwm_lf, "B", speed)
        self._motor(self.LR_IN1, self.LR_IN2, self.pwm_lr, "B", speed)
        self._motor(self.RF_IN1, self.RF_IN2, self.pwm_rf, "B", speed)
        self._motor(self.RR_IN1, self.RR_IN2, self.pwm_rr, "B", speed)

    def backward(self, speed):
        # Logical BACKWARD uses raw F.
        self._motor(self.LF_IN1, self.LF_IN2, self.pwm_lf, "F", speed)
        self._motor(self.LR_IN1, self.LR_IN2, self.pwm_lr, "F", speed)
        self._motor(self.RF_IN1, self.RF_IN2, self.pwm_rf, "F", speed)
        self._motor(self.RR_IN1, self.RR_IN2, self.pwm_rr, "F", speed)

    def turn_left(self, speed):
        # TURN FIX:
        # Previous TURN_LEFT made the vehicle turn right.
        # Use the previous right-turn motor pattern for logical left turn.
        self._motor(self.LF_IN1, self.LF_IN2, self.pwm_lf, "B", speed)
        self._motor(self.LR_IN1, self.LR_IN2, self.pwm_lr, "B", speed)
        self._motor(self.RF_IN1, self.RF_IN2, self.pwm_rf, "F", speed)
        self._motor(self.RR_IN1, self.RR_IN2, self.pwm_rr, "F", speed)

    def turn_right(self, speed):
        # TURN FIX:
        # Previous TURN_RIGHT made the vehicle turn left.
        # Use the previous left-turn motor pattern for logical right turn.
        self._motor(self.LF_IN1, self.LF_IN2, self.pwm_lf, "F", speed)
        self._motor(self.LR_IN1, self.LR_IN2, self.pwm_lr, "F", speed)
        self._motor(self.RF_IN1, self.RF_IN2, self.pwm_rf, "B", speed)
        self._motor(self.RR_IN1, self.RR_IN2, self.pwm_rr, "B", speed)

    def cleanup(self):
        self.stop()
        print("[SAFE_PI_VEHICLE] cleanup: STOP, pins kept LOW", flush=True)

def role_pi_vehicle(args):
    rx = bind_udp(args.listen_port)
    rx.settimeout(0.1)

    motor = MotorDriver(dry_run=args.dry_run)
    last_cmd_time = now_ms()
    last_seq = -1

    print("[SAFE_PI_VEHICLE] start")
    print(f"[SAFE_PI_VEHICLE] listen UDP :{args.listen_port}")
    print(f"[SAFE_PI_VEHICLE] dry_run={args.dry_run}")

    try:
        while True:
            try:
                data, addr = rx.recvfrom(4096)
                text = data.decode(errors="ignore").strip()
            except socket.timeout:
                if now_ms() - last_cmd_time > args.cmd_timeout_ms:
                    motor.stop()
                continue

            msg_type, kv = parse_kv(text)
            if msg_type != "CMD":
                print(f"[SAFE_PI_VEHICLE IGNORE] {addr} {text}", flush=True)
                continue

            seq = safe_int(kv.get("seq", 0), 0)
            ttl = safe_int(kv.get("ttl", 0), 0)
            mode = kv.get("mode", "UNKNOWN")
            drive = kv.get("drive", "STOP")
            speed = safe_int(kv.get("speed", 0), 0)
            fault = kv.get("fault", "NONE")
            if ttl <= 0:
                print(f"[SAFE_PI_VEHICLE BAD_TTL] {text}", flush=True)
                motor.stop()
                continue

            print(f"[SAFE_PI_VEHICLE CMD_RX] {text}", flush=True)

            if drive == "FORWARD":
                motor.forward(speed)
                print(f"[SAFE_PI_VEHICLE MOTOR] FORWARD speed={speed}", flush=True)
            elif drive == "BACKWARD":
                motor.backward(speed)
                print(f"[SAFE_PI_VEHICLE MOTOR] BACKWARD speed={speed}", flush=True)
            elif drive in ("TURN_LEFT", "AVOID_LEFT"):
                motor.turn_left(speed)
                print(f"[SAFE_PI_VEHICLE MOTOR] TURN_LEFT speed={speed}", flush=True)
            elif drive in ("TURN_RIGHT", "AVOID_RIGHT"):
                motor.turn_right(speed)
                print(f"[SAFE_PI_VEHICLE MOTOR] TURN_RIGHT speed={speed}", flush=True)
            else:
                motor.stop()
                print(f"[SAFE_PI_VEHICLE MOTOR] STOP mode={mode} fault={fault}", flush=True)

    except KeyboardInterrupt:
        print("[SAFE_PI_VEHICLE] KeyboardInterrupt -> STOP", flush=True)
    finally:
        motor.cleanup()

# ============================================================
# CLI
# ============================================================

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--role", required=True, choices=[
        "pc_gateway",
        "pi_perception",
        "pi_frame",
        "topst",
        "pi_vehicle",
    ])

    # PC Gateway
    parser.add_argument("--perception-port", type=int, default=6002)
    parser.add_argument("--cmd-port", type=int, default=5006)
    parser.add_argument("--topst-ip", default="192.168.50.20")
    parser.add_argument("--topst-port", type=int, default=5005)
    parser.add_argument("--pi-ip", default="192.168.0.12")
    parser.add_argument("--pi-cmd-port", type=int, default=5006)
    parser.add_argument("--force-obstacle0", action="store_true")
    parser.add_argument("--force-id", type=int, default=None)
    parser.add_argument("--force-area", type=int, default=None)
    parser.add_argument("--controller-keepalive", action="store_true")
    parser.add_argument("--controller-period", type=float, default=0.2)
    parser.add_argument("--target-mask", type=int, default=4)

    # Pi perception relay
    parser.add_argument("--listen-port", type=int, default=5005)
    parser.add_argument("--pc-ip", default="192.168.50.10")
    parser.add_argument("--pc-port", type=int, default=5006)

    # Pi frame
    parser.add_argument("--aig-ip", default="192.168.60.2")
    parser.add_argument("--aig-port", type=int, default=7000)
    parser.add_argument("--dev", default="/dev/video1")
    parser.add_argument("--width", type=int, default=320)
    parser.add_argument("--height", type=int, default=240)
    parser.add_argument("--fps", type=float, default=10.0)
    parser.add_argument("--cam-width", type=int, default=640)
    parser.add_argument("--cam-height", type=int, default=480)
    parser.add_argument("--cam-fps", type=int, default=15)

    # TOPST
    parser.add_argument("--targets", default="2")
    parser.add_argument("--auto-start", action="store_true")
    parser.add_argument("--send-hz", type=float, default=5.0)
    parser.add_argument("--ttl-ms", type=int, default=2000)
    parser.add_argument("--center", type=int, default=160)
    parser.add_argument("--deadband", type=int, default=35)
    parser.add_argument("--speed", type=int, default=35)
    parser.add_argument("--reach-area", type=int, default=3500)
    parser.add_argument("--reach-count", type=int, default=3)
    parser.add_argument("--perception-timeout-ms", type=int, default=3000)
    parser.add_argument("--ignore-obstacle", action="store_true")

    # Pi vehicle
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--cmd-timeout-ms", type=int, default=1000)

    args = parser.parse_args()

    if args.role == "pc_gateway":
        role_pc_gateway(args)
    elif args.role == "pi_perception":
        role_pi_perception(args)
    elif args.role == "pi_frame":
        role_pi_frame(args)
    elif args.role == "topst":
        role_topst(args)
    elif args.role == "pi_vehicle":
        role_pi_vehicle(args)

if __name__ == "__main__":
    main()
