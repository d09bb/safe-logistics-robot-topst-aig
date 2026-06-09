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
            "start": 0,
            "estop": 0,
            "deadman": 1,
            "manual": 0,
            "joy_x": 512,
            "joy_y": 512,
            "target_mask": 0,
        }

        self.last_p_time = 0
        self.last_c_time = 0

        # HARD-CODED ROUTE:
        # Physical ArUco IDs are 0 -> 1 -> 2.
        # target_mask means "hold 5 seconds at that marker", not "visit only selected markers".
        self.route = [0, 1, 2]
        self.hold_targets = set()

        self.started = False
        self.target_locked = False
        self.current_target = 0
        self.visited = set()
        self.finished = False
        self.reached_count = 0

        # Last valid current-target memory for flicker tolerance.
        self.last_target_seen_ms = 0
        self.last_target_cx = 160
        self.last_target_area = 0

        # Obstacle debounce.
        self.obstacle_seen_count = 0
        self.obstacle_clear_count = 0
        self.obstacle_latched = False

        # Phase state.
        # NAV: normal ArUco tracking/search
        # HOLD: 5 sec stop at selected marker
        # TURN_RIGHT / WAIT / BLIND_FORWARD: hard-coded transition between markers
        # FINISH: mission done
        self.phase = "IDLE"
        self.phase_until_ms = 0
        self.transition_from = -1
        self.transition_to = -1

        # Start -> 0 blind forward.
        self.start0_forward_started_ms = 0
        self.start0_target_seen = False

        # Ignore ultrasonic obstacle only during marker-board transition.
        self.obstacle_ignore_until_ms = 0
        self.obstacle_ignore_reason = "NONE"

    def targets_from_mask(self, mask):
        out = []
        for i in self.route:
            if mask & (1 << i):
                out.append(i)
        return out

    def start_and_lock(self):
        if self.target_locked:
            return

        mask = safe_int(self.latest_c.get("target_mask", 0), 0)

        self.hold_targets = set(self.targets_from_mask(mask))
        self.started = True
        self.target_locked = True
        self.current_target = 0
        self.visited = set()
        self.finished = False
        self.reached_count = 0
        self.phase = "NAV"
        self.phase_until_ms = 0
        self.transition_from = -1
        self.transition_to = -1
        self.start0_forward_started_ms = 0
        self.start0_target_seen = False
        self.obstacle_ignore_until_ms = 0
        self.obstacle_ignore_reason = "NONE"

        print(
            f"[SAFE_TOPST LOCK] route={self.route}, hold_targets={sorted(self.hold_targets)}, "
            f"current_target={self.current_target}",
            flush=True,
        )

    def next_target_after(self, reached):
        try:
            idx = self.route.index(reached)
        except ValueError:
            return None
        if idx + 1 >= len(self.route):
            return None
        return self.route[idx + 1]


def make_cmd(seq, ttl, mode, target, drive, speed, steer="CENTER", servo=90, buzzer="OFF", fault="NONE"):
    return (
        f"CMD seq={seq} ttl={ttl} mode={mode} target={target} "
        f"drive={drive} speed={speed} steer={steer} servo={servo} buzzer={buzzer} fault={fault}"
    )


def decide_follow(cx, center, deadband, speed, area=0):
    """
    ArUco follow with very gentle left/right correction.

    Policy:
    - Keep ArUco near the camera center.
    - If ArUco is left/right, issue TURN_LEFT / TURN_RIGHT.
    - But turn speed is intentionally very low.
    - Hard-coded transition turns are not handled here.
    """
    cx = safe_int(cx, center)
    center = safe_int(center, 320)
    area = safe_int(area, 0)
    speed = safe_int(speed, 35)

    # Frame is 640 wide. If old center=160 is passed, force 320.
    if center < 250:
        center = 320

    err = cx - center

    # CENTER zone: not too wide, so the robot still tries to face ArUco.
    # center=320, band=100:
    #   cx < 220  -> gentle TURN_LEFT
    #   220~420   -> FORWARD
    #   cx > 420  -> gentle TURN_RIGHT
    band = max(safe_int(deadband, 0), 100)

    # Tiny/noisy marker: do not steer from it.
    if area > 0 and area < 150:
        return "FORWARD", speed, "CENTER"

    # Very gentle turning command.
    # Pi side will also limit GO_TO_TARGET turn PWM.
    if err < -band:
        return "TURN_LEFT", 15, "LEFT"

    if err > band:
        return "TURN_RIGHT", 15, "RIGHT"

    return "FORWARD", speed, "CENTER"


def decide_manual(joy_x, joy_y, speed):
    """
    Manual joystick command.

    Current calibrated state:
    - forward/backward axis is correct
    - left/right direction is reversed

    Therefore:
    - raw joy_x is used for forward/backward
    - raw joy_y is used for left/right
    - left/right return values are swapped
    """
    raw_x = safe_int(joy_x, 512)
    raw_y = safe_int(joy_y, 512)

    low = 350
    high = 700

    # 현재 컨트롤러는 X/Y 축이 교환되어 들어온다.
    fb = raw_x
    lr = raw_y

    # 앞/뒤는 현재 맞는 상태
    if fb < low:
        return "FORWARD", speed, "CENTER"
    if fb > high:
        return "BACKWARD", speed, "CENTER"

    # 좌/우만 서로 바뀐 상태이므로 여기만 반대로 둔다.
    if lr < low:
        return "TURN_RIGHT", speed, "RIGHT"
    if lr > high:
        return "TURN_LEFT", speed, "LEFT"

    return "STOP", 0, "CENTER"


def start_transition(state, from_target, now):
    """
    Start hard-coded transition:
    0 -> 1: right turn 4s, wait 0.5s, forward 1s
    1 -> 2: right turn 4s, wait 0.5s, forward 2s
    """
    to_target = state.next_target_after(from_target)

    if to_target is None:
        state.finished = True
        state.phase = "FINISH"
        state.current_target = from_target
        print(f"[MISSION_FINISH] reached={from_target}", flush=True)
        return

    turn_ms = int(os.environ.get("TOPST_TRANSITION_TURN_MS", "4000"))
    ignore_ms = int(os.environ.get("TOPST_TRANSITION_OBSTACLE_IGNORE_MS", "7000"))

    state.current_target = to_target
    state.transition_from = from_target
    state.transition_to = to_target
    state.phase = "TURN_RIGHT"
    state.phase_until_ms = now + turn_ms
    state.reached_count = 0
    state.last_target_seen_ms = 0
    state.last_target_cx = 160
    state.last_target_area = 0

    state.obstacle_latched = False
    state.obstacle_seen_count = 0
    state.obstacle_clear_count = 0
    state.obstacle_ignore_until_ms = now + ignore_ms
    state.obstacle_ignore_reason = f"TRANSITION_{from_target}_TO_{to_target}"

    print(
        f"[TRANSITION_START] {from_target}->{to_target} "
        f"TURN_RIGHT={turn_ms}ms obstacle_ignore={ignore_ms}ms",
        flush=True,
    )


def begin_reached_target(state, reached, now):
    state.visited.add(reached)
    state.reached_count = 0

    print(
        f"[TARGET_REACHED] reached={reached} visited={sorted(state.visited)} "
        f"hold_targets={sorted(state.hold_targets)}",
        flush=True,
    )

    # Final target 2.
    if reached == 2:
        state.finished = True
        state.phase = "FINISH"
        state.current_target = 2
        return

    # If user selected this marker, hold for 5 seconds first.
    if reached in state.hold_targets:
        hold_ms = int(os.environ.get("TOPST_HOLD_MS", "5000"))
        state.phase = "HOLD"
        state.phase_until_ms = now + hold_ms
        state.current_target = reached
        state.transition_from = reached
        print(f"[ARRIVAL_HOLD_START] target={reached} ms={hold_ms}", flush=True)
        return

    # Otherwise immediately start hard-coded turn to next marker.
    start_transition(state, reached, now)


def parse_targets(s):
    # Kept for CLI compatibility. Actual route is hard-coded to 0,1,2.
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


def role_topst(args):
    state = SafeTopstState(parse_targets(args.targets))

    rx = bind_udp(args.listen_port, blocking=False)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print("[SAFE_TOPST] start")
    print(f"[SAFE_TOPST] listen UDP :{args.listen_port}")
    print(f"[SAFE_TOPST] CMD -> PC {args.pc_ip}:{args.pc_port}")
    print("[SAFE_TOPST] HARD_CODED_ROUTE=[0,1,2]")
    print("[SAFE_TOPST] target_mask means hold target, not route selection")
    print(f"[SAFE_TOPST] reach_area={args.reach_area}, reach_count={args.reach_count}")
    print(f"[SAFE_TOPST] auto_start={args.auto_start}, ignore_obstacle={args.ignore_obstacle}")

    if args.auto_start:
        state.latest_c["start"] = 1
        state.start_and_lock()

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
        joy_x = safe_int(c.get("joy_x", 512), 512)
        joy_y = safe_int(c.get("joy_y", 512), 512)

        raw_obstacle = safe_int(p.get("obstacle", 0), 0)
        worker = safe_int(p.get("worker", 0), 0)
        fault = safe_int(p.get("fault", 0), 0)

        # During transition, ignore ultrasonic obstacle only.
        obstacle_ignore = (manual != 1 and now < state.obstacle_ignore_until_ms)

        if obstacle_ignore:
            obstacle = 0
            if raw_obstacle == 1:
                print(
                    f"[OBSTACLE_IGNORE] reason={state.obstacle_ignore_reason} "
                    f"remain={state.obstacle_ignore_until_ms - now} raw_obstacle=1",
                    flush=True,
                )
        else:
            obstacle = raw_obstacle
            if state.obstacle_ignore_until_ms > 0:
                print(f"[OBSTACLE_IGNORE_DONE] reason={state.obstacle_ignore_reason}", flush=True)
                state.obstacle_ignore_until_ms = 0
                state.obstacle_ignore_reason = "NONE"

        # Obstacle debounce.
        if obstacle == 1:
            state.obstacle_seen_count += 1
            state.obstacle_clear_count = 0
        else:
            state.obstacle_clear_count += 1
            if state.obstacle_clear_count >= args.obstacle_clear_count:
                state.obstacle_seen_count = 0
                state.obstacle_latched = False

        if state.obstacle_seen_count >= args.obstacle_confirm_count:
            state.obstacle_latched = True

        obstacle_active = state.obstacle_latched

        mode = "SEARCH_TARGET"
        drive = "STOP"
        speed = 0
        steer = "CENTER"
        buzzer = "OFF"
        fault_text = "NONE"
        servo = 90
        target = state.current_target

        # Absolute safety first.
        if not state.started:
            mode = "IDLE"
            fault_text = "WAIT_START"

        elif estop == 1:
            mode = "SAFETY_STOP"
            drive = "STOP"
            fault_text = "ESTOP"

        elif deadman == 0:
            mode = "SAFETY_STOP"
            drive = "STOP"
            fault_text = "DEADMAN"

        elif worker == 1:
            mode = "SAFETY_STOP"
            drive = "STOP"
            fault_text = "WORKER"

        elif fault == 1:
            mode = "SAFETY_STOP"
            drive = "STOP"
            fault_text = "AIG_FAULT"

        elif state.finished or state.phase == "FINISH":
            mode = "FINISH"
            drive = "STOP"
            speed = 0
            buzzer = "FINISH"
            fault_text = "NONE"

        # Manual mode has priority over ultrasonic obstacle.
        # E-STOP / deadman / worker / fault are still handled above this branch.
        # This allows the operator to manually move the vehicle away from an obstacle.
        elif manual == 1:
            mode = "MANUAL"
            drive, speed, steer = decide_manual(
                joy_x,
                joy_y,
                int(os.environ.get("TOPST_MANUAL_SPEED", "45"))
            )
            fault_text = "MANUAL"

        # Arrival gate before obstacle hold.
        # If the current target ArUco is close enough, treat it as TARGET_REACHED
        # even when ultrasonic/ToF sees the marker board as an obstacle.
        elif aruco == 1 and marker_id == state.current_target and area >= args.reach_area:
            if state.current_target == 0:
                state.start0_target_seen = True

            state.last_target_seen_ms = now
            state.last_target_cx = cx
            state.last_target_area = area
            state.reached_count += 1

            mode = "ARRIVAL_CONFIRM"
            target = state.current_target
            drive = "STOP"
            speed = 0
            steer = "CENTER"
            buzzer = "GOAL"
            fault_text = f"ARRIVAL_CONFIRM_{state.reached_count}_{args.reach_count}"

            if state.reached_count >= args.reach_count:
                mode = "TARGET_REACHED"
                fault_text = "TARGET_REACHED"
                begin_reached_target(state, state.current_target, now)

        # Obstacle stops only automatic driving.
        elif (not args.ignore_obstacle) and obstacle_active:
            mode = "OBSTACLE_HOLD"
            drive = "STOP"
            speed = 0
            steer = "CENTER"
            buzzer = "WARN"
            fault_text = "OBSTACLE"

        # Hold at selected target.
        elif state.phase == "HOLD":
            if now < state.phase_until_ms:
                mode = "ARRIVAL_HOLD"
                target = state.current_target
                drive = "STOP"
                speed = 0
                buzzer = "GOAL"
                fault_text = "ARRIVAL_HOLD"
            else:
                reached = state.transition_from
                print(f"[ARRIVAL_HOLD_DONE] target={reached}", flush=True)
                start_transition(state, reached, now)
                mode = "TRANSITION_TURN"
                target = state.current_target
                drive = "TURN_RIGHT"
                speed = int(os.environ.get("TOPST_TRANSITION_TURN_SPEED", "70"))
                steer = "RIGHT"
                fault_text = f"TURN_{reached}_TO_{state.current_target}"

        # Hard-coded transition.
        elif state.phase in ("TURN_RIGHT", "WAIT", "BLIND_FORWARD"):
            turn_speed = int(os.environ.get("TOPST_TRANSITION_TURN_SPEED", "70"))
            fwd_speed = int(os.environ.get("TOPST_TRANSITION_FORWARD_SPEED", "35"))
            wait_ms = int(os.environ.get("TOPST_TRANSITION_WAIT_MS", "500"))

            if now >= state.phase_until_ms:
                if state.phase == "TURN_RIGHT":
                    state.phase = "WAIT"
                    state.phase_until_ms = now + wait_ms
                    print(f"[TRANSITION_STAGE] WAIT ms={wait_ms}", flush=True)

                elif state.phase == "WAIT":
                    state.phase = "BLIND_FORWARD"
                    if state.transition_from == 0 and state.transition_to == 1:
                        fwd_ms = int(os.environ.get("TOPST_TRANSITION_0_1_FORWARD_MS", "1000"))
                    elif state.transition_from == 1 and state.transition_to == 2:
                        fwd_ms = int(os.environ.get("TOPST_TRANSITION_1_2_FORWARD_MS", "2000"))
                    else:
                        fwd_ms = 1000
                    state.phase_until_ms = now + fwd_ms
                    print(f"[TRANSITION_STAGE] FORWARD ms={fwd_ms}", flush=True)

                elif state.phase == "BLIND_FORWARD":
                    state.phase = "NAV"
                    state.phase_until_ms = 0
                    state.reached_count = 0
                    state.last_target_seen_ms = 0
                    print(f"[TRANSITION_DONE] target={state.current_target}", flush=True)

            if state.phase == "TURN_RIGHT":
                mode = "TRANSITION_TURN_RIGHT"
                target = state.current_target
                drive = "TURN_RIGHT"
                speed = turn_speed
                steer = "RIGHT"
                fault_text = f"TURN_{state.transition_from}_TO_{state.transition_to}"

            elif state.phase == "WAIT":
                mode = "TRANSITION_WAIT"
                target = state.current_target
                drive = "STOP"
                speed = 0
                steer = "CENTER"
                fault_text = "TRANSITION_WAIT"

            elif state.phase == "BLIND_FORWARD":
                mode = "TRANSITION_FORWARD"
                target = state.current_target
                drive = "FORWARD"
                speed = fwd_speed
                steer = "CENTER"
                fault_text = f"FORWARD_TO_{state.current_target}"

            else:
                mode = "SEARCH_TARGET"
                target = state.current_target
                drive = "STOP"
                speed = 0
                fault_text = "TRANSITION_DONE"

        # Current target seen.
        elif aruco == 1 and marker_id == state.current_target:
            if state.current_target == 0:
                state.start0_target_seen = True

            state.last_target_seen_ms = now
            state.last_target_cx = cx
            state.last_target_area = area

            if area >= args.reach_area:
                state.reached_count += 1
            else:
                state.reached_count = 0

            if state.reached_count >= args.reach_count:
                mode = "TARGET_REACHED"
                target = state.current_target
                drive = "STOP"
                speed = 0
                buzzer = "GOAL"
                fault_text = "TARGET_REACHED"
                begin_reached_target(state, state.current_target, now)
            else:
                mode = "GO_TO_TARGET"
                target = state.current_target
                drive, speed, steer = decide_follow(cx, args.center, args.deadband, args.speed, area)
                fault_text = "NONE"

        # Flicker tolerance.
        elif state.last_target_seen_ms > 0 and (now - state.last_target_seen_ms) <= args.target_lost_hold_ms:
            mode = "GO_TO_TARGET_HOLD"
            target = state.current_target
            drive, speed, steer = decide_follow(state.last_target_cx, args.center, args.deadband, args.speed, state.last_target_area)
            fault_text = "TARGET_HOLD"

        # Start -> 0 blind forward.
        elif state.current_target == 0 and not state.start0_target_seen:
            start0_ms = int(os.environ.get("TOPST_START0_FORWARD_MS", "3000"))
            start0_speed = int(os.environ.get("TOPST_START0_FORWARD_SPEED", "35"))

            if state.start0_forward_started_ms <= 0:
                state.start0_forward_started_ms = now
                print(f"[START0_FORWARD_START] ms={start0_ms} speed={start0_speed}", flush=True)

            elapsed = now - state.start0_forward_started_ms

            if elapsed <= start0_ms:
                mode = "START0_FORWARD"
                target = 0
                drive = "FORWARD"
                speed = start0_speed
                steer = "CENTER"
                fault_text = "START0_FORWARD"
            else:
                mode = "START0_SEARCH"
                target = 0
                drive = "STOP"
                speed = 0
                steer = "CENTER"
                buzzer = "WARN"
                fault_text = "TARGET0_NOT_FOUND_AFTER_BLIND"

        else:
            mode = "SEARCH_TARGET"
            target = state.current_target
            drive = "STOP"
            speed = 0
            fault_text = "TARGET_NOT_FOUND"

        cmd = make_cmd(
            seq=seq,
            ttl=args.ttl_ms,
            mode=mode,
            target=target,
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
                f"phase={state.phase} p_age={p_age} aruco={aruco} id={marker_id} "
                f"cx={cx} area={area} raw_obs={raw_obstacle} obs_active={obstacle_active}",
                flush=True,
            )
        except Exception as e:
            print(f"[SAFE_TOPST CMD_ERR] {e} | {cmd}", flush=True)

        seq += 1


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
        self.pwms = {}

        # BCM pins
        self.motors = {
            "LF": {"in1": 5, "in2": 6, "pwm": 12},
            "LR": {"in1": 16, "in2": 20, "pwm": 13},
            "RF": {"in1": 23, "in2": 24, "pwm": 18},
            "RR": {"in1": 21, "in2": 26, "pwm": 19},
        }

        if not dry_run:
            try:
                import RPi.GPIO as GPIO
                self.gpio = GPIO
                GPIO.setwarnings(False)
                GPIO.setmode(GPIO.BCM)

                for name, m in self.motors.items():
                    GPIO.setup(m["in1"], GPIO.OUT)
                    GPIO.setup(m["in2"], GPIO.OUT)
                    GPIO.setup(m["pwm"], GPIO.OUT)
                    pwm = GPIO.PWM(m["pwm"], pwm_freq)
                    pwm.start(0)
                    self.pwms[name] = pwm

                self.stop()
                print("[SAFE_PI_VEHICLE] GPIO motor initialized", flush=True)

            except Exception as e:
                print(f"[SAFE_PI_VEHICLE] GPIO init failed, dry-run fallback: {e}", flush=True)
                self.dry_run = True

    def set_motor(self, name, direction, speed):
        speed = max(0, min(100, int(speed)))

        if self.dry_run:
            return

        GPIO = self.gpio
        m = self.motors[name]

        if direction == "F":
            GPIO.output(m["in1"], GPIO.HIGH)
            GPIO.output(m["in2"], GPIO.LOW)
        elif direction == "B":
            GPIO.output(m["in1"], GPIO.LOW)
            GPIO.output(m["in2"], GPIO.HIGH)
        else:
            GPIO.output(m["in1"], GPIO.LOW)
            GPIO.output(m["in2"], GPIO.LOW)
            speed = 0

        self.pwms[name].ChangeDutyCycle(speed)

    def stop(self):
        for name in self.motors:
            self.set_motor(name, "S", 0)

    def forward(self, speed):
        for name in self.motors:
            self.set_motor(name, "F", speed)

    def backward(self, speed):
        for name in self.motors:
            self.set_motor(name, "B", speed)

    def turn_left(self, speed):
        # left side backward, right side forward
        self.set_motor("LF", "B", speed)
        self.set_motor("LR", "B", speed)
        self.set_motor("RF", "F", speed)
        self.set_motor("RR", "F", speed)

    def turn_right(self, speed):
        # left side forward, right side backward
        self.set_motor("LF", "F", speed)
        self.set_motor("LR", "F", speed)
        self.set_motor("RF", "B", speed)
        self.set_motor("RR", "B", speed)

    def cleanup(self):
        self.stop()
        if not self.dry_run and self.gpio:
            self.gpio.cleanup()

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

            if seq <= last_seq:
                print(f"[SAFE_PI_VEHICLE OLD_SEQ] seq={seq} last={last_seq}", flush=True)
                continue

            last_seq = seq
            last_cmd_time = now_ms()

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
    parser.add_argument("--pi-ip", default="192.168.0.24")
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
    parser.add_argument("--reach-area", type=int, default=60000)
    parser.add_argument("--reach-count", type=int, default=3)
    parser.add_argument("--perception-timeout-ms", type=int, default=3000)
    parser.add_argument("--target-lost-hold-ms", type=int, default=800)
    parser.add_argument("--obstacle-confirm-count", type=int, default=3)
    parser.add_argument("--obstacle-clear-count", type=int, default=2)
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
