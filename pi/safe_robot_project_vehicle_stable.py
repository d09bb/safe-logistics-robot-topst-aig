#!/usr/bin/env python3
import socket
import time
import RPi.GPIO as GPIO

# Camera pan servo config
SERVO_PIN = 17
SERVO_CENTER = 90

# Final safe speed policy for heavy vehicle.
# Use a short kick only when starting/changing direction,
# then keep low cruise speed for safety.
FINAL_SAFE_KICK_PWM = 70
FINAL_AUTO_CRUISE_SPEED = 40
FINAL_MANUAL_STRAIGHT_SPEED = 35
FINAL_MANUAL_TURN_SPEED = 30

# Final drive speed tuning for heavy vehicle.
# Low PWM cannot overcome static friction, so every moving command has a minimum PWM.
FINAL_AUTO_GAIN = 1.20
FINAL_AUTO_MAX = 65
FINAL_AUTO_MIN_MOVE = 56

FINAL_MANUAL_STRAIGHT_GAIN = 1.45
FINAL_MANUAL_STRAIGHT_MAX = 85
FINAL_MANUAL_STRAIGHT_MIN_MOVE = 62

FINAL_MANUAL_TURN_GAIN = 1.15
FINAL_MANUAL_TURN_MAX = 65
FINAL_MANUAL_TURN_MIN_MOVE = 58

# One-cycle kick when starting or changing direction.
FINAL_START_BOOST = 75

# Auto mode servo-body alignment.
# If the camera sees the marker far from the vehicle center direction,
# rotate the vehicle first while slowly returning the camera to center.
AUTO_SERVO_ALIGN_DEADBAND_DEG = 35
AUTO_SERVO_ALIGN_STEP_DEG = 5
AUTO_SERVO_ALIGN_MIN_SPEED = 35

# After one body-align turn pulse, force forward briefly.
# This prevents "turn-stop-search-turn" oscillation.
AUTO_SERVO_ALIGN_FORWARD_COMMIT_MS = 800
AUTO_SERVO_ALIGN_FORWARD_SPEED = 35
align_forward_until_ms = 0



SERVO_LEFT = 150
SERVO_RIGHT = 20

# 5-degree smooth scan
SERVO_SCAN_STEP = 5
SERVO_SCAN_INTERVAL_MS = 150
SERVO_SCAN_START_DELAY_MS = 800

# Physical scan limits. SERVO_LEFT may be larger than SERVO_RIGHT.
SERVO_SCAN_MIN = min(SERVO_LEFT, SERVO_RIGHT)
SERVO_SCAN_MAX = max(SERVO_LEFT, SERVO_RIGHT)

# Camera pan servo

# ============================================================
# Pin map: motor_test.py 기준
# ============================================================
LF_IN1, LF_IN2, LF_PWM = 5, 6, 12
LR_IN1, LR_IN2, LR_PWM = 16, 20, 13
RF_IN1, RF_IN2, RF_PWM = 23, 24, 18
RR_IN1, RR_IN2, RR_PWM = 21, 26, 19

MOTORS = {
    "LF": (LF_IN1, LF_IN2, LF_PWM),
    "LR": (LR_IN1, LR_IN2, LR_PWM),
    "RF": (RF_IN1, RF_IN2, RF_PWM),
    "RR": (RR_IN1, RR_IN2, RR_PWM),
}

# ============================================================
# Direction map
# 현재 확인 기준:
# raw F = 실제 후진
# raw B = 실제 전진
# TURN_LEFT/TURN_RIGHT는 네가 "됨"이라고 한 마지막 기준으로 설정
# ============================================================

def raw_forward_for_vehicle():
    return "B"

def raw_backward_for_vehicle():
    return "F"

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

pwms = {}

servo_pwm = None
current_servo_angle = None
servo_last_cmd_ms = 0
servo_search_start_ms = 0
servo_scan_angle = SERVO_CENTER
servo_scan_dir = 1
servo_scan_last_step_ms = 0
servo_last_mode = "INIT"
servo_freeze_until_ms = 0
servo_lock_until_ms = 0
last_target_servo_angle = SERVO_CENTER
last_target_scan_dir = 1
last_search_target = -1
servo_last_mode = "INIT"
servo_freeze_until_ms = 0


def setup():
    global servo_pwm

    GPIO.setup(SERVO_PIN, GPIO.OUT)
    servo_pwm = GPIO.PWM(SERVO_PIN, 50)
    servo_pwm.start(0)
    for name, (in1, in2, pwm_pin) in MOTORS.items():
        GPIO.setup(in1, GPIO.OUT)
        GPIO.setup(in2, GPIO.OUT)
        GPIO.setup(pwm_pin, GPIO.OUT)

        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.LOW)

        pwm = GPIO.PWM(pwm_pin, 1000)
        pwm.start(0)
        pwms[name] = pwm

    stop_all()
    set_servo_angle(SERVO_CENTER, force=True)
    print("[VEHICLE_STABLE] GPIO initialized from motor_test style, servo center=90", flush=True)

def set_raw(name, raw_dir, speed):
    in1, in2, pwm_pin = MOTORS[name]
    speed = max(0, min(100, int(speed)))

    if raw_dir == "F":
        GPIO.output(in1, GPIO.HIGH)
        GPIO.output(in2, GPIO.LOW)
        pwms[name].ChangeDutyCycle(speed)
    elif raw_dir == "B":
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.HIGH)
        pwms[name].ChangeDutyCycle(speed)
    else:
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.LOW)
        pwms[name].ChangeDutyCycle(0)

def stop_all():
    for name in MOTORS:
        set_raw(name, "S", 0)

def forward(speed):
    r = raw_forward_for_vehicle()
    for name in MOTORS:
        set_raw(name, r, speed)

def backward(speed):
    r = raw_backward_for_vehicle()
    for name in MOTORS:
        set_raw(name, r, speed)

def turn_left(speed):
    # SWAPPED: actual vehicle left turn
    set_raw("LF", "B", speed)
    set_raw("LR", "B", speed)
    set_raw("RF", "F", speed)
    set_raw("RR", "F", speed)
def turn_right(speed):
    # SWAPPED: actual vehicle right turn
    set_raw("LF", "F", speed)
    set_raw("LR", "F", speed)
    set_raw("RF", "B", speed)
    set_raw("RR", "B", speed)

def spin_vehicle(direction, duration=0.30):
    """
    Rough body spin before symmetric servo scan.
    Keep duration short for safety. Tune later after floor test.
    """
    print(f"[VEHICLE_SPIN] direction={direction} duration={duration}", flush=True)

    if direction == "LEFT":
        turn_left(FINAL_START_BOOST)
    elif direction == "RIGHT":
        turn_right(FINAL_START_BOOST)
    else:
        stop_all()
        return

    time.sleep(0.10)

    if direction == "LEFT":
        turn_left(FINAL_MANUAL_TURN_SPEED)
    elif direction == "RIGHT":
        turn_right(FINAL_MANUAL_TURN_SPEED)

    time.sleep(max(0.0, duration - 0.10))
    stop_all()
    print("[VEHICLE_SPIN] done -> STOP", flush=True)

def now_ms():
    return int(time.time() * 1000)

def set_servo_angle(angle, force=False):
    global current_servo_angle, servo_last_cmd_ms

    angle = max(0, min(180, int(angle)))
    t = now_ms()

    # 같은 각도면 다시 PWM을 주지 않음
    if (not force) and current_servo_angle == angle:
        return

    # 각도 변경 간격 제한
    if (not force) and (t - servo_last_cmd_ms) < 100:
        return

    duty = 2.5 + (angle / 180.0) * 10.0

    if servo_pwm is not None:
        servo_pwm.ChangeDutyCycle(duty)
        time.sleep(0.10)
        servo_pwm.ChangeDutyCycle(0)

    current_servo_angle = angle
    servo_last_cmd_ms = t

    print(f"[SERVO] angle={angle} duty={duty:.2f}", flush=True)

def update_servo_by_mode(mode, steer="CENTER", servo_cmd=None):
    """
    Servo policy:
    - GO_TO_TARGET:
      ArUco found. Keep current angle and remember this angle/direction.
    - SEARCH_TARGET:
      Search from the last angle where ArUco was found.
      First scan toward the same side where it was last seen.
    - SAFETY_STOP / LOCAL_SAFETY_STOP / FINISH / TARGET_REACHED:
      Center camera to SERVO_CENTER.
    """
    global servo_search_start_ms
    global servo_scan_angle, servo_scan_dir, servo_scan_last_step_ms
    global servo_last_mode, servo_freeze_until_ms, servo_lock_until_ms
    global last_target_servo_angle, last_target_scan_dir

    t = now_ms()

    if mode == "MANUAL":
        # Manual mode: do not move camera servo.
        # Joystick should control only vehicle drive, not camera pan.
        servo_lock_until_ms = 0
        servo_freeze_until_ms = 0
        servo_search_start_ms = 0
        servo_scan_last_step_ms = 0
        servo_last_mode = "MANUAL"
        return

    if mode == "GO_TO_TARGET":
        # Use TOPST CMD steer as the primary target-side memory.
        # Servo convention: LEFT=0, CENTER=40, RIGHT=150.
        # scan_dir +1 increases angle toward RIGHT, -1 decreases toward LEFT.
        _steer = str(steer or "CENTER").upper()

        # Physical servo calibration:
        # RIGHT = 20, CENTER = 90, LEFT = 150.
        # Increasing angle scans toward LEFT.
        # Decreasing angle scans toward RIGHT.
        if _steer == "RIGHT":
            last_target_servo_angle = SERVO_RIGHT
            last_target_scan_dir = -1
        elif _steer == "LEFT":
            last_target_servo_angle = SERVO_LEFT
            last_target_scan_dir = 1
        elif _steer == "CENTER":
            last_target_servo_angle = SERVO_CENTER
            last_target_scan_dir = 1

        # 현재 보고 있는 각도를 마지막 ArUco 발견 방향으로 저장
        if current_servo_angle is None:
            last_target_servo_angle = SERVO_CENTER
        else:
            last_target_servo_angle = current_servo_angle

        # 중심보다 큰 쪽에서 봤으면 다음에도 큰 각도 방향으로 먼저 탐색
        # 중심보다 작은 쪽에서 봤으면 다음에도 작은 각도 방향으로 먼저 탐색
        # Fallback direction only when steer was not explicitly provided.
        if str(steer or "").upper() not in ("LEFT", "CENTER", "RIGHT"):
            if last_target_servo_angle > SERVO_CENTER:
                last_target_scan_dir = 1
            elif last_target_servo_angle < SERVO_CENTER:
                last_target_scan_dir = -1
            else:
                last_target_scan_dir = 1

        # ArUco 발견 직후 SEARCH_TARGET 튐 방지
        servo_lock_until_ms = t + 1000
        servo_freeze_until_ms = servo_lock_until_ms
        servo_search_start_ms = 0
        servo_scan_last_step_ms = 0
        servo_last_mode = "GO_TO_TARGET"

        print(
            f"[SERVO_MEMORY] found_angle={last_target_servo_angle} next_dir={last_target_scan_dir}",
            flush=True
        )
        return

    if mode == "SEARCH_TARGET":
        # 방금 ArUco를 봤으면 SEARCH가 잠깐 들어와도 움직이지 않음
        if t < servo_lock_until_ms:
            print(
                f"[SERVO_LOCK_HOLD] ignore SEARCH angle={current_servo_angle} "
                f"last={last_target_servo_angle} dir={last_target_scan_dir}",
                flush=True
            )
            return

        # SEARCH 첫 진입: 마지막으로 본 각도에서 시작
        if servo_last_mode != "SEARCH_TARGET":
            servo_search_start_ms = t
            servo_scan_angle = last_target_servo_angle
            servo_scan_dir = last_target_scan_dir
            servo_scan_last_step_ms = t
            servo_last_mode = "SEARCH_TARGET"

            # 현재 각도를 마지막 각도로 맞춘 뒤 시작
            set_servo_angle(servo_scan_angle, force=True)

            print(
                f"[SERVO_SEARCH_ENTER] start_angle={servo_scan_angle} dir={servo_scan_dir}",
                flush=True
            )
            return

        # 아주 짧은 대기 후 스캔
        if (t - servo_search_start_ms) < SERVO_SCAN_START_DELAY_MS:
            return

        if (t - servo_scan_last_step_ms) >= SERVO_SCAN_INTERVAL_MS:
            servo_scan_angle += servo_scan_dir * SERVO_SCAN_STEP

            if servo_scan_angle >= SERVO_SCAN_MAX:
                servo_scan_angle = SERVO_SCAN_MAX
                servo_scan_dir = -1

            elif servo_scan_angle <= SERVO_SCAN_MIN:
                servo_scan_angle = SERVO_SCAN_MIN
                servo_scan_dir = 1

            servo_scan_last_step_ms = t
            set_servo_angle(servo_scan_angle, force=True)

            print(
                f"[SERVO_SCAN] angle={servo_scan_angle} dir={servo_scan_dir}",
                flush=True
            )

        return

    if mode in (
        "IDLE",
        "WAIT_START",
        "OBSTACLE_HOLD",
        "ARRIVAL_HOLD",
        "SAFETY_STOP",
        "LOCAL_SAFETY_STOP",
        "FINISH",
        "TARGET_REACHED",
    ):
        servo_lock_until_ms = 0
        servo_freeze_until_ms = 0
        servo_search_start_ms = 0
        servo_scan_angle = SERVO_CENTER
        servo_scan_dir = last_target_scan_dir
        servo_scan_last_step_ms = 0
        servo_last_mode = mode
        set_servo_angle(SERVO_CENTER, force=True)
        print(f"[SERVO_CENTER] mode={mode} angle={SERVO_CENTER}", flush=True)
        return

    # AUTO_AVOID / TEST / UNKNOWN: 현재 각도 유지
    servo_last_mode = mode
    return


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

def to_int(v, default=0):
    try:
        return int(v)
    except Exception:
        return default


def apply_auto_servo_alignment(mode, drive, speed):
    """
    Auto alignment policy:
    - If camera servo is off-center, give only one short body-turn pulse.
    - Then force FORWARD for a short interval.
    - This lets the robot approach the marker instead of rotating forever.
    """
    global current_servo_angle, align_forward_until_ms

    t = now_ms()

    if mode not in ("GO_TO_TARGET", "GO_TO_TARGET_HOLD"):
        align_forward_until_ms = 0
        return drive, speed

    if current_servo_angle is None:
        return drive, speed

    # Forward commit window after a turn pulse.
    if t < align_forward_until_ms:
        print(
            f"[AUTO_SERVO_ALIGN_FORWARD_COMMIT] angle={current_servo_angle} "
            f"until={align_forward_until_ms} now={t}",
            flush=True,
        )
        return "FORWARD", max(int(speed), AUTO_SERVO_ALIGN_FORWARD_SPEED)

    err = int(current_servo_angle) - SERVO_CENTER

    if abs(err) <= AUTO_SERVO_ALIGN_DEADBAND_DEG:
        return drive, speed

    if err > 0:
        # Camera is looking left, so rotate vehicle left briefly,
        # while moving the camera back toward center.
        new_angle = max(SERVO_CENTER, int(current_servo_angle) - AUTO_SERVO_ALIGN_STEP_DEG)
        set_servo_angle(new_angle, force=True)
        align_forward_until_ms = t + AUTO_SERVO_ALIGN_FORWARD_COMMIT_MS
        print(
            f"[AUTO_SERVO_ALIGN_TURN_PULSE] marker_side=LEFT angle={current_servo_angle} "
            f"err={err} override=TURN_LEFT forward_until={align_forward_until_ms}",
            flush=True,
        )
        return "TURN_LEFT", max(int(speed), AUTO_SERVO_ALIGN_MIN_SPEED)

    # Camera is looking right, so rotate vehicle right briefly,
    # while moving the camera back toward center.
    new_angle = min(SERVO_CENTER, int(current_servo_angle) + AUTO_SERVO_ALIGN_STEP_DEG)
    set_servo_angle(new_angle, force=True)
    align_forward_until_ms = t + AUTO_SERVO_ALIGN_FORWARD_COMMIT_MS
    print(
        f"[AUTO_SERVO_ALIGN_TURN_PULSE] marker_side=RIGHT angle={current_servo_angle} "
        f"err={err} override=TURN_RIGHT forward_until={align_forward_until_ms}",
        flush=True,
    )
    return "TURN_RIGHT", max(int(speed), AUTO_SERVO_ALIGN_MIN_SPEED)


def apply_drive(drive, speed, mode="UNKNOWN"):
    if drive == "FORWARD":
        forward(speed)
        return "FORWARD"
    elif drive == "BACKWARD":
        backward(speed)
        return "BACKWARD"
    elif drive in ("TURN_LEFT", "AVOID_LEFT"):
        turn_left(speed)
        return "TURN_LEFT"
    elif drive in ("TURN_RIGHT", "AVOID_RIGHT"):
        turn_right(speed)
        return "TURN_RIGHT"
    else:
        stop_all()
        return "STOP"

def main():
    global last_search_target, last_target_servo_angle, servo_last_mode
    listen_port = 5006
    timeout_ms = 3000

    setup()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", listen_port))
    sock.settimeout(0.1)

    last_cmd_time = int(time.time() * 1000)
    last_drive = "STOP"
    last_speed = 0

    print(f"[VEHICLE_STABLE] listen UDP 0.0.0.0:{listen_port}", flush=True)
    print("[VEHICLE_STABLE] no seq filtering; execute latest CMD directly", flush=True)

    try:
        while True:
            now = int(time.time() * 1000)

            try:
                data, addr = sock.recvfrom(4096)
                text = data.decode(errors="ignore").strip()
            except socket.timeout:
                if now - last_cmd_time > timeout_ms:
                    if last_drive != "STOP":
                        print("[VEHICLE_STABLE] CMD timeout -> STOP", flush=True)
                    stop_all()
                    set_servo_angle(SERVO_CENTER)
                    last_drive = "STOP"
                    last_speed = 0
                continue

            msg_type, kv = parse_kv(text)

            if msg_type != "CMD":
                print(f"[VEHICLE_STABLE IGNORE] {addr} {text}", flush=True)
                continue

            drive = kv.get("drive", "STOP")
            speed = to_int(kv.get("speed", 0), 0)
            mode = kv.get("mode", "UNKNOWN")

            target_id = to_int(kv.get("target", -1), -1)
            search_dir = str(kv.get("dir", "CENTER")).upper()

            # Mapping-based rough search:
            # When TOPST enters SEARCH_TARGET for a new target, spin the body once,
            # then let the servo scan symmetrically from SERVO_CENTER.
            if mode == "SEARCH_TARGET":
                if target_id != last_search_target and target_id in (0, 1, 2):
                    if search_dir == "LEFT":
                        spin_vehicle("LEFT", 0.08)
                    elif search_dir == "RIGHT":
                        spin_vehicle("RIGHT", 0.08)

                    last_search_target = target_id
                    last_target_servo_angle = SERVO_CENTER
                    servo_last_mode = "INIT"
            # Do not reset last_search_target on every GO_TO_TARGET.
            # Otherwise, when ArUco flickers:
            #   SEARCH_TARGET -> GO_TO_TARGET -> SEARCH_TARGET
            # the same target spins again and again.
            elif mode in (
                "IDLE",
                "WAIT_START",
                "FINISH",
                "COMPLETION",
                "SAFETY_STOP",
                "ARRIVAL_HOLD",
            ):
                last_search_target = -1

            # EXTREME_TURN_ONLY_PWM100
            # AUTO/MANUAL both: only left/right turn commands use max PWM 100.
            # Forward/backward/stop are not changed.
            if drive in ("TURN_LEFT", "TURN_RIGHT", "AVOID_LEFT", "AVOID_RIGHT"):
                speed = 70
                print(f"[EXTREME_TURN_ONLY_PWM100] mode={mode} drive={drive} speed={speed}", flush=True)

            # Final speed profile for heavy vehicle.
            # STOP remains STOP. Moving commands get minimum PWM and one-cycle start boost.
            if drive != "STOP" and speed > 0:
                if mode == "MANUAL":
                    if drive in ("TURN_LEFT", "TURN_RIGHT", "AVOID_LEFT", "AVOID_RIGHT"):
                        speed = int(speed * FINAL_MANUAL_TURN_GAIN)
                        if speed < FINAL_MANUAL_TURN_MIN_MOVE:
                            speed = FINAL_MANUAL_TURN_MIN_MOVE
                        if speed > FINAL_MANUAL_TURN_MAX:
                            speed = FINAL_MANUAL_TURN_MAX
                    else:
                        speed = int(speed * FINAL_MANUAL_STRAIGHT_GAIN)
                        if speed < FINAL_MANUAL_STRAIGHT_MIN_MOVE:
                            speed = FINAL_MANUAL_STRAIGHT_MIN_MOVE
                        if speed > FINAL_MANUAL_STRAIGHT_MAX:
                            speed = FINAL_MANUAL_STRAIGHT_MAX
                else:
                    speed = int(speed * FINAL_AUTO_GAIN)
                    if speed < FINAL_AUTO_MIN_MOVE:
                        speed = FINAL_AUTO_MIN_MOVE
                    if speed > FINAL_AUTO_MAX:
                        speed = FINAL_AUTO_MAX

                # Smooth start boost:
                # Apply boost only when starting from STOP.
                # Do not re-boost on every FORWARD/TURN command change.
                if last_drive == "STOP":
                    if speed < FINAL_START_BOOST:
                        speed = FINAL_START_BOOST
            fault = kv.get("fault", "NONE")
            steer = kv.get("steer", "CENTER")
            servo_cmd = to_int(kv.get("servo", -1), -1)

            last_cmd_time = now
            # SERVO_STOP_STATE_GUARD
            # Servo must be fixed only in:
            # - E-STOP / SAFETY_STOP
            # - MANUAL mode
            # - FINISH / COMPLETION after all targets are visited
            #
            # Important:
            # SEARCH_TARGET must be allowed to scan.
            # Therefore, do NOT stop servo only because drive=STOP or speed=0.
            stop_servo_modes = (
                "SAFETY_STOP",
                "MANUAL",
                "FINISH",
                "COMPLETION",
            )

            if mode in stop_servo_modes:
                if current_servo_angle != SERVO_CENTER:
                    set_servo_angle(SERVO_CENTER, force=True)
                    print(f"[SERVO_FIXED_MODE_CENTER] mode={mode} angle={SERVO_CENTER}", flush=True)
                else:
                    if servo_pwm is not None:
                        servo_pwm.ChangeDutyCycle(0)

                servo_search_start_ms = 0
                servo_scan_last_step_ms = 0
                servo_scan_angle = SERVO_CENTER
                servo_last_mode = mode
            else:
                update_servo_by_mode(mode, steer=steer, servo_cmd=servo_cmd)

            # AUTO_SERVO_BODY_ALIGN is intentionally disabled.
            # Direction is controlled only by TOPST ArUco cx decision.
            # Servo angle must not override FORWARD/TURN commands.
            # drive, speed = apply_auto_servo_alignment(mode, drive, speed)

            # FINAL_TURN_DIRECTION_SPEED_FIX
            # Forward/backward keep the previous safe cruise policy.
            # Only left/right tank turn commands are forced to max PWM 100.
            turn_cmd = drive in ("TURN_LEFT", "TURN_RIGHT", "AVOID_LEFT", "AVOID_RIGHT")

            if drive == "STOP" or speed <= 0:
                speed = 0
            else:
                if turn_cmd:
                    speed = 70
                    print(f"[FINAL_TURN_PWM100] mode={mode} drive={drive} speed={speed}", flush=True)
                else:
                    if mode == "MANUAL":
                        cruise_speed = FINAL_MANUAL_STRAIGHT_SPEED
                    else:
                        cruise_speed = FINAL_AUTO_CRUISE_SPEED

                    if last_drive == "STOP":
                        speed = FINAL_SAFE_KICK_PWM
                    else:
                        speed = cruise_speed

            # AUTO_TURN_SPEED_LIMIT_PATCH
            # Reduce AUTO mode left/right turn speed only.
            # MANUAL turn, forward, backward, and stop are unchanged.
            AUTO_TURN_SPEED_LIMIT = 70
            if mode != "MANUAL" and drive in ("TURN_LEFT", "TURN_RIGHT", "AVOID_LEFT", "AVOID_RIGHT"):
                if speed > AUTO_TURN_SPEED_LIMIT:
                    speed = AUTO_TURN_SPEED_LIMIT
                print(f"[AUTO_TURN_SPEED_LIMIT] mode={mode} drive={drive} speed={speed}", flush=True)
            applied = apply_drive(drive, speed, mode)

            last_drive = applied
            last_speed = speed if applied != "STOP" else 0

            print(
                f"[VEHICLE_STABLE CMD] mode={mode} drive={drive} speed={speed} "
                f"applied={applied} fault={fault}",
                flush=True
            )

    except KeyboardInterrupt:
        print("[VEHICLE_STABLE] KeyboardInterrupt -> STOP", flush=True)
    finally:
        stop_all()
        if servo_pwm is not None:
            servo_pwm.ChangeDutyCycle(0)
            servo_pwm.stop()
        print("[VEHICLE_STABLE] pins LOW, servo stopped, no GPIO.cleanup()", flush=True)

if __name__ == "__main__":
    main()
