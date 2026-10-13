# OpenMV 长轨道纠偏测试：基于当前 correction_only_integration.py
# 配合 MD 最新 V5 / STM32 VISION_TARGET_TEST_V5_1300。
# UART1 115200 8N1；rectify启动，finishrectify/stopcorrect/STOP停止。
# 上电只等待命令；target只记录通知并保持激光关闭。
# 1300mm总行程（包含纠偏路程）、3帧启动、10帧达标、
# 到站退出、yaw左90度均由STM32负责，OpenMV仅输出VERR。
# 不发送finishtarget，不处理11/22/33，不自动启动激光。
# 识别算法、阈值、X/T符号、采集配置与诊断日志沿用联调版。
import sensor, image, time, math

from pyb import UART, Pin, Timer



UART_ID = 1

UART_BAUD = 115200

SERVO_PIN = "P7"

SERVO_CHANNEL = 1

SERVO_MAX_ANGLE = 180

SERVO_REVERSE_DIRECTION = True

SERVO_MIN_PULSE_US = 500

SERVO_MAX_PULSE_US = 2500

CORRECTION_PITCH_DEG = 180

LASER_PIN = "P5"

LASER_OFF_LEVEL = 0

LIGHT_PIN = "P9"

LIGHT_ON_LEVEL = 1

TRACK_GRAY_LAB = (42, 100, -17, 14, -18, 9)

TRACK_SCAN_ROWS = (0.30, 0.36, 0.42, 0.48, 0.54, 0.60, 0.66)

TRACK_CONTROL_ROW = 0.48

TRACK_CENTER_OFFSET_QVGA_PX = 15.0

MIN_SAMPLES = 4

MAX_RESIDUAL_QVGA_PX = 15.0

LATERAL_SIGN = 1.0

HEADING_SIGN = -1.0

SEND_PERIOD_MS = 0

MAX_FRAME_AGE_MS = 150

LOG_PERIOD_MS = 250



uart = UART(UART_ID, UART_BAUD, bits=8, parity=None, stop=1,

            timeout=20, timeout_char=20, read_buf_len=256)

pitch_timer = Timer(4, freq=50)

pitch_pwm = pitch_timer.channel(SERVO_CHANNEL, Timer.PWM, pin=Pin(SERVO_PIN))



def set_pitch(mechanical_deg):

    actual = SERVO_MAX_ANGLE - mechanical_deg if SERVO_REVERSE_DIRECTION else mechanical_deg

    pulse_us = int(round(SERVO_MIN_PULSE_US +

                         (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) *

                         actual / SERVO_MAX_ANGLE))

    pitch_pwm.pulse_width(((pitch_timer.period() + 1) * pulse_us) // 20000)

    print("PITCH", mechanical_deg, "deg, pulse_us", pulse_us)



# 纠偏联调固定使用 180° 俯仰

set_pitch(CORRECTION_PITCH_DEG)

laser = Pin(LASER_PIN, Pin.OUT_PP)

laser.value(LASER_OFF_LEVEL)

light = Pin(LIGHT_PIN, Pin.OUT_PP)

light.value(LIGHT_ON_LEVEL)

print("LASER: OFF", LASER_PIN, "LIGHT: ON", LIGHT_PIN)

sensor.reset()

sensor.set_pixformat(sensor.RGB565)

sensor.set_framesize(sensor.VGA)

sensor.set_hmirror(False)

sensor.set_vflip(False)

sensor.set_auto_exposure(True)

sensor.set_auto_gain(True)

sensor.set_auto_whitebal(True)

sensor.skip_frames(time=1500)

sensor.set_auto_exposure(False)

sensor.set_auto_gain(False)

sensor.set_auto_whitebal(False)

sensor.skip_frames(time=200)

clock = time.clock()



correcting = False                # 先等待纠偏命令
phase = "WAIT_RECTIFY"

# 靶标阶段参数（沿用 target_shoot_standalone.py）
QR_TASK_CODE = 222
TASK_COLOR = 1
COLORS = {1: (0,45,14,81,-76,79), 2: (34,56,-67,-17,-41,84), 3: (11,59,-28,-5,-33,-6)}
TARGET_ROI = (0, 0, 320, 200)
MIN_PIXELS, MIN_AREA = 80, 120
CONFIRM_FRAMES = 3
FIRST_ZONE = (70, 0, 210, 200)
SECOND_ZONE = (205, 0, 320, 200)
TARGET_PITCH_DEG = 80
PITCH_SETTLE_MS = 600
LASER_ON_MS = 1000
pitch_ready = time.ticks_add(time.ticks_ms(), PITCH_SETTLE_MS)
target_stable = 0
target_choice = 0
target_shot_requested = False
print("OPENMV_LONG_TRACK_V5_1300 READY: WAIT RECTIFY")

last_tx = 0

last_log = 0

rx_buf = bytearray()

track_samples = []

track_reason = "WAIT"

track_residual = None

frame_count = 0

invalid_count = 0

good_streak = 0
correction_good_streak = 0

last_status = "WAIT"



class TrackEdge:

    def __init__(self, p): self.p = p

    def line(self): return self.p



def fit(points):

    my = sum(y for y, x in points) / len(points)

    mx = sum(x for y, x in points) / len(points)

    den = sum((y - my) ** 2 for y, x in points)

    if den <= 0: return 0.0, mx

    m = sum((y - my) * (x - mx) for y, x in points) / den

    return m, mx - m * my



def detect_track(img):

    global track_samples, track_reason, track_residual

    track_residual = None

    w, h = img.width(), img.height()

    sx = w / 320.0

    reference = w / 2.0 + TRACK_CENTER_OFFSET_QVGA_PX * sx

    band = max(4, int(h * .018))

    previous = reference

    samples = []

    for fraction in TRACK_SCAN_ROWS:

        y = int(h * fraction)

        candidates = []

        for blob in img.find_blobs([TRACK_GRAY_LAB], roi=(0, y, w, band),

                                    x_stride=2, y_stride=1, merge=False,

                                    pixels_threshold=30, area_threshold=40):

            x, _, bw, _ = blob.rect()

            if x <= 2 or x + bw >= w - 2 or not .18*w <= bw <= .94*w:

                continue

            if blob.pixels() / float(bw * band) < .55:

                continue

            mid = x + (bw - 1) / 2.0

            if abs(mid - previous) < .25*w:

                candidates.append((abs(mid - previous), x, x + bw - 1))

        if candidates:

            _, left, right = min(candidates)

            samples.append((y + band // 2, left, right))

            previous = (left + right) / 2.0

    track_samples = samples

    if len(samples) < MIN_SAMPLES:

        track_reason = "FEW SAMPLES"; return None

    if samples[-1][0] - samples[0][0] < .15*h:

        track_reason = "SHORT SPAN"; return None

    centers = [(y, (l+r)/2.0) for y, l, r in samples]

    m, c = fit(centers)

    track_residual = max(abs(x - (m*y + c)) for y, x in centers)

    if track_residual > MAX_RESIDUAL_QVGA_PX*sx:

        track_reason = "BAD FIT"; return None

    left_m, left_c = fit([(y, l) for y, l, r in samples])

    right_m, right_c = fit([(y, r) for y, l, r in samples])

    control_y = int(h * TRACK_CONTROL_ROW) + band // 2

    center = m * control_y + c

    error = LATERAL_SIGN * (reference - center)

    heading = HEADING_SIGN * math.degrees(math.atan(m))

    track_reason = "VALID"

    left = TrackEdge((int(left_m*samples[0][0]+left_c), samples[0][0],

                      int(left_m*samples[-1][0]+left_c), samples[-1][0]))

    right = TrackEdge((int(right_m*samples[0][0]+right_c), samples[0][0],

                       int(right_m*samples[-1][0]+right_c), samples[-1][0]))

    return error, heading, left, right



def send(frame):
    try: uart.write(frame)

    except Exception as e: print("UART TX ERROR", e)



def handle_command(cmd):
    global frame_count, invalid_count, good_streak, last_status, last_log, last_tx
    global correcting, phase, pitch_ready, target_stable, target_choice, target_shot_requested, correction_good_streak
    cmd = cmd.strip().lower()
    valid_commands = (b"rectify", b"correct", b"shortcorrect", b"longcorrect",
                      b"finishrectify", b"stopcorrect", b"finishcorrect", b"stop",
                      b"target", b"finishtarget", b"reset")
    if cmd not in valid_commands:
        return
    print("RX", repr(cmd))
    if cmd in (b"rectify", b"correct", b"shortcorrect", b"longcorrect"):
        frame_count = invalid_count = good_streak = correction_good_streak = 0
        last_status = "WAIT"; last_log = time.ticks_ms()
        set_pitch(CORRECTION_PITCH_DEG); correcting = True; phase = "CORRECTING"; last_tx = time.ticks_ms() - SEND_PERIOD_MS
        print("CORRECTION START")
    elif cmd in (b"finishrectify", b"stopcorrect", b"finishcorrect"):
        correcting = False; phase = "WAIT_TARGET"
        print("CORRECTION STOP, WAIT TARGET")
    elif cmd == b"target" and phase == "WAIT_TARGET":
        set_pitch(TARGET_PITCH_DEG)
        pitch_ready = time.ticks_add(time.ticks_ms(), PITCH_SETTLE_MS)
        target_stable = target_choice = 0; target_shot_requested = False
        phase = "TARGET_SEARCH"; print("TARGET SEARCH START")
    elif cmd == b"finishtarget" and phase in ("TARGET_WAIT_FINISH", "TARGET_SEARCH"):
        laser.value(1); send(b"laser\r\n")
        deadline = time.ticks_add(time.ticks_ms(), LASER_ON_MS)
        while time.ticks_diff(deadline, time.ticks_ms()) > 0: time.sleep_ms(5)
        laser.value(LASER_OFF_LEVEL); send(b"ready\r\n")
        phase = "WAIT_TARGET"; target_shot_requested = False; print("TARGET FINISH")
    elif cmd == b"reset":
        correcting = False; phase = "WAIT_RECTIFY"; target_shot_requested = False
        laser.value(LASER_OFF_LEVEL)

def poll_uart():

    global rx_buf

    while uart.any():

        data = uart.read(128)

        if not data: return

        for b in data:

            if b in (10, 13):

                if rx_buf: handle_command(bytes(rx_buf)); rx_buf = bytearray()

            elif len(rx_buf) < 32: rx_buf.append(b)

            else: rx_buf = bytearray()



def draw_overlay(img, result):

    w, h = img.width(), img.height(); sx = w / 320.0

    ref = w / 2.0 + TRACK_CENTER_OFFSET_QVGA_PX * sx; top = int(.29*h)

    img.draw_line(int(ref), top, int(ref), h-1, color=255, thickness=3)

    if result:

        x, t, left, right = result

        img.draw_line(left.line(), color=255, thickness=2)

        img.draw_line(right.line(), color=255, thickness=2)

        img.draw_string(2, 2, "VERR X:{:.1f} T:{:.1f}".format(x, t), color=255)

    else: img.draw_string(2, 2, "INVALID " + track_reason, color=255)



def frame_status(result, age):

    if result is None:

        return track_reason

    x, heading, _, _ = result

    reasons = []

    if not math.isfinite(x) or not math.isfinite(heading):

        reasons.append("NONFINITE")

    if abs(x) > 320:

        reasons.append("X_RANGE")

    if abs(heading) > 15:

        reasons.append("T_RANGE")

    if age > MAX_FRAME_AGE_MS:

        reasons.append("STALE")

    return "+".join(reasons) if reasons else "VALID"




def target_candidates(img):
    return [b for b in img.find_blobs([COLORS[TASK_COLOR]], roi=TARGET_ROI, pixels_threshold=MIN_PIXELS, area_threshold=MIN_AREA, merge=True) if b.w() >= 8 and b.h() >= 8]
def choose_target(bs):
    first = [b for b in bs if FIRST_ZONE[0] <= b.cx() < FIRST_ZONE[2]]
    right = [b for b in bs if SECOND_ZONE[0] <= b.cx() < SECOND_ZONE[2]]
    if first: return 1, max(first, key=lambda b:b.pixels())
    if right: return 2, max(right, key=lambda b:b.pixels())
    return 0, None
while True:
    clock.tick(); poll_uart(); captured = time.ticks_ms(); img = sensor.snapshot(); poll_uart()
    if phase == "WAIT_RECTIFY": img.draw_string(2, 2, "WAIT RECTIFY", color=255); continue
    if phase == "CORRECTING":
        if time.ticks_diff(time.ticks_ms(), last_tx) < SEND_PERIOD_MS: continue
        result = detect_track(img); poll_uart()
        if not correcting: continue
        age = max(0, time.ticks_diff(time.ticks_ms(), captured)); status = frame_status(result, age)
        if status == "VALID":
            frame = ("VERR 1 {:.1f} {:.1f} 80 {}\r\n".format(result[0], result[1], age)).encode("ascii")
            send(frame)
        last_tx = time.ticks_ms(); draw_overlay(img, result)
        continue
    if phase == "TARGET_SEARCH":
        bs = target_candidates(img); idx, blob = choose_target(bs); target_stable = target_stable + 1 if idx else 0
        if idx and target_stable >= CONFIRM_FRAMES and time.ticks_diff(time.ticks_ms(), pitch_ready) >= 0 and not target_shot_requested:
            target_choice = idx; send((str({1:11, 2:22}[idx]) + "\r\n").encode()); target_shot_requested = True; phase = "TARGET_WAIT_FINISH"
        for b in bs: img.draw_rectangle(b.rect(), color=(255,0,0)); img.draw_cross(b.cx(), b.cy(), color=(0,255,0))
    img.draw_string(2, 2, phase + " C" + str(TASK_COLOR) + " K" + str(target_choice), color=255)










