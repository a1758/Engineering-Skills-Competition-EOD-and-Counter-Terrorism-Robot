# OpenMV 纠偏双模式：同一文件包含测试版与联调版
# RUN_MODE="TEST"：配合STM32 V7，上电等待rectify。
# RUN_MODE="INTEGRATION"：保留原联调方式，上电直接发送VERR。
# 两种模式共用识别、诊断与停止命令；上电直接发送数据可能启动底盘纠偏。
# STM32停车先T后X，3帧启动，连续5帧达标且停稳后finishrectify。
# STM32从纠偏完成位置前进1650mm；15秒纠偏超时与1秒失联保护由STM32执行。
# OpenMV保留原始X/T算法与符号，不自行结束或控制底盘。
# target切到80度；颜色识别尚未接入，不发送猜测的11/22/33。
# finishtarget由STM32发送，PE2激光1000ms由STM32控制；本机P5保持关闭。
import sensor, image, time, math
from pyb import UART, Pin, Timer

# 只修改这一项切换模式，不需要换文件。
RUN_MODE = "TEST"  # "TEST" 或 "INTEGRATION"
if RUN_MODE not in ("TEST", "INTEGRATION"):
    raise ValueError("RUN_MODE must be TEST or INTEGRATION")
MODE_ID = "OPENMV_CORRECTION_V7_" + RUN_MODE

UART_ID = 1
UART_BAUD = 115200
SERVO_PIN = "P7"
SERVO_CHANNEL = 1
SERVO_MAX_ANGLE = 180
SERVO_REVERSE_DIRECTION = True
SERVO_MIN_PULSE_US = 500
SERVO_MAX_PULSE_US = 2500
CORRECTION_PITCH_DEG = 180
TARGET_PITCH_DEG = 80
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
SEND_PERIOD_MS = 70
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

correcting = (RUN_MODE == "INTEGRATION")
mode = "CORRECTING" if correcting else "WAIT RECTIFY"
last_valid_tx = None
print(MODE_ID, "READY:", mode)
last_tx = 0
last_log = 0
rx_buf = bytearray()
track_samples = []
track_reason = "WAIT"
track_residual = None
frame_count = 0
invalid_count = 0
good_streak = 0
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
    global frame_count, invalid_count, good_streak, last_status, last_log
    global correcting, last_tx, last_valid_tx, mode
    cmd = cmd.strip().lower()
    if cmd in (b"rectify", b"correct", b"shortcorrect", b"longcorrect"):
        frame_count = 0
        invalid_count = 0
        good_streak = 0
        last_valid_tx = None
        last_status = "WAIT"
        last_log = time.ticks_ms()
        set_pitch(CORRECTION_PITCH_DEG)
        last_tx = time.ticks_ms() - SEND_PERIOD_MS
        correcting = True
        mode = "CORRECTING"
        print("CORRECTION START", MODE_ID)
    elif cmd in (b"finishrectify", b"finishcorrect"):
        correcting = False
        mode = "WAIT TARGET"
        print("CORRECTION STOP", cmd, "frames", frame_count,
              "invalid", invalid_count, "streak", good_streak)
    elif cmd in (b"stopcorrect", b"stop"):
        correcting = False
        good_streak = 0
        last_valid_tx = None
        mode = "STOPPED"
        laser.value(LASER_OFF_LEVEL)
        print("STOP", cmd)
    elif cmd == b"target":
        if mode != "WAIT TARGET":
            print("TARGET IGNORED IN", mode)
            return
        correcting = False
        laser.value(LASER_OFF_LEVEL)
        set_pitch(TARGET_PITCH_DEG)
        mode = "TARGET: NO COLOR CODE"
        print("TARGET READY: PITCH 80; COLOR RECOGNITION NOT CONFIGURED")
    elif cmd == b"finishtarget":
        if mode != "TARGET: NO COLOR CODE":
            print("FINISHTARGET IGNORED IN", mode)
            return
        correcting = False
        laser.value(LASER_OFF_LEVEL)
        mode = "TARGET FINISHED"
        print("TARGET FINISHED; STM32 CONTROLS PE2 LASER")

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

while True:
    clock.tick(); poll_uart(); captured = time.ticks_ms(); img = sensor.snapshot(); poll_uart()
    if not correcting:
        img.draw_string(2, 2, mode, color=255)
        continue
    now = time.ticks_ms()
    if time.ticks_diff(now, last_tx) < SEND_PERIOD_MS:
        continue
    result = detect_track(img)
    # 发送前处理停止命令，并将接收处理耗时计入数据年龄。
    poll_uart()
    if not correcting:
        img.draw_string(2, 2, mode, color=255)
        continue
    age = max(0, time.ticks_diff(time.ticks_ms(), captured))
    status = frame_status(result, age)
    if status == "VALID":
        x, heading, _, _ = result
        frame = ("VERR 1 {:.1f} {:.1f} 80 {}\r\n".format(
            x, heading, age)).encode("ascii")
    else:
        frame = b"VERR 0 0.0 0.0 0 0\r\n"
    send(frame); last_tx = time.ticks_ms()
    frame_count += 1
    if status != "VALID":
        invalid_count += 1
    # 诊断计数也遵守150ms相邻有效帧限制，但不代替STM32停稳判定。
    if status != "VALID" or (last_valid_tx is not None and
            time.ticks_diff(last_tx, last_valid_tx) > MAX_FRAME_AGE_MS):
        good_streak = 0
    if status == "VALID":
        last_valid_tx = last_tx
    else:
        last_valid_tx = None
    if status == "VALID" and abs(result[0]) <= 20 and abs(result[1]) <= 3:
        good_streak += 1
    else:
        good_streak = 0
    # streak仅用于诊断；结束与达标判定仍由STM32执行。
    if time.ticks_diff(last_tx, last_log) >= LOG_PERIOD_MS or status != last_status:
        raw = ("X:{:.2f} T:{:.2f}".format(result[0], result[1])
               if result is not None else "X:NA T:NA")
        print("VERR", repr(frame), "reason", status, "fit", track_reason,
              "raw", raw, "age_ms", age, "samples", len(track_samples),
              "residual_px", track_residual, "frames", frame_count,
              "invalid", invalid_count, "streak", good_streak)
        last_log = last_tx
    last_status = status
    draw_overlay(img, result)
    if status != "VALID":
        img.draw_string(2, 20, "REJECT " + status, color=255)
