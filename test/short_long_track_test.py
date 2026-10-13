# STM32 纠偏流程：收到 rectify 后 OpenMV 只测量并发送 VERR；
# STM32 收到连续有效帧后自行低速前进、横移和旋转，达标后发送 finishrectify。
# OpenMV 不发送底盘速度，也不自行判断完成条件。
# 短轨道算法复用于长轨道；STM32 UART 联调测试文件
# OpenMV + STM32 联调测试
# STM32 -> OpenMV: qr
# OpenMV -> STM32: finishqr（仅二维码确认成功后发送）
# qr 握手沿用 qr_correction_test.py；rectify 开启 VERR，finishrectify 停止。

import sensor, image, time, math
from pyb import UART, Timer, Pin

# 运行模式：
# DEBUG：独立观察纠偏，不依赖 STM32；INTEGRATION：与 STM32 串口联调。
OPERATION_MODE = "INTEGRATION"
print("BUILD: QR_FIXED_LAB_INTEGRATION_V3; file: short_long_track_test.py")
DEBUG_CORRECTION = OPERATION_MODE != "INTEGRATION"
if OPERATION_MODE not in ("DEBUG", "INTEGRATION", "CALIBRATE"):
    raise ValueError("OPERATION_MODE must be DEBUG, INTEGRATION or CALIBRATE")
# CALIBRATE 前必须将车摆正居中，采样期间不移动。
CALIBRATION_FRAMES = 30
CALIBRATION_POINTS = []
CALIBRATION_DONE = False
SEND_TRACK_POSITION = False  # STM32 接入 TRACK 解析后改为 True
# 补光灯：P9，按你的硬件若有效电平相反改为 0；不使用时设为 None。
LIGHT_PIN = "P9"
LIGHT_ENABLE = True  # 调试和联调均开启补光，False 关闭
LIGHT_ON_LEVEL = 1
LIGHT_OFF_LEVEL = 0
LIGHT_WARMUP_MS = 1500
TRACK_STAGE = "SHORT"  # 同一短轨道算法用于短轨道和夹球后的长轨道
TRACK_CENTER_OFFSET_QVGA_PX = 15.0  # 参考中心偏置，以 320 像素宽度为基准
HEADING_ZERO_DEG = 0.0               # 车摆正时测得的 T，用作零点
CENTER_LINE_THICKNESS = 3  # 参考中心线粗细（像素），只影响显示
DEBUG_CENTER_TOL_QVGA_PX = 10.0  # VGA 下中心左右各 20 像素
DEBUG_HEADING_TOL_DEG = 3.0
DEBUG_PRINT_MS = 500
LAST_DEBUG_MS = 0
# 当前画面上方受反光影响，允许 4/7 条扫描带参与拟合。
DEBUG_MIN_SAMPLES = 4
# 灰色区域有明显透视，近处宽度远大于远处；只拒绝极端误检。
DEBUG_MAX_FIT_RESIDUAL_QVGA_PX = 15.0
DEBUG_WIDTH_TOLERANCE = 1.00
DEBUG_OUTLIER_RESIDUAL_QVGA_PX = 10.0
DEBUG_VALID_STREAK_REQUIRED = 3
DEBUG_VALID_STREAK = 0
# LAB 初值需用 IDE 阈值编辑器选取灰色路面，排除浅黄色背景。
TRACK_GRAY_LAB = (42, 100, -17, 14, -18, 9)
TRACK_SCAN_ROWS = (0.30, 0.36, 0.42, 0.48, 0.54, 0.60, 0.66)
TRACK_CONTROL_ROW = 0.48
TRACK_MIN_SPAN = 0.15  # 四条间隔 0.06 的扫描带，容许整数取整误差
TRACK_VERSION = "CORRECTION_ONLY_FIXED_LAB_V1"
TRACK_DETECT_MODE = "ADAPTIVE_CENTER"  # ADAPTIVE_CENTER / GRADIENT / BLOBS
GRADIENT_STEP = 4
GRADIENT_MIN_CONTRAST = 8
QR_PITCH_DEG = 80
CORRECTION_PITCH_DEG = 180
# 采用已验证的 Timer4/P7 脉宽控制，不使用 pyb.Servo.angle() 的相对角度映射。
SERVO_CHANNEL = 1
SERVO_PIN = "P7"
SERVO_MAX_ANGLE = 180
SERVO_REVERSE_DIRECTION = True
SERVO_MIN_PULSE_US = 500
SERVO_MAX_PULSE_US = 2500
PITCH_SETTLE_MS = 700
PITCH_READY_AT = 0
CAMERA_WARMUP_MS = 1500
CAMERA_STATE = "READY"
CAMERA_READY_AT = 0
CAMERA_FRAME_AFTER = 0

# 执行器初始化：激光关闭，俯仰云台固定 85 度。
# 激光控制引脚为 P5；若激光模块低电平有效，将 LASER_OFF_LEVEL 改为 1。
LASER_OFF_LEVEL = 0
laser = Pin("P5", Pin.OUT_PP)
laser.value(LASER_OFF_LEVEL)
light = None
if LIGHT_PIN is not None:
    light = Pin(LIGHT_PIN, Pin.OUT_PP)
    light.value(LIGHT_ON_LEVEL if LIGHT_ENABLE else LIGHT_OFF_LEVEL)
    print("LIGHT:", "ON" if LIGHT_ENABLE else "OFF", "pin:", LIGHT_PIN,
          "level:", light.value())
    if LIGHT_ENABLE:
        time.sleep_ms(LIGHT_WARMUP_MS)
pitch_timer = Timer(4, freq=50)
pitch_pwm = pitch_timer.channel(SERVO_CHANNEL, Timer.PWM, pin=Pin(SERVO_PIN))
def pitch_angle_to_us(angle):
    if not 0 <= angle <= SERVO_MAX_ANGLE:
        raise ValueError("pitch angle outside configured range")
    actual = SERVO_MAX_ANGLE - angle if SERVO_REVERSE_DIRECTION else angle
    return int(round(SERVO_MIN_PULSE_US +
                     (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) *
                     actual / SERVO_MAX_ANGLE))

def set_pitch(mechanical_deg):
    global PITCH_READY_AT
    pulse_us = pitch_angle_to_us(mechanical_deg)
    pitch_pwm.pulse_width(((pitch_timer.period() + 1) * pulse_us) // 20000)
    PITCH_READY_AT = time.ticks_add(time.ticks_ms(), PITCH_SETTLE_MS)
    actual = SERVO_MAX_ANGLE - mechanical_deg if SERVO_REVERSE_DIRECTION else mechanical_deg
    print("PITCH logical:", mechanical_deg, "actual:", actual,
          "pulse_us:", pulse_us, "pin:", SERVO_PIN)

set_pitch(CORRECTION_PITCH_DEG if DEBUG_CORRECTION else QR_PITCH_DEG)

# STM32 USART3 与 OpenMV UART 编号无关：PD8 -> P0 RX，PD9 <- P1 TX。
# UART3 的 P5 RX 与激光引脚冲突，因此使用 UART1。
uart = None
if not DEBUG_CORRECTION:
    uart = UART(1, 115200, bits=8, parity=None, stop=1,
                timeout=20, timeout_char=20, read_buf_len=256)
    print("MODE: INTEGRATION; UART1 115200 8N1")
else:
    print("MODE: DEBUG; UART disabled")
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
print("TRACK STAGE:", TRACK_STAGE)
print("TRACK:", TRACK_VERSION, TRACK_DETECT_MODE, "RGB565", TRACK_GRAY_LAB)
# 小尺寸二维码使用 VGA 保留更多黑白格细节。
sensor.set_framesize(sensor.VGA)
# 与 main_before.py 保持相同的相机方向：不做水平镜像，也不做垂直翻转。
sensor.set_hmirror(False)
sensor.set_vflip(False)
# 先在补光环境下完成自动曝光/增益/白平衡，再锁定参数。
sensor.set_auto_exposure(True)
sensor.set_auto_gain(True)
sensor.set_auto_whitebal(True)
sensor.skip_frames(time=1500)
sensor.set_auto_exposure(False)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.skip_frames(time=200)
clock = time.clock()

def prepare_pose_camera():
    global CAMERA_STATE, TRACK_THRESHOLD, L_ADAPT_VALID, DEBUG_VALID_STREAK
    CAMERA_STATE = "SETTLING"
    TRACK_THRESHOLD = TRACK_GRAY_LAB
    L_ADAPT_VALID = 0
    DEBUG_VALID_STREAK = 0
    print(time.ticks_ms(), "CAMERA: waiting for pitch; discard old frames")

def pose_camera_ready(captured_ms):
    global CAMERA_STATE, CAMERA_READY_AT, CAMERA_FRAME_AFTER
    now = time.ticks_ms()
    if time.ticks_diff(captured_ms, PITCH_READY_AT) < 0:
        return False
    if CAMERA_STATE == "SETTLING":
        sensor.set_auto_exposure(True)
        sensor.set_auto_gain(True)
        sensor.set_auto_whitebal(True)
        CAMERA_READY_AT = time.ticks_add(now, CAMERA_WARMUP_MS)
        CAMERA_STATE = "WARMING"
        print(now, "CAMERA: adapting exposure/gain/WB at new pose")
        return False
    if CAMERA_STATE == "WARMING":
        if time.ticks_diff(now, CAMERA_READY_AT) < 0:
            return False
        sensor.set_auto_exposure(False)
        sensor.set_auto_gain(False)
        sensor.set_auto_whitebal(False)
        CAMERA_FRAME_AFTER = time.ticks_add(time.ticks_ms(), 200)
        CAMERA_STATE = "FLUSHING"
        return False
    if CAMERA_STATE == "FLUSHING":
        if time.ticks_diff(captured_ms, CAMERA_FRAME_AFTER) < 0:
            return False
        CAMERA_STATE = "READY"
        print(now, "CAMERA READY: exposure_us:", sensor.get_exposure_us(),
              "gain_db:", sensor.get_gain_db(), "rgb_gain_db:", sensor.get_rgb_gain_db())
    return CAMERA_STATE == "READY"

TRACK_ROI = (0, 120, 640, 360)
TRACK_MIN_BLOB_PIXELS = 50
TRACK_CENTER_POINTS = []
TRACK_THRESHOLD = TRACK_GRAY_LAB
TRACK_REASON = "WAIT"
L_ADAPT_VALID = 0

def fit(points):
    my = sum(y for y, x in points) / len(points)
    mx = sum(x for y, x in points) / len(points)
    den = sum((y - my) ** 2 for y, x in points)
    if den <= 0: return 0.0, mx
    m = sum((y - my) * (x - mx) for y, x in points) / den
    return m, mx - m * my


def adaptive_center_error(img):
    global TRACK_SAMPLES, TRACK_REASON, TRACK_CENTER_POINTS
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
    TRACK_SAMPLES = samples
    TRACK_CENTER_POINTS = [((l+r)/2.0, y) for y, l, r in samples]
    if len(samples) < DEBUG_MIN_SAMPLES:
        TRACK_REASON = "FEW SAMPLES"; return None
    if samples[-1][0] - samples[0][0] < .15*h:
        TRACK_REASON = "SHORT SPAN"; return None
    centers = [(y, (l+r)/2.0) for y, l, r in samples]
    m, c = fit(centers)
    if max(abs(x - (m*y + c)) for y, x in centers) > DEBUG_MAX_FIT_RESIDUAL_QVGA_PX*sx:
        TRACK_REASON = "BAD FIT"; return None
    left_m, left_c = fit([(y, l) for y, l, r in samples])
    right_m, right_c = fit([(y, r) for y, l, r in samples])
    control_y = int(h * TRACK_CONTROL_ROW) + band // 2
    center = m * control_y + c
    error = LATERAL_SIGN * (reference - center)
    heading = HEADING_SIGN * math.degrees(math.atan(m)) - HEADING_ZERO_DEG
    TRACK_REASON = "VALID"
    left = TrackEdge((int(left_m*samples[0][0]+left_c), samples[0][0],
                      int(left_m*samples[-1][0]+left_c), samples[-1][0]))
    right = TrackEdge((int(right_m*samples[0][0]+right_c), samples[0][0],
                       int(right_m*samples[-1][0]+right_c), samples[-1][0]))
    return error, heading, left, right



CORRECTING = False
LAST_VERR_MS = 0
LAST_VISION_LOG_MS = 0
VISION_LOG_PERIOD_MS = 1000
# 当前 180/220 仅是旧程序的估算值。按实际高度、俯角标定后才能启用。
CORRECTION_CALIBRATED = False
MM_PER_QVGA_PIXEL = 180.0 / 220.0
# 正常未镜像前视：车在路线右侧时，路线中心落在画面左侧。
LATERAL_SIGN = 1.0
# 车头左偏时路线下端相对上端偏左；实机左偏测试后核对此符号。
HEADING_SIGN = -1.0
CORRECTION_PERIOD_MS = 70
RX_WORD = bytearray()
COMMANDS = (b"qr", b"rectify", b"finishrectify", b"correct", b"shortcorrect", b"longcorrect", b"stopcorrect", b"finishcorrect",
            b"route_reset", b"finishstart", b"finishqr", b"finishglobule", b"globule")
RX_STATE = "BOUNDARY"
RX_LINE = bytearray()
RX_LIMIT = 256
# 留出 STM32 30 秒超时之前的回传余量。
QR_TIMEOUT_MS = 28000
QR_STATE = "WAIT"
STM32_READY_SEEN = False
MISSION_STARTED = False
QR_STARTED = 0
QR_RESULT = None
QR_CANDIDATE = None
QR_HITS = 0
# 比赛流程要求首次有效解码后立即通知 STM32 平移；不再等待第二帧确认。
QR_CONFIRM_FRAMES = 1
QR_CONFIRM_GAP_MS = 700
QR_LAST_HIT = 0
STATUS = "WAIT QR CMD"
STATUS_COLOR = (255, 255, 0)

def fail_task(message):
    global QR_STATE, STATUS, STATUS_COLOR, CORRECTING
    CORRECTING = False
    QR_STATE = "FAILED"
    STATUS = message
    STATUS_COLOR = (255, 0, 0)
    print(time.ticks_ms(), message)


def request_qr():
    global QR_STATE, QR_STARTED, STATUS, STATUS_COLOR, CORRECTING, MISSION_STARTED
    if QR_STATE != "WAIT":
        print(time.ticks_ms(), "duplicate qr ignored:", QR_STATE)
        return
    CORRECTING = False
    MISSION_STARTED = True
    set_pitch(QR_PITCH_DEG)
    prepare_pose_camera()
    QR_STARTED = time.ticks_ms()
    QR_STATE = "SCANNING"
    STATUS = "QR SCANNING"
    STATUS_COLOR = (0, 255, 255)
    print(QR_STARTED, "QR CMD RECEIVED / SCANNING")


def handle_status(data):
    global STM32_READY_SEEN
    try:
        line = data.decode("ascii")
    except UnicodeError:
        return
    print(time.ticks_ms(), "STM32 STATUS:", line)
    parts = line.split()
    if len(parts) >= 2 and parts[0].isdigit():
        state = parts[1].upper()
        if state == "READY":
            if not MISSION_STARTED and QR_STATE == "WAIT":
                STM32_READY_SEEN = True
                print(time.ticks_ms(), "STM32 INITIAL READY: waiting for qr/rectify")
                return
            STM32_READY_SEEN = True
            fail_task("STM32 RESTART - REINITIALIZE BOTH")
        elif state in ("ERROR", "STOPPED"):
            fail_task("STM32 FAULT / RESTART")


def handle_command(word):
    global CORRECTING, STATUS, STATUS_COLOR, MISSION_STARTED
    print(time.ticks_ms(), "COMMAND PARSED:", repr(word),
          "QR_STATE:", QR_STATE, "CORRECTING:", CORRECTING)
    if word == b"qr":
        request_qr()
    elif word in (b"rectify", b"correct", b"shortcorrect", b"longcorrect"):
        if QR_STATE in ("FAILED", "SCANNING"):
            print(time.ticks_ms(), "COMMAND REJECTED:", repr(word),
                  "reason: QR state", QR_STATE)
            return
        # 三个启动命令均使用同一检测算法，不等待静止十帧。
        MISSION_STARTED = True
        if not CORRECTING:
            set_pitch(CORRECTION_PITCH_DEG)
            prepare_pose_camera()
        CORRECTING = True
        # 支持第一次平移之后及后续路段再次开启，不依赖二维码轮次。
        STATUS, STATUS_COLOR = "STATIC ALIGN", (255, 255, 0)
        print(time.ticks_ms(), "CORRECTION START; calibrated:", CORRECTION_CALIBRATED)
    elif word in (b"finishrectify", b"stopcorrect", b"finishcorrect", b"route_reset"):
        was_correcting = CORRECTING
        CORRECTING = False
        if was_correcting:
            write_frame(b"VERR 0 0.0 0.0 0 0\r\n")
        if word == b"route_reset":
            # 本地不自行重置二维码轮次，双方重新初始化后再做整轮测试。
            fail_task("ROUTE RESET - RESTART BOTH")
        elif QR_STATE != "FAILED":
            STATUS, STATUS_COLOR = "CORRECTION STOPPED", (255, 255, 0)


def feed_rx(data):
    """固定命令收齐即处理；状态/未知文本隔离到行尾。"""
    global RX_STATE, RX_LINE, RX_WORD
    for byte in data:
        if byte == 0 and RX_STATE == "BOUNDARY":
            # 启动时已观察到孤立 NUL；不能让它吞掉下一条命令。
            print(time.ticks_ms(), "RX IGNORED: isolated NUL at boundary")
            continue
        if byte in (10, 13):
            if RX_STATE == "LINE":
                handle_status(RX_LINE)
            elif RX_STATE in ("DROP", "WORD"):
                print(time.ticks_ms(), "RX LINE DISCARDED; state:", RX_STATE,
                      "partial:", repr(bytes(RX_WORD)))
            RX_STATE = "BOUNDARY"
            RX_LINE, RX_WORD = bytearray(), bytearray()
        elif RX_STATE == "BOUNDARY" and (byte == 35 or 48 <= byte <= 57):
            RX_STATE = "LINE"
            RX_LINE = bytearray([byte])
        elif RX_STATE in ("BOUNDARY", "WORD"):
            RX_WORD.append(byte)
            word = bytes(RX_WORD)
            if word in COMMANDS:
                RX_STATE, RX_WORD = "BOUNDARY", bytearray()
                handle_command(word)
            elif any(command.startswith(word) for command in COMMANDS):
                RX_STATE = "WORD"
            else:
                print(time.ticks_ms(), "RX UNKNOWN PREFIX:", repr(word),
                      "discarding until CR/LF")
                RX_STATE, RX_WORD = "DROP", bytearray()
        elif RX_STATE == "LINE":
            if len(RX_LINE) < RX_LIMIT:
                RX_LINE.append(byte)
            else:
                fail_task("RX STATUS TOO LONG")
                RX_LINE = bytearray()
                RX_STATE = "DROP"


def poll_uart():
    if uart is None:
        return
    # 每轮工作有上限，防止串口连续输出阻塞图像处理。
    for _ in range(4):
        if not uart.any():
            break
        data = uart.read(256)
        if data:
            print(time.ticks_ms(), "UART RAW:", repr(data), "RX_STATE:", RX_STATE)
            feed_rx(data)
            print(time.ticks_ms(), "RX AFTER:", RX_STATE, "CORRECTING:", CORRECTING)

def line_angle(line):
    # 线段端点方向统一为从上到下，避免 0/180 度跨界平均错误。
    x1, y1, x2, y2 = line.line()
    if y2 < y1:
        x1, y1, x2, y2 = x2, y2, x1, y1
    return math.degrees(math.atan2(x2 - x1, y2 - y1))

class TrackEdge:
    def __init__(self, points):
        self.points = points
    def line(self):
        return self.points


TRACK_SAMPLES = []

def detect_track_raw(img):
    global TRACK_SAMPLES
    if TRACK_DETECT_MODE == "ADAPTIVE_CENTER":
        return adaptive_center_error(img)
    width, height = img.width(), img.height()
    sx = width / 320.0
    reference = width / 2.0 + TRACK_CENTER_OFFSET_QVGA_PX * sx
    samples = []
    previous = reference
    band = max(3, int(height * 0.0125))
    # 扫描灰色路面横截面，不寻找横条上的直线。
    for fraction in TRACK_SCAN_ROWS:
        y = int(height * fraction)
        if TRACK_DETECT_MODE == "GRADIENT":
            step = max(2, int(GRADIENT_STEP * sx))
            band_y = min(height - 1, y + band // 2)
            values = []
            for px in range(0, width, step):
                r, g, b = img.get_pixel(px, band_y)
                values.append((0.299 * r + 0.587 * g + 0.114 * b, px))
            gradients = []
            for i in range(1, len(values)):
                gradients.append((abs(values[i][0] - values[i - 1][0]),
                                  values[i - 1][1], values[i][1]))
            left_edges = [item for item in gradients
                          if width * 0.08 < item[1] < reference - width * 0.05]
            right_edges = [item for item in gradients
                           if reference + width * 0.05 < item[2] < width * 0.92]
            if left_edges and right_edges:
                left_edge = max(left_edges, key=lambda item: item[0])
                right_edge = max(right_edges, key=lambda item: item[0])
                if (left_edge[0] >= GRADIENT_MIN_CONTRAST and
                        right_edge[0] >= GRADIENT_MIN_CONTRAST):
                    left = left_edge[2]
                    right = right_edge[1]
                    if right - left > width * 0.20:
                        midpoint = (left + right) / 2.0
                        if abs(midpoint - previous) <= width * 0.25:
                            previous = midpoint
                            samples.append((y + band // 2, left, right))
            continue
        candidates = []
        for blob in img.find_blobs([TRACK_GRAY_LAB], roi=(0, y, width, band),
                                   x_stride=2, y_stride=1,
                                   pixels_threshold=max(10, int(15 * sx)),
                                   area_threshold=max(10, int(20 * sx)), merge=False):
            x, _, w, _ = blob.rect()
            # 两侧都须留有可见背景；整幅灰色/仅一侧边界不作为有效轨道。
            if w < width * 0.12 or w > width * 0.95 or x <= 2 or x + w >= width - 2:
                continue
            if blob.pixels() / float(w * band) < 0.35:
                continue
            midpoint = x + w / 2.0
            if abs(midpoint - previous) > width * 0.20:
                continue
            candidates.append((abs(midpoint - previous), x, x + w - 1))
        if candidates:
            _, left, right = min(candidates)
            previous = (left + right) / 2.0
            samples.append((y + band // 2, left, right))
    TRACK_SAMPLES = samples
    if len(samples) < DEBUG_MIN_SAMPLES or samples[-1][0] - samples[0][0] < height * 0.35:
        return None
    def fit(points):
        my = sum(y for y, x in points) / len(points)
        mx = sum(x for y, x in points) / len(points)
        denominator = sum((y - my) ** 2 for y, x in points)
        slope = sum((y - my) * (x - mx) for y, x in points) / denominator
        return slope, mx - slope * my
    centers = [(y, (left + right) / 2.0) for y, left, right in samples]
    slope, intercept = fit(centers)
    # 先拟合中心线，再剔除单个明显偏离点，避免一条误检边界拉歪整条线。
    residuals = [abs(x - (slope * y + intercept)) for y, x in centers]
    worst = max(range(len(residuals)), key=lambda i: residuals[i])
    if residuals[worst] > DEBUG_OUTLIER_RESIDUAL_QVGA_PX * sx and len(centers) > DEBUG_MIN_SAMPLES:
        centers = [item for i, item in enumerate(centers) if i != worst]
        samples = [item for i, item in enumerate(samples) if i != worst]
        slope, intercept = fit(centers)
    widths = [right - left for _, left, right in samples]
    median_width = sorted(widths)[len(widths) // 2]
    if median_width <= width * 0.20 or any(abs(item - median_width) > median_width * DEBUG_WIDTH_TOLERANCE
                                            for item in widths):
        return None
    if max(abs(x - (slope * y + intercept)) for y, x in centers) > DEBUG_MAX_FIT_RESIDUAL_QVGA_PX * sx:
        return None
    edges = []
    for index in (1, 2):
        m, c = fit([(row[0], row[index]) for row in samples])
        edges.append(TrackEdge((int(m * samples[0][0] + c), samples[0][0],
                                int(m * (height - 1) + c), height - 1)))
    center_bottom = slope * (height - 1) + intercept
    err = LATERAL_SIGN * (reference - center_bottom) * MM_PER_QVGA_PIXEL / sx
    yaw = HEADING_SIGN * math.degrees(math.atan(slope)) - HEADING_ZERO_DEG
    return err, yaw, edges[0], edges[1]


def track_error(img):
    # 长短轨道使用相同的当前帧检测，无旧结果保持或十帧静止门槛。
    return detect_track_raw(img)


def parse_qr(payload):
    # 接收任意三个 ASCII 数字，保留前导零及各位任务编号。
    # 编号的颜色/形状含义由比赛规则决定，此处不限定为 1/2/3。
    raw = payload.strip()
    if len(raw) != 3 or any(c not in "0123456789" for c in raw):
        return None
    return {"raw": raw, "ball_color": int(raw[0]),
            "target_color": int(raw[1]), "hostage_shape": int(raw[2])}


def check_timeout():
    if QR_STATE == "SCANNING" and time.ticks_diff(time.ticks_ms(), QR_STARTED) >= QR_TIMEOUT_MS:
        fail_task("QR TIMEOUT - NO finishqr")


def process_qr(img):
    global QR_CANDIDATE, QR_HITS, QR_LAST_HIT, QR_RESULT
    global QR_STATE, STATUS, STATUS_COLOR
    check_timeout()
    if QR_STATE != "SCANNING":
        return
    tasks = []
    for qr in img.find_qrcodes():
        img.draw_rectangle(qr.rect(), color=255, thickness=2)
        payload = qr.payload()
        print(time.ticks_ms(), "QR DECODED:", repr(payload))
        task = parse_qr(payload)
        if task is not None:
            tasks.append(task)
        else:
            print(time.ticks_ms(), "QR REJECTED: expected three digits")
    now = time.ticks_ms()
    if len(tasks) != 1:
        QR_CANDIDATE, QR_HITS = None, 0
        return
    task = tasks[0]
    if task["raw"] != QR_CANDIDATE or time.ticks_diff(now, QR_LAST_HIT) > QR_CONFIRM_GAP_MS:
        QR_CANDIDATE, QR_HITS = task["raw"], 0
    QR_LAST_HIT = now
    QR_HITS += 1
    if QR_HITS < QR_CONFIRM_FRAMES:
        return
    # 检测耗时期间仍可能收到故障，发送前再检查一次。
    poll_uart()
    check_timeout()
    if QR_STATE != "SCANNING":
        return
    QR_RESULT = task  # 本轮结果保存在内存，不发二维码内容。
    print(time.ticks_ms(), "QR CONFIRMED:", QR_RESULT)
    frame = b"finishqr\r\n"
    offset = 0
    started = time.ticks_ms()
    while offset < len(frame):
        poll_uart()
        check_timeout()
        if QR_STATE != "SCANNING":
            return
        if time.ticks_diff(time.ticks_ms(), started) >= 100:
            fail_task("finishqr TX TIMEOUT")
            return
        sent = uart.write(frame[offset:])
        print(time.ticks_ms(), "UART TX:", repr(frame[offset:]), "written:", sent)
        if not sent:
            fail_task("finishqr TX FAILED")
            return
        offset += sent
    QR_STATE = "DONE"
    STATUS = "QR OK - finishqr SENT"
    STATUS_COLOR = (0, 255, 0)
    # STM32 不单独回 ACK；本轮不再启动第二次识别。


def write_frame(frame):
    if uart is None:
        return False
    offset = 0
    started = time.ticks_ms()
    while offset < len(frame):
        if time.ticks_diff(time.ticks_ms(), started) >= 100:
            fail_task("VERR TX TIMEOUT")
            return False
        sent = uart.write(frame[offset:])
        if not sent:
            fail_task("VERR TX FAILED")
            return False
        offset += sent
    return True


def send_verr(img, captured_ms):
    global LAST_VERR_MS, LAST_VISION_LOG_MS
    if not CORRECTING or CAMERA_STATE != "READY" or time.ticks_diff(captured_ms, PITCH_READY_AT) < 0:
        return
    now = time.ticks_ms()
    if time.ticks_diff(now, LAST_VERR_MS) < CORRECTION_PERIOD_MS:
        return
    result = track_error(img)
    # 边线检测期间仍处理停止/故障，避免发出迟到的有效纠偏数据。
    poll_uart()
    if not CORRECTING:
        return
    age_ms = max(0, time.ticks_diff(time.ticks_ms(), captured_ms))
    frame = b"VERR 0 0.0 0.0 0 0\r\n"
    reject_reason = TRACK_REASON if result is None else "VALID"
    quality = 0
    # 联调画面也显示与 DEBUG 模式相同的参考线和轨道中心。
    width, height = img.width(), img.height()
    sx = width / 320.0
    top = int(70 * height / 240.0)
    reference = width / 2.0 + TRACK_CENTER_OFFSET_QVGA_PX * sx
    img.draw_rectangle((0, top, width, height - top), color=255)
    img.draw_line(int(reference), top, int(reference), height - 1,
                  color=255, thickness=CENTER_LINE_THICKNESS)
    tolerance_px = DEBUG_CENTER_TOL_QVGA_PX * sx
    for bound in (reference - tolerance_px, reference + tolerance_px):
        if 0 <= bound < width:
            img.draw_line(int(bound), top, int(bound), height - 1,
                          color=(255, 180, 0), thickness=1)
    if result is not None:
        ey, etheta, left, right = result
        # 有效范围按当前 STM32 文档：有限数值、|X|<=320、|T|<=15。
        # |X|<=20、|T|<=3° 是 STM32 的完成条件，不是 OpenMV 的有效帧条件。
        # 例如 X=50、T=6 仍应发送 VERR 1，供 STM32 进行纠偏。
        in_range = (math.isfinite(ey) and math.isfinite(etheta) and
                    abs(ey) <= 320 and abs(etheta) <= 15)
        quality = 80 if in_range else 0
        if quality < 50:
            reject_reason = "X/T OUT OF SEND RANGE"
        elif age_ms > 150:
            reject_reason = "FRAME TOO OLD"
        elif TRACK_DETECT_MODE != "ADAPTIVE_CENTER" and not CORRECTION_CALIBRATED:
            reject_reason = "NOT CALIBRATED"
        if (TRACK_DETECT_MODE == "ADAPTIVE_CENTER" or CORRECTION_CALIBRATED) and quality >= 50 and age_ms <= 150:
            frame = ("VERR 1 {:.1f} {:.1f} {} {}\r\n".format(
                ey, etheta, quality, age_ms)).encode("ascii")
        img.draw_line(left.line(), color=255, thickness=2)
        img.draw_line(right.line(), color=255, thickness=2)
        def bottom_x(line):
            x1, y1, x2, y2 = line.line()
            control_y = int(height * TRACK_CONTROL_ROW) + max(4, int(height * .018)) // 2
            if y2 == y1:
                return (x1 + x2) / 2.0
            return x1 + (x2 - x1) * (control_y - y1) / (y2 - y1)
        lane_center = (bottom_x(left) + bottom_x(right)) / 2.0
        if 0 <= lane_center < width:
            img.draw_cross(int(lane_center),
                           int(height * TRACK_CONTROL_ROW) + max(4, int(height * .018)) // 2,
                           color=255)
        img.draw_string(2, 16, "VERR X:{:.1f} T:{:.1f}".format(ey, etheta),
                        color=255)
        img.draw_string(2, 34, "CENTER:{} HEADING:{}".format(
            "OK" if abs(ey) <= DEBUG_CENTER_TOL_QVGA_PX * sx else "OFF",
            "OK" if abs(etheta) <= DEBUG_HEADING_TOL_DEG else "OFF"),
            color=255 if abs(ey) <= DEBUG_CENTER_TOL_QVGA_PX * sx and
            abs(etheta) <= DEBUG_HEADING_TOL_DEG else (255, 150, 0))
    write_frame(frame)
    if SEND_TRACK_POSITION:
        reference = img.width()/2.0 + TRACK_CENTER_OFFSET_QVGA_PX*img.width()/320.0
        if result is not None and frame.startswith(b"VERR 1 "):
            center = reference - result[0]/LATERAL_SIGN
            position = "TRACK 1 {:.1f} {:.1f} {:.1f} {}\r\n".format(
                center, reference, result[1], age_ms).encode("ascii")
        else:
            position = b"TRACK 0 0 0 0 0\r\n"
        write_frame(position)
    tx_ms = time.ticks_ms()
    interval_ms = time.ticks_diff(tx_ms, LAST_VERR_MS) if LAST_VERR_MS else 0
    LAST_VERR_MS = tx_ms
    if time.ticks_diff(tx_ms, LAST_VISION_LOG_MS) >= VISION_LOG_PERIOD_MS:
        print(tx_ms, "VERR TX:", repr(frame), "interval_ms:", interval_ms,
              "VISION:", reject_reason, "samples:", len(TRACK_SAMPLES))
        LAST_VISION_LOG_MS = tx_ms



def calibrate_reference(result, width):
    global CALIBRATION_DONE
    if CALIBRATION_DONE or result is None:
        if result is None:
            CALIBRATION_POINTS[:] = []
        return
    reference = width/2.0 + TRACK_CENTER_OFFSET_QVGA_PX*width/320.0
    center = reference - result[0]/LATERAL_SIGN
    raw_angle = result[1] + HEADING_ZERO_DEG
    CALIBRATION_POINTS.append((center, raw_angle))
    if len(CALIBRATION_POINTS) < CALIBRATION_FRAMES:
        return
    xs = sorted(p[0] for p in CALIBRATION_POINTS)
    angles = sorted(p[1] for p in CALIBRATION_POINTS)
    # 拒绝车体移动或检测波动大的校准，不输出错误零点。
    if xs[-1]-xs[0] > 10*width/320.0 or angles[-1]-angles[0] > 4:
        print("CALIBRATION UNSTABLE: hold vehicle still; retrying")
        CALIBRATION_POINTS[:] = []
        return
    center = xs[len(xs)//2]
    angle = angles[len(angles)//2]
    print("CALIBRATION DONE: reference_x=", center)
    print("TRACK_CENTER_OFFSET_QVGA_PX =", (center-width/2.0)*320.0/width)
    print("HEADING_ZERO_DEG =", angle)
    print("Copy these parameters into this file, then use INTEGRATION.")
    CALIBRATION_DONE = True


def debug_correction(img, captured_ms):
    global LAST_DEBUG_MS, DEBUG_VALID_STREAK
    # 先检测，再叠加标记，避免画出来的线影响检测。
    result = track_error(img)
    if OPERATION_MODE == "CALIBRATE":
        calibrate_reference(result, img.width())
    width, height = img.width(), img.height()
    sx = width / 320.0
    ref = width / 2.0 + TRACK_CENTER_OFFSET_QVGA_PX * sx
    top = int(70 * height / 240.0)
    img.draw_rectangle((0, top, width, height - top), color=255)
    img.draw_line(int(ref), top, int(ref), height - 1, color=255,
                  thickness=CENTER_LINE_THICKNESS)
    tolerance_px = DEBUG_CENTER_TOL_QVGA_PX * sx
    for bound in (ref - tolerance_px, ref + tolerance_px):
        if 0 <= bound < width:
            img.draw_line(int(bound), top, int(bound), height - 1,
                          color=(255, 180, 0), thickness=1)
    now = time.ticks_ms()
    img.draw_string(2, 34, "LAB L:%d..%d %s" % (TRACK_THRESHOLD[0],TRACK_THRESHOLD[1],TRACK_REASON), color=255)
    for y, left, right in TRACK_SAMPLES:
        img.draw_line(left, y, right, y, color=255)
    if result is None:
        DEBUG_VALID_STREAK = 0
        img.draw_string(2, 2, "DEBUG: INVALID (%d/%d SAMPLES)" %
                        (len(TRACK_SAMPLES), DEBUG_MIN_SAMPLES), color=255)
        message = "INVALID samples=%d/%d" % (len(TRACK_SAMPLES), DEBUG_MIN_SAMPLES)
    else:
        DEBUG_VALID_STREAK += 1
        _, yaw, left, right = result
        def bottom_x(line):
            x1, y1, x2, y2 = line.line()
            control_y = int(height*TRACK_CONTROL_ROW) + max(4,int(height*.018))//2
            return x1 + (x2 - x1) * (control_y - y1) / (y2 - y1)
        lane_center = (bottom_x(left) + bottom_x(right)) / 2.0
        # 显示当前 VGA 图像像素，不使用毫米估算比例。
        error_px = result[0] if TRACK_DETECT_MODE == "ADAPTIVE_CENTER" else LATERAL_SIGN * (ref - lane_center)
        lane_center = ref - error_px/LATERAL_SIGN
        center_ok = abs(error_px) <= DEBUG_CENTER_TOL_QVGA_PX * sx
        heading_ok = abs(yaw) <= DEBUG_HEADING_TOL_DEG
        img.draw_line(left.line(), color=255, thickness=2)
        img.draw_line(right.line(), color=255, thickness=2)
        if 0 <= lane_center < width:
            img.draw_cross(int(lane_center), int(height*TRACK_CONTROL_ROW)+max(4,int(height*.018))//2, color=255)
        stable = DEBUG_VALID_STREAK >= DEBUG_VALID_STREAK_REQUIRED
        img.draw_string(2, 2, "DEBUG X:{:.1f}px T:{:.1f}deg {}".format(
            error_px, yaw, "STABLE" if stable else "WAIT"), color=255)
        img.draw_string(2, 18, "CENTER:{} HEADING:{}".format(
            "OK" if center_ok else "OFF", "OK" if heading_ok else "OFF"),
            color=255 if center_ok and heading_ok else (255, 150, 0))
        message = "X_px={:.1f} T_deg={:.1f} center_ok={} heading_ok={} stable={}".format(
            error_px, yaw, center_ok, heading_ok, stable)
    if time.ticks_diff(now, LAST_DEBUG_MS) >= DEBUG_PRINT_MS:
        print(now, "CORRECTION DEBUG:", message, "processing_ms:",
              time.ticks_diff(now, captured_ms))
        LAST_DEBUG_MS = now


while True:
    poll_uart()
    check_timeout()
    clock.tick()
    captured_ms = time.ticks_ms()
    img = sensor.snapshot()
    # Snapshot 后处理命令，再检查姿态和相机；切换前的图像不得发送。
    poll_uart()
    if not pose_camera_ready(captured_ms):
        img.draw_string(2, 2, "CAMERA " + CAMERA_STATE, color=255)
        continue
    if DEBUG_CORRECTION:
        debug_correction(img, captured_ms)
        continue
    if QR_STATE == "SCANNING":
        process_qr(img)
    elif CORRECTING:
        send_verr(img, captured_ms)
    img.draw_string(2, 2, STATUS, color=max(STATUS_COLOR))
