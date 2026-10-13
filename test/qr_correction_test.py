# OpenMV + STM32 联调测试
# STM32 -> OpenMV: qr
# OpenMV -> STM32: finishqr（仅二维码确认成功后发送）
# 纯命令词协议，不主动发送周期数据。

import sensor, image, time, math
from pyb import UART, Servo, Pin

# 执行器初始化：激光关闭，俯仰云台固定 85 度。
# 激光控制引脚为 P5；若激光模块低电平有效，将 LASER_OFF_LEVEL 改为 1。
LASER_OFF_LEVEL = 0
laser = Pin("P5", Pin.OUT_PP)
laser.value(LASER_OFF_LEVEL)
pitch_servo = Servo(1)
pitch_servo.angle(85)

# STM32 USART3 与 OpenMV UART 编号无关：PD8 -> P0 RX，PD9 <- P1 TX。
# UART3 的 P5 RX 与激光引脚冲突，因此使用 UART1。
uart = UART(1, 115200, bits=8, parity=None, stop=1,
            timeout=20, timeout_char=20, read_buf_len=256)
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
# 小尺寸二维码使用 VGA 保留更多黑白格细节。
sensor.set_framesize(sensor.VGA)
# 与 main_before.py 保持相同的相机方向：不做水平镜像，也不做垂直翻转。
sensor.set_hmirror(False)
sensor.set_vflip(False)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
clock = time.clock()

RX_STATE = "BOUNDARY"
RX_LINE = bytearray()
RX_LIMIT = 256
# 留出 STM32 30 秒超时之前的回传余量。
QR_TIMEOUT_MS = 28000
QR_STATE = "WAIT"
QR_STARTED = 0
QR_RESULT = None
QR_CANDIDATE = None
QR_HITS = 0
QR_CONFIRM_FRAMES = 2
QR_CONFIRM_GAP_MS = 700
QR_LAST_HIT = 0
STATUS = "WAIT QR CMD"
STATUS_COLOR = (255, 255, 0)

def fail_task(message):
    global QR_STATE, STATUS, STATUS_COLOR
    QR_STATE = "FAILED"
    STATUS = message
    STATUS_COLOR = (255, 0, 0)
    print(time.ticks_ms(), message)


def request_qr():
    global QR_STATE, QR_STARTED, STATUS, STATUS_COLOR
    if QR_STATE != "WAIT":
        print(time.ticks_ms(), "duplicate qr ignored:", QR_STATE)
        return
    QR_STARTED = time.ticks_ms()
    QR_STATE = "SCANNING"
    STATUS = "QR SCANNING"
    STATUS_COLOR = (0, 255, 255)
    print(QR_STARTED, "QR CMD RECEIVED / SCANNING")


def handle_status(data):
    try:
        line = data.decode("ascii")
    except UnicodeError:
        return
    print(time.ticks_ms(), "STM32 STATUS:", line)
    parts = line.split()
    if len(parts) >= 2 and parts[0].isdigit():
        if parts[1].upper() in ("ERROR", "STOPPED", "READY"):
            fail_task("STM32 FAULT / RESTART")


def feed_rx(data):
    """固定 qr 收齐即处理；状态/未知文本始终隔离到行尾。"""
    global RX_STATE, RX_LINE
    for byte in data:
        if byte in (10, 13):
            if RX_STATE == "LINE":
                handle_status(RX_LINE)
            # q 后出现换行是非法拆分，不能与下一行的 r 拼接。
            RX_STATE = "BOUNDARY"
            RX_LINE = bytearray()
        elif RX_STATE == "BOUNDARY":
            if byte == 113:  # q
                RX_STATE = "Q"
            elif byte == 35 or 48 <= byte <= 57:  # # 或数字状态行
                RX_STATE = "LINE"
                RX_LINE = bytearray([byte])
            else:
                RX_STATE = "DROP"
        elif RX_STATE == "Q":
            if byte == 114:  # r
                RX_STATE = "BOUNDARY"
                request_qr()
            else:
                RX_STATE = "DROP"
        elif RX_STATE == "LINE":
            if len(RX_LINE) < RX_LIMIT:
                RX_LINE.append(byte)
            else:
                fail_task("RX STATUS TOO LONG")
                RX_LINE = bytearray()
                RX_STATE = "DROP"


def poll_uart():
    # 每轮工作有上限，防止串口连续输出阻塞图像处理。
    for _ in range(4):
        if not uart.any():
            break
        data = uart.read(256)
        if data:
            print(time.ticks_ms(), "UART RAW:", repr(data))
            feed_rx(data)

def line_angle(line):
    a = line.theta()
    return a - 180 if a > 90 else a

def track_error(img):
    lines = img.find_lines(roi=(0, 70, 320, 170), threshold=1000,
                            theta_margin=15, rho_margin=15)
    left = right = None
    ls = rs = 0
    for ln in lines:
        x1, y1, x2, y2 = ln.line()
        length = math.sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1))
        a = line_angle(ln)
        if length < 35 or abs(a) < 20:
            continue
        xb = (x1+x2)/2 if y1 == y2 else x1+(x2-x1)*(239-y1)/(y2-y1)
        score = length * abs(math.sin(math.radians(a)))
        if xb < 160 and score > ls:
            left, ls = ln, score
        elif xb >= 160 and score > rs:
            right, rs = ln, score
    if left is None or right is None:
        return None
    def bx(ln):
        x1,y1,x2,y2 = ln.line()
        return (x1+x2)/2 if y1 == y2 else x1+(x2-x1)*(239-y1)/(y2-y1)
    err = ((bx(left)+bx(right))/2 - 160) * (180.0/220.0)
    yaw = (line_angle(left)+line_angle(right))/2
    return err, yaw, left, right

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
        img.draw_rectangle(qr.rect(), color=(255, 0, 0), thickness=2)
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


while True:
    poll_uart()
    check_timeout()
    clock.tick()
    img = sensor.snapshot()
    poll_uart()
    if QR_STATE == "SCANNING":
        process_qr(img)
    img.draw_string(2, 2, STATUS, color=STATUS_COLOR)
