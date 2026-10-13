# OpenMV ball-only handshake. STM32 owns yaw, rack, lift and chassis.
# TEST simulates STM32 completion locally; LINK receives real UART commands.
import sensor
import time
import pyb

RUN_MODE = "TEST"  # TEST / LINK
TASK_CODE = "222"
UART_ID = 1
UART_BAUD = 115200
OBJECT_ROI = (0, 180, 320, 60)
CONFIRM_HITS = 3
SCAN_TIMEOUT_MS = 2500
SCAN_SETTLE_MS = 500
OPEN_WAIT_MS = 1000
GRAB_OPEN_MS = 3000
CLOSE_WAIT_MS = 1000
RELEASE_WAIT_MS = 1000
SERVO_MAX_ANGLE = 270  # 200-degree opening requires a 270-degree claw servo
REVERSE_DIRECTION = False  # same claw mapping as main.py
TEST_MECHANISM_WAIT_MS = 3000  # simulated STM32 operations; TEST chassis stays still
TEST_REPEAT = False
COLORS = {1: (59, 0, 22, 127, -128, 127),
          2: (46, 65, -44, -18, -128, 127),
          3: (5, 40, -10, 35, -60, -15)}


def angle_to_us(angle):
    if not 0 <= angle <= SERVO_MAX_ANGLE:
        raise ValueError("claw angle outside configured range")
    actual = SERVO_MAX_ANGLE - angle if REVERSE_DIRECTION else angle
    return int(round(500 + 2000 * actual / SERVO_MAX_ANGLE))


def value(obj, name):
    v = getattr(obj, name)
    return v() if callable(v) else v


def target_blob(img, wanted):
    blobs = img.find_blobs([COLORS[wanted]], roi=OBJECT_ROI,
                           pixels_threshold=60, area_threshold=100, merge=False)
    valid = []
    for b in blobs:
        x, y, w, h = value(b, "rect")
        if w >= 5 and h >= 3 and value(b, "pixels") / float(w*h) >= 0.2:
            valid.append(b)
    return max(valid, key=lambda b: value(b, "pixels")) if valid else None


class Lines:
    def __init__(self):
        self.buf = bytearray()
        self.drop = False

    def feed(self, data):
        out = []
        for b in data:
            if b in (10, 13):
                if self.buf and not self.drop:
                    try:
                        out.append(self.buf.decode("ascii").strip().lower())
                    except UnicodeError:
                        pass
                self.buf = bytearray()
                self.drop = False
            elif not self.drop:
                if len(self.buf) >= 80:
                    self.buf = bytearray()
                    self.drop = True
                else:
                    self.buf.append(b)
        return out


def write_all(uart, data):
    off, start = 0, time.ticks_ms()
    while off < len(data):
        n = uart.write(data[off:])
        if n:
            off += n
        if off < len(data) and time.ticks_diff(time.ticks_ms(), start) > 100:
            return False
    return True


class BallFlow:
    def __init__(self, set_claw, send):
        self.set_claw, self.send = set_claw, send
        self.state = "WAIT_BALL"
        self.wrong = 0
        self.hits = 0
        self.deadline = None

    def emit(self, message):
        if not self.send(message):
            self.state = "TX_ERROR"
            print("TX ERROR; restart only after checking both controllers")
            return False
        return True

    def command(self, cmd, now):
        print("RX:", cmd, "state:", self.state)
        if cmd == "ball" and self.state in ("WAIT_BALL", "DONE", "NO_TARGET"):
            self.wrong = 0
            self.hits = 0
            self.set_claw(200)
            self.state = "OPEN_200"
            self.deadline = time.ticks_add(now, OPEN_WAIT_MS)
        elif cmd == "down" and self.state == "WAIT_DOWN":
            # STM32 must finish lowering AND the next 150mm move before down.
            self.hits = 0
            self.state = "SCAN_SETTLE"
            self.deadline = time.ticks_add(now, SCAN_SETTLE_MS)
        elif cmd == "grab" and self.state == "WAIT_GRAB":
            self.set_claw(200)
            self.state = "GRAB_OPEN_200"
            self.deadline = time.ticks_add(now, GRAB_OPEN_MS)
        elif cmd == "home" and self.state == "WAIT_HOME":
            self.set_claw(140)
            self.state = "RELEASE"
            self.deadline = time.ticks_add(now, RELEASE_WAIT_MS)
        else:
            print("Ignored unexpected command")

    def tick(self, now):
        if self.deadline is None or time.ticks_diff(now, self.deadline) < 0:
            return
        self.deadline = None
        if self.state == "OPEN_200":
            self.state = "WAIT_DOWN"
            self.emit("finishball")
        elif self.state == "SCAN_SETTLE":
            self.state = "SCAN"
            self.deadline = time.ticks_add(now, SCAN_TIMEOUT_MS)
        elif self.state == "SCAN":
            if self.wrong < 2:
                self.wrong += 1
                self.state = "WAIT_DOWN"
                self.emit("wrong")
            else:
                # No fourth position and no third wrong; stop and report locally.
                self.state = "NO_TARGET"
                print("NO_TARGET: third position failed; no further wrong sent")
        elif self.state == "GRAB_OPEN_200":
            self.set_claw(80)
            self.state = "CLOSE_80"
            self.deadline = time.ticks_add(now, CLOSE_WAIT_MS)
        elif self.state == "CLOSE_80":
            self.state = "WAIT_HOME"
            self.emit(("none", "one", "two")[self.wrong])
        elif self.state == "RELEASE":
            self.state = "DONE"
            self.emit("finishhome")
            print("Release complete; STM32 may start the correction stage")

    def observe(self, detected):
        if self.state != "SCAN":
            return
        self.hits = self.hits + 1 if detected else 0
        if self.hits >= CONFIRM_HITS:
            self.deadline = None
            self.state = "WAIT_GRAB"
            self.emit("right")


def run():
    if RUN_MODE not in ("TEST", "LINK"):
        raise ValueError("RUN_MODE must be TEST or LINK")
    if len(TASK_CODE) != 3 or any(c not in "123" for c in TASK_CODE):
        raise ValueError("invalid task code")
    wanted = int(TASK_CODE[0])
    laser = pyb.Pin("P5", pyb.Pin.OUT_PP)
    laser.low()
    light = pyb.Pin("P9", pyb.Pin.OUT_PP)
    light.low()  # disable fill light in both TEST and LINK modes
    # P7 is left untouched: STM32 performs rack/lift/yaw sequence.
    timer = pyb.Timer(4, freq=50)
    claw = timer.channel(2, pyb.Timer.PWM, pin=pyb.Pin("P8"))
    def set_claw(angle):
        us = angle_to_us(angle)
        claw.pulse_width(((timer.period()+1) * us) // 20000)
        print("P8 angle:", angle, "pulse us:", us)
    uart = None
    if RUN_MODE == "LINK":
        uart = pyb.UART(UART_ID, UART_BAUD, bits=8, parity=None, stop=1,
                        timeout=20, timeout_char=20, read_buf_len=256)
    def send(message):
        print("TX:" if uart else "TEST simulated TX:", message)
        return write_all(uart, (message + "\r\n").encode("ascii")) if uart else True
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    sensor.set_vflip(False)
    sensor.set_hmirror(False)
    sensor.skip_frames(time=1500)
    sensor.set_auto_gain(False)
    sensor.set_auto_whitebal(False)
    flow, receiver = BallFlow(set_claw, send), Lines()
    test_due = time.ticks_ms()
    test_state = None
    if RUN_MODE == "TEST":
        print("TEST operates real claw; UART disabled; STM32 completions simulated")
        flow.command("ball", time.ticks_ms())
    print("Task:", TASK_CODE, "ball color:", wanted)
    while True:
        now = time.ticks_ms()
        if uart and uart.any():
            for cmd in receiver.feed(uart.read(256) or b""):
                flow.command(cmd, now)
        if RUN_MODE == "TEST":
            if flow.state != test_state:
                test_state = flow.state
                test_due = time.ticks_add(now, TEST_MECHANISM_WAIT_MS)
            if time.ticks_diff(now, test_due) >= 0:
                commands = {"WAIT_DOWN": "down", "WAIT_GRAB": "grab", "WAIT_HOME": "home"}
                cmd = commands.get(flow.state)
                if flow.state == "DONE" and TEST_REPEAT:
                    cmd = "ball"
                if cmd:
                    flow.command(cmd, now)
        flow.tick(time.ticks_ms())
        img = sensor.snapshot()
        b = target_blob(img, wanted) if flow.state == "SCAN" else None
        # Observe before timeout evaluation, so a valid final frame can win.
        if flow.state == "SCAN":
            flow.observe(b is not None)
        flow.tick(time.ticks_ms())
        img.draw_rectangle(OBJECT_ROI, color=(0,255,255))
        if b:
            img.draw_rectangle(value(b, "rect"), color=(0,255,0))
        img.draw_string(3,3, RUN_MODE + " " + flow.state, color=(0,255,0))
        img.draw_string(3,18,"QR:%s color:%d wrong:%d hits:%d" %
                        (TASK_CODE,wanted,flow.wrong,flow.hits), color=(255,255,0))


if __name__ == "__main__":
    run()
