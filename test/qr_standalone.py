# OpenMV H7 Plus standalone QR test / STM32 integration.
# TEST: no UART transmission. LINK: wait for qr\r\n, reply finishqr\r\n.
# UART1: P1 TX -> STM32 PD9 RX; P0 RX <- PD8 TX; common GND.
import sensor
import time
import pyb

RUN_MODE = "LINK"  # TEST / LINK
CAMERA_ROTATE_180 = False  # same mounting as main.py
UART_ID = 1
UART_BAUD = 115200
CONFIRM_HITS = 2
CONFIRM_GAP_MS = 1500  # allow for slower full-frame VGA decoding
PITCH_ENABLE = True  # P7 camera pitch; same 10-degree QR pose as main.py
PITCH_ANGLE = 10
LIGHT_ENABLE = False  # P9 LED driver enable, same wiring as main.py
LASER_PIN = "P5"  # same wiring as main.py; LOW turns the laser off


def parse_payload(raw):
    raw = raw.strip()
    if len(raw) != 3 or any(ch not in "123" for ch in raw):
        return None
    return raw


class Confirmation:
    def __init__(self):
        self.reset()

    def reset(self):
        self.payload = None
        self.hits = 0
        self.last_hit = None

    def update(self, payload, now):
        if self.last_hit is not None and time.ticks_diff(now, self.last_hit) > CONFIRM_GAP_MS:
            self.reset()
        if payload is None:
            return False
        self.hits = self.hits + 1 if payload == self.payload else 1
        self.payload = payload
        self.last_hit = now
        return self.hits >= CONFIRM_HITS


class LineReceiver:
    def __init__(self):
        self.buffer = bytearray()
        self.dropping = False

    def feed(self, data):
        lines = []
        for byte in data:
            if byte in (10, 13):
                if self.buffer and not self.dropping:
                    try:
                        lines.append(self.buffer.decode("ascii").strip().lower())
                    except UnicodeError:
                        pass
                self.buffer = bytearray()
                self.dropping = False
            elif not self.dropping:
                if len(self.buffer) >= 96:
                    self.buffer = bytearray()
                    self.dropping = True
                else:
                    self.buffer.append(byte)
        return lines


def write_all(uart, data):
    offset = 0
    started = time.ticks_ms()
    while offset < len(data):
        sent = uart.write(data[offset:])
        if sent:
            offset += sent
        if offset < len(data) and time.ticks_diff(time.ticks_ms(), started) > 100:
            return False
    return True


def run():
    laser = pyb.Pin(LASER_PIN, pyb.Pin.OUT_PP)
    laser.low()
    if RUN_MODE not in ("TEST", "LINK"):
        raise ValueError("RUN_MODE must be TEST or LINK")
    # Keep timer/channel objects alive throughout the loop.
    pitch_timer = pitch_channel = light = None
    if PITCH_ENABLE:
        if not 0 <= PITCH_ANGLE <= 180:
            raise ValueError("invalid pitch angle")
        pitch_timer = pyb.Timer(4, freq=50)
        pitch_channel = pitch_timer.channel(1, pyb.Timer.PWM, pin=pyb.Pin("P7"))
        pitch_channel.pulse_width(int(round(500 + 2000 * PITCH_ANGLE / 180)))
    if LIGHT_ENABLE:
        light = pyb.Pin("P9", pyb.Pin.OUT_PP)
        light.high()

    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.VGA)
    sensor.set_vflip(CAMERA_ROTATE_180)
    sensor.set_hmirror(CAMERA_ROTATE_180)
    sensor.skip_frames(time=2000)
    sensor.set_auto_gain(False)
    sensor.set_auto_whitebal(False)
    uart = None
    if RUN_MODE == "LINK":
        uart = pyb.UART(UART_ID, UART_BAUD, bits=8, parity=None, stop=1,
                        timeout=20, timeout_char=20, read_buf_len=256)
    receiver = LineReceiver()
    confirmation = Confirmation()
    active = RUN_MODE == "TEST"
    locked = None
    pending_reply = False
    next_retry = time.ticks_ms()
    last_status = time.ticks_ms()
    clock = time.clock()
    print("QR", RUN_MODE, "VGA full frame; pitch:", PITCH_ANGLE)
    if uart:
        print("Waiting for qr\\r\\n; reply finishqr\\r\\n; no automatic start")

    while True:
        clock.tick()
        if uart and uart.any():
            for command in receiver.feed(uart.read(256) or b""):
                print("RX:", command)
                if command == "qr":
                    # A repeated request after completion resends the cached reply.
                    if locked is not None and not active:
                        pending_reply = True
                    elif not active:
                        confirmation.reset()
                        active = True
                elif command == "reset":
                    confirmation.reset()
                    active = False
                    locked = None
                    pending_reply = False

        img = sensor.snapshot()
        hit = None
        if active:
            # Decode before drawing: overlays must not alter QR pixels.
            codes = img.find_qrcodes()
            for code in codes:
                payload = parse_payload(code.payload())
                if payload is not None and hit is None:
                    hit = payload
                img.draw_rectangle(code.rect(), color=(0, 255, 0) if payload else (255, 0, 0))
            now = time.ticks_ms()
            if confirmation.update(hit, now):
                if locked != confirmation.payload:
                    print("QR LOCKED:", confirmation.payload,
                          "ball/target/hostage:", tuple(int(c) for c in confirmation.payload))
                locked = confirmation.payload
                if uart:
                    active = False
                    pending_reply = True

        now = time.ticks_ms()
        if pending_reply and time.ticks_diff(now, next_retry) >= 0:
            if write_all(uart, b"finishqr\r\n"):
                print("TX: finishqr; saved QR:", locked)
                pending_reply = False
            else:
                print("TX FAILED; retry in 1 second")
            next_retry = time.ticks_add(now, 1000)

        state = "SCAN" if active else ("DONE" if locked else "WAIT qr")
        img.draw_string(4, 4, "%s %s" % (RUN_MODE, state), color=(0, 255, 0), scale=2)
        img.draw_string(4, 28, "hit:%s %d/%d" % (hit or "---", confirmation.hits, CONFIRM_HITS),
                        color=(0, 255, 0), scale=2)
        if locked:
            img.draw_string(4, 52, "last QR: " + locked, color=(255, 255, 0), scale=2)
        if time.ticks_diff(now, last_status) >= 1000:
            print("QR", state, "hit:", hit, "last:", locked, "fps:", clock.fps())
            last_status = now


if __name__ == "__main__":
    run()
