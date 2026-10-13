# OpenMV standalone target test: fixed three-target decision flow
import sensor, image, time, pyb

RUN_MODE = "INTEGRATION"                 # DEBUG or INTEGRATION
DEBUG_DRAW = True
TASK_COLOR = 1                      # 1 red, 2 green, 3 blue
COLORS = {1: (0,45,14,81,-76,79), 2: (34,56,-67,-17,-41,84), 3: (11,59,-28,-5,-33,-6)}
ROI = (0, 0, 320, 200)
MIN_PIXELS, MIN_AREA = 80, 120
CONFIRM_FRAMES = 3
# First target is near image centre; second is on image right.
FIRST_ZONE = (70, 0, 210, 200)
SECOND_ZONE = (205, 0, 320, 200)
PITCH_DEG = 80
PITCH_SETTLE_MS = 600
LASER_ON_MS = 500
LASER_PIN = "P5"
UART_ID, UART_BAUD = 1, 115200
# STM32 -> OpenMV: target, finishtarget, reset
# OpenMV -> STM32: 11 / 22 / 33, laser, ready

sensor.reset(); sensor.set_pixformat(sensor.RGB565); sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=1500); sensor.set_auto_gain(False); sensor.set_auto_whitebal(False)
clock = time.clock()
laser = pyb.Pin(LASER_PIN, pyb.Pin.OUT_PP); laser.low()
timer = pyb.Timer(4, freq=50); pitch = timer.channel(1, pyb.Timer.PWM, pin=pyb.Pin("P7"))
def set_pitch(deg):
    us = 500 + int(2000 * max(0, min(180, deg)) / 180)
    pitch.pulse_width(int((timer.period()+1) * us / 20000))
set_pitch(PITCH_DEG); pitch_ready = time.ticks_add(time.ticks_ms(), PITCH_SETTLE_MS)
uart = pyb.UART(UART_ID, UART_BAUD, timeout=20, timeout_char=20)

def tx(s):
    print("TX", s)
    if RUN_MODE == "INTEGRATION": uart.write(s + "\r\n")
def off(): laser.low()
def reset_flow():
    global state, stable, choice
    off(); state, stable, choice = "WAIT_TARGET", 0, 0

def command(s):
    global state, stable, choice, pitch_ready
    print("RX", s)
    if s == "reset": reset_flow()
    elif s == "target" and state == "WAIT_TARGET":
        set_pitch(PITCH_DEG); pitch_ready = time.ticks_add(time.ticks_ms(), PITCH_SETTLE_MS)
        state, stable, choice = "SEARCH", 0, 0
    elif s == "finishtarget" and state == "WAIT_FINISH":
        laser.high(); tx("laser")
        deadline = time.ticks_add(time.ticks_ms(), LASER_ON_MS)
        while time.ticks_diff(deadline, time.ticks_ms()) > 0: time.sleep_ms(5)
        off(); tx("ready"); reset_flow()

def read_uart():
    if RUN_MODE != "INTEGRATION": return
    while uart.any():
        raw = uart.readline()
        if raw:
            try: command(raw.decode().strip())
            except: pass

def candidates(img):
    bs = img.find_blobs([COLORS[TASK_COLOR]], roi=ROI, pixels_threshold=MIN_PIXELS,
                        area_threshold=MIN_AREA, merge=True)
    return [b for b in bs if b.w() >= 8 and b.h() >= 8]

def choose_target(bs):
    # Centre target has priority; otherwise use the right target.
    first = [b for b in bs if FIRST_ZONE[0] <= b.cx() < FIRST_ZONE[2]]
    right = [b for b in bs if SECOND_ZONE[0] <= b.cx() < SECOND_ZONE[2]]
    if first: return 1, max(first, key=lambda b:b.pixels())
    if right: return 2, max(right, key=lambda b:b.pixels())
    return 0, None

state = "SEARCH" if RUN_MODE == "DEBUG" else "WAIT_TARGET"
stable, choice, shot_requested = 0, 0, False
while True:
    clock.tick(); read_uart(); img = sensor.snapshot()
    if state == "SEARCH":
        bs = candidates(img); idx, blob = choose_target(bs)
        # If the first two positions do not match, select the third fixed target.
        if idx == 0:
            stable += 1
            if stable >= CONFIRM_FRAMES and time.ticks_diff(time.ticks_ms(), pitch_ready) >= 0:
                idx, blob = 3, None
        else:
            stable += 1
        if idx and stable >= CONFIRM_FRAMES and not shot_requested:
            choice = idx; tx(str({1:11, 2:22, 3:33}[idx]))
            shot_requested = True; state = "WAIT_FINISH"
        if DEBUG_DRAW:
            for b in bs: img.draw_rectangle(b.rect(), color=(255,0,0)); img.draw_cross(b.cx(), b.cy(), color=(0,255,0))
    img.draw_string(2, 2, "%s C%d K%d" % (state, TASK_COLOR, choice), color=(255,255,0))
    if RUN_MODE == "DEBUG" and state == "WAIT_FINISH":
        time.sleep_ms(300); command("finishtarget")
