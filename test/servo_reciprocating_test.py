# OpenMV H7 Plus: one positional servo moves back and forth continuously.
# Upright camera preview. Timer4 CH1=P7, CH2=P8. No light control.
from pyb import Pin, Timer
import time

SERVO_CHANNEL = 1  # 1=P7 pitch; 2=P8 claw (current robot wiring)
START_ANGLE = 180
END_ANGLE = 180
HOLD_MS = 1000
SERVO_MAX_ANGLE = 180  # set to actual servo specification
REVERSE_DIRECTION = True  # logical angle -> maximum angle minus logical angle
MIN_PULSE_US = 500
MAX_PULSE_US = 2500
PINS = {1: 'P7', 2: 'P8'}


def make_camera():
    try:
        import csi
    except ImportError:
        import sensor
        sensor.reset()
        sensor.set_pixformat(sensor.RGB565)
        sensor.set_framesize(sensor.QVGA)
        sensor.set_hmirror(False)
        sensor.set_vflip(False)
        sensor.skip_frames(time=2000)
        return sensor
    cam = csi.CSI()
    cam.reset()
    cam.pixformat(csi.RGB565)
    cam.framesize(csi.QVGA)
    cam.hmirror(False)
    cam.vflip(False)
    cam.snapshot(time=2000)
    return cam


def angle_to_us(angle):
    if not 0 <= angle <= SERVO_MAX_ANGLE:
        raise ValueError('Angle outside configured servo range')
    actual_angle = SERVO_MAX_ANGLE-angle if REVERSE_DIRECTION else angle
    return int(round(MIN_PULSE_US +
                     (MAX_PULSE_US-MIN_PULSE_US)*actual_angle/SERVO_MAX_ANGLE))


if SERVO_CHANNEL not in PINS:
    raise ValueError('Choose channel 1=P7 or 2=P8')
if SERVO_MAX_ANGLE <= 0 or HOLD_MS < 100:
    raise ValueError('Invalid angle range or hold time')
angle_to_us(START_ANGLE)
angle_to_us(END_ANGLE)

tim = Timer(4, freq=50)
servo = tim.channel(SERVO_CHANNEL, Timer.PWM, pin=Pin(PINS[SERVO_CHANNEL]))
try:
    cam = make_camera()
    while True:
        for angle in (START_ANGLE, END_ANGLE):
            us = angle_to_us(angle)
            servo.pulse_width(((tim.period()+1)*us)//20000)
            print(PINS[SERVO_CHANNEL], 'logical angle:', angle, 'pulse:', us, 'us')
            started = time.ticks_ms()
            while time.ticks_diff(time.ticks_ms(), started) < HOLD_MS:
                img = cam.snapshot()
                actual = SERVO_MAX_ANGLE-angle if REVERSE_DIRECTION else angle
                img.draw_string(2, 2, '%s INPUT:%s ACTUAL:%s' %
                                (PINS[SERVO_CHANNEL], angle, actual), color=(255, 0, 0))
finally:
    servo.pulse_width(0)
    tim.deinit()
