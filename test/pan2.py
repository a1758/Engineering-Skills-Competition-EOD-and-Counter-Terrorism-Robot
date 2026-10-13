# OpenMV: P8 claw opens once, then closes and holds; live camera preview.
from pyb import Pin, Timer
import time

# Set these angles to match the actual claw travel.
CLAW_OPEN_ANGLE = 200 # adjust to actual open position
CLAW_CLOSE_ANGLE = 200
HOLD_MS = 3000
MIN_PULSE_US = 500
MAX_PULSE_US = 2500
SERVO_MAX_ANGLE = 270
CLAW_PIN = 'P8'  # Timer4 channel 2
CAMERA_ROTATE_180 = True  # upside-down camera mounting


def make_camera():
    try:
        import csi
    except ImportError:
        import sensor
        sensor.reset()
        sensor.set_pixformat(sensor.RGB565)
        sensor.set_framesize(sensor.QVGA)
        sensor.set_hmirror(CAMERA_ROTATE_180)
        sensor.set_vflip(CAMERA_ROTATE_180)
        sensor.skip_frames(time=2000)
        return sensor
    cam = csi.CSI()
    cam.reset()
    cam.pixformat(csi.RGB565)
    cam.framesize(csi.QVGA)
    cam.hmirror(CAMERA_ROTATE_180)
    cam.vflip(CAMERA_ROTATE_180)
    cam.snapshot(time=2000)
    return cam


def angle_to_us(angle):
    if not 0 <= angle <= SERVO_MAX_ANGLE:
        raise ValueError('Angle must be 0..270')
    return int(round(MIN_PULSE_US + (MAX_PULSE_US-MIN_PULSE_US)*angle/SERVO_MAX_ANGLE))


def set_angle(servo, angle):
    us = angle_to_us(angle)
    servo.pulse_width(((tim.period()+1)*us)//20000)
    print('Angle:', angle, 'Pulse:', us, 'us')


angle_to_us(CLAW_OPEN_ANGLE)
angle_to_us(CLAW_CLOSE_ANGLE)
if HOLD_MS <= 0:
    raise ValueError('HOLD_MS must be positive')

tim = Timer(4, freq=50)
claw = tim.channel(2, Timer.PWM, pin=Pin(CLAW_PIN))
try:
    opened = True
    set_angle(claw, CLAW_OPEN_ANGLE)
    started = time.ticks_ms()
    # Initialize the camera after opening the claw; the 5-second hold
    # starts when the servo reaches the open command.
    cam = make_camera()
    while True:
        if opened and time.ticks_diff(time.ticks_ms(), started) >= HOLD_MS:
            set_angle(claw, CLAW_CLOSE_ANGLE)
            opened = False
        img = cam.snapshot()  # live framebuffer in OpenMV IDE
        angle = CLAW_OPEN_ANGLE if opened else CLAW_CLOSE_ANGLE
        img.draw_string(2, 2, 'P8 %s: %s' % ('OPEN' if opened else 'CLOSE', angle), color=(255, 0, 0))
finally:
    claw.pulse_width(0)
    tim.deinit()
