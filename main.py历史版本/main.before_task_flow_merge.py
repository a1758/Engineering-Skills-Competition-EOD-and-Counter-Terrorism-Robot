# OpenMV H7 Plus / MicroPython. See 视觉方案.md before running on the robot.
import time
import math
import gc
CAMERA_ROTATE_180 = False  # match pan.py: camera mounted upside down
QR_VGA_ENABLE = True
CAMERA_SIZE_SETTLE_FRAMES = 3

# All LAB thresholds are initial values, NOT competition calibration values.
COLORS = {1: (0, 45, 14, 81, -76, 79),
          2: (34, 56, -67, -17, -41, 84),
          3: (11, 59, -28, -5, -33, -6)}
FLOOR_LAB = (21, 100, -54, 127, -102, 3)  # recalibrate on track under constant light
WHITE_LAB = (45, 100, -25, 25, -30, 30)  # initial neutral white; calibrate with lighting
HOSTAGE_LAB = (45, 100, -25, 25, -30, 30)
HOSTAGE_ROI = (65, 40, 190, 190)  # QVGA; entire object must fit inside
HOSTAGE_DEBUG_BINARY = True  # debug preview only, after recognition finishes
HOSTAGE_PROFILE_ROWS = (0.25, 0.32, 0.39, 0.46, 0.53, 0.60, 0.67, 0.74, 0.80)
BOOT_MODE = "TRACK"  # TRACK / QR / BALL / TARGET / HOSTAGE / IDLE
DEBUG_DRAW = True
CONFIRM_FRAMES = 3
OBJECT_ROI = (0, 0, 320, 200)  # adjust for the stopped, object-facing pose
TRACK_ROWS = (218, 185, 150)  # near -> far; QVGA only
TRACK_WIDTH_LIMITS = ((70, 290), (45, 260), (25, 230))
TRACK_REF_X = 160  # camera optical centre projected on vehicle centreline
TRACK_MIN_RUN = 20
TRACK_JOIN_GAP = 2
TRACK_PAIR_MAX_SHIFT = 65
TRACK_WARN_PX = 18  # visual margin threshold, not physical safety clearance
PROFILE_MIN_WIDTH = 10
UART_ENABLE = True
UART_ID = 1  # verified: P0 RX <- STM32 PD8 TX; P1 TX -> STM32 PD9 RX
UART_BAUD = 115200
UART_TIMEOUT_MS = 20
UART_TIMEOUT_CHAR_MS = 20  # same settings as successful uart_light_test.py
UART_READ_BUF_LEN = 256
STM32_PROTOCOL = "FIXED_ROUTE"  # qr/globule line handshake; legacy modes retained
SEND_START_ON_BOOT = True  # send START until STM32 confirms
START_DELAY_MS = 5000
START_REPEAT_MS = 1000  # match successful standalone test cadence
UART_DEBUG_RAW = True  # disable after communication is verified
START_WAIT_WARNING_MS = 10000  # diagnostic only; NOT a chassis arrival deadline
START_COMMAND = b"START"  # exactly 5 ASCII bytes; no spaces or CR/LF
ROUTE_V3_ENABLE = True  # requires STM32 VIS V3 / VCMD / VEVT extension
ACTION_LINK = None  # runtime handle for explicit action requests
MISSION = None  # runtime handle; arrive() only after real chassis stop
SEND_INTERVAL_MS = 50
MAX_RESULT_AGE_MS = 200
MODE_IDS = {"IDLE": 0, "TRACK": 1, "QR": 2, "BALL": 3,
            "TARGET": 4, "HOSTAGE": 5, "BIN": 6}
WARNING_IDS = {"": 0, "EDGE_LOST": 1, "PIXEL_MARGIN": 2,
               "BODY_MARGIN": 3, "STALE": 4, "GIMBAL_SETTLING": 5,
               "GROUND_CALIBRATION": 6, "TRACK_GEOMETRY": 7}
INVALID_I16 = -32768
RX_FRAME_LIMIT = 96
RX_TIMEOUT_MS = 200
TX_FRAME_LIMIT = 320
TX_TIMEOUT_MS = 100
COMMAND_QUEUE_LIMIT = 4
GIMBAL_ENABLE = True  # set True after P7 claw / P8 pitch wiring & pulse calibration
PAN_CHANNEL = 2  # Timer4 CH2=P8: 270-degree claw (legacy variable name)
TILT_CHANNEL = 1  # Timer4 CH1=P7: camera pitch
SERVO_MIN_PULSE_US = 500
SERVO_MAX_PULSE_US = 2500  # supplied 180-degree positional servo mapping
TILT_LIMIT_DEG = (0, 180)  # narrow to actual safe mechanical range
CLAW_LIMIT_DEG = (0, 180)
CLAW_MAX_ANGLE = 180  # MG90S positional version; verify purchased specification
TILT_MAX_ANGLE = 180  # pitch servo range; independent of claw
TILT_REVERSE_DIRECTION = True
CLAW_REVERSE_DIRECTION = False
TILT_ANGLES = {"TRACK": 100, "QR": 10, "BALL": 80,
               "TARGET": 20, "HOSTAGE": 53, "IDLE": 100, "BIN": 71}
CLAW_OPEN_ANGLE = 133.333333  # preserves latest 200/270 pulse (~1981us)
CLAW_CLOSE_ANGLE = 60.333333  # preserves latest 90.5/270 pulse (~1170us)
# Fixed-height, fixed-pitch image-space edge holding. No invented mm distances.
FINAL_TRACK_ANGLE = 30
EDGE_X_SAMPLES = (60, 90, 120, 150, 180, 210, 240, 270)
EDGE_Y_RANGE = (35, 145)  # excludes wheels/claw at bottom of supplied images
EDGE_MIN_FLOOR_ROWS = 10
EDGE_MAX_FIT_ERROR_PX = 5
EDGE_MAX_DEVIATION_PX = 20
EDGE_REFERENCES = {segment: None for segment in range(1, 9)}
# Each stopped route segment learns its OWN safe edge Y; alternatively fill
# measured references per segment. Never reuse left/right viewing geometry.
# None learns a reference at a STOPPED arrival, after 3 consistent frames.
# Learning holds the existing distance; it cannot certify that distance is safe.
TRACK_PROFILES = {"CENTER": 0, "FRONT": 1, "SIDE": 2}
CORNER_END_LIMITS = {4: None, 7: None}
# After real-image calibration use ("LEFT", x) or ("RIGHT", x).
# LEFT triggers when the road's visible left end reaches x or greater;
# RIGHT triggers when its right end reaches x or less. None disables visual stop.
# Compatibility metadata only: ground calibration still stores pulse poses.
POSE_US = {mode: (1500, int(round(500+2000*((TILT_MAX_ANGLE-angle)
           if TILT_REVERSE_DIRECTION else angle)/TILT_MAX_ANGLE)))
           for mode, angle in TILT_ANGLES.items()}
# Placeholder poses deliberately identical until measured; order (pan, tilt).
GIMBAL_SETTLE_MS = 600
# P7_LED -> LED driver EN. Constant GPIO HIGH frees Timer4 for servos.
LIGHT_MODE = "ON"  # ON=constant full brightness, OFF=off, PWM=legacy dimming
LIGHT_PIN = "P9"  # disconnect/isolate LED board P7 before using claw PWM
LIGHT_TIMER_ID = 4
LIGHT_TIMER_CHANNEL = 1
LIGHT_DUTY = 25
LASER_PIN = "P5"  # laser control moved by user: HIGH=ON, LOW=OFF; not UART3 RX
LASER_AIM_CALIBRATED = False  # enable only after camera/laser offset calibration
LASER_AIM_WINDOW = (160, 120, 4, 4)  # cx,cy,tolerance x/y, QVGA pixels
LASER_ON_MS = 500  # bounded illumination; tune for receiver during testing

# CAD interpretation, pending confirmation: wheel-centre spans + wheel extent.
CAD_LATERAL_CENTRES_MM = 196.60
CAD_LONGITUDINAL_CENTRES_MM = 164.0
WHEEL_HALF_WIDTH_MM = 23.30
WHEEL_DIAMETER_MM = 80.0
VEHICLE_WIDTH_MM = 240.0  # latest CAD outside width 239.2mm, rounded up
VEHICLE_LENGTH_MM = 240.0  # user-selected conservative value for roller measurement
VEHICLE_HALF_WIDTH_MM = VEHICLE_WIDTH_MM / 2.0
VEHICLE_HALF_LENGTH_MM = VEHICLE_LENGTH_MM / 2.0
FOOTPRINT_CONFIRMED = True  # accepted 240x240mm cruising envelope, plus extra below
FOOTPRINT_EXTRA_MM = 3.0  # extra outward envelope allowance, not stopping margin
TRACK_WIDTH_MM = 300.0
TRACK_WIDTH_TOLERANCE_MM = 35.0
EDGE_STRAIGHT_TOLERANCE_MM = 8.0
# User confirmed: optical centre height above ground, range 125..210 mm.
CAMERA_HEIGHT_RANGE_MM = (125.0, 210.0)
CAMERA_HEIGHT_MM = None  # current measured optical height; fixed while cruising
CAMERA_X_MM = 0.0  # provisional near vehicle centre; calibration uses vehicle frame
CAMERA_Y_MM = 0.0
HEIGHT_MATCH_TOLERANCE_MM = 1.0
# Optional calibrated ground map: (a,b,c,d,e,f,g,h).
# Ground X=(a*x+b*y+c)/(g*x+h*y+1), Y likewise; units mm.
# X right, Y forward; origin actual footprint centre, NOT camera centre.
# Set metadata after measuring this exact camera height/servo pose.
GROUND_H = None
GROUND_CALIB_HEIGHT_MM = None
GROUND_CALIB_POSE_US = None
# Alternative separate calibrations for multiple fixed optical heights:
# {125.0: {"h": (...8 coefficients...), "pose_us": (pan, tilt)}, ...}
GROUND_PROFILES = {}
SAFETY_MARGIN_MM = 15


def value(obj, name):
    """Old OpenMV results use methods; current csi examples use properties."""
    v = getattr(obj, name)
    return v() if callable(v) else v


def angle_to_us(angle, max_angle=180, reverse=False):
    if not 0 <= angle <= max_angle:
        raise ValueError("servo angle outside configured range")
    angle = max_angle-angle if reverse else angle
    return int(round(SERVO_MIN_PULSE_US +
                     (SERVO_MAX_PULSE_US-SERVO_MIN_PULSE_US)*angle/max_angle))


def camera_warmup(cam, service=None):
    began = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(),began)<2000:
        if service:
            service()
        cam.snapshot()
    if service:
        service()


def make_camera(service=None):
    try:
        import csi
    except ImportError:
        import sensor
        sensor.reset()
        sensor.set_pixformat(sensor.RGB565)
        sensor.set_framesize(sensor.QVGA)
        sensor.set_hmirror(CAMERA_ROTATE_180)
        sensor.set_vflip(CAMERA_ROTATE_180)
        camera_warmup(sensor, service)
        sensor.set_auto_gain(False)
        sensor.set_auto_whitebal(False)
        return sensor
    cam = csi.CSI()
    cam.reset()
    cam.pixformat(csi.RGB565)
    cam.framesize(csi.QVGA)
    cam.hmirror(CAMERA_ROTATE_180)
    cam.vflip(CAMERA_ROTATE_180)
    camera_warmup(cam, service)
    cam.auto_gain(False)
    cam.auto_whitebal(False)
    return cam


class CameraModes:
    """Change resolution only for QR. Discard transition frames before use."""
    def __init__(self, cam):
        self.cam = cam
        self.vga = False

    def snapshot(self, mode):
        vga = mode == "QR" and QR_VGA_ENABLE
        if vga != self.vga:
            if hasattr(self.cam, "set_framesize"):
                import sensor
                self.cam.set_framesize(sensor.VGA if vga else sensor.QVGA)
            else:
                import csi
                self.cam.framesize(csi.VGA if vga else csi.QVGA)
            for _ in range(CAMERA_SIZE_SETTLE_FRAMES):
                self.cam.snapshot()
            self.vga = vga
        return self.cam.snapshot()


class Hardware:
    def __init__(self):
        self.pan = self.tilt = self.light = None
        if LASER_PIN is not None and LASER_PIN == LIGHT_PIN:
            raise ValueError("laser and light cannot share a control pin")
        self.laser_pin = None
        if LASER_PIN is not None:
            from pyb import Pin
            self.laser_pin = Pin(LASER_PIN, Pin.OUT_PP)
            self.laser_pin.low()
        self.ready_at = time.ticks_ms()
        # Validate before configuring any PWM output.
        pins = {1: "P7", 2: "P8"}
        if GIMBAL_ENABLE and (PAN_CHANNEL not in pins or TILT_CHANNEL not in pins or PAN_CHANNEL == TILT_CHANNEL):
            raise ValueError("invalid servo channels")
        if LIGHT_PIN is not None:
            if LIGHT_MODE not in ("ON", "OFF", "PWM"):
                raise ValueError("invalid light mode")
            if LIGHT_MODE == "PWM" and (LIGHT_TIMER_ID is None or LIGHT_TIMER_CHANNEL is None):
                raise ValueError("LED PWM timer/channel not configured")
            if GIMBAL_ENABLE and ((LIGHT_MODE == "PWM" and LIGHT_TIMER_ID == 4) or
                                   LIGHT_PIN in (pins[PAN_CHANNEL], pins[TILT_CHANNEL])):
                raise ValueError("LED pin/timer conflicts with gimbal")
            if not 0 <= LIGHT_DUTY <= 100:
                raise ValueError("LED duty must be 0..100")
        if GIMBAL_ENABLE:
            from pyb import Timer, Pin
            self.servo_timer = Timer(4, freq=50)
            self.pan = self.servo_timer.channel(PAN_CHANNEL, Timer.PWM,
                                                pin=Pin(pins[PAN_CHANNEL]))
            self.tilt = self.servo_timer.channel(TILT_CHANNEL, Timer.PWM,
                                                 pin=Pin(pins[TILT_CHANNEL]))
        if LIGHT_PIN is not None:
            from pyb import Timer, Pin
            if LIGHT_MODE == "PWM":
                self.light_timer = Timer(LIGHT_TIMER_ID, freq=1000)
                self.light = self.light_timer.channel(LIGHT_TIMER_CHANNEL, Timer.PWM,
                                                      pin=Pin(LIGHT_PIN),
                                                      pulse_width_percent=LIGHT_DUTY)
            else:
                self.light = Pin(LIGHT_PIN, Pin.OUT_PP)
                self.light.value(1 if LIGHT_MODE == "ON" else 0)

    def set_pose(self, mode):
        if self.pan is None:
            return
        self.set_tilt_angle(TILT_ANGLES[mode])
        self.ready_at = time.ticks_add(time.ticks_ms(), GIMBAL_SETTLE_MS)

    def set_tilt_angle(self, angle):
        if not TILT_LIMIT_DEG[0] <= angle <= TILT_LIMIT_DEG[1]:
            raise ValueError("tilt angle outside safe limits")
        us = angle_to_us(angle, TILT_MAX_ANGLE, TILT_REVERSE_DIRECTION)
        if self.tilt is None:
            return False
        self.tilt.pulse_width(((self.servo_timer.period()+1)*us)//20000)
        return True

    def claw(self, angle):
        if self.pan is None:
            return False
        if not CLAW_LIMIT_DEG[0] <= angle <= CLAW_LIMIT_DEG[1]:
            raise ValueError("claw angle outside safe limits")
        us = angle_to_us(angle, CLAW_MAX_ANGLE, CLAW_REVERSE_DIRECTION)
        self.pan.pulse_width(((self.servo_timer.period()+1)*us)//20000)
        return True

    def laser(self, enabled):
        if self.laser_pin is None:
            return False
        self.laser_pin.value(1 if enabled else 0)
        return True

    def ready(self, now):
        return time.ticks_diff(now, self.ready_at) >= 0


class Confirmation:
    def __init__(self):
        self.reset()

    def reset(self):
        self.key = None
        self.pos = None
        self.count = 0

    def update(self, key, pos=None):
        if key is None:
            self.reset()
            return False
        close = pos is None or self.pos is None or (
            abs(pos[0] - self.pos[0]) <= 20 and
            abs(pos[1] - self.pos[1]) <= 20)
        self.count = self.count + 1 if key == self.key and close else 1
        self.key, self.pos = key, pos
        return self.count >= CONFIRM_FRAMES


def parse_qr(raw):
    raw = raw.strip()
    if len(raw) != 3 or any(c not in "123" for c in raw):
        return None
    return {"raw": raw, "ball_color": int(raw[0]),
            "target_color": int(raw[1]), "hostage_shape": int(raw[2])}


def find_qr(img):
    candidates = []
    gray = img.copy()
    gray = gray.to_grayscale()
    for code in gray.find_qrcodes():
        task = parse_qr(value(code, "payload"))
        if task:
            candidates.append((task, value(code, "rect")))
    # Ambiguous task codes in one frame must not latch a random task.
    if not candidates or len(set(t[0]["raw"] for t in candidates)) != 1:
        return None
    task, rect = max(candidates, key=lambda t: t[1][2] * t[1][3])
    display_rect = rect
    sx, sy = 320.0/img.width(), 240.0/img.height()
    rect = tuple(int(round(v*(sx if i%2==0 else sy))) for i,v in enumerate(rect))
    return {"task": task, "rect": rect, "cx": rect[0] + rect[2] // 2,
            "cy": rect[1] + rect[3] // 2, "display_rect": display_rect}


def color_objects(img, target=False):
    out = []
    for color in (1, 2, 3):
        for b in img.find_blobs([COLORS[color]], roi=OBJECT_ROI,
                                pixels_threshold=60, area_threshold=100,
                                merge=False):
            x, y, w, h = value(b, "rect")
            pixels = value(b, "pixels")
            aspect = w / float(h)
            density = pixels / float(w * h)
            # Ball stem may elongate the blob; allow a taller silhouette.
            if min(w, h) < 10 or not 0.5 <= aspect <= 1.6:
                continue
            if not (0.12 if target else 0.4) <= density <= 1.0:
                continue
            out.append({"color": color, "cx": x + w // 2,
                        "cy": y + h // 2, "rect": (x, y, w, h),
                        "pixels": pixels, "density": density,
                        "aim_kind": "ring_bbox_center" if target else "blob_center"})
    return out


def white_pixel(img, x, y):
    p = img.get_pixel(x, y)
    return (p[0] > 127) if isinstance(p, tuple) else p > 127


def row_runs(img, y, x0=0, x1=320, join_gap=0):
    runs = []
    start = None
    last = None
    for x in range(x0, x1):
        if white_pixel(img, x, y):
            if start is None:
                start = x
            last = x
        elif start is not None and x - last > join_gap:
            runs.append((start, last))
            start = None
    if start is not None:
        runs.append((start, last))
    return runs


def profile_fit(values):
    n = len(values)
    mid = (n-1)/2.0
    mean = sum(values)/float(n)
    slope = sum((i-mid)*(v-mean) for i,v in enumerate(values))/sum((i-mid)**2 for i in range(n))
    residual = math.sqrt(sum((v-mean-slope*(i-mid))**2 for i,v in enumerate(values))/n)
    return slope, residual


def shape_metrics(widths, centers=None):
    if len(widths)<5 or min(widths)<PROFILE_MIN_WIDTH:
        return {"shape":0,"reason":"PROFILE_MISSING"}
    n=len(widths)
    mean=sum(widths)/float(n)
    slope,error=profile_fit(widths)
    taper=abs(slope*(n-1))/mean
    top=sum(widths[:2])/2.0
    bottom=sum(widths[-2:])/2.0
    middle=sum(widths[n//2-1:n//2+2])/3.0
    bulge=min((middle-top)/mean,(middle-bottom)/mean)
    edge_error=error/mean
    left=right=None
    if centers is not None:
        left=[c-w/2.0 for c,w in zip(centers,widths)]
        right=[c+w/2.0 for c,w in zip(centers,widths)]
        edge_error=max(profile_fit(left)[1],profile_fit(right)[1])/mean
    reason="AMBIGUOUS"
    shape=0
    # Mid-body bulge above BOTH ends, with nonlinear side edges.
    if bulge>=0.075 and edge_error>=0.022 and error/mean>=0.025:
        if left is None or (left[n//2]<((left[0]+left[-1])/2)-mean*0.035 and
                            right[n//2]>((right[0]+right[-1])/2)+mean*0.035):
            shape,reason=3,"CURVED_BULGE"
    elif edge_error<=0.045 and error/mean<=0.055:
        if taper<=0.12 and (max(widths)-min(widths))/mean<=0.18:
            shape,reason=1,"PARALLEL_SIDES"
        elif 0.18<=taper<=0.90 and abs(bottom-top)>=max(4,mean*0.12):
            shape,reason=2,"STRAIGHT_TAPER"
    return {"shape":shape,"reason":reason,"taper":taper,
            "edge_error":edge_error,"bulge":bulge}


def classify_profile(widths, top_diameter=None):
    # Compatibility wrapper; top diameter is deliberately not a hard gate.
    return shape_metrics(widths)["shape"]


def hostage_objects(img, diagnostics=None):
    out = []
    iw, ih = img.width(), img.height()
    x0,y0,rw,rh = HOSTAGE_ROI
    x0,y0 = max(0,x0),max(0,y0)
    rw,rh = min(rw,iw-x0),min(rh,ih-y0)
    if rw <= 0 or rh <= 0:
        raise ValueError('HOSTAGE_ROI outside image')
    if diagnostics is not None:
        diagnostics.update(roi=(x0,y0,rw,rh), candidates=0, rejected=[], reason='NO_WHITE_BLOB')
    # Work on a copy: no binary/debug drawing can corrupt other detectors.
    mask = img.copy()
    mask.binary([HOSTAGE_LAB])
    for b in mask.find_blobs([(50, 100, -128, 127, -128, 127)],
                             roi=(x0,y0,rw,rh), pixels_threshold=120,
                             area_threshold=180, merge=False):
        x, y, w, h = value(b, "rect")
        reason = None
        if x <= x0+1 or y <= y0+1 or x+w >= x0+rw-1 or y+h >= y0+rh-1:
            reason = 'ROI_TOUCH'  # cropped object or merged pale background
        elif w < 14 or h < 25:
            reason = 'TOO_SMALL'
        elif not 0.3 <= w/float(h) <= 1.15:
            reason = 'BAD_ASPECT'
        if diagnostics is not None:
            diagnostics['candidates'] += 1
        if reason:
            if diagnostics is not None and len(diagnostics['rejected']) < 6:
                diagnostics['rejected'].append(((x,y,w,h),reason))
                diagnostics['reason'] = reason
            continue
        widths = []
        centers = []
        # Avoid the top ellipse and rounded bottom; use the body sides.
        sampled_y = []
        for fraction in HOSTAGE_PROFILE_ROWS:
            samples = []
            sample_centers = []
            yy = y + int((h - 1) * fraction)
            sampled_y.append(yy)
            for dy in (-2, -1, 0, 1, 2):
                runs = [r for r in row_runs(mask, yy + dy, x, x + w)
                        if r[1]-r[0]+1 >= max(3,int(w*0.04))]
                if runs:
                    left, right = runs[0][0], runs[-1][1]
                    samples.append(right - left + 1)
                    sample_centers.append((left + right) / 2.0)
            widths.append(sorted(samples)[len(samples)//2] if samples else 0)
            centers.append(sum(sample_centers)/len(sample_centers) if sample_centers else -999)
        metrics = shape_metrics(widths, centers)
        shape = metrics["shape"]
        out.append({"shape": shape, "cx": x + w//2, "cy": y + h//2,
                    "rect": (x, y, w, h), "width_profile": widths,
                    "profile_centers": centers, "profile_y": sampled_y,
                    "pixels": value(b, "pixels"), "shape_metrics": metrics,
                    "upper_lower_ratio": sum(widths[:2])/float(sum(widths[-2:])) if sum(widths[-2:]) else None})
    if diagnostics is not None and out:
        diagnostics['reason'] = 'CLASSIFIED' if any(o['shape'] for o in out) else 'SHAPE_UNKNOWN'
    return out


def active_ground_map():
    if CAMERA_HEIGHT_MM is None:
        return None
    current_pose = (1500, angle_to_us(TILT_ANGLES["TRACK"], TILT_MAX_ANGLE,
                                    TILT_REVERSE_DIRECTION))
    matches = [(abs(height-CAMERA_HEIGHT_MM), profile) for height, profile in GROUND_PROFILES.items()
               if abs(height-CAMERA_HEIGHT_MM) <= HEIGHT_MATCH_TOLERANCE_MM and
               tuple(profile["pose_us"]) == current_pose]
    if matches:
        return min(matches, key=lambda item: item[0])[1]["h"]
    if (GROUND_H is not None and GROUND_CALIB_HEIGHT_MM is not None and
            abs(CAMERA_HEIGHT_MM-GROUND_CALIB_HEIGHT_MM) <= HEIGHT_MATCH_TOLERANCE_MM and
            GROUND_CALIB_POSE_US is not None and
            tuple(GROUND_CALIB_POSE_US) == current_pose):
        return GROUND_H
    return None


def set_cruise_height(height_mm):
    """Call only after lift has stopped; this records position, does not drive lift."""
    global CAMERA_HEIGHT_MM
    if height_mm is None:
        CAMERA_HEIGHT_MM = None  # invalidate physical mapping during lift motion
    elif not CAMERA_HEIGHT_RANGE_MM[0] <= height_mm <= CAMERA_HEIGHT_RANGE_MM[1]:
        raise ValueError("camera height outside known range")
    else:
        CAMERA_HEIGHT_MM = float(height_mm)


def ground_point(x, y, mapping=None):
    mapping = active_ground_map() if mapping is None else mapping
    if mapping is None:
        return None
    a, b, c, d, e, f, g, h = mapping
    den = g*x + h*y + 1
    if abs(den) < 1e-6:
        return None
    return ((a*x+b*y+c)/den, (d*x+e*y+f)/den)


def ground_clearance(left, right):
    """Signed inward distances for a full rectangular footprint, straight road.

    Returns None for inconsistent width / curved edges / singular geometry.
    Points near->far, vehicle-aligned X right/Y forward, mm.
    """
    if len(left) != 3 or len(right) != 3:
        return None  # all three bands required for physical-edge consistency
    def inward_line(points, inside):
        p, q = points[0], points[-1]
        dx, dy = q[0]-p[0], q[1]-p[1]
        length = math.sqrt(dx*dx + dy*dy)
        if length < 20:
            return None
        nx, ny = -dy/length, dx/length
        if nx*(inside[0]-p[0])+ny*(inside[1]-p[1]) < 0:
            nx, ny = -nx, -ny
        offset = -nx*p[0]-ny*p[1]
        if any(abs(nx*r[0]+ny*r[1]+offset) > EDGE_STRAIGHT_TOLERANCE_MM for r in points):
            return None
        return nx, ny, offset
    ln = inward_line(left, right[len(right)//2])
    rn = inward_line(right, left[len(left)//2])
    if ln is None or rn is None:
        return None
    # Require opposing, nearly parallel edges before extrapolating to chassis.
    if ln[0]*rn[0] + ln[1]*rn[1] > -0.985:
        return None
    for l, r in zip(left, right):
        width = ln[0]*r[0]+ln[1]*r[1]+ln[2]
        if abs(width-TRACK_WIDTH_MM) > TRACK_WIDTH_TOLERANCE_MM:
            return None
    def min_corner(line):
        nx, ny, offset = line
        # Support function gives minimum over all FOUR corners, preserving sign.
        return (offset - abs(nx)*(VEHICLE_HALF_WIDTH_MM+FOOTPRINT_EXTRA_MM)
                - abs(ny)*(VEHICLE_HALF_LENGTH_MM+FOOTPRINT_EXTRA_MM))
    return min_corner(ln), min_corner(rn)


def track_result(img):
    mask = img.copy()
    mask.binary([FLOOR_LAB])
    rows = []
    previous_center = TRACK_REF_X
    for index, y in enumerate(TRACK_ROWS):
        lo, hi = TRACK_WIDTH_LIMITS[index]
        # Check three adjacent scanlines and use the middle measured pair.
        choices = []
        for dy in (-2, 0, 2):
            runs = row_runs(mask, y + dy, join_gap=TRACK_JOIN_GAP)
            runs = [r for r in runs if r[0] > 2 and r[1] < 317 and
                    lo <= r[1]-r[0]+1 <= hi and
                    abs((r[0]+r[1])/2 - previous_center) <= TRACK_PAIR_MAX_SHIFT]
            if runs:
                choices.append(min(runs, key=lambda r: abs((r[0]+r[1])/2-previous_center)))
        if len(choices) < 2:
            rows.append(None)
            continue
        left, right = sorted(choices, key=lambda r: (r[0]+r[1]))[len(choices)//2]
        previous_center = (left + right) / 2.0
        rows.append({"y": y, "left_px": left, "right_px": right,
                     "center_px": previous_center, "width_px": right-left+1})
    # Require near and far rows. No stale edges or synthetic zero-error output.
    valid = rows[0] is not None and rows[-1] is not None
    result = {"valid": valid, "rows": rows, "error_px": None,
              "heading_image_deg": None, "left_margin_px": None,
              "right_margin_px": None, "clearance_mm": None,
              "ground_error_mm": None, "ground_heading_deg": None,
              "camera_height_mm": CAMERA_HEIGHT_MM,
              "footprint_mm": (VEHICLE_WIDTH_MM, VEHICLE_LENGTH_MM),
              "stop_required": not valid, "warning": "EDGE_LOST" if not valid else ""}
    if not valid:
        return result
    near, far = rows[0], rows[-1]
    left_margin = TRACK_REF_X - near["left_px"]
    right_margin = near["right_px"] - TRACK_REF_X
    result.update({"error_px": near["center_px"]-TRACK_REF_X,
                   "heading_image_deg": math.atan2(far["center_px"]-near["center_px"],
                                                   near["y"]-far["y"]) * 180/math.pi,
                   "left_margin_px": left_margin, "right_margin_px": right_margin})
    # Positive error: track centre appears right of vehicle reference.
    if min(left_margin, right_margin) < TRACK_WARN_PX:
        result["stop_required"] = True
        result["warning"] = "PIXEL_MARGIN"
    mapping = active_ground_map()
    configured = GROUND_H is not None or bool(GROUND_PROFILES)
    if configured and (mapping is None or not FOOTPRINT_CONFIRMED):
        result["stop_required"] = True
        result["warning"] = "GROUND_CALIBRATION"
    elif mapping is not None and FOOTPRINT_CONFIRMED:
        left = [ground_point(r["left_px"], r["y"], mapping) for r in rows if r]
        right = [ground_point(r["right_px"], r["y"], mapping) for r in rows if r]
        clearance = ground_clearance(left, right) if None not in left + right else None
        if clearance is None:
            result["stop_required"] = True
            result["warning"] = "TRACK_GEOMETRY"
        else:
            result["clearance_mm"] = clearance
            centres = [((l[0]+r[0])/2, (l[1]+r[1])/2) for l, r in zip(left, right)]
            dx, dy = centres[-1][0]-centres[0][0], centres[-1][1]-centres[0][1]
            result["ground_heading_deg"] = math.atan2(dx, dy)*180/math.pi
            if abs(dy) > 1e-6:
                result["ground_error_mm"] = centres[0][0]-dx*centres[0][1]/dy
            if min(clearance) < SAFETY_MARGIN_MM:
                result["stop_required"] = True
                result["warning"] = "BODY_MARGIN"
    return result


def single_edge_result(img, profile, reference):
    """Find grey-road upper boundary, not wheel/claw silhouette at image bottom."""
    mask = img.copy()
    mask.binary([FLOOR_LAB])
    points = []
    ymin, ymax = EDGE_Y_RANGE
    for x in EDGE_X_SAMPLES:
        candidates = []
        for y in range(ymin+4, ymax-EDGE_MIN_FLOOR_ROWS):
            above = sum(white_pixel(mask, x, yy) for yy in range(y-4, y))
            below = sum(white_pixel(mask, x, yy) for yy in range(y, y+EDGE_MIN_FLOOR_ROWS))
            if above <= 1 and below >= EDGE_MIN_FLOOR_ROWS-1:
                candidates.append(y)
        if candidates:
            target = reference if reference is not None else (ymin+ymax)/2
            points.append((x, min(candidates, key=lambda y: abs(y-target))))
    result = {"valid": False, "rows": [], "profile": profile,
              "error_px": None, "heading_image_deg": None,
              "clearance_mm": None, "stop_required": True, "warning": "EDGE_LOST",
              "edge_points": points, "edge_y": None, "reference_y": reference}
    if len(points) < 5:
        return result
    mx = sum(p[0] for p in points)/len(points)
    my = sum(p[1] for p in points)/len(points)
    denom = sum((x-mx)**2 for x,y in points)
    slope = sum((x-mx)*(y-my) for x,y in points)/denom
    residual = math.sqrt(sum((y-my-slope*(x-mx))**2 for x,y in points)/len(points))
    if residual > EDGE_MAX_FIT_ERROR_PX:
        return result
    edge_y = my+slope*(160-mx)
    error = edge_y-reference if reference is not None else None
    result.update(valid=True, edge_y=edge_y, error_px=error,
                  heading_image_deg=math.atan(slope)*180/math.pi,
                  stop_required=error is None or abs(error)>EDGE_MAX_DEVIATION_PX,
                  warning="GROUND_CALIBRATION" if error is None else
                          ("PIXEL_MARGIN" if abs(error)>EDGE_MAX_DEVIATION_PX else ""))
    return result


def corner_end_seen(img, track, segment):
    limit = CORNER_END_LIMITS.get(segment)
    if limit is None or not track.get("valid") or track.get("edge_y") is None:
        return False
    mask = img.copy()
    mask.binary([FLOOR_LAB])
    side, stop_x = limit
    votes = 0
    for offset in (12, 20, 28):
        y = int(track["edge_y"])+offset
        if y >= EDGE_Y_RANGE[1]:
            continue
        runs = [r for r in row_runs(mask,y,join_gap=2) if r[1]-r[0]>=80]
        if not runs:
            continue
        left,right = max(runs,key=lambda r:r[1]-r[0])
        votes += int((side == "LEFT" and left >= stop_x) or
                     (side == "RIGHT" and right <= stop_x))
    return votes >= 2


class Vision:
    def __init__(self, hardware=None):
        self.hardware = hardware
        self.mode = BOOT_MODE
        self.task = None
        self.confirm = Confirmation()
        self.seq = 0
        self.track_profile = "CENTER"
        self.route_segment = 0
        self.edge_references = dict(EDGE_REFERENCES)
        if hardware:
            hardware.set_pose(self.mode)

    def set_mode(self, mode):
        if mode not in MODE_IDS:
            raise ValueError("unknown vision mode")
        self.mode = mode
        self.confirm.reset()
        if self.hardware:
            self.hardware.set_pose(mode)

    def reset_task(self):
        self.task = None
        self.confirm.reset()

    def process(self, img, now):
        self.seq += 1
        packet = {"seq": self.seq, "capture_ms": now, "mode": self.mode,
                  "valid": False, "locked": False, "task": self.task,
                  "objects": [], "selected": None, "track": None,
                  "max_age_ms": MAX_RESULT_AGE_MS}
        packet["pose_ready"] = self.hardware is None or self.hardware.ready(now)
        if not packet["pose_ready"]:
            packet["reason"] = "GIMBAL_SETTLING"
            self.confirm.reset()
            return packet
        if self.mode == "TRACK":
            packet["track"] = (track_result(img) if self.track_profile == "CENTER" else
                               single_edge_result(img, self.track_profile,
                                                  self.edge_references.get(self.route_segment)))
            packet["track"]["profile"] = self.track_profile
            packet["valid"] = packet["track"]["valid"]
        elif self.mode == "QR":
            r = find_qr(img)
            packet["valid"] = r is not None
            locked = self.confirm.update(r["task"]["raw"] if r else None)
            if r:
                packet["selected"] = r
            if locked:
                self.task = r["task"]
            packet["locked"] = locked
        elif self.mode == "BIN":
            blobs = img.find_blobs([(0,35,-15,15,-15,15)], roi=OBJECT_ROI,
                                    pixels_threshold=250, area_threshold=400, merge=False)
            objs = []
            for b in blobs:
                x,y,w,h = value(b,"rect")
                if min(w,h)>=20 and 0.4<=w/float(h)<=2:
                    objs.append({"cx":x+w//2,"cy":y+h//2,"rect":(x,y,w,h),
                                 "pixels":value(b,"pixels")})
            packet["objects"] = objs
            r = max(objs,key=lambda o:o["pixels"]) if objs else None
            packet["selected"] = r
            packet["valid"] = r is not None
            packet["locked"] = self.confirm.update("BLACK" if r else None,
                                (r["cx"],r["cy"]) if r else None)
        elif self.mode in ("BALL", "TARGET", "HOSTAGE"):
            shape_mode = self.mode == "HOSTAGE"
            if shape_mode:
                packet['hostage_debug'] = {}
                objs = hostage_objects(img, packet['hostage_debug'])
            else:
                objs = color_objects(img, self.mode == "TARGET")
            packet["objects"] = objs
            field = "shape" if shape_mode else "color"
            task_field = {"BALL": "ball_color", "TARGET": "target_color",
                          "HOSTAGE": "hostage_shape"}[self.mode]
            wanted = self.task[task_field] if self.task else None
            candidates = [o for o in objs if o[field] != 0 and
                          (wanted is None or o[field] == wanted)]
            r = max(candidates, key=lambda o: o["pixels"]) if candidates else None
            packet["selected"] = r
            packet["valid"] = r is not None
            packet["locked"] = self.confirm.update(r[field] if r else None,
                                                     (r["cx"], r["cy"]) if r else None)
            packet["matches_task"] = wanted is not None and r is not None
        packet["task"] = self.task
        return packet


def crc16(data):
    """CRC-16/CCITT-FALSE: poly 1021, init FFFF, refin/out false, xorout 0."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


# Calibrate these on the real gripper / camera before enabling automation.
AUTO_GRIP_CALIBRATED = False
GRIP_WAIT_MS = 1000
SEARCH_TIMEOUT_MS = 4000
PURPLE_LAB = (15, 85, 12, 75, -75, -5)  # calibrate on real finish mat
PURPLE_ROI = (20, 100, 280, 140)
PURPLE_MIN_PIXELS = 350
CORNER_STOP_CLEARANCE_MM = 18.0  # requires actual ground calibration
ALIGN_WINDOWS = {"BALL": (160, 130, 20, 20, 25),
                 "HOSTAGE": (160, 120, 20, 20, 25)}
# (cx, cy, x tolerance, y tolerance, minimum box width), QVGA pixels.
# Event requests are persistent until STM32 acknowledges their event ID.
# STM32 owns chassis/horizontal mechanism motion; OpenMV owns P5 laser.
class Mission(Vision):
    def __init__(self, hardware=None):
        super().__init__(hardware)
        self.stage = "CRUISE"
        self.position = 0
        self.event = None
        self.event_seq = 0
        self.started = time.ticks_ms()
        self.observed = Confirmation()
        self.gripping_until = None
        self.carrying = False
        self.corner_count = 0
        self.corner_enabled = False
        self.scanned = set()
        self.bin_ready = False
        self.laser_until = None
        self.finish_confirm = Confirmation()
        self.finish_enabled = False
        self.corner_seen = False
        self.edge_confirm = Confirmation()
        self.route_segment = 0

    def horizontal_request(self, signed_degrees):
        # Placeholder only: +right / -left. STM32 integration added later.
        self.set_mode("IDLE")
        self.stage = "WAIT_HORIZONTAL"
        self.emit(18, signed_degrees)

    def start_track_segment(self, profile, segment, learn=False):
        self.route_segment = segment
        self.track_profile = profile
        self.set_mode("TRACK")
        if profile == "CENTER" and self.hardware:
            self.hardware.set_tilt_angle(FINAL_TRACK_ANGLE)
            self.hardware.ready_at = time.ticks_add(time.ticks_ms(), GIMBAL_SETTLE_MS)
        self.edge_confirm.reset()
        self.corner_count = 0
        self.stage = "EDGE_ACQUIRE" if learn and profile != "CENTER" else "CRUISE"
        self.corner_enabled = False

    def laser_off(self):
        if self.hardware and hasattr(self.hardware, "laser"):
            self.hardware.laser(False)
        self.laser_until = None

    def service_laser(self, now):
        if self.laser_until is not None and time.ticks_diff(now, self.laser_until) >= 0:
            self.laser_off()

    def set_mode(self, mode):
        self.laser_off()
        super().set_mode(mode)

    def reset_task(self):
        super().reset_task()
        self.event = None
        self.scanned = set()
        self.gripping_until = None
        self.stage = "CRUISE"
        self.carrying = False
        self.bin_ready = False
        self.finish_enabled = False
        self.finish_confirm.reset()
        self.corner_enabled = False
        self.corner_seen = False
        self.set_mode("TRACK")
        self.track_profile = "CENTER"
        self.route_segment = 0
        self.edge_references = dict(EDGE_REFERENCES)
        self.edge_confirm.reset()

    def emit(self, action, arg=0):
        if self.event is None:
            self.event_seq = (self.event_seq+1) & 65535
            self.event = (self.event_seq, action, arg)

    def acknowledge(self, event_id):
        if self.event and self.event[0] == event_id:
            self.event = None
            return True
        return False

    def arrive(self, code):
        # New arrival means chassis stopped. Never infer arrival from elapsed time.
        if self.event is not None or self.gripping_until is not None:
            return False
        if code == 0:
            self.stage = "CRUISE"
            self.set_mode("TRACK")
        elif code == 1:
            self.stage = "READ_QR"
            self.set_mode("QR")
        elif code == 70:  # first lateral leg STOPPED -> head left90
            self.horizontal_request(-90)
        elif code == 71:  # left90 completed, still stopped -> first forward leg
            self.start_track_segment("SIDE", 2, learn=True)
        elif code == 72:  # first forward leg stopped -> head right90
            self.horizontal_request(90)
        elif code == 73:  # right90 completed, stopped -> second lateral leg
            self.start_track_segment("FRONT", 3, learn=True)
        elif code == 74:  # second lateral leg stopped -> head left90
            self.horizontal_request(-90)
        elif code == 75:  # left90 complete -> second forward leg
            self.start_track_segment("SIDE", 4, learn=True)
        elif code in (76, 78):  # corner stopped -> head right90; chassis right90 separately
            self.horizontal_request(90)
        elif code in (77, 79):  # head/chassis turns complete -> approach ball/hostage
            self.start_track_segment("SIDE", 5 if code == 77 else 8, learn=True)
        elif code == 80:  # first target stopped -> head left180
            self.horizontal_request(-180)
        elif 10 <= code <= 12 or 20 <= code <= 22 or 30 <= code <= 32:
            if self.task is None:
                return False
            group = code//10
            self.stage = {1:"BALL", 2:"TARGET", 3:"HOSTAGE"}[group]
            self.position = code%10
            self.set_mode(self.stage)
            if self.hardware and not self.carrying and self.stage in ("BALL", "HOSTAGE"):
                self.hardware.claw(CLAW_OPEN_ANGLE)
        elif code == 40:  # returned to second ball station, chassis stopped
            self.stage = "TO_BIN"
            self.set_mode("IDLE")
            self.emit(6, 180)  # STM32 rotates horizontal mechanism right180, NOT chassis
        elif code == 41:  # horizontal mechanism right180 complete, facing bin
            if not self.carrying:
                return False
            self.stage = "BIN"
            self.set_mode("BIN")
        elif code == 42:  # horizontal mechanism left90 complete after bucket release
            self.start_track_segment("SIDE", 6, learn=True)
        elif code == 50:  # target completed; keep horizontal pose, restore pitch100
            self.laser_off()
            self.start_track_segment("SIDE", 7, learn=True)
        elif code == 51:  # horizontal mechanism right90 complete after target
            self.stage = "CRUISE"
            self.set_mode("TRACK")
            self.corner_enabled = True
            self.corner_seen = False
            self.corner_count = 0
        elif code == 52:  # chassis right90 and straight route toward hostage begins
            self.stage = "CRUISE"
            self.set_mode("TRACK")
            self.corner_enabled = False
        elif code == 61:  # mechanism right90 complete after hostage grip
            if not self.carrying:
                return False
            self.start_track_segment("CENTER", 9)
            self.stage = "RETURN_CRUISE"
            self.finish_enabled = True
            self.finish_confirm.reset()
        elif code == 60:  # terminal arrival, chassis stopped
            self.stage = "DONE"
            self.set_mode("IDLE")
            self.emit(9)
        else:
            return False
        self.started = time.ticks_ms()
        self.observed.reset()
        return True

    def release_at_bin(self):
        # Explicit STM32 permission after physical bucket alignment.
        if self.stage != "BIN" or not self.bin_ready or self.event is not None or not AUTO_GRIP_CALIBRATED:
            return False
        if self.hardware is None or not self.hardware.claw(CLAW_OPEN_ANGLE):
            return False
        self.stage = "RELEASING"
        self.gripping_until = time.ticks_add(time.ticks_ms(), GRIP_WAIT_MS)
        return True

    def process(self, img, now):
        # Check deadline before expensive image processing as well as after.
        self.service_laser(now)
        packet = super().process(img, now)
        self.bin_ready = self.stage == "BIN" and packet["locked"] and packet["pose_ready"]
        if self.gripping_until is not None:
            if time.ticks_diff(now, self.gripping_until) >= 0:
                self.gripping_until = None
                if self.stage == "RELEASING":
                    self.carrying = False
                    self.stage = "WAIT_ROUTE"
                    self.set_mode("IDLE")
                    self.emit(7, 90)  # STM32 horizontal mechanism left90; then arrival42
                else:
                    self.carrying = True
                    self.stage = "WAIT_ROUTE"
                    self.set_mode("IDLE")  # no track claim while head still sideways
                    self.emit(4 if self.mode_before_grip == "BALL" else 5,
                              1 if self.mode_before_grip == "BALL" else 90)
                    # 4:return ball index1; 5:rotate mechanism right90, arrival61
        fresh = time.ticks_diff(time.ticks_ms(), now) <= MAX_RESULT_AGE_MS
        self.bin_ready = self.bin_ready and fresh
        if self.stage == "WAIT_TARGET":
            obj = packet.get("selected") or {}
            x,y,tx,ty = LASER_AIM_WINDOW
            aim_ok = (fresh and packet["pose_ready"] and packet["locked"] and
                      packet.get("matches_task") and
                      abs(obj.get("cx",-999)-x)<=tx and abs(obj.get("cy",-999)-y)<=ty)
            if not aim_ok or (self.laser_until is not None and
                             time.ticks_diff(time.ticks_ms(),self.laser_until)>=0):
                self.laser_off()
        if self.event is None and fresh and packet["pose_ready"] and self.gripping_until is None:
            if self.stage == "READ_QR" and packet["locked"]:
                self.start_track_segment("FRONT", 1, learn=True)
                self.emit(1, 0)  # task saved locally; never transmit QR contents
            elif self.stage == "EDGE_ACQUIRE":
                track = packet.get("track") or {}
                edge_y = track.get("edge_y")
                if edge_y is not None and self.edge_confirm.pos is not None and abs(edge_y-self.edge_confirm.pos[1]) > 3:
                    self.edge_confirm.reset()
                stable = self.edge_confirm.update(self.track_profile if track.get("valid") else None,
                                                  (160, edge_y) if edge_y is not None else None)
                if stable:
                    if self.edge_references.get(self.route_segment) is None:
                        self.edge_references[self.route_segment] = edge_y
                    self.stage = "CRUISE"
                    self.emit(17, self.route_segment)  # reference ready; STM32 may start leg
            elif self.stage in ("BALL", "TARGET", "HOSTAGE"):
                field = "shape" if self.stage == "HOSTAGE" else "color"
                known = [o for o in packet["objects"] if o.get(field,0)]
                # One station ROI must contain one object; multiple is ambiguous.
                obs = known[0] if len(known) == 1 else None
                stable = self.observed.update(obs[field] if obs else None,
                            (obs["cx"],obs["cy"]) if obs else None)
                desired = self.task[{"BALL":"ball_color","TARGET":"target_color",
                                     "HOSTAGE":"hostage_shape"}[self.stage]]
                packet["verdict"] = (1 if obs[field] == desired else 2) if stable else 0
                if stable and obs[field] != desired:
                    self.scanned.add((self.stage,self.position))
                    next_pos = next((p for p in range(3) if (self.stage,p) not in self.scanned), None)
                    self.emit(2, next_pos) if next_pos is not None else self.emit(10)
                    self.stage = "WAIT_ROUTE"
                elif stable and packet["locked"] and packet.get("matches_task"):
                    if self.stage == "TARGET":
                        x,y,tx,ty = LASER_AIM_WINDOW
                        aimed = abs(obs["cx"]-x)<=tx and abs(obs["cy"]-y)<=ty
                        if not aimed:
                            self.emit(11,self.position)  # STM32 stopped micro-adjustment
                        elif not LASER_AIM_CALIBRATED:
                            self.emit(15,self.position)  # laser aim calibration required
                        elif self.hardware and self.hardware.laser(True):
                            self.laser_until = time.ticks_add(time.ticks_ms(),LASER_ON_MS)
                            self.emit(3, self.position)  # laser started; NOT success proof
                            self.stage = "WAIT_TARGET"
                    else:
                        x,y,tx,ty,min_w = ALIGN_WINDOWS[self.stage]
                        aligned = (abs(obs["cx"]-x)<=tx and abs(obs["cy"]-y)<=ty and
                                   obs["rect"][2]>=min_w)
                        if not aligned:
                            self.emit(11,self.position)  # request stopped micro-adjustment
                        elif not AUTO_GRIP_CALIBRATED:
                            self.emit(12,self.position)  # review/calibrate grip first
                        elif self.hardware and self.hardware.claw(CLAW_CLOSE_ANGLE):
                            self.mode_before_grip = self.stage
                            self.gripping_until = time.ticks_add(now, GRIP_WAIT_MS)
                            self.stage = "GRIPPING"
                elif time.ticks_diff(now,self.started) > SEARCH_TIMEOUT_MS:
                    self.emit(13,self.position)  # unseen/ambiguous, NOT wrong target
            elif self.stage in ("RETURN_CRUISE", "WAIT_FINISH_STOP") and self.finish_enabled:
                blobs = img.find_blobs([PURPLE_LAB], roi=PURPLE_ROI,
                                      pixels_threshold=PURPLE_MIN_PIXELS,
                                      area_threshold=500, merge=False)
                candidates = [b for b in blobs if value(b,"w")>=25 and value(b,"h")>=12]
                b = max(candidates,key=lambda b:value(b,"pixels")) if candidates else None
                stable = self.finish_confirm.update("PURPLE" if b else None,
                          (value(b,"cx"),value(b,"cy")) if b else None)
                packet["purple_seen"] = stable
                if stable and self.stage == "RETURN_CRUISE":
                    self.emit(16)  # slow down / stop within purple; STM32 owns stop
                    self.stage = "WAIT_FINISH_STOP"
                elif stable and b and value(b,"y")+value(b,"h") >= 210:
                    self.emit(19)  # near purple ROI; STM32 must confirm actual chassis position
                    self.finish_enabled = False
            elif self.stage == "CRUISE" and self.corner_enabled:
                track = packet.get("track") or {}
                rows = track.get("rows") or []
                curve = False
                if len(rows)==3 and all(rows):
                    n,m,f = rows
                    predicted = n["center_px"]+(f["center_px"]-n["center_px"])*(n["y"]-m["y"])/(n["y"]-f["y"])
                    curve = abs(m["center_px"]-predicted)>12
                self.corner_count = self.corner_count+1 if curve else 0
                if self.corner_count>=CONFIRM_FRAMES:
                    self.corner_seen = True
                clearance = track.get("clearance_mm")
                if self.corner_seen and clearance is not None and min(clearance)<=CORNER_STOP_CLEARANCE_MM:
                    self.emit(14)  # stop request then STM32 chassis right90
                    self.corner_enabled=False
                elif self.corner_seen:
                    packet["corner_candidate"] = True  # no fabricated mm distance
        if self.stage == "CRUISE" and self.route_segment in CORNER_END_LIMITS and self.event is None:
            seen = corner_end_seen(img, packet.get("track") or {}, self.route_segment)
            self.corner_count = self.corner_count+1 if seen else 0
            if self.corner_count >= CONFIRM_FRAMES:
                self.emit(14, self.route_segment)
                self.stage = "WAIT_CORNER_STOP"
        if self.stage in ("EDGE_ACQUIRE", "WAIT_CORNER_STOP", "WAIT_HORIZONTAL") and packet.get("track"):
            packet["track"]["stop_required"] = True
        packet["mission_stage"] = self.stage
        packet["mission_event"] = self.event
        packet["station"] = self.position
        packet["route_segment"] = self.route_segment
        return packet


def frame_bytes(body):
    data = body.encode("ascii")
    return b"$" + data + ("*%04X#" % crc16(data)).encode("ascii")


def number(n, scale=1):
    if n is None:
        return INVALID_I16
    # Reserve -32768 exclusively for unavailable data.
    return max(-32767, min(32767, int(round(n * scale))))


def encode_packet(packet):
    """VIS V3: V2 fields plus track profile, segment, edge Y, reference Y, acquiring."""
    mode = packet.get("mode", "IDLE")
    pose = packet.get("pose_ready", False)
    fresh = not packet.get("stale", False)
    valid = bool(packet.get("valid", False) and pose and fresh)
    locked = bool(packet.get("locked", False) and valid)
    task = packet.get("task") or {}
    track = packet.get("track") or {}
    track_valid = mode == "TRACK" and valid and track.get("valid", False)
    clearance = track.get("clearance_mm") if track_valid else None
    stop = mode == "TRACK" and (not track_valid or track.get("stop_required", True))
    flags = (int(valid) | (int(locked) << 1) | (int(pose) << 2) |
             (int(bool(task)) << 3) | (int(packet.get("matches_task", False) and valid) << 4) |
             (int(stop) << 5) | (int(not fresh) << 6) | (int(clearance is not None) << 7))
    obj = packet.get("selected") if valid else None
    obj = obj or {}
    rect = obj.get("rect")
    cx, cy = obj.get("cx"), obj.get("cy")
    rows = track.get("rows") or []
    near = rows[0] if track_valid and rows else None
    far = rows[-1] if track_valid and rows else None
    warning = packet.get("reason", track.get("warning", ""))
    if not fresh:
        warning = "STALE"
    verdict = packet.get("verdict", 0) if pose and fresh else 0
    fields = [3, packet.get("seq", 0) & 0xFFFF, MODE_IDS[mode], flags,
              0, 0, 0, number(packet.get("processing_ms", 0)),
              verdict, number(cx), number(cy),
              number(rect[2] if rect else None), number(rect[3] if rect else None),
              number(cx - 160 if cx is not None else None),
              number(cy - 120 if cy is not None else None),
              min(255, len(packet.get("objects", []))),
              number(track.get("error_px") if track_valid else None),
              number(track.get("heading_image_deg") if track_valid else None, 100),
              number(track.get("left_margin_px") if track_valid else None),
              number(track.get("right_margin_px") if track_valid else None),
              number(near["left_px"] if near else None),
              number(near["right_px"] if near else None),
              number(far["left_px"] if far else None),
              number(far["right_px"] if far else None),
              number(clearance[0] if clearance is not None else None, 10),
              number(clearance[1] if clearance is not None else None, 10),
              WARNING_IDS.get(warning, 0),
              TRACK_PROFILES.get(track.get("profile", "CENTER"), 0),
              packet.get("route_segment", 0),
              number(track.get("edge_y") if track_valid else None),
              number(track.get("reference_y") if track_valid else None),
              int(packet.get("mission_stage") == "EDGE_ACQUIRE")]
    return frame_bytes("VIS:" + ",".join(str(n) for n in fields))


class SerialLink:
    """Bounded stream parser + latest telemetry + bounded ACK queue.

    Call poll and flush even when no visual frame is due. Partial writes never
    interleave with another frame. No motor-driver commands are forwarded.
    """
    def __init__(self, uart):
        self.uart = uart
        self.rx = bytearray()
        self.receiving = False
        self.rx_time = 0
        self.acks = []
        self.latest = None
        self.pending = None
        self.offset = 0
        self.tx_started = 0
        self.bad_frames = 0
        self.tx_errors = 0
        self.command_history = []  # last 8 accepted commands, duplicate ACK replay

    def handle_frame(self, data, vision):
        try:
            text = data.decode("ascii")
            body, checksum = text.rsplit("*", 1)
            if len(checksum) != 4 or any(c not in "0123456789abcdefABCDEF" for c in checksum):
                raise ValueError("checksum text")
            if crc16(body.encode("ascii")) != int(checksum, 16):
                raise ValueError("CRC")
            if not body.startswith("VCMD:"):
                raise ValueError("prefix")
            tokens = body[5:].split(",")
            if len(tokens) != 4:
                raise ValueError("field count")
            if any(not t or any(c not in "0123456789" for c in t) for t in tokens):
                raise ValueError("integer text")
            version, seq, command, arg = [int(t) for t in tokens]
            if version != 1 or not 0 <= seq <= 65535:
                raise ValueError("version/sequence")
        except (ValueError, UnicodeError):
            self.bad_frames += 1
            return
        # Backpressure: do not execute a command whose ACK cannot be queued.
        if len(self.acks) >= COMMAND_QUEUE_LIMIT:
            return
        for old_seq, old_cmd, old_arg, old_ack in self.command_history:
            if old_seq == seq:
                if (old_cmd, old_arg) == (command, arg):
                    self.acks.append(old_ack)
                else:
                    self.acks.append(frame_bytes("VACK:1,%d,2,%d" % (seq, MODE_IDS[vision.mode])))
                return
        status = 0
        if command == 1:
            mode = next((k for k in MODE_IDS if MODE_IDS[k] == arg), None)
            if mode is None:
                status = 1
            elif vision.mode != mode:
                vision.set_mode(mode)  # repeated mode command is idempotent
        elif command == 2 and arg == 0:
            vision.reset_task()
        elif command == 3 and arg == 0:
            pass  # ping, returns current mode
        elif command == 4 and isinstance(vision, Mission):
            status = 0 if vision.arrive(arg) else 1
        elif command == 5 and isinstance(vision, Mission):
            status = 0 if vision.acknowledge(arg) else 1
        elif command == 6 and arg == 0 and isinstance(vision, Mission):
            status = 0 if vision.release_at_bin() else 1
        elif command == 7 and arg in (0,1) and isinstance(vision, Mission):
            vision.corner_enabled = bool(arg)
            vision.corner_count = 0
        else:
            status = 1
        self.latest = None  # discard any unsent pre-command telemetry
        ack = frame_bytes("VACK:1,%d,%d,%d" % (seq, status, MODE_IDS[vision.mode]))
        self.acks.append(ack)
        self.command_history.append((seq, command, arg, ack))
        if len(self.command_history) > 8:
            self.command_history.pop(0)

    def poll(self, vision, now):
        if self.receiving and time.ticks_diff(now, self.rx_time) > RX_TIMEOUT_MS:
            self.rx = bytearray()
            self.receiving = False
            self.bad_frames += 1
        if self.uart is None or not self.uart.any():
            return
        data = self.uart.read(128)  # bounded per-loop work
        if not data:
            return
        for byte in data:
            if byte == 36:  # '$' always resynchronizes
                self.rx = bytearray()
                self.receiving = True
                self.rx_time = now
            elif self.receiving:
                self.rx_time = now
                if byte == 35:  # '#'
                    self.receiving = False
                    self.handle_frame(self.rx, vision)
                    self.rx = bytearray()
                elif len(self.rx) >= RX_FRAME_LIMIT:
                    self.rx = bytearray()
                    self.receiving = False
                    self.bad_frames += 1
                else:
                    self.rx.append(byte)

    def submit(self, packet, now):
        payload = encode_packet(packet)
        event = packet.get("mission_event")
        if event:
            payload += frame_bytes("VEVT:1,%d,%d,%d" % event)
        if len(payload) > TX_FRAME_LIMIT:
            raise ValueError("visual frame too long")
        self.latest = (payload, now)  # replace, never queue old results

    def flush(self, now):
        if self.uart is None:
            self.latest = None
            self.acks = []
            return
        if self.pending is not None and time.ticks_diff(now, self.tx_started) > TX_TIMEOUT_MS:
            # The next '$' resynchronizes STM32's parser after a broken frame.
            self.pending = None
            self.tx_errors += 1
        if self.pending is None:
            if self.acks:
                self.pending = self.acks.pop(0)
            elif self.latest is not None:
                payload, queued_at = self.latest
                self.latest = None
                if time.ticks_diff(now, queued_at) > MAX_RESULT_AGE_MS:
                    return
                self.pending = payload
            else:
                return
            self.offset = 0
            self.tx_started = now
        try:
            sent = self.uart.write(self.pending[self.offset:])
        except OSError:
            self.tx_errors += 1
            return  # retain suffix; never interleave frames
        if sent:
            self.offset += sent
            if self.offset >= len(self.pending):
                self.pending = None


class FixedRouteMission(Mission):
    """Only the two waiting points supported by current STM32 route firmware."""
    def __init__(self, hardware=None):
        super().__init__(hardware)
        self.set_mode('IDLE')
        self.stage = 'WAIT_QR_REQUEST'
        self.reply = None
        self.deadline = None

    def fail_route(self, reason):
        self.laser_off()
        self.reply = None
        self.gripping_until = None
        self.stage = 'ROUTE_ERROR'
        self.set_mode('IDLE')
        print('Fixed route stopped:', reason)

    def request(self, name, now):
        if name == 'qr' and self.stage == 'WAIT_QR_REQUEST':
            self.task = None
            self.stage = 'FIXED_QR'
            self.set_mode('QR')
        elif name == 'globule' and self.stage == 'WAIT_BALL_REQUEST' and self.task:
            self.stage = 'FIXED_BALL'
            self.set_mode('BALL')
            if self.hardware:
                self.hardware.claw(CLAW_OPEN_ANGLE)
        else:
            return  # duplicate/out-of-order requests never restart work or grip
        self.deadline = time.ticks_add(now, 29000)  # STM32 owns the 30s timeout

    def process(self, img, now):
        packet = Vision.process(self, img, now)
        current = time.ticks_ms()
        if self.deadline is not None and time.ticks_diff(current,self.deadline)>=0:
            self.fail_route('LOCAL_VISION_TIMEOUT')
            self.deadline = None
        fresh = time.ticks_diff(current,now) <= MAX_RESULT_AGE_MS
        if self.stage == 'FIXED_QR' and fresh and packet['locked']:
            self.reply = b'finishqr'
            self.stage = 'QR_REPLY_PENDING'
            self.set_mode('TRACK')  # restores configured track pitch only
        elif self.stage == 'FIXED_BALL' and fresh and packet['pose_ready']:
            objects = packet['objects']
            if len(objects)==1 and packet['locked'] and packet.get('matches_task'):
                obj = packet['selected']
                x,y,tx,ty,min_w = ALIGN_WINDOWS['BALL']
                aligned = abs(obj['cx']-x)<=tx and abs(obj['cy']-y)<=ty and obj['rect'][2]>=min_w
                if not aligned:
                    packet['reason'] = 'BALL_ALIGNMENT_REQUIRED'
                elif not AUTO_GRIP_CALIBRATED:
                    packet['reason'] = 'GRIP_CALIBRATION_REQUIRED'
                elif self.hardware and self.hardware.claw(CLAW_CLOSE_ANGLE):
                    self.gripping_until = time.ticks_add(current,GRIP_WAIT_MS)
                    self.stage = 'FIXED_GRIPPING'
        elif self.stage == 'FIXED_GRIPPING' and time.ticks_diff(current,self.gripping_until)>=0:
            self.carrying = True
            self.gripping_until = None
            self.reply = b'finishglobule'
            self.stage = 'BALL_REPLY_PENDING'
            self.set_mode('IDLE')
        packet['mission_stage'] = self.stage
        packet['mission_event'] = None
        return packet


def uart_write_all(uart, data):
    """Complete-write loop from the verified uart_start_pc_test.py."""
    if uart is None:
        return False
    offset = 0
    while offset < len(data):
        sent = uart.write(data[offset:])
        if not sent:
            print('UART TX failed')
            return False
        offset += sent
    return True


def uart_read_chunk(uart):
    """Read once, display once, then give the same bytes to the route parser."""
    if uart is None or not uart.any():
        return b''
    data = uart.read(64) or b''
    if data and UART_DEBUG_RAW:
        print('RX RAW:', repr(data))
        print('RX HEX:', ' '.join('%02X' % byte for byte in data))
        try:
            print('RX TEXT:', data.decode('ascii'))
        except UnicodeError:
            print('RX TEXT: non-ASCII; see RAW/HEX')
    return data


class FixedRouteLink:
    def __init__(self, uart, vision):
        self.uart, self.vision = uart, vision
        self.rx = bytearray()
        self.dropping = False
        self.start_at = None
        self.start_attempted = False
        self.last_start_at = None
        self.start_status = 'NOT_SENT'
        self.start_confirmed = False
        self.start_sequence = 0
        self.next_start = time.ticks_add(time.ticks_ms(), START_DELAY_MS)
        self.rx_bytes = 0
        self.rx_last_at = None
        self.rx_lines = 0
        self.wait_warned = False

    def send(self, data):
        try:
            sent = uart_write_all(self.uart, data)
        except OSError as error:
            print('UART TX error:', error)
            sent = False
        if not sent:
            self.vision.fail_route('UART_TX_FAILED')
            return False
        print('OpenMV TX -> STM32:', repr(data), 'bytes:', len(data))
        return True

    def send_start(self):
        if not SEND_START_ON_BOOT or self.start_confirmed or self.vision.stage != 'WAIT_QR_REQUEST':
            return False
        self.start_attempted = True
        self.start_sequence += 1
        if not self.send(START_COMMAND):
            print('TX', self.start_sequence, 'FAILED')
            self.start_status = 'TX_FAILED'
            return False
        print('TX', self.start_sequence, 'OK HEX:',
              ' '.join('%02X' % byte for byte in START_COMMAND))
        self.last_start_at = time.ticks_ms()
        self.next_start = time.ticks_add(self.last_start_at, START_REPEAT_MS)
        if self.start_at is None:
            self.start_at = self.last_start_at
        self.start_status = 'SENT_NOT_CONFIRMED'
        print('START sent; repeating every',START_REPEAT_MS,'ms until finishstart')
        return True

    def check_start_wait(self, now):
        if (self.start_at is not None and not self.start_confirmed and not self.wait_warned and
                self.vision.stage == 'WAIT_QR_REQUEST' and
                time.ticks_diff(now,self.start_at) >= START_WAIT_WARNING_MS):
            self.wait_warned = True
            print('START_WAIT_WARNING:',self.start_status,
                  'RX bytes:',self.rx_bytes,'lines:',self.rx_lines,
                  'unfinished line:',bytes(self.rx),
                  'No qr yet; START retries active; check STM32 reply/preparation')

    def handle_line(self, line, now):
        self.rx_lines += 1
        print('STM32 RX:',line)
        text = line.strip().lower()
        if text == 'finishstart':
            if self.start_attempted and not self.start_confirmed and self.vision.stage == 'WAIT_QR_REQUEST':
                self.start_confirmed = True
                self.start_status = 'START_CONFIRMED'
                print('START confirmed; retries stopped, waiting for qr')
        elif text in ('qr','globule'):
            self.vision.request(text,now)
            if text == 'qr' and self.vision.stage == 'FIXED_QR':
                self.start_status = 'QR_REQUEST_RECEIVED'
        elif 'error' in text.split():
            self.start_status = 'STM32_ERROR'
            self.vision.fail_route(line)
        elif text == '0 done route' and self.vision.stage == 'WAIT_ROUTE_DONE':
            self.vision.stage = 'DONE'
        elif text.startswith('0 ready') and self.vision.stage != 'WAIT_QR_REQUEST':
            self.vision.fail_route('STM32_RESTART')

    def parse_received(self, now):
        # START ACK has a unique long token. Recover even after stray bytes.
        ack = b'finishstart'
        while True:
            index = bytes(self.rx).lower().find(ack)
            if index < 0:
                break
            prefix = bytes(self.rx[:index])
            if prefix:
                # Keep errors/status lines meaningful rather than hiding faults.
                for line in prefix.replace(b'\r', b'\n').split(b'\n'):
                    if line:
                        self.handle_line(line.decode('ascii', 'replace'), now)
            del self.rx[:index + len(ack)]
            self.handle_line('finishstart', now)

        # Parse complete diagnostic lines first. No substring qr matching.
        while True:
            separators = [i for i, byte in enumerate(self.rx) if byte in (10, 13)]
            if not separators:
                break
            end = separators[0]
            line = bytes(self.rx[:end])
            del self.rx[:end + 1]
            if line:
                self.handle_line(line.decode('ascii', 'replace'), now)

        # Bare task words have no delimiters. Brief silence distinguishes qr
        # from qrcode within a burst. Arbitrarily delayed debug text remains
        # ambiguous: STM32 must keep task words separate from debug text.
        quiet = (self.rx_last_at is not None and
                 time.ticks_diff(now, self.rx_last_at) >= 30)
        if not quiet or not self.rx:
            return
        token = bytes(self.rx).strip().lower()
        commands = []
        remainder = token
        while remainder:
            match = next((word for word in (b'qr', b'globule')
                          if remainder.startswith(word)), None)
            if match is None:
                break
            commands.append(match)
            remainder = remainder[len(match):]
        if commands and not remainder:
            self.rx = bytearray()
            for command in commands:
                self.handle_line(command.decode('ascii'), now)
        elif b'error' in token.split() or token == b'0 done route':
            self.rx = bytearray()
            self.handle_line(token.decode('ascii', 'replace'), now)
        elif time.ticks_diff(now, self.rx_last_at) >= 200:
            print('UART ignored unterminated data:', repr(bytes(self.rx)))
            self.rx = bytearray()

    def poll(self, vision, now):
        # Drain bounded batches before parsing or scheduling another START.
        if self.uart is not None:
            for _ in range(4):
                if not self.uart.any():
                    break
                data = uart_read_chunk(self.uart)
                if not data:
                    break
                self.rx_bytes += len(data)
                self.rx_last_at = now
                self.rx.extend(data)
                self.parse_received(now)
                if len(self.rx) > 256:
                    print('UART RX overflow; preserving trailing token prefix')
                    # Long unknown data must not permanently block later ACKs.
                    self.rx = self.rx[-32:]
        self.parse_received(now)
        self.check_start_wait(now)

    def submit(self, packet, now):
        pass  # fixed route firmware has no VIS/VEVT/correction interface

    def flush(self, now):
        v = self.vision
        # Same next_send/ticks_diff schedule as uart_start_pc_test.py.
        # Unlike the standalone sender, a full ACK stops all START attempts.
        if (SEND_START_ON_BOOT and not self.start_confirmed and
                v.stage == 'WAIT_QR_REQUEST' and
                time.ticks_diff(now, self.next_start) >= 0):
            self.send_start()
        if v.reply is not None:
            if v.deadline is not None and time.ticks_diff(now,v.deadline)>=0:
                v.fail_route('REPLY_TOO_LATE')
                return
            reply = v.reply
            if self.send(reply):
                v.reply = None
                v.deadline = None
                v.stage = 'WAIT_BALL_REQUEST' if reply==b'finishqr' else 'WAIT_ROUTE_DONE'


class ActionsLink:
    """ACTIONS_V2 bridge. No invented chassis translation/arrival commands."""
    def __init__(self, uart):
        self.uart = uart
        self.rx = bytearray()
        self.dropping = False
        self.next_id = 0
        self.pending = None
        self.yaw_position = None  # -90 left, 0 front, +90 right
        self.error = None
        self.last_status = None
        self.blocked_event = None
        self.yaw_event_goal = None
        self.route_link = SerialLink(uart)
        self.route_ready = False

    def handle_route_frame(self, frame, vision):
        # Reuse bounded CRC parser, duplicate history and command ACK handling.
        before = len(self.route_link.acks)
        try:
            body = frame.split(b'*',1)[0]
            tokens = body.split(b':',1)[1].split(b',')
            command = int(tokens[2])
        except (ValueError, IndexError):
            command = None
        if command == 5 and vision.event and vision.event[1] in (18,6,7,5):
            return  # yaw event completion belongs exclusively to physical DONE
        if command in (1,2,4,6) and self.pending:
            return  # no mode/reset/arrival/release change during mechanism motion
        self.route_link.handle_frame(frame, vision)
        if len(self.route_link.acks) > before:
            ack_body = self.route_link.acks[-1].split(b'*',1)[0]
            if ack_body.split(b',')[-2] == b'0':
                self.route_ready = True

    def send_start(self):
        if self.uart is None:
            return False
        offset = 0
        started = time.ticks_ms()
        while offset < len(START_COMMAND):
            sent = self.uart.write(START_COMMAND[offset:])
            if not sent or time.ticks_diff(time.ticks_ms(), started) > 100:
                self.error = "START_TX_FAILED"
                return False
            offset += sent
        print("STM32 TX: start (sent, execution not confirmed)")
        return True

    def request_action(self, command, vision=None, event=None, arrival=None, goal=None):
        # ZERO must be explicitly requested only after physically setting yaw forward.
        allowed = ("BASE_L90", "BASE_R90", "YAW_L90", "YAW_HOME",
                   "YAW_R90", "YAW_R180", "ZERO", "STOP", "STATUS", "CAL", "HELP")
        parts = command.split()
        if not parts or (command not in allowed and not
                (len(parts)==2 and parts[0] in ("LIFT", "RACK", "SPEED") and
                 parts[1].lstrip('-').isdigit())):
            raise ValueError("unsupported ACTIONS_V2 command")
        if self.pending is not None or self.uart is None:
            return False
        auxiliary = parts[0] in ("ZERO", "STOP", "STATUS", "CAL", "HELP", "SPEED")
        if auxiliary and event is not None:
            return False
        self.next_id += 1
        if self.next_id > 4294967295:
            raise ValueError("action IDs exhausted; reset both controllers")
        ident = 0 if auxiliary else self.next_id
        frame = ((command if auxiliary else "%d %s" % (ident,command))+"\r\n").encode('ascii')
        offset = 0
        started = time.ticks_ms()
        while offset < len(frame):
            sent = self.uart.write(frame[offset:])
            if not sent or time.ticks_diff(time.ticks_ms(),started)>100:
                self.error = "TX_LOST"
                self.yaw_position = None
                return False
            offset += sent
        if command == "STOP":
            self.yaw_position = None
        if parts[0] not in ("STATUS", "HELP"):
            self.pending = (ident, command, started, event, arrival, goal)
        print("STM32 TX:", frame)
        return True

    def handle_line(self, line, vision):
        self.last_status = line
        if line.startswith('#'):
            # Only trust reported zero reference and actual encoder position.
            fields = dict(t.split('=',1) for t in line.split() if '=' in t)
            if fields.get('ZERO') == '0':
                self.yaw_position = None
            return
        print("STM32 RX:", line)
        parts = line.split()
        if len(parts)<2 or not parts[0].isdigit():
            return
        ident, state = int(parts[0]), parts[1].upper()
        if state == 'READY':
            self.pending = None
            self.yaw_position = None
            self.error = "STM32_RESTART_REQUIRES_ZERO"
            return
        if ident == 0 and state == 'ERROR' and self.pending is None:
            self.error = ' '.join(parts[2:]) or 'STM32_ERROR'
            vision.laser_off()
            return
        if not self.pending or ident != self.pending[0]:
            return
        _,command,_,event,arrival,goal = self.pending
        if state == 'ERROR':
            self.error = ' '.join(parts[2:]) or 'ERROR'
            self.pending = None
            self.yaw_position = None
            vision.laser_off()
            return
        if state != 'DONE':
            return
        self.pending = None
        self.error = None
        if command == 'ZERO':
            self.yaw_position = 0
        elif goal is not None:
            self.yaw_position = goal
        if event is not None:
            vision.acknowledge(event[0])
        if arrival is not None and not vision.arrive(arrival):
            self.error = "ARRIVAL_REJECTED"

    def poll(self, vision, now):
        if self.uart is not None and self.uart.any():
            for byte in self.uart.read(256) or b'':
                if byte == 36 and ROUTE_V3_ENABLE:  # '$' starts CRC route frame
                    self.rx = bytearray(b'$')
                    self.dropping = False
                elif byte == 35 and self.rx[:1] == b'$' and ROUTE_V3_ENABLE:
                    self.handle_route_frame(bytes(self.rx[1:]),vision)
                    self.rx = bytearray()
                    self.dropping = False
                elif byte in (10,13):
                    if self.rx and not self.dropping:
                        try:
                            self.handle_line(self.rx.decode('ascii'),vision)
                        except UnicodeError:
                            self.error = "RX_NON_ASCII"
                    self.rx = bytearray()
                    self.dropping = False
                elif not self.dropping:
                    if len(self.rx)>=256:
                        self.dropping = True
                        self.rx = bytearray()
                        self.error = "LINE_DISCARDED"
                    else:
                        self.rx.append(byte)
        if self.pending and time.ticks_diff(now,self.pending[2])>20000:
            self.error = "ACTION_REPLY_TIMEOUT"
            self.pending = None
            self.yaw_position = None
            vision.laser_off()
            if self.uart:
                self.uart.write(b'STOP\r\n')

    def submit(self, packet, now):
        if ROUTE_V3_ENABLE and self.route_ready:
            copy = dict(packet)
            event = copy.get('mission_event')
            # Horizontal events are consumed by the ACTIONS_V2 bridge, not twice.
            if event and event[1] in (18,6,7,5):
                copy['mission_event'] = None
            self.route_link.submit(copy, now)

    def flush(self, now):
        if ROUTE_V3_ENABLE:
            self.route_link.flush(now)

    def dispatch(self, vision):
        event = vision.event
        if not event or self.pending or self.error:
            return
        _,action,arg = event
        arrival = None
        if action == 18:
            if self.yaw_event_goal is None or self.yaw_event_goal[0] != event[0]:
                self.yaw_event_goal = (event[0], None if self.yaw_position is None else self.yaw_position+arg)
            goal = self.yaw_event_goal[1]
            arrival = {1:71,2:73,3:75}.get(vision.route_segment)
            # At corners the chassis must also turn: do not infer arrival77/79.
        elif action == 6:
            goal,arrival = 90,41
        elif action == 7:
            goal,arrival = 0,42
        elif action == 5:
            goal,arrival = 0,61
        else:
            if self.blocked_event != event:
                print("LOCAL mission event (no STM32 API):",event)
                self.blocked_event = event
            return
        source = self.yaw_position
        command = None
        if goal == 0 and source in (-90,90): command = 'YAW_HOME'
        elif goal == -90 and source == 0: command = 'YAW_L90'
        elif goal == 90 and source == 0: command = 'YAW_R90'
        elif goal == 90 and source == -90: command = 'YAW_R180'
        elif goal == -90 and source == 90:
            # No YAW_L180 in ACTIONS_V2: first HOME, then dispatch same event again.
            self.request_action('YAW_HOME', vision, goal=0)
            return
        if command is None:
            if self.blocked_event != event:
                print("YAW blocked: need explicit ZERO / supported pose",source,goal,event)
                self.blocked_event = event
            return
        self.request_action(command,vision,event,arrival,goal)


def poll_control(link, vision, now):
    link.poll(vision, now)
    if isinstance(link, ActionsLink):
        link.dispatch(vision)


def draw_result(img, packet):
    # Always called after all recognition is finished.
    debug = packet.get('hostage_debug')
    if packet['mode'] == 'HOSTAGE' and HOSTAGE_DEBUG_BINARY:
        img.binary([HOSTAGE_LAB])
    if debug:
        img.draw_rectangle(debug['roi'], color=(255,255,0))
        img.draw_string(2,16,debug['reason'],color=(255,100,0))
        for rect,reason in debug['rejected']:
            img.draw_rectangle(rect,color=(255,0,0))
            img.draw_string(rect[0],max(30,rect[1]),reason,color=(255,0,0))
    elif packet['mode'] == 'HOSTAGE':
        img.draw_string(2,16,packet.get('reason','WAIT_FRAME'),color=(255,100,0))
    img.draw_string(2, 2, packet["mode"], color=(255, 0, 0))
    for o in packet["objects"]:
        img.draw_rectangle(o["rect"], color=(0, 255, 0))
        img.draw_cross(o["cx"], o["cy"], color=(255, 0, 0))
        metrics = o.get("shape_metrics")
        if metrics:
            x,y,w,h = o["rect"]
            name = {0:"UNKNOWN",1:"CYLINDER",2:"CONE",3:"DRUM"}[o["shape"]]
            img.draw_string(x,max(14,y-12),name,color=(255,255,0))
            img.draw_string(x,min(img.height()-12,y+h),metrics["reason"],color=(255,255,0))
            for i,width in enumerate(o["width_profile"]):
                yy = o['profile_y'][i]
                center = o['profile_centers'][i]
                if width > 0 and center >= 0:
                    img.draw_line(int(center-width/2),yy,int(center+width/2),yy,color=(0,255,255))
    r = packet["selected"]
    if r:
        img.draw_rectangle(r.get("display_rect", r["rect"]), color=(255, 255, 0))
    if packet["track"]:
        track = packet["track"]
        for x,y in track.get("edge_points", []):
            img.draw_cross(x,y,color=(0,255,255))
        ref = track.get("reference_y")
        if ref is not None:
            img.draw_line(40,int(ref),280,int(ref),color=(255,255,0))
        img.draw_string(2,16,'%s SEG:%s' % (track.get("profile","CENTER"),
                                         packet.get("route_segment",0)),color=(255,255,0))
        for row in packet["track"]["rows"]:
            if row:
                img.draw_line(row["left_px"], row["y"], row["right_px"], row["y"], color=(0, 255, 0))
                img.draw_cross(int(row["center_px"]), row["y"], color=(255, 0, 0))


def make_uart():
    """Use the exact configuration verified by uart_start_pc_test.py."""
    if not UART_ENABLE:
        return None
    from pyb import UART
    return UART(UART_ID, UART_BAUD, bits=8, parity=None, stop=1,
                timeout=20, timeout_char=20, read_buf_len=256)


def wait_start_handshake(link, vision):
    """Test-script receive/send loop, with route ACK and error handling."""
    print('BOOT TEST-BASED v3: UART1; START sender from standalone test')
    print('Receiving now; first START in', START_DELAY_MS, 'ms')
    print('Expected TX HEX: 53 54 41 52 54')
    last_report = time.ticks_ms()
    while not link.start_confirmed:
        # Read/display/parser share one read. Never discard task bytes.
        link.poll(vision, time.ticks_ms())
        if vision.stage == 'ROUTE_ERROR':
            raise RuntimeError('STM32 startup error; see preceding RX TEXT')
        if vision.stage != 'WAIT_QR_REQUEST' and not link.start_confirmed:
            raise RuntimeError('Task arrived before finishstart')
        link.flush(time.ticks_ms())
        now = time.ticks_ms()
        if time.ticks_diff(now, last_report) >= 5000:
            print('BOOT UART status: RX bytes:', link.rx_bytes)
            last_report = now
        time.sleep_ms(5)
    print('BOOT UART PASS: finishstart received; START stopped')


def main():
    global ACTION_LINK, MISSION
    print('BOOT: initializing UART')
    uart = make_uart()
    if uart is not None:
        print('UART1: P1 TX, P0 RX; 115200 8N1')
    early_vision = early_link = None
    if STM32_PROTOCOL == 'FIXED_ROUTE':
        early_vision = FixedRouteMission()
        early_link = FixedRouteLink(uart,early_vision)
        print('BOOT: OpenMV sends START until finishstart; STM32 starts chassis')
        if SEND_START_ON_BOOT:
            if uart is None:
                raise ValueError('UART must be enabled for route startup')
            wait_start_handshake(early_link, early_vision)

    def service_boot_uart():
        if early_link:
            now = time.ticks_ms()
            early_link.poll(early_vision,now)
            early_link.flush(now)

    print('BOOT: initializing light and servos')
    hardware = Hardware()  # illumination must be on before AWB/AGC warmup
    print('BOOT: initializing camera')
    cam = make_camera(service_boot_uart)
    camera_modes = CameraModes(cam)
    print('BOOT: camera ready')
    if STM32_PROTOCOL not in ("FIXED_ROUTE", "ACTIONS_V2", "VISION_V3"):
        raise ValueError("unknown STM32 protocol")
    vision = early_vision if early_vision is not None else Mission(hardware)
    if early_vision is not None:
        vision.hardware = hardware
        hardware.set_pose(vision.mode)
        vision.confirm.reset()  # camera warming frames did not perform recognition
    link = (early_link if STM32_PROTOCOL == 'FIXED_ROUTE' else
            (ActionsLink(uart) if STM32_PROTOCOL == "ACTIONS_V2" else SerialLink(uart)))
    ACTION_LINK = link if isinstance(link, ActionsLink) else None
    MISSION = vision
    if isinstance(link, FixedRouteLink):
        # Preserve buffered boot replies; never discard them as stale data.
        for _ in range(4):
            if uart is None or not uart.any():
                break
            link.poll(vision,time.ticks_ms())
    start_due = None
    if SEND_START_ON_BOOT and isinstance(link, ActionsLink):
        start_due = time.ticks_add(time.ticks_ms(), START_DELAY_MS)
        print('BOOT: waiting', START_DELAY_MS, 'ms before START; payload:', repr(START_COMMAND))
    last_send = time.ticks_ms()
    last_print = last_send
    frames = 0
    while True:
        vision.service_laser(time.ticks_ms())
        poll_control(link, vision, time.ticks_ms())
        if start_due is not None and time.ticks_diff(time.ticks_ms(),start_due) >= 0:
            start_due = None  # legacy ActionsLink first send only
            if vision.stage != 'ROUTE_ERROR':
                link.send_start()
        link.flush(time.ticks_ms())
        img = camera_modes.snapshot(vision.mode)
        now = time.ticks_ms()
        vision.service_laser(now)
        try:
            packet = vision.process(img, now)
        except BaseException:
            vision.laser_off()
            raise
        done = time.ticks_ms()
        packet["processing_ms"] = time.ticks_diff(done, now)
        packet["stale"] = packet["processing_ms"] > MAX_RESULT_AGE_MS
        if packet["stale"]:
            packet["valid"] = False
            packet["locked"] = False
            if packet["track"]:
                packet["track"]["stop_required"] = True
                packet["track"]["valid"] = False
                packet["track"]["warning"] = "STALE"
        if time.ticks_diff(done, last_send) >= SEND_INTERVAL_MS:
            link.submit(packet, done)
            last_send = done
        link.flush(time.ticks_ms())
        if time.ticks_diff(done, last_print) >= 500:
            print(packet)
            last_print = done
        if DEBUG_DRAW:
            draw_result(img, packet)
        frames += 1
        if frames % 30 == 0:
            gc.collect()


if __name__ == "__main__":
    try:
        main()
    finally:
        if LASER_PIN is not None:
            from pyb import Pin
            Pin(LASER_PIN, Pin.OUT_PP).low()

