from machine import UART, Pin
import time
import sys
import select

# =====================================================================
# Kerfur neck controller - Pico (proof-of-concept)
# Receives "yaw,pitch\n" over USB serial from the host bridge node,
# drives the 6-axis Stewart platform. Physical inputs for home/neutral/
# goto-home and a tracking-enable jumper.
# =====================================================================

# ---------------------------------------------------------------------
# CRC8-ATM (poly 0x07), over all bytes except the CRC itself
# ---------------------------------------------------------------------
def crc8(data):
    crc = 0
    for b in data:
        for _ in range(8):
            if (crc >> 7) ^ (b & 1):
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
            b >>= 1
    return crc

# ---------------------------------------------------------------------
# STEP/DIR pin mapping  (DIR, STEP) per axis
# ---------------------------------------------------------------------
STEP_DIR_PINS = {
    "A": (7, 6),
    "B": (11, 10),
    "C": (15, 14),
    "D": (27, 26),
    "E": (21, 20),
    "F": (17, 16),
}

step_pins = {}
dir_pins = {}
for name, (d, s) in STEP_DIR_PINS.items():
    dir_pins[name] = Pin(d, Pin.OUT)
    step_pins[name] = Pin(s, Pin.OUT)
    dir_pins[name].value(0)
    step_pins[name].value(0)

# ---------------------------------------------------------------------
# TMC2209 UART register access
# ---------------------------------------------------------------------
def read_reg(bus, addr, reg):
    req = bytearray([0x05, addr, reg & 0x7F])
    req.append(crc8(req))
    while bus.any():
        bus.read()
    bus.write(req)
    time.sleep_ms(20)
    resp = bus.read()
    if resp is None or len(resp) < 12:
        return None
    reply = resp[4:12]                      # drop 4-byte echo, keep 8-byte reply
    if reply[0] != 0x05 or reply[1] != 0xFF:
        return None
    if crc8(reply[:7]) != reply[7]:
        return None
    return (reply[3] << 24) | (reply[4] << 16) | (reply[5] << 8) | reply[6]

def write_reg(bus, addr, reg, val):
    data = [(val >> 24) & 0xFF, (val >> 16) & 0xFF, (val >> 8) & 0xFF, val & 0xFF]
    req = bytearray([0x05, addr, reg | 0x80])
    req += bytearray(data)
    req.append(crc8(req))
    while bus.any():
        bus.read()
    bus.write(req)
    time.sleep_ms(20)
    bus.read()   # discard echo

# Register addresses
IFCNT      = 0x02
GCONF      = 0x00
CHOPCONF   = 0x6C
IHOLD_IRUN = 0x10

# ---------------------------------------------------------------------
# Buses and per-axis address map
# ---------------------------------------------------------------------
busABC = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))
busDEF = UART(1, baudrate=115200, tx=Pin(4), rx=Pin(5))

AXES = {
    "A": (busABC, 0), "B": (busABC, 1), "C": (busABC, 2),
    "D": (busDEF, 0), "E": (busDEF, 1), "F": (busDEF, 2),
}

# Config values (1/16 microstep -> 3200 steps/rev, zero hold, IRUN=10)
GCONF_VAL      = 0x000000C0   # pdn_disable + mstep_reg_select
CHOPCONF_VAL   = 0x54000053   # MRES=4 (1/16), intpol, TOFF=3, vsense=1
IHOLD_IRUN_VAL = 0x00020A00   # IHOLD=0, IRUN=10, IHOLDDELAY=2

def configure_driver(bus, addr):
    before = read_reg(bus, addr, IFCNT)
    if before is None:
        print("  addr %d: NO RESPONSE (check VM power/wiring)" % addr)
        return False
    write_reg(bus, addr, GCONF, GCONF_VAL)
    write_reg(bus, addr, CHOPCONF, CHOPCONF_VAL)
    write_reg(bus, addr, IHOLD_IRUN, IHOLD_IRUN_VAL)
    after = read_reg(bus, addr, IFCNT)
    ok = (after == ((before + 3) & 0xFF))
    print("  addr %d: IFCNT %d -> %d  %s"
          % (addr, before, after, "OK" if ok else "MISMATCH"))
    return ok

def configure_all():
    print("Configuring all six drivers:")
    all_ok = True
    for name, (bus, addr) in AXES.items():
        print("Axis %s" % name)
        if not configure_driver(bus, addr):
            all_ok = False
    return all_ok

# ---------------------------------------------------------------------
# Position tracking + coordinated motion
# ---------------------------------------------------------------------
MICROSTEPS_PER_REV = 3200          # 200 full * 16 microsteps
position = {name: 0 for name in STEP_DIR_PINS}

def move_all_to(targets, delay_us=250):
    """Coordinated (Bresenham) absolute move of multiple axes; they arrive together."""
    deltas = {}
    for name, target in targets.items():
        delta = target - position[name]
        dir_pins[name].value(1 if delta > 0 else 0)
        deltas[name] = abs(delta)
    time.sleep_us(2)   # DIR settle

    max_steps = max(deltas.values()) if deltas else 0
    if max_steps == 0:
        return

    error = {name: max_steps // 2 for name in deltas}
    for _ in range(max_steps):
        for name in deltas:
            error[name] -= deltas[name]
            if error[name] < 0:
                error[name] += max_steps
                step_pins[name].value(1)
        time.sleep_us(2)
        for name in deltas:
            step_pins[name].value(0)
        time.sleep_us(delay_us)

    for name, target in targets.items():
        position[name] = target

# ---------------------------------------------------------------------
# Home / neutral helpers (declare-current-position, no motion)
# ---------------------------------------------------------------------
NEUTRAL = 40000   # comfortable mid-travel, found empirically

def set_home_here(names=None):
    if names is None:
        names = list(STEP_DIR_PINS.keys())
    for name in names:
        position[name] = 0
    print("Home set (current position = 0) for: %s" % ", ".join(names))

def set_neutral_here(names=None):
    if names is None:
        names = list(STEP_DIR_PINS.keys())
    for name in names:
        position[name] = NEUTRAL
    print("Neutral set (current position = %d) for: %s" % (NEUTRAL, ", ".join(names)))

def goto_neutral():
    move_all_to({name: 40000 for name in STEP_DIR_PINS})
    print("Moved to neutral (40000).")

# ---------------------------------------------------------------------
# Pose control (Dropbear sign-matrix, mapped to A-F)
# ---------------------------------------------------------------------
DROPBEAR_MATRIX = {
    1: {"X": -1, "Y": +1, "Z": +1, "pitch": +1, "roll": +1},
    2: {"X": +1, "Y": -1, "Z": -1, "pitch": +1, "roll": +1},
    3: {"X": -1, "Y": -1, "Z": -1, "pitch": -1, "roll": +1},
    4: {"X": +1, "Y": +1, "Z": -1, "pitch": -1, "roll": -1},
    5: {"X": -1, "Y": +1, "Z": -1, "pitch": +1, "roll": -1},
    6: {"X": +1, "Y": -1, "Z": +1, "pitch": +1, "roll": -1},
}
AXIS_TO_MOTOR = {"A": 6, "B": 5, "C": 4, "D": 1, "E": 2, "F": 3}

POSE_SCALE  = 15000
HEIGHT_SCALE = 400

def move_head(angleX=0, angleY=0, angleZ=0, pitch=0, roll=0, height=0, delay_us=10):
    """Sign-matrix pose -> six leg targets around NEUTRAL."""
    targets = {}
    for axis, motor in AXIS_TO_MOTOR.items():
        m = DROPBEAR_MATRIX[motor]
        val = (m["X"] * angleX + m["Y"] * angleY + m["Z"] * angleZ
               + m["pitch"] * pitch + m["roll"] * roll) * POSE_SCALE
        val += height * HEIGHT_SCALE
        targets[axis] = int(NEUTRAL + val)
    move_all_to(targets, delay_us=delay_us)

# ---------------------------------------------------------------------
# Gaze tracking filter (dead zone + smoothing + clamp)
# NOTE: MAX_* at 1.0 allows full-unit pose. With POSE_SCALE=10000 a single
# axis swings +/-10000 from NEUTRAL, and stacked pitch+yaw can reach
# +/-20000. If a stacked command ever binds a leg, lower these.
# ---------------------------------------------------------------------
MAX_YAW   = 1.0
MAX_PITCH = 1.0
DEAD_ZONE = 0.1     # ignore gaze changes smaller than this (hysteresis)
SMOOTH    = 0.15    # 0..1; lower = lazier/smoother follow
MOVE_THRESHOLD = 0.01

class GazeTracker:
    def __init__(self):
        self.cur_yaw = 0.0
        self.cur_pitch = 0.0
        self.tgt_yaw = 0.0
        self.tgt_pitch = 0.0

    def update_target(self, gaze_x, gaze_y):
        if abs(gaze_x - self.tgt_yaw) > DEAD_ZONE:
            self.tgt_yaw = gaze_x
        if abs(gaze_y - self.tgt_pitch) > DEAD_ZONE:
            self.tgt_pitch = gaze_y
        self.tgt_yaw   = max(-MAX_YAW,   min(MAX_YAW,   self.tgt_yaw))
        self.tgt_pitch = max(-MAX_PITCH, min(MAX_PITCH, self.tgt_pitch))

    def step(self):
        new_yaw   = self.cur_yaw   + (self.tgt_yaw   - self.cur_yaw)   * SMOOTH
        new_pitch = self.cur_pitch + (self.tgt_pitch - self.cur_pitch) * SMOOTH
        if (abs(new_yaw - self.cur_yaw) > MOVE_THRESHOLD or
                abs(new_pitch - self.cur_pitch) > MOVE_THRESHOLD):
            self.cur_yaw = new_yaw
            self.cur_pitch = new_pitch
            move_head(angleX=-self.cur_yaw, pitch=-self.cur_pitch)

    def relax_to_neutral(self):
        """Ease target back to centered (used when tracking is off / gaze stale)."""
        self.tgt_yaw = 0.0
        self.tgt_pitch = 0.0

tracker = GazeTracker()

# ---------------------------------------------------------------------
# Physical inputs  (to GND, internal pull-ups; pressed/jumpered = 0)
# ---------------------------------------------------------------------
in_set_home     = Pin(8,  Pin.IN, Pin.PULL_UP)
in_set_neutral  = Pin(9,  Pin.IN, Pin.PULL_UP)
in_goto_neutral    = Pin(12, Pin.IN, Pin.PULL_UP)
in_track_jumper = Pin(13, Pin.IN, Pin.PULL_UP)

led = Pin("LED", Pin.OUT)   # heartbeat: blinks as gaze data is received

_btn_last = {"home": 1, "neutral": 1, "goto": 1}

def poll_buttons():
    """Momentary buttons fire once on press edge (1 -> 0)."""
    h = in_set_home.value()
    if h == 0 and _btn_last["home"] == 1:
        set_home_here()
    _btn_last["home"] = h

    n = in_set_neutral.value()
    if n == 0 and _btn_last["neutral"] == 1:
        set_neutral_here()
    _btn_last["neutral"] = n

    g = in_goto_neutral.value()
    if g == 0 and _btn_last["goto"] == 1:
        goto_neutral()
    _btn_last["goto"] = g

def tracking_enabled():
    """Jumper is a level: jumpered (0) = tracking ON."""
    return in_track_jumper.value() == 0

# ---------------------------------------------------------------------
# Serial input: read newest "yaw,pitch" line, drop stale ones
# ---------------------------------------------------------------------
_spoll = select.poll()
_spoll.register(sys.stdin, select.POLLIN)

def serial_latest():
    latest = None
    while _spoll.poll(0):
        line = sys.stdin.readline()
        if line:
            latest = line.strip()
    return latest

# ---------------------------------------------------------------------
# Startup
# ---------------------------------------------------------------------
print("Kerfur neck booting...")
configure_all()
print("Ready. Jumper GP13 to GND to enable tracking.")

# ---------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------
while True:
    poll_buttons()

    if tracking_enabled():
        line = serial_latest()
        if line:
            try:
                x_s, y_s = line.split(",")
                tracker.update_target(float(x_s), float(y_s))
                led.toggle()           # heartbeat on valid data
            except Exception:
                pass                   # malformed line -> skip (fail soft)
        tracker.step()
    else:
        serial_latest()               # drain serial so it doesn't pile up
        tracker.relax_to_neutral()    # ease target to center while idle
        tracker.step()                # keep easing toward neutral
        led.off()

    time.sleep_ms(10)