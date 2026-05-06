# main.py — Raspberry Pi Pico 2 W (NOT MAIN CODE FOR JETSON, SEE README.md)
# ─────────────────────────────────────────────────────────────────────────────
# Sensor fusion output node:
#   • Reads ultrasonic distance and streams it to the Jetson over USB serial
#   • Receives CMD messages from the Jetson and drives the PWM buzzer
#
# Serial protocol (newline-terminated, 115200 baud):
#   Pico  → Jetson : "US,<float_inches>\n"      e.g.  "US,7.23\n"
#   Jetson → Pico  : "CMD,<type>,<param>\n"
#       CMD,NONE,0          — silence
#       CMD,PARK,<0-100>    — parking proximity (100 = closest)
#       CMD,COLLISION,<1|2> — collision (1=slow, 2=stop)
#       CMD,LANE,<1|2>      — lane departure (1=left, 2=right)
#
# Buzzer tones:
#   PARK        — 2000 Hz, beep interval scales with severity
#   COLLISION,1 — 3000 Hz, 120ms on / 180ms off
#   COLLISION,2 — 3800 Hz, 220ms on / 80ms off (urgent)
#   LANE        — 1200 Hz, triple-pulse pattern then pause
# ─────────────────────────────────────────────────────────────────────────────

import sys
import time
import uselect
from machine import Pin, PWM

# Pin assignments
TRIG_PIN   = 3
ECHO_PIN   = 2
BUZZER_PIN = 15

TRIG   = Pin(TRIG_PIN, Pin.OUT)
ECHO   = Pin(ECHO_PIN, Pin.IN)
buzzer = PWM(Pin(BUZZER_PIN))
buzzer.duty_u16(0)

# Ultrasonic constants
SPEED_INCHES_PER_US = 0.0135
US_SAMPLE_COUNT     = 5
US_SAMPLE_DELAY_MS  = 5
US_REPORT_EVERY_MS  = 100   # stream readings at ~10 Hz

# Buzzer pattern constants
LANE_ON_MS    = 80
LANE_OFF_MS   = 80
LANE_PAUSE_MS = 600

# Serial input buffer
_serial_buf = ""

# Alert state
_cmd_type  = "NONE"
_cmd_param = 0

# Buzzer state machine
_bsm_state      = "OFF"
_bsm_deadline   = 0
_lane_pulse_idx = 0

# Timing
_last_us_report = time.ticks_ms()


# Ultrasonic
def _single_us_reading():
    from machine import time_pulse_us
    TRIG.low()
    time.sleep_us(2)
    TRIG.high()
    time.sleep_us(10)
    TRIG.low()
    try:
        duration = time_pulse_us(ECHO, 1, 30000)
        if duration <= 0:
            return None
        return (duration * SPEED_INCHES_PER_US) / 2.0
    except OSError:
        return None


def get_distance_inches():
    readings = []
    for _ in range(US_SAMPLE_COUNT):
        d = _single_us_reading()
        if d is not None:
            readings.append(d)
        time.sleep_ms(US_SAMPLE_DELAY_MS)
    if not readings:
        return None
    readings.sort()
    return readings[len(readings) // 2]


# Serial helpers — using sys.stdin.buffer for reliable operation outside REPL
_poll = uselect.poll()
_poll.register(sys.stdin, uselect.POLLIN)

def serial_readline_nonblocking():
    global _serial_buf
    try:
        while _poll.poll(0):
            char = sys.stdin.read(1)
            if char:
                _serial_buf += char
                if "\n" in _serial_buf:
                    line, _serial_buf = _serial_buf.split("\n", 1)
                    return line.strip()
    except Exception as e:
        sys.stdout.write("ERR:{}\n".format(e))
    return None


def parse_command(line):
    parts = line.split(",")
    if len(parts) == 3 and parts[0] == "CMD":
        try:
            return parts[1], int(parts[2])
        except ValueError:
            pass
    return None, None


def send_us(dist_inches):
    msg = "US,{:.2f}\n".format(dist_inches)
    sys.stdout.buffer.write(msg.encode("utf-8"))


# Buzzer helpers
def _buzzer_on(freq, duty=30000):
    buzzer.freq(freq)
    buzzer.duty_u16(duty)


def _buzzer_off():
    buzzer.duty_u16(0)


def _park_timing_ms(severity):
    ratio  = max(0, min(100, severity)) / 100.0
    off_ms = int(550 - ratio * 500)    # 550ms gap → 50ms gap as severity rises
    return 50, off_ms


# Buzzer state machine — non-blocking, called every main loop iteration
def update_buzzer_sm():
    global _bsm_state, _bsm_deadline, _lane_pulse_idx

    now = time.ticks_ms()
    if time.ticks_diff(_bsm_deadline, now) > 0:
        return  # still in current phase

    # NONE — silence
    if _cmd_type == "NONE":
        _buzzer_off()
        _bsm_state      = "OFF"
        _bsm_deadline   = now + 50
        _lane_pulse_idx = 0
        return

    # PARK — 2000 Hz, interval scales with severity
    if _cmd_type == "PARK":
        on_ms, off_ms = _park_timing_ms(_cmd_param)
        if _cmd_param >= 100:
            _buzzer_on(2000)
            _bsm_deadline = now + 50
        elif _bsm_state == "OFF":
            _buzzer_on(2000)
            _bsm_state    = "ON"
            _bsm_deadline = now + on_ms
        else:
            _buzzer_off()
            _bsm_state    = "OFF"
            _bsm_deadline = now + off_ms
        return

    # COLLISION,1 — 3000 Hz, 120ms on / 180ms off
    if _cmd_type == "COLLISION" and _cmd_param == 1:
        if _bsm_state == "OFF":
            _buzzer_on(3000)
            _bsm_state    = "ON"
            _bsm_deadline = now + 120
        else:
            _buzzer_off()
            _bsm_state    = "OFF"
            _bsm_deadline = now + 180
        return

    # COLLISION,2 — 3800 Hz, 220ms on / 80ms off (urgent)
    if _cmd_type == "COLLISION" and _cmd_param == 2:
        if _bsm_state == "OFF":
            _buzzer_on(3800)
            _bsm_state    = "ON"
            _bsm_deadline = now + 220
        else:
            _buzzer_off()
            _bsm_state    = "OFF"
            _bsm_deadline = now + 80
        return

    # LANE — 1200 Hz, triple-pulse then pause
    if _cmd_type == "LANE":
        if _lane_pulse_idx in (0, 2, 4):
            _buzzer_on(1200)
            _bsm_state      = "ON"
            _lane_pulse_idx += 1
            _bsm_deadline   = now + LANE_ON_MS
        elif _lane_pulse_idx in (1, 3):
            _buzzer_off()
            _bsm_state      = "OFF"
            _lane_pulse_idx += 1
            _bsm_deadline   = now + LANE_OFF_MS
        else:
            _buzzer_off()
            _bsm_state      = "OFF"
            _lane_pulse_idx = 0
            _bsm_deadline   = now + LANE_PAUSE_MS
        return

    # Fallback — unknown command, silence
    _buzzer_off()
    _bsm_state    = "OFF"
    _bsm_deadline = now + 50


# Main loop
def main():
    global _cmd_type, _cmd_param, _last_us_report

    sys.stdout.buffer.write(b"Pico ADAS firmware ready.\n")

    # Initialize deadline to now so state machine starts immediately
    global _bsm_deadline
    _bsm_deadline = time.ticks_ms()

    while True:
        now = time.ticks_ms()

        line = serial_readline_nonblocking()
        if line:
            t, p = parse_command(line)
            if t is not None:
                _cmd_type  = t
                _cmd_param = p

        update_buzzer_sm()

        if time.ticks_diff(now, _last_us_report) >= US_REPORT_EVERY_MS:
            _last_us_report = now
            dist = get_distance_inches()
            if dist is not None:
                send_us(dist)

        time.sleep_ms(2)


main()