"""
simple_path_following.py

As simple as it gets:
  1. Drive forward at BASE_SPEED until GPS distance = user input length.
  2. Stop. Point turn right for TURN_DURATION_SEC seconds.
  3. Drive forward at BASE_SPEED for the same distance.
  4. Stop.

No PID. No encoders. No heading math. No cross-track correction.
One speed value sent identically to all four wheels at all times.
GPS distance is the only stop trigger.

Tuning — the only three numbers
--------------------------------
  BASE_SPEED        m/s for all four wheels. Start at 0.15.
  TURN_SPEED_MS     m/s for each side during point turn. Start at 0.10.
  TURN_DURATION_SEC Seconds for a 90° right turn. Calibrate with stopwatch.
"""

import math
import time
import serial


# ---------------------------------------------------------------------------
# Serial config
# ---------------------------------------------------------------------------
SERIAL_PORT = "COM15"
BAUD_RATE   = 9600


def open_serial():
    return serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)


# ---------------------------------------------------------------------------
# Tuning
# ---------------------------------------------------------------------------
BASE_SPEED        = 0.15   # m/s — all four wheels, straight segments
TURN_SPEED_MS     = 0.10   # m/s — each side during point turn
TURN_DURATION_SEC = 10.0   # seconds for 90° right turn (fixed per request)
STOP_MARGIN_M     = 0.30   # stop this many metres before GPS target


# ---------------------------------------------------------------------------
# Speed -> PWM
# ---------------------------------------------------------------------------
MAX_SPEED_MS = 1.5

def speed_to_pwm(speed_ms: float) -> int:
    return int(min(255, max(0.0, speed_ms) / MAX_SPEED_MS * 255.0))

PWM_BASE = speed_to_pwm(BASE_SPEED)
PWM_TURN = speed_to_pwm(TURN_SPEED_MS)


# ---------------------------------------------------------------------------
# Serial commands
# ---------------------------------------------------------------------------

def all_stop(ser):
    """Zero all four wheels and paint arm off."""
    ser.write(b"L1:0,L2:0,R1:0,R2:0,ARM:0\n")


def all_forward(ser, arm: bool = True):
    """All four wheels at BASE_SPEED."""
    arm_val = 1 if arm else 0
    cmd = f"L1:{PWM_BASE},L2:{PWM_BASE},R1:{PWM_BASE},R2:{PWM_BASE},ARM:{arm_val}\n"
    ser.write(cmd.encode())


def turn_right(ser):
    """Left side forward, right side reverse. Paint arm off."""
    cmd = f"L1:{PWM_TURN},L2:{PWM_TURN},R1:-{PWM_TURN},R2:-{PWM_TURN},ARM:0\n"
    ser.write(cmd.encode())


# ---------------------------------------------------------------------------
# Geometry
# ---------------------------------------------------------------------------

def dist_m(a: tuple, b: tuple) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)


# ---------------------------------------------------------------------------
# States
# ---------------------------------------------------------------------------
STATE_SEG1   = "SEG1_FORWARD"
STATE_TURN   = "TURNING"
STATE_SEG2   = "SEG2_FORWARD"
STATE_DONE   = "DONE"


# ---------------------------------------------------------------------------
# SimplePathFollower
# ---------------------------------------------------------------------------

class SimplePathFollower:
    """
    Dead-simple two-segment runner.

    follower = SimplePathFollower(length_m)
    follower.set_start(gnss.get_position())

    while not follower.is_done():
        result = follower.step(gnss.get_position())
        # use result["state"] and result["command"] to drive motors

    After the turn completes, call:
        follower.set_seg2_start(gnss.get_position())
    before continuing the loop.
    """

    def __init__(self, length_m: float):
        self.length_m  = length_m
        self._state    = STATE_SEG1
        self._start    = None
        self._turn_start_time = None

        print(f"\n[Simple] Length     : {length_m:.2f} m each segment")
        print(f"[Simple] Base PWM   : {PWM_BASE}  ({BASE_SPEED} m/s)")
        print(f"[Simple] Turn PWM   : {PWM_TURN}  ({TURN_SPEED_MS} m/s)")
        print(f"[Simple] Turn time  : {TURN_DURATION_SEC} s")
        print(f"[Simple] Stop margin: {STOP_MARGIN_M} m")

    def set_start(self, pos: tuple):
        """Set GPS start for the current segment."""
        self._start = pos
        print(f"[Simple] Segment start: E={pos[0]:.3f}  N={pos[1]:.3f}")

    def is_done(self) -> bool:
        return self._state == STATE_DONE

    def current_state(self) -> str:
        return self._state

    def step(self, current_pos: tuple) -> dict:
        """
        One control cycle. Returns dict with:
            state       : current state string
            command     : "forward" | "turn" | "stop"
            dist        : GPS distance from segment start (m)
            remaining   : metres left in segment (m)
            turn_elapsed: seconds into turn (0 if not turning)
            turn_pct    : 0–100 during turn, else 0
        """
        if self._state == STATE_DONE:
            return self._make(current_pos, "stop", 0, 0, 0, 0)

        # ---- Straight segments ----------------------------------------
        if self._state in (STATE_SEG1, STATE_SEG2):
            d         = dist_m(self._start, current_pos)
            remaining = self.length_m - d

            if d >= (self.length_m - STOP_MARGIN_M):
                print(f"\n[Simple] {self._state} complete — "
                      f"{d:.2f} m travelled.")

                if self._state == STATE_SEG1:
                    self._state           = STATE_TURN
                    self._turn_start_time = time.time()
                    print(f"[Simple] Starting {TURN_DURATION_SEC}s right turn ...")
                else:
                    self._state = STATE_DONE
                    print("[Simple] DONE.")

                return self._make(current_pos, "stop", d, remaining, 0, 0)

            return self._make(current_pos, "forward", d, remaining, 0, 0)

        # ---- Turn --------------------------------------------------------
        if self._state == STATE_TURN:
            elapsed  = time.time() - self._turn_start_time
            pct      = min(100.0, elapsed / TURN_DURATION_SEC * 100.0)

            if elapsed >= TURN_DURATION_SEC:
                print("[Simple] Turn complete.")
                self._state = STATE_SEG2
                return self._make(current_pos, "stop", 0, 0, elapsed, 100.0)

            return self._make(current_pos, "turn", 0, 0, elapsed, pct)

        return self._make(current_pos, "stop", 0, 0, 0, 0)

    def _make(self, pos, command, dist, remaining,
              turn_elapsed, turn_pct) -> dict:
        return {
            "state"        : self._state,
            "command"      : command,
            "dist"         : round(dist, 3),
            "remaining"    : round(remaining, 3),
            "easting"      : round(pos[0], 3),
            "northing"     : round(pos[1], 3),
            "turn_elapsed" : round(turn_elapsed, 2),
            "turn_pct"     : round(turn_pct, 1),
        }
