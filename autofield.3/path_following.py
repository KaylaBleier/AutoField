"""
path_following.py  (v4 — encoder/time toggle)

Top-level switches
------------------
  USE_ENCODERS   False → no encoder data used anywhere. Both wheels get
                         BASE_SPEED during straights (no PID drift correction).
                         Turn runs for TURN_DURATION_SEC seconds then stops.
                 True  → encoder ticks used for straight PID correction AND
                         turn completion (counts to TICKS_PER_90).

  These are the only two lines you need to change to switch modes:

      USE_ENCODERS   = False   # ← toggle here
      TURN_MODE      = "time"  # ← "time" or "ticks"

  TURN_MODE is ignored when USE_ENCODERS = False (always uses time).
  TURN_MODE = "ticks" requires USE_ENCODERS = True.

State machine
-------------
  STRAIGHT_1  →  TURNING  →  STRAIGHT_2  →  DONE

  STRAIGHT : both wheels at BASE_SPEED. PID correction only active when
             USE_ENCODERS = True and drift exceeds DRIFT_DEADBAND.
  TURNING  : left forward, right reverse at TURN_SPEED_MS.
             Ends after TURN_DURATION_SEC (time mode) or TICKS_PER_90
             left-wheel ticks (tick mode).
  DONE     : motors stop, paint arm off.

Encoder serial protocol  (Arduino → Pi)
----------------------------------------
  "ENC:<left_cumulative>,<right_cumulative>\\n"
  Ignored entirely when USE_ENCODERS = False.

Motor command format  (Pi → Arduino)
--------------------------------------
  "L1:<v>,L2:<v>,R1:<v>,R2:<v>,ARM:<v>\\n"
  Values 0–255 are speed setpoints. Arduino closes velocity loop per wheel.
  Negative v_R_ms during turn signals Arduino to reverse the right side.
"""

import math
import time
import serial


# ===========================================================================
# TOP-LEVEL SWITCHES  ← change these two lines to switch modes
# ===========================================================================
USE_ENCODERS = False   # True  = use encoder ticks for PID + turn counting
                       # False = no encoders, time-based turn, no PID

TURN_MODE    = "time"  # "time"  = turn for TURN_DURATION_SEC seconds
                       # "ticks" = turn until TICKS_PER_90 ticks (requires
                       #           USE_ENCODERS = True)
# ===========================================================================


# ---------------------------------------------------------------------------
# Serial config
# ---------------------------------------------------------------------------
SERIAL_PORT = "COM15"
BAUD_RATE   = 9600


def open_serial():
    return serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)


def send_motor_commands(ser, v_L_ms: float, v_R_ms: float, arm: bool):
    """
    Send speed setpoints to Arduino.
    Positive values → forward. Negative v_R_ms → right side reverses.
    Format: "L1:<v>,L2:<v>,R1:<v>,R2:<v>,ARM:<v>\\n"  (values 0–255)
    """
    pwm_L   = speed_to_pwm(v_L_ms)
    pwm_R   = speed_to_pwm(v_R_ms)
    arm_val = 1 if arm else 0
    cmd = f"L1:{pwm_L},L2:{pwm_L},R1:{pwm_R},R2:{pwm_R},ARM:{arm_val}\n"
    ser.write(cmd.encode())


# ---------------------------------------------------------------------------
# Speed <-> PWM
# ---------------------------------------------------------------------------
MAX_SPEED_MS = 1.5


def speed_to_pwm(speed_ms: float) -> int:
    speed_ms = max(0.0, speed_ms)
    return int(min(255, (speed_ms / MAX_SPEED_MS) * 255.0))


# ---------------------------------------------------------------------------
# Tuning constants
# ---------------------------------------------------------------------------

# --- Straight driving ---
BASE_SPEED        = 0.20   # m/s, both wheels during straight

# PID — only active when USE_ENCODERS = True
# Set any gain to 0.0 to disable that term individually
KP_ENC            = 0.003
KI_ENC            = 0.0    # disabled — set > 0 to enable
KD_ENC            = 0.0    # disabled — set > 0 to enable
MAX_CORRECTION_MS = 0.10   # clamp on total PID correction (m/s)
DRIFT_DEADBAND    = 2      # ticks — ignore drift smaller than this

STOP_MARGIN_M     = 0.30   # halt this many metres before GPS target

# --- Point turn ---
TURN_SPEED_MS     = 0.10   # m/s magnitude for each side during point turn

# Time-based turn (used when TURN_MODE = "time" or USE_ENCODERS = False)
TURN_DURATION_SEC = 3.0    # *** CALIBRATE *** seconds for a 90° right turn
                           # Method: run a turn with stopwatch, adjust until
                           # visually 90°. Repeat 3x and average.

# Tick-based turn (used when TURN_MODE = "ticks" AND USE_ENCODERS = True)
TICKS_PER_90      = 2000   # *** CALIBRATE *** left-wheel ticks for 90° right
                           # Method: run a turn, read left tick count at 90°.
                           # Repeat 3x and average.

# --- GPS heading blend ---
GPS_HEADING_BLEND = 0.0    # 0 = no GPS heading blend (safest). 0.1–0.2 = light blend.
MIN_GPS_MOVE_M    = 0.25   # min GPS displacement (m) before updating heading


# ---------------------------------------------------------------------------
# States
# ---------------------------------------------------------------------------
STATE_STRAIGHT_1 = "STRAIGHT_1"
STATE_TURNING    = "TURNING"
STATE_STRAIGHT_2 = "STRAIGHT_2"
STATE_DONE       = "DONE"


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def dist_m(a: tuple, b: tuple) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)


def heading_from_points(p1: tuple, p2: tuple) -> float:
    dx, dy = p2[0] - p1[0], p2[1] - p1[1]
    if abs(dx) < 1e-9 and abs(dy) < 1e-9:
        return 0.0
    return math.atan2(dy, dx)


def normalize_angle(a: float) -> float:
    while a >  math.pi: a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a


def heading_to_compass(rad: float) -> float:
    """Radians CCW-from-East → compass degrees CW-from-North."""
    return (90.0 - math.degrees(rad)) % 360.0


# ---------------------------------------------------------------------------
# Encoder helpers
# ---------------------------------------------------------------------------

def parse_encoder_line(line: str):
    line = line.strip()
    if not line.startswith("ENC:"):
        return None
    try:
        parts = line[4:].split(",")
        return int(parts[0]), int(parts[1])
    except (IndexError, ValueError):
        return None


def read_encoders(ser) -> tuple | None:
    """
    Drain Arduino serial buffer, return last valid ENC:<L>,<R> as (L, R).
    Returns None if no encoder line was waiting.
    Always returns None immediately when USE_ENCODERS = False.
    """
    if not USE_ENCODERS:
        # Still drain the buffer so it doesn't back up, just discard
        while ser.in_waiting:
            try:
                ser.readline()
            except serial.SerialException:
                break
        return None

    result = None
    while ser.in_waiting:
        try:
            raw = ser.readline().decode("ascii", errors="replace")
        except serial.SerialException:
            break
        parsed = parse_encoder_line(raw)
        if parsed is not None:
            result = parsed
    return result


# ---------------------------------------------------------------------------
# PathFollower  (v4)
# ---------------------------------------------------------------------------

class PathFollower:
    """
    Two-segment lawnmower:  STRAIGHT_1 → TURNING (90° right) → STRAIGHT_2 → DONE

    Quick-start
    -----------
    follower = PathFollower(seg1_length_m, seg2_length_m)
    follower.set_segment_start(fix1)
    follower.set_target_heading(fix1, fix2)

    while not follower.is_finished():
        enc = read_encoders(ser)           # None when USE_ENCODERS = False
        ticks_L, ticks_R = enc if enc else (None, None)
        result = follower.step(gnss.get_position(), ticks_L, ticks_R)
        send_motor_commands(ser, result["v_L_ms"], result["v_R_ms"],
                            result["paint_arm"])
    """

    def __init__(self, seg1_length_m: float, seg2_length_m: float):
        self.seg1_length = seg1_length_m
        self.seg2_length = seg2_length_m

        self._state = STATE_STRAIGHT_1

        # Heading
        self._ref_heading  = 0.0
        self._gps_heading  = 0.0
        self._prev_gps_pos = None

        # Segment start
        self._seg_start = None

        # Encoder state
        self._prev_L  = None
        self._prev_R  = None
        self._delta_L = 0
        self._delta_R = 0

        # PID state
        self._integral   = 0.0
        self._prev_drift = 0.0
        self._integral_clamp = 5.0

        # Turn state
        self._turn_start_time = None   # for time-based turn
        self._turn_start_L    = None   # for tick-based turn

        # Resolve effective turn mode at init so it's clear in logs
        if not USE_ENCODERS and TURN_MODE == "ticks":
            print("[PathFollower] WARNING: TURN_MODE='ticks' requires "
                  "USE_ENCODERS=True. Falling back to time-based turn.")
        self._effective_turn_mode = (
            "ticks" if (USE_ENCODERS and TURN_MODE == "ticks") else "time"
        )

        print(f"[PathFollower] USE_ENCODERS = {USE_ENCODERS}")
        print(f"[PathFollower] Turn mode    = {self._effective_turn_mode}"
              + (f"  ({TURN_DURATION_SEC}s)"
                 if self._effective_turn_mode == "time"
                 else f"  ({TICKS_PER_90} ticks)"))
        print(f"[PathFollower] Segment 1    = {seg1_length_m:.2f} m")
        print(f"[PathFollower] Segment 2    = {seg2_length_m:.2f} m")

    # ------------------------------------------------------------------
    # Public setup
    # ------------------------------------------------------------------

    def set_segment_start(self, pos: tuple):
        """Record GPS start point for the current segment."""
        self._seg_start    = pos
        self._prev_gps_pos = pos
        print(f"[PathFollower] Segment start: E={pos[0]:.3f}  N={pos[1]:.3f}")

    def set_target_heading(self, fix1: tuple, fix2: tuple):
        """Lock reference heading from two nudge fixes."""
        d = dist_m(fix1, fix2)
        if d < 0.10:
            print(f"[PathFollower] WARNING: nudge too close ({d:.3f} m) — "
                  "keeping previous heading.")
            return
        self._ref_heading = heading_from_points(fix1, fix2)
        self._gps_heading = self._ref_heading
        self._integral    = 0.0
        self._prev_drift  = 0.0
        print(f"[PathFollower] Heading locked: "
              f"{math.degrees(self._ref_heading):.1f}° from East  "
              f"({heading_to_compass(self._ref_heading):.1f}° compass)")

    def is_finished(self) -> bool:
        return self._state == STATE_DONE

    def current_state(self) -> str:
        return self._state

    # ------------------------------------------------------------------
    # Internal: encoders
    # ------------------------------------------------------------------

    def _update_encoders(self, ticks_L: int, ticks_R: int):
        if self._prev_L is None:
            self._prev_L  = ticks_L
            self._prev_R  = ticks_R
            self._delta_L = 0
            self._delta_R = 0
            return
        self._delta_L = ticks_L - self._prev_L
        self._delta_R = ticks_R - self._prev_R
        self._prev_L  = ticks_L
        self._prev_R  = ticks_R

    # ------------------------------------------------------------------
    # Internal: GPS heading blend
    # ------------------------------------------------------------------

    def _update_gps_heading(self, pos: tuple):
        if GPS_HEADING_BLEND == 0.0 or self._prev_gps_pos is None:
            return
        d = dist_m(self._prev_gps_pos, pos)
        if d < MIN_GPS_MOVE_M:
            return
        new_hdg = heading_from_points(self._prev_gps_pos, pos)
        self._gps_heading = normalize_angle(
            (1.0 - GPS_HEADING_BLEND) * self._gps_heading
            + GPS_HEADING_BLEND * new_hdg
        )
        self._prev_gps_pos = pos

    # ------------------------------------------------------------------
    # Internal: PID correction (straight)
    # ------------------------------------------------------------------

    def _pid_correction(self) -> float:
        """
        Returns m/s correction from encoder drift.
        Always returns 0.0 when USE_ENCODERS = False.
        Positive = left ran more = slow left / speed right.
        """
        if not USE_ENCODERS:
            return 0.0

        drift = float(self._delta_L - self._delta_R)

        if abs(drift) <= DRIFT_DEADBAND:
            self._integral   *= 0.95   # decay integrator in dead-band
            self._prev_drift  = drift
            return 0.0

        p_term = KP_ENC * drift

        self._integral += drift
        self._integral  = max(-self._integral_clamp,
                               min(self._integral_clamp, self._integral))
        i_term = KI_ENC * self._integral

        d_term = KD_ENC * (drift - self._prev_drift)
        self._prev_drift = drift

        correction = p_term + i_term + d_term
        return max(-MAX_CORRECTION_MS, min(MAX_CORRECTION_MS, correction))

    # ------------------------------------------------------------------
    # Internal: straight step (shared by seg 1 and seg 2)
    # ------------------------------------------------------------------

    def _step_straight(self, current_pos: tuple,
                       seg_length: float,
                       next_state: str) -> dict:

        self._update_gps_heading(current_pos)

        gps_dist  = dist_m(self._seg_start, current_pos)
        remaining = seg_length - gps_dist

        # Transition check
        if gps_dist >= (seg_length - STOP_MARGIN_M):
            print(f"\n[PathFollower] {self._state} complete — "
                  f"GPS dist={gps_dist:.2f} m  target={seg_length:.2f} m")
            self._state = next_state

            if next_state == STATE_TURNING:
                self._turn_start_time = time.time()
                self._turn_start_L    = self._prev_L
                print(f"[PathFollower] Starting 90° right point turn "
                      f"({self._effective_turn_mode} mode) ...")

            elif next_state == STATE_DONE:
                print("[PathFollower] All segments complete. DONE.")

            return self._stopped_dict(current_pos)

        # PID correction (0.0 when USE_ENCODERS = False)
        correction = self._pid_correction()
        v_L_ms = max(0.0, BASE_SPEED - correction)
        v_R_ms = max(0.0, BASE_SPEED + correction)

        heading_err = normalize_angle(self._gps_heading - self._ref_heading)

        return {
            "v_L_ms"          : round(v_L_ms, 4),
            "v_R_ms"          : round(v_R_ms, 4),
            "paint_arm"       : True,
            "status"          : "running",
            "state"           : self._state,
            "easting"         : round(current_pos[0], 3),
            "northing"        : round(current_pos[1], 3),
            "dist_from_start" : round(gps_dist, 3),
            "dist_remaining"  : round(remaining, 3),
            "heading_deg"     : round(heading_to_compass(self._gps_heading), 1),
            "heading_ref_deg" : round(heading_to_compass(self._ref_heading), 1),
            "heading_err_deg" : round(math.degrees(heading_err), 2),
            "enc_drift"       : float(self._delta_L - self._delta_R)
                                if USE_ENCODERS else 0.0,
            "correction_ms"   : round(correction, 4),
            "turn_progress_pct": 0.0,
        }

    # ------------------------------------------------------------------
    # Internal: turn step
    # ------------------------------------------------------------------

    def _step_turning(self) -> dict:
        """
        Right point turn: left wheel forward, right wheel reverse.
        Completion determined by time or ticks depending on _effective_turn_mode.
        """
        done     = False
        progress = 0.0

        if self._effective_turn_mode == "time":
            elapsed  = time.time() - self._turn_start_time
            progress = min(1.0, elapsed / TURN_DURATION_SEC)
            done     = elapsed >= TURN_DURATION_SEC

        else:  # ticks
            if self._prev_L is None or self._turn_start_L is None:
                print("[PathFollower] WARNING: no encoder data for tick turn.")
                return self._turn_status_dict(0.0, 0.0, progress=0.0)
            ticks_so_far = abs(self._prev_L - self._turn_start_L)
            progress     = min(1.0, ticks_so_far / TICKS_PER_90)
            done         = ticks_so_far >= TICKS_PER_90

        if done:
            print(f"[PathFollower] Turn complete.")
            self._state      = STATE_STRAIGHT_2
            self._integral   = 0.0
            self._prev_drift = 0.0
            self._delta_L    = 0
            self._delta_R    = 0
            return self._turn_status_dict(0.0, 0.0, progress=1.0)

        # Still turning: left forward, right reverse
        return self._turn_status_dict(TURN_SPEED_MS, TURN_SPEED_MS,
                                      progress=progress)

    def _turn_status_dict(self, v_left: float, v_right: float,
                          progress: float) -> dict:
        return {
            "v_L_ms"           : v_left,
            "v_R_ms"           : -v_right,   # negative = Arduino reverses right
            "paint_arm"        : False,
            "status"           : "running",
            "state"            : self._state,
            "easting"          : 0.0,
            "northing"         : 0.0,
            "dist_from_start"  : 0.0,
            "dist_remaining"   : 0.0,
            "heading_deg"      : 0.0,
            "heading_ref_deg"  : 0.0,
            "heading_err_deg"  : 0.0,
            "enc_drift"        : 0.0,
            "correction_ms"    : 0.0,
            "turn_progress_pct": round(progress * 100, 1),
        }

    # ------------------------------------------------------------------
    # Main step
    # ------------------------------------------------------------------

    def step(self, current_pos: tuple,
             ticks_L=None,
             ticks_R=None) -> dict:
        """
        Run one control cycle.

        Parameters
        ----------
        current_pos : (easting, northing)
        ticks_L     : cumulative left  encoder ticks, or None
        ticks_R     : cumulative right encoder ticks, or None
                      Both ignored when USE_ENCODERS = False.
        """
        if self._state == STATE_DONE:
            return self._stopped_dict(current_pos)

        if USE_ENCODERS and ticks_L is not None and ticks_R is not None:
            self._update_encoders(ticks_L, ticks_R)

        if self._state == STATE_STRAIGHT_1:
            return self._step_straight(current_pos,
                                       self.seg1_length,
                                       next_state=STATE_TURNING)
        if self._state == STATE_TURNING:
            return self._step_turning()

        if self._state == STATE_STRAIGHT_2:
            return self._step_straight(current_pos,
                                       self.seg2_length,
                                       next_state=STATE_DONE)

        return self._stopped_dict(current_pos)

    # ------------------------------------------------------------------

    def _stopped_dict(self, pos: tuple) -> dict:
        status = "finished" if self._state == STATE_DONE else "transitioning"
        return {
            "v_L_ms"           : 0.0,
            "v_R_ms"           : 0.0,
            "paint_arm"        : False,
            "status"           : status,
            "state"            : self._state,
            "easting"          : round(pos[0], 3),
            "northing"         : round(pos[1], 3),
            "dist_from_start"  : 0.0,
            "dist_remaining"   : 0.0,
            "heading_deg"      : round(heading_to_compass(self._gps_heading), 1),
            "heading_ref_deg"  : round(heading_to_compass(self._ref_heading), 1),
            "heading_err_deg"  : 0.0,
            "enc_drift"        : 0.0,
            "correction_ms"    : 0.0,
            "turn_progress_pct": 0.0,
        }


# ---------------------------------------------------------------------------
# Smoke test
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import random
    random.seed(42)

    print("PathFollower v4 smoke test")
    print(f"USE_ENCODERS={USE_ENCODERS}  TURN_MODE={TURN_MODE}\n")

    follower = PathFollower(seg1_length_m=10.0, seg2_length_m=8.0)
    follower.set_segment_start((500.0, 1000.0))
    follower.set_target_heading((500.0, 1000.0), (500.8, 1000.01))

    e, n    = 500.0, 1000.0
    tL = tR = 0
    step_n  = 0
    seg2_ready = False

    print(f"{'#':>4}  {'State':<12}  {'E':>8}  {'N':>8}  "
          f"{'Dist':>5}  {'Rem':>5}  {'vL':>6}  {'vR':>6}  ARM  Extra")
    print("-" * 80)

    while not follower.is_finished():
        step_n += 1
        state = follower.current_state()

        if state == STATE_STRAIGHT_1:
            e  += 0.4 + random.uniform(-0.01, 0.01)
            n  += random.uniform(-0.02, 0.02)
            tL += 40 + random.randint(0, 2)
            tR += 38 + random.randint(0, 2)

        elif state == STATE_TURNING:
            tL += 45
            tR += 45

        elif state == STATE_STRAIGHT_2:
            if not seg2_ready:
                follower.set_segment_start((e, n))
                follower.set_target_heading((e, n), (e + 0.02, n + 0.8))
                seg2_ready = True
            n  += 0.4 + random.uniform(-0.01, 0.01)
            e  += random.uniform(-0.02, 0.02)
            tL += 40 + random.randint(0, 2)
            tR += 38 + random.randint(0, 2)

        r = follower.step((e, n), tL, tR)

        extra = (f"turn {r['turn_progress_pct']:.0f}%"
                 if state == STATE_TURNING else "")
        arm   = "ON " if r["paint_arm"] else "OFF"

        print(f"{step_n:>4}  {r['state']:<12}  "
              f"{r['easting']:>8.3f}  {r['northing']:>8.3f}  "
              f"{r['dist_from_start']:>5.2f}  {r['dist_remaining']:>5.2f}  "
              f"{r['v_L_ms']:>6.3f}  {r['v_R_ms']:>6.3f}  {arm}  {extra}")

        if step_n > 300:
            print("(step limit)")
            break
        time.sleep(0.01)

    print("\n[Smoke test] complete.")
