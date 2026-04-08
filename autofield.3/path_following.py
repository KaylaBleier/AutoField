"""
path_following.py  (v3 — state machine: STRAIGHT → TURNING → STRAIGHT → DONE)

States
------
STRAIGHT  : Both wheels at BASE_SPEED. Encoder drift P/I/D correction applied
            outside a dead-band. GPS distance triggers transition to TURNING.
TURNING   : Point turn in place — right wheel reverse, left wheel forward.
            Encoder tick count measures rotation. Stops when TICKS_PER_90
            ticks accumulated. Paint arm OFF during turn.
STRAIGHT  : Same as first straight, new heading locked from post-turn nudge.
            GPS distance triggers transition to DONE.
DONE      : Motors stop, paint arm off.

Calibration required before test day
-------------------------------------
  TICKS_PER_90   Drive a point turn, count ticks when visually at 90°.
                 Repeat 3x and average. This is the most important number.
  TURN_SPEED_MS  Start at 0.10 m/s. Raise if turn is too slow/inconsistent.
  BASE_SPEED     Start at 0.20 m/s. Raise once straight driving works.
  KP_ENC         Start at 0.003. Raise if rover wanders, lower if it wiggles.
  DRIFT_DEADBAND Ticks of L/R difference to ignore — filters noise.
                 Start at 2, raise if motors hunt constantly.

Encoder serial protocol  (Arduino → Pi)
----------------------------------------
  "ENC:<left_cumulative>,<right_cumulative>\\n"
  Ticks are cumulative signed integers (positive = forward).
  Pass ticks_L=None, ticks_R=None to step() if not yet implemented —
  straight driving falls back to GPS-only, turn will not work without encoders.

Motor command format  (Pi → Arduino)
--------------------------------------
  "L1:<v>,L2:<v>,R1:<v>,R2:<v>,ARM:<v>\\n"
  Values 0–255 are speed setpoints. Arduino closes velocity loop per wheel.
"""

import math
import serial


# ---------------------------------------------------------------------------
# Serial config
# ---------------------------------------------------------------------------
SERIAL_PORT = "COM15"
BAUD_RATE   = 9600


def open_serial():
    return serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)


def send_motor_commands(ser, v_L_ms: float, v_R_ms: float, arm: bool):
    """
    Convert m/s wheel speed setpoints to 0–255 and send to Arduino.
    Negative speeds map to 0 — direction is controlled by Arduino pins.

    Format: "L1:<v>,L2:<v>,R1:<v>,R2:<v>,ARM:<v>\\n"
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
KP_ENC            = 0.0  # P gain on encoder drift
KI_ENC            = 0.0    # I gain — set 0.0 to disable
KD_ENC            = 0.0    # D gain — set 0.0 to disable
MAX_CORRECTION_MS = 0.10   # clamp on total PID correction (m/s)
DRIFT_DEADBAND    = 2      # ticks — ignore drift smaller than this
STOP_MARGIN_M     = 0.30   # halt this many metres before GPS target

# --- Point turn ---
TICKS_PER_90      = 2000   # *** CALIBRATE THIS *** ticks for a 90° right turn
                           # Method: command a turn, count ticks at 90°, repeat 3x
TURN_SPEED_MS     = 0.10   # m/s magnitude for each side during point turn
                           # left wheel forward, right wheel reverse

# --- GPS heading blend (optional, 0 = encoder-only) ---
GPS_HEADING_BLEND = 0.0
MIN_GPS_MOVE_M    = 0.25


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
    Call once per cycle BEFORE sending motor commands.
    """
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
# PathFollower  (v3)
# ---------------------------------------------------------------------------

class PathFollower:
    """
    Two-segment lawnmower runner:
        STRAIGHT_1  →  TURNING (90° right point turn)  →  STRAIGHT_2  →  DONE

    Construction
    ------------
    follower = PathFollower(seg1_length_m, seg2_length_m)
    follower.set_heading_straight1(fix1, fix2)   # from nudge before seg 1
    # ... run seg 1 ...
    follower.set_heading_straight2(fix1, fix2)   # from nudge before seg 2
                                                 # (or auto from turn encoders)

    step() call
    -----------
    result = follower.step(current_pos, ticks_L, ticks_R)
    send_motor_commands(ser, result["v_L_ms"], result["v_R_ms"],
                             result["paint_arm"])
    """

    def __init__(self, seg1_length_m: float, seg2_length_m: float):
        self.seg1_length = seg1_length_m
        self.seg2_length = seg2_length_m

        # State machine
        self._state = STATE_STRAIGHT_1

        # Heading references (radians, CCW from East)
        self._ref_heading  = 0.0
        self._gps_heading  = 0.0
        self._prev_gps_pos = None

        # Segment start point — updated at each state transition
        self._seg_start = None

        # Encoder state
        self._prev_L  = None
        self._prev_R  = None
        self._delta_L = 0
        self._delta_R = 0

        # PID state (straight driving)
        self._integral   = 0.0
        self._prev_drift = 0.0
        self._integral_clamp = 5.0

        # Turn state
        self._turn_ticks_accumulated = 0
        self._turn_start_L = None
        self._turn_start_R = None

        print("[PathFollower] Initialised.")
        print(f"  Segment 1 : {seg1_length_m:.2f} m")
        print(f"  Turn      : 90° right point turn ({TICKS_PER_90} ticks)")
        print(f"  Segment 2 : {seg2_length_m:.2f} m")

    # ------------------------------------------------------------------
    # Public setup — call before starting each straight
    # ------------------------------------------------------------------

    def set_segment_start(self, pos: tuple):
        """
        Record the GPS start point for the current segment.
        Call this once when the rover is stationary at the start of
        each straight (after fix_1 is confirmed, before ENTER to go).
        """
        self._seg_start   = pos
        self._prev_gps_pos = pos
        print(f"[PathFollower] Segment start set: "
              f"E={pos[0]:.3f}  N={pos[1]:.3f}")

    def set_target_heading(self, fix1: tuple, fix2: tuple):
        """
        Lock the reference heading from two nudge fixes.
        Call once per straight segment after the operator nudges the rover.
        """
        d = dist_m(fix1, fix2)
        if d < 0.10:
            print(f"[PathFollower] WARNING: nudge fixes too close "
                  f"({d:.3f} m) — keeping previous heading.")
            return
        self._ref_heading = heading_from_points(fix1, fix2)
        self._gps_heading = self._ref_heading
        # Reset PID integrator for new segment
        self._integral   = 0.0
        self._prev_drift = 0.0
        print(f"[PathFollower] Heading locked: "
              f"{math.degrees(self._ref_heading):.1f}° from East  "
              f"({heading_to_compass(self._ref_heading):.1f}° compass)")

    # ------------------------------------------------------------------
    def is_finished(self) -> bool:
        return self._state == STATE_DONE

    def current_state(self) -> str:
        return self._state

    # ------------------------------------------------------------------
    # Internal: encoder update
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
    # Internal: PID straight correction
    # ------------------------------------------------------------------

    def _pid_correction(self) -> float:
        """
        Compute m/s speed correction from encoder drift.
        Returns 0.0 if drift is within dead-band.
        Positive correction → slow left, speed right (rover drifted right).
        """
        drift = float(self._delta_L - self._delta_R)

        if abs(drift) <= DRIFT_DEADBAND:
            # Inside dead-band — decay integrator slowly, no correction
            self._integral *= 0.95
            self._prev_drift = drift
            return 0.0

        # P
        p_term = KP_ENC * drift

        # I
        self._integral += drift
        self._integral  = max(-self._integral_clamp,
                               min(self._integral_clamp, self._integral))
        i_term = KI_ENC * self._integral

        # D
        d_term = KD_ENC * (drift - self._prev_drift)
        self._prev_drift = drift

        correction = p_term + i_term + d_term
        return max(-MAX_CORRECTION_MS, min(MAX_CORRECTION_MS, correction))

    # ------------------------------------------------------------------
    # Internal: straight segment step
    # ------------------------------------------------------------------

    def _step_straight(self, current_pos: tuple,
                       seg_length: float,
                       next_state: str) -> dict:
        """Shared logic for STRAIGHT_1 and STRAIGHT_2."""

        self._update_gps_heading(current_pos)

        gps_dist  = dist_m(self._seg_start, current_pos)
        remaining = seg_length - gps_dist

        # --- Stop / transition check ---
        if gps_dist >= (seg_length - STOP_MARGIN_M):
            print(f"\n[PathFollower] {self._state} complete — "
                  f"GPS dist={gps_dist:.2f} m  target={seg_length:.2f} m")
            self._state = next_state

            if next_state == STATE_TURNING:
                # Capture encoder baseline for turn
                self._turn_ticks_accumulated = 0
                self._turn_start_L = self._prev_L
                self._turn_start_R = self._prev_R
                print("[PathFollower] Starting 90° right point turn ...")

            elif next_state == STATE_DONE:
                print("[PathFollower] All segments complete. DONE.")

            return self._stopped_dict(current_pos, paint=False)

        # --- PID correction ---
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
            "enc_drift"       : float(self._delta_L - self._delta_R),
            "correction_ms"   : round(correction, 4),
        }

    # ------------------------------------------------------------------
    # Internal: turning step
    # ------------------------------------------------------------------

    def _step_turning(self) -> dict:
        """
        Point turn: left wheel forward, right wheel reverse.
        Counts ticks on the LEFT wheel (forward direction) to measure rotation.
        Transitions to STRAIGHT_2 when TICKS_PER_90 accumulated.
        """
        if self._prev_L is None:
            # No encoder data yet — can't turn safely
            print("[PathFollower] WARNING: no encoder data during turn. "
                  "Holding position.")
            return self._turn_dict(0, 0)

        # Ticks accumulated since turn started (left wheel, forward)
        ticks_since_start = abs(self._prev_L - self._turn_start_L)
        self._turn_ticks_accumulated = ticks_since_start

        progress = min(1.0, ticks_since_start / TICKS_PER_90)

        if ticks_since_start >= TICKS_PER_90:
            print(f"[PathFollower] Turn complete — "
                  f"{ticks_since_start} ticks accumulated.")
            self._state = STATE_STRAIGHT_2
            # Reset PID state for new straight
            self._integral   = 0.0
            self._prev_drift = 0.0
            self._delta_L    = 0
            self._delta_R    = 0
            return self._turn_dict(0, 0, progress=1.0, done=True)

        # Left forward, right reverse — right 90° point turn
        return self._turn_dict(TURN_SPEED_MS, TURN_SPEED_MS, progress=progress)

    def _turn_dict(self, v_left: float, v_right: float,
                   progress: float = 0.0, done: bool = False) -> dict:
        """
        During a point turn the Arduino needs to drive left forward and
        right in reverse. Because direction is set by Arduino direction pins
        (not the PWM sign), we still send positive magnitudes — but we need
        to tell the Arduino which side to reverse.

        For now we encode this by sending v_R as negative in the serial
        command. Update send_motor_commands (or add a new function) on the
        Arduino side to interpret a negative value as "reverse this side".

        If your Arduino protocol doesn't support signed values yet, use
        send_turn_command() below instead, which sends a dedicated TURN message.
        """
        return {
            "v_L_ms"               : v_left,
            "v_R_ms"               : -v_right,   # negative = reverse right side
            "paint_arm"            : False,        # arm UP during turn
            "status"               : "done" if done else "running",
            "state"                : self._state,
            "easting"              : 0.0,          # not meaningful mid-turn
            "northing"             : 0.0,
            "dist_from_start"      : 0.0,
            "dist_remaining"       : 0.0,
            "heading_deg"          : 0.0,
            "heading_ref_deg"      : 0.0,
            "heading_err_deg"      : 0.0,
            "enc_drift"            : 0.0,
            "correction_ms"        : 0.0,
            "turn_progress_pct"    : round(progress * 100, 1),
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
        ticks_L     : cumulative left  encoder ticks (or None)
        ticks_R     : cumulative right encoder ticks (or None)

        Returns
        -------
        dict — see _step_straight / _step_turning for keys.
        Always contains: v_L_ms, v_R_ms, paint_arm, status, state.
        """
        if self._state == STATE_DONE:
            return self._stopped_dict(current_pos, paint=False)

        # Update encoders every cycle regardless of state
        if ticks_L is not None and ticks_R is not None:
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

        # Should never reach here
        return self._stopped_dict(current_pos, paint=False)

    # ------------------------------------------------------------------

    def _stopped_dict(self, pos: tuple, paint: bool = False) -> dict:
        return {
            "v_L_ms"          : 0.0,
            "v_R_ms"          : 0.0,
            "paint_arm"       : paint,
            "status"          : "finished" if self._state == STATE_DONE else "transitioning",
            "state"           : self._state,
            "easting"         : round(pos[0], 3),
            "northing"        : round(pos[1], 3),
            "dist_from_start" : 0.0,
            "dist_remaining"  : 0.0,
            "heading_deg"     : round(heading_to_compass(self._gps_heading), 1),
            "heading_ref_deg" : round(heading_to_compass(self._ref_heading), 1),
            "heading_err_deg" : 0.0,
            "enc_drift"       : 0.0,
            "correction_ms"   : 0.0,
            "turn_progress_pct": 0.0,
        }


# ---------------------------------------------------------------------------
# Standalone smoke test
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import time, random
    random.seed(42)

    print("PathFollower v3 — state machine smoke test\n")

    follower = PathFollower(seg1_length_m=20.0, seg2_length_m=15.0)

    # Nudge fixes for segment 1 (heading roughly East)
    follower.set_segment_start((500.0, 1000.0))
    follower.set_target_heading((500.0, 1000.0), (500.8, 1000.02))

    e, n    = 500.0, 1000.0
    ticks_L = 0
    ticks_R = 0
    step_n  = 0

    print(f"{'#':>4}  {'State':<12}  {'E':>8}  {'N':>8}  "
          f"{'Dist':>5}  {'Rem':>5}  {'vL':>6}  {'vR':>6}  "
          f"{'Drift':>5}  ARM  Extra")
    print("-" * 85)

    while not follower.is_finished():
        step_n += 1
        state = follower.current_state()

        # Simulate position and encoders per state
        if state == "STRAIGHT_1":
            e += 0.4 + random.uniform(-0.01, 0.01)
            n += random.uniform(-0.02, 0.02)
            ticks_L += 40 + random.randint(0, 2)
            ticks_R += 38 + random.randint(0, 2)

        elif state == "TURNING":
            # Simulate turn ticks accumulating
            ticks_L += 45
            ticks_R += 45

        elif state == "STRAIGHT_2":
            # Now heading North after right turn
            n += 0.4 + random.uniform(-0.01, 0.01)
            e += random.uniform(-0.02, 0.02)
            ticks_L += 40 + random.randint(0, 2)
            ticks_R += 38 + random.randint(0, 2)

            # Set up seg 2 start and heading on first entry
            if follower._seg_start is None or \
               follower._seg_start == (500.0, 1000.0):
                follower.set_segment_start((e, n))
                follower.set_target_heading((e, n), (e + 0.02, n + 0.8))

        r = follower.step((e, n), ticks_L, ticks_R)

        extra = ""
        if "turn_progress_pct" in r and state == "TURNING":
            extra = f"turn {r['turn_progress_pct']:.0f}%"

        arm = "ON " if r["paint_arm"] else "OFF"
        print(f"{step_n:>4}  {r['state']:<12}  "
              f"{r['easting']:>8.3f}  {r['northing']:>8.3f}  "
              f"{r['dist_from_start']:>5.2f}  {r['dist_remaining']:>5.2f}  "
              f"{r['v_L_ms']:>6.3f}  {r['v_R_ms']:>6.3f}  "
              f"{r['enc_drift']:>+5.0f}  {arm}  {extra}")

        if step_n > 200:  # safety limit for smoke test
            print("(smoke test step limit reached)")
            break

        time.sleep(0.02)

    print("\n[Smoke test] complete.")
