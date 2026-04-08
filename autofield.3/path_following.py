"""
path_following.py  (v2 — heading-lock + encoder-based P steering)

Strategy
--------
Replaces the Stanley cross-track controller with a simpler, more robust
approach for a short straight run on a skid-steer rover:

  1. Lock a TARGET HEADING from two GPS nudge fixes (fix_1 → fix_2).
  2. Drive both sides at BASE_SPEED (m/s).
  3. Each cycle, read cumulative encoder ticks from the Arduino.
     Compute the per-cycle tick difference (delta_L - delta_R).
     Apply a P gain to get a speed correction and adjust L/R setpoints.
     Encoders run at high rate and catch drift between GPS updates.
  4. Optionally blend in the GPS-derived heading as a slow sanity check.
  5. Stop when GPS distance from fix_1 reaches (target_dist - STOP_MARGIN_M).

All Stanley / cross-track / IC / vector math is removed.

Encoder serial protocol  (Arduino → Pi, before each motor command)
------------------------------------------------------------------
    Arduino sends:  "ENC:<left_cumulative>,<right_cumulative>\\n"
    Ticks are cumulative signed integers (positive = forward).
    If the Arduino doesn't send ENC lines yet, pass ticks_L=None and
    ticks_R=None to step() — it will fall back to GPS-distance only.

Motor command format  (Pi → Arduino) — unchanged from v1
---------------------------------------------------------
    "L1:<v>,L2:<v>,R1:<v>,R2:<v>,ARM:<v>\\n"
    Values 0–255 are speed setpoints; Arduino closes the velocity loop.

Tuning guide
------------
    BASE_SPEED        Start at 0.15–0.20 m/s. Raise once steering works.
    KP_ENC            Start at 0.003. Raise if rover wanders; lower if it
                      oscillates side-to-side. Typical range 0.001–0.010.
    STOP_MARGIN_M     0.3 m is safe for most ground speeds.
    GPS_HEADING_BLEND 0.0 = pure encoder steering (recommended to start).
                      0.1–0.2 adds a slow GPS sanity pull.
"""

import math
import serial


# ---------------------------------------------------------------------------
# Serial config — unchanged from v1
# ---------------------------------------------------------------------------
SERIAL_PORT = "COM15"
BAUD_RATE   = 9600


def open_serial():
    return serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)


def send_motor_commands(ser, v_L_ms: float, v_R_ms: float, arm: bool):
    """
    Convert m/s wheel speed setpoints to 0–255 and send to Arduino.
    The Arduino closes the velocity loop internally via its P controller.

    Format: "L1:<v>,L2:<v>,R1:<v>,R2:<v>,ARM:<v>\\n"
    Both left motors share v_L_ms; both right motors share v_R_ms.
    Negative speeds are clamped to 0 — direction is set by Arduino pins.

    Parameters
    ----------
    v_L_ms : float  Left-side target speed  (m/s)
    v_R_ms : float  Right-side target speed (m/s)
    arm    : bool   True = paint arm active
    """
    pwm_L   = speed_to_pwm(v_L_ms)
    pwm_R   = speed_to_pwm(v_R_ms)
    arm_val = 1 if arm else 0
    cmd = f"L1:{pwm_L},L2:{pwm_L},R1:{pwm_R},R2:{pwm_R},ARM:{arm_val}\n"
    ser.write(cmd.encode())


# ---------------------------------------------------------------------------
# Speed <-> PWM setpoint conversion  (unchanged from v1)
# ---------------------------------------------------------------------------
MAX_SPEED_MS = 1.5   # m/s that maps to setpoint 255


def speed_to_pwm(speed_ms: float) -> int:
    speed_ms = max(0.0, speed_ms)
    return int(min(255, (speed_ms / MAX_SPEED_MS) * 255.0))


# ---------------------------------------------------------------------------
# Tuning constants
# ---------------------------------------------------------------------------
BASE_SPEED        = 0.20   # m/s — forward speed sent to both sides
KP_ENC            = 0.003  # P gain: (delta_L - delta_R) ticks -> m/s correction
                           # positive drift (left ran more) -> slow left, speed right
KI_ENC            = 0.0001 # start at zero, raise very slowly
KD_ENC            = 0.001  # start at zero, raise if oscillating

MAX_CORRECTION_MS = 0.10   # clamp on the per-cycle speed correction (m/s)

STOP_MARGIN_M     = 0.30   # stop this many metres before the GPS target to
                           # account for deceleration / latency

GPS_HEADING_BLEND = 0.0    # fraction of GPS heading folded into reference
                           # each cycle. 0 = encoder-only (safest to start)
MIN_GPS_MOVE_M    = 0.25   # minimum GPS displacement (m) to trust a new
                           # heading reading — suppresses GPS jitter


# ---------------------------------------------------------------------------
# Geometry helpers  (only the ones we still need)
# ---------------------------------------------------------------------------

def dist_m(a: tuple, b: tuple) -> float:
    """Euclidean distance between two (easting, northing) points."""
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)


def heading_from_points(p1: tuple, p2: tuple) -> float:
    """
    Bearing in radians (CCW from East, UTM convention) from p1 to p2.
    Returns 0.0 if points are identical.
    """
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    if abs(dx) < 1e-9 and abs(dy) < 1e-9:
        return 0.0
    return math.atan2(dy, dx)


def normalize_angle(a: float) -> float:
    """Wrap angle to [-pi, pi]."""
    while a >  math.pi: a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a


def heading_to_compass(rad: float) -> float:
    """Convert radians (CCW from East) to compass degrees (CW from North)."""
    return (90.0 - math.degrees(rad)) % 360.0


# ---------------------------------------------------------------------------
# Encoder line parser
# ---------------------------------------------------------------------------

def parse_encoder_line(line: str):
    """
    Parse "ENC:<left>,<right>" sent by the Arduino.
    Returns (left_ticks, right_ticks) as ints, or None on failure.
    """
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
    Non-blocking drain of the Arduino serial buffer.
    Returns the LAST valid ENC:<L>,<R> line as (left, right), or None.
    Call once per control cycle BEFORE sending motor commands.
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
# PathFollower  (v2 — heading-lock)
# ---------------------------------------------------------------------------

class PathFollower:
    """
    Straight-line runner using encoder P steering and GPS stop trigger.

    Typical call sequence
    ---------------------
    follower = PathFollower([fix1, fix2_target])
    follower.set_target_heading(nudge_fix1, nudge_fix2)   # lock heading

    while not follower.is_finished():
        enc = read_encoders(ser)
        ticks_L, ticks_R = enc if enc else (None, None)
        pos = gnss.get_position()
        result = follower.step(pos, ticks_L, ticks_R)
        send_motor_commands(ser, result["v_L_ms"], result["v_R_ms"],
                            result["paint_arm"])
    """

    def __init__(self, waypoints: list):
        if len(waypoints) < 2:
            raise ValueError("Need at least 2 waypoints.")

        self.start_pt    = waypoints[0]   # fix_1 (easting, northing)
        self.target_pt   = waypoints[1]   # fix_2 target
        self.target_dist = dist_m(self.start_pt, self.target_pt)

        # Reference heading — overwrite with set_target_heading() after nudge
        self._ref_heading  = heading_from_points(self.start_pt, self.target_pt)
        self._gps_heading  = self._ref_heading
        self._prev_gps_pos = self.start_pt

        # Encoder state
        self._prev_L  = None   # previous cumulative left  ticks
        self._prev_R  = None   # previous cumulative right ticks
        self._delta_L = 0      # this-cycle left  tick increment
        self._delta_R = 0      # this-cycle right tick increment

        # PID state var - to be tuned
        self._integral_drift  = 0.0   # accumulated drift over time
        self._prev_enc_drift  = 0.0   # last cycle's drift, for derivative
        self._integral_clamp  = 5.0   # max absolute value of integral term
                               # prevents windup if rover is held still

        self._finished = False

        print(f"[PathFollower] Target dist : {self.target_dist:.2f} m")
        print(f"[PathFollower] Ref heading : "
              f"{math.degrees(self._ref_heading):.1f} deg from East  "
              f"({heading_to_compass(self._ref_heading):.1f} deg compass)")
        print(f"[PathFollower] Will stop at: "
              f"{max(0, self.target_dist - STOP_MARGIN_M):.2f} m from start")

    # ------------------------------------------------------------------
    # Lock heading from nudge fixes
    # ------------------------------------------------------------------

    def set_target_heading(self, fix1: tuple, fix2: tuple):
        """
        Override the reference heading from two physical nudge fixes.
        Call this once after the operator nudges the rover ~1 m forward.

        fix1, fix2 : (easting, northing)
        """
        d = dist_m(fix1, fix2)
        if d < 0.10:
            print(f"[PathFollower] WARNING: nudge fixes too close "
                  f"({d:.3f} m) — keeping previous heading.")
            return
        self._ref_heading = heading_from_points(fix1, fix2)
        self._gps_heading = self._ref_heading
        print(f"[PathFollower] Heading locked from nudge: "
              f"{math.degrees(self._ref_heading):.1f} deg from East  "
              f"({heading_to_compass(self._ref_heading):.1f} deg compass)")

    # ------------------------------------------------------------------
    def is_finished(self) -> bool:
        return self._finished

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _update_encoders(self, ticks_L: int, ticks_R: int):
        """Store per-cycle tick deltas; initialise baseline on first call."""
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

    def _update_gps_heading(self, pos: tuple):
        """
        Blend GPS-derived heading into reference if rover moved enough.
        Skipped entirely when GPS_HEADING_BLEND == 0.0.
        """
        if GPS_HEADING_BLEND == 0.0:
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
    # Main step
    # ------------------------------------------------------------------

    def step(self, current_pos: tuple,
             ticks_L=None,
             ticks_R=None) -> dict:
        """
        Run one control cycle.

        Parameters
        ----------
        current_pos : (easting, northing) — latest GPS fix
        ticks_L     : cumulative left  encoder ticks from Arduino (or None)
        ticks_R     : cumulative right encoder ticks from Arduino (or None)

        Returns
        -------
        dict with keys:
            v_L_ms          left  speed setpoint (m/s) -> send_motor_commands()
            v_R_ms          right speed setpoint (m/s) -> send_motor_commands()
            paint_arm       bool — True while running
            status          "running" | "finished"
            easting         float
            northing        float
            dist_from_start GPS distance from fix_1 (m)
            dist_remaining  metres left to target
            heading_deg     GPS-blended heading (compass deg, CW from North)
            heading_ref_deg reference heading   (compass deg, CW from North)
            heading_err_deg GPS heading - reference (degrees, signed)
            enc_drift       delta_L - delta_R this cycle (ticks)
            correction_ms   speed correction applied this cycle (m/s)
            have_enc        bool — True if encoder data was available
        """
        if self._finished:
            return self._stopped_dict(current_pos)

        # ---- 1. Encoders ------------------------------------------------
        if ticks_L is not None and ticks_R is not None:
            self._update_encoders(ticks_L, ticks_R)
            have_enc = True
        else:
            self._delta_L = 0
            self._delta_R = 0
            have_enc = False

        # ---- 2. GPS heading blend (optional) ----------------------------
        self._update_gps_heading(current_pos)

        # ---- 3. Stop check ----------------------------------------------
        gps_dist  = dist_m(self.start_pt, current_pos)
        remaining = self.target_dist - gps_dist

        if gps_dist >= (self.target_dist - STOP_MARGIN_M):
            self._finished = True
            print(f"\n[PathFollower] STOP — "
                  f"GPS dist={gps_dist:.2f} m  target={self.target_dist:.2f} m")
            return self._stopped_dict(current_pos)

        # ---- 4. P steering correction -----------------------------------
        #
        # enc_drift > 0  -> left wheel ran more -> rover veered RIGHT
        #                -> slow left, speed up right to pull back left
        # enc_drift < 0  -> right wheel ran more -> rover veered LEFT
        #                -> slow right, speed up left
        #
        enc_drift  = float(self._delta_L - self._delta_R)

        # Proportional
        p_term = KP_ENC * enc_drift

        # Integral — accumulate, then clamp to prevent windup
        self._integral_drift += enc_drift
        self._integral_drift  = max(-self._integral_clamp,
                                    min(self._integral_clamp,
                                        self._integral_drift))
        i_term = KI_ENC * self._integral_drift

        # Derivative — rate of change of drift
        d_term = KD_ENC * (enc_drift - self._prev_enc_drift)
        self._prev_enc_drift = enc_drift

        correction = p_term + i_term + d_term
        correction = max(-MAX_CORRECTION_MS, min(MAX_CORRECTION_MS, correction))

        v_L_ms = max(0.0, BASE_SPEED - correction)
        v_R_ms = max(0.0, BASE_SPEED + correction)

        # ---- 5. Telemetry -----------------------------------------------
        heading_err = normalize_angle(self._gps_heading - self._ref_heading)

        return {
            "v_L_ms"          : round(v_L_ms, 4),
            "v_R_ms"          : round(v_R_ms, 4),
            "paint_arm"       : True,
            "status"          : "running",
            "easting"         : round(current_pos[0], 3),
            "northing"        : round(current_pos[1], 3),
            "dist_from_start" : round(gps_dist, 3),
            "dist_remaining"  : round(remaining, 3),
            "heading_deg"     : round(heading_to_compass(self._gps_heading), 1),
            "heading_ref_deg" : round(heading_to_compass(self._ref_heading), 1),
            "heading_err_deg" : round(math.degrees(heading_err), 2),
            "enc_drift"       : enc_drift,
            "correction_ms"   : round(correction, 4),
            "have_enc"        : have_enc,
            "p_term"          : round(p_term, 5),
            "i_term"          : round(i_term, 5),
            "d_term"          : round(d_term, 5),
        }

    # ------------------------------------------------------------------
    def _stopped_dict(self, pos: tuple) -> dict:
        gps_dist = dist_m(self.start_pt, pos)
        return {
            "v_L_ms"          : 0.0,
            "v_R_ms"          : 0.0,
            "paint_arm"       : False,
            "status"          : "finished",
            "easting"         : round(pos[0], 3),
            "northing"        : round(pos[1], 3),
            "dist_from_start" : round(gps_dist, 3),
            "dist_remaining"  : round(self.target_dist - gps_dist, 3),
            "heading_deg"     : round(heading_to_compass(self._gps_heading), 1),
            "heading_ref_deg" : round(heading_to_compass(self._ref_heading), 1),
            "heading_err_deg" : 0.0,
            "enc_drift"       : 0.0,
            "correction_ms"   : 0.0,
            "have_enc"        : False,
        }


# ---------------------------------------------------------------------------
# Smoke test — run standalone to verify logic without hardware
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import time, random
    random.seed(7)

    print("PathFollower v2 — standalone smoke test (no hardware)")
    print("Simulating 20 m eastward run with encoder drift\n")

    wp1 = (500.000, 1000.000)
    wp2 = (520.000, 1000.000)
    follower = PathFollower([wp1, wp2])

    # Simulate nudge: rover nudged ~0.8 m east with a tiny northing wobble
    follower.set_target_heading((500.000, 1000.000), (500.800, 1000.030))

    e, n    = 500.0, 1000.0
    ticks_L = 0
    ticks_R = 0

    hdr = (f"{'#':>4}  {'E':>9}  {'N':>9}  {'Dist':>5}  {'Rem':>5}  "
           f"{'HdgGPS':>7}  {'HRef':>6}  {'HErr':>5}  "
           f"{'Drift':>5}  {'Corr':>7}  {'vL':>6}  {'vR':>6}")
    print(hdr)
    print("-" * len(hdr))

    step = 0
    while not follower.is_finished():
        step += 1
        e += 0.5 + random.uniform(-0.02, 0.02)
        n += random.uniform(-0.04, 0.04)

        # Left ticks slightly faster -> rover drifts right -> correction pulls left
        ticks_L += 50 + random.randint(0, 2)
        ticks_R += 47 + random.randint(0, 2)

        r = follower.step((e, n), ticks_L, ticks_R)

        print(f"{step:>4}  "
              f"{r['easting']:>9.3f}  {r['northing']:>9.3f}  "
              f"{r['dist_from_start']:>5.2f}  {r['dist_remaining']:>5.2f}  "
              f"{r['heading_deg']:>7.1f}  {r['heading_ref_deg']:>6.1f}  "
              f"{r['heading_err_deg']:>+5.1f}  "
              f"{r['enc_drift']:>+5.0f}  {r['correction_ms']:>+7.4f}  "
              f"{r['v_L_ms']:>6.4f}  {r['v_R_ms']:>6.4f}")

        if r["status"] == "finished":
            break
        time.sleep(0.02)

    print("\n[Smoke test] complete.")
