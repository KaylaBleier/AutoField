"""
path_following.py  (v5 — single speed, GPS stop, timed turn)

All four wheels receive the same speed setpoint at all times during
straight segments. There is no differential, no drift correction, and
no encoder dependency. GPS distance is the only stop trigger.
The 90° right point turn is purely time-based.

When encoders or an IMU are available and confirmed working, drift
correction can be added back on top of this foundation.

State machine
-------------
  STRAIGHT_1  →  TURNING  →  STRAIGHT_2  →  DONE

Tuning — the only three numbers you need to calibrate
------------------------------------------------------
  BASE_SPEED        m/s sent to all four wheels during straights.
                    Start low (~0.15), raise once GPS stop is working.

  TURN_SPEED_MS     m/s magnitude during point turn.
                    Start low (~0.10). Consistent speed = consistent turn.

  TURN_DURATION_SEC Seconds for a 90° right point turn at TURN_SPEED_MS.
                    Calibrate: run a turn, time it with a stopwatch,
                    adjust until visually 90°. Repeat 3x and average.
                    *** If you change TURN_SPEED_MS, recalibrate this. ***

Motor command format  (Pi → Arduino)
--------------------------------------
  "L1:<v>,L2:<v>,R1:<v>,R2:<v>,ARM:<v>\\n"
  Values 0–255. Arduino closes per-wheel velocity loop.
  During a straight: L1=L2=R1=R2 always.
  During a turn:     L1=L2 forward, R1=R2 reverse (right side).
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


def send_motor_commands(ser, pwm_all: int, arm: bool):
    """
    Send the same speed setpoint to all four wheels.
    Used during straight segments — no differential at all.

    Parameters
    ----------
    pwm_all : int   Single speed setpoint [0, 255] for all wheels.
    arm     : bool  True = paint arm active.
    """
    pwm_all = int(max(0, min(255, pwm_all)))
    arm_val = 1 if arm else 0
    cmd = f"L1:{pwm_all},L2:{pwm_all},R1:{pwm_all},R2:{pwm_all},ARM:{arm_val}\n"
    ser.write(cmd.encode())


def send_turn_command(ser, pwm_left: int, pwm_right: int, arm: bool):
    """
    Send differential speed for point turn.
    Left side forward, right side reverse (handled by Arduino direction pins).
    The Arduino must interpret a negative R value as reverse — update
    Arduino code if it only accepts 0–255 unsigned.

    Parameters
    ----------
    pwm_left  : int  Left side setpoint  [0, 255] (forward).
    pwm_right : int  Right side setpoint [0, 255] (reverse, sign handled by Arduino).
    arm       : bool Paint arm (off during turn).
    """
    pwm_left  = int(max(0, min(255, pwm_left)))
    pwm_right = int(max(0, min(255, pwm_right)))
    arm_val   = 1 if arm else 0
    cmd = f"L1:{pwm_left},L2:{pwm_left},R1:-{pwm_right},R2:-{pwm_right},ARM:{arm_val}\n"
    ser.write(cmd.encode())


# ---------------------------------------------------------------------------
# Speed -> PWM
# ---------------------------------------------------------------------------
MAX_SPEED_MS = 1.5   # m/s that maps to setpoint 255


def speed_to_pwm(speed_ms: float) -> int:
    speed_ms = max(0.0, speed_ms)
    return int(min(255, (speed_ms / MAX_SPEED_MS) * 255.0))


# ---------------------------------------------------------------------------
# Tuning constants  ← the only numbers to adjust
# ---------------------------------------------------------------------------
BASE_SPEED        = 0.15   # m/s — all four wheels during straight segments
TURN_SPEED_MS     = 0.10   # m/s — each side during point turn
TURN_DURATION_SEC = 3.0    # *** CALIBRATE *** seconds for a 90° right turn
STOP_MARGIN_M     = 0.30   # stop this many metres before GPS target


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
    """Straight-line distance between two (easting, northing) points."""
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)


def heading_from_points(p1: tuple, p2: tuple) -> float:
    """Bearing in radians (CCW from East) from p1 to p2."""
    dx, dy = p2[0] - p1[0], p2[1] - p1[1]
    if abs(dx) < 1e-9 and abs(dy) < 1e-9:
        return 0.0
    return math.atan2(dy, dx)


def heading_to_compass(rad: float) -> float:
    """Radians CCW-from-East → compass degrees CW-from-North."""
    return (90.0 - math.degrees(rad)) % 360.0


# ---------------------------------------------------------------------------
# PathFollower  (v5)
# ---------------------------------------------------------------------------

class PathFollower:
    """
    Two-segment lawnmower runner.

    Usage
    -----
    follower = PathFollower(seg1_length_m, seg2_length_m)

    # Before segment 1:
    follower.set_segment_start(fix1)
    follower.set_target_heading(fix1, nudge_fix2)

    # Before segment 2 (after turn completes):
    follower.set_segment_start(fix1)
    follower.set_target_heading(fix1, nudge_fix2)

    # Each cycle:
    result = follower.step(gnss.get_position())
    if result["state"] == STATE_TURNING:
        send_turn_command(ser, result["pwm_turn"], result["pwm_turn"], False)
    else:
        send_motor_commands(ser, result["pwm_all"], result["paint_arm"])
    """

    def __init__(self, seg1_length_m: float, seg2_length_m: float):
        self.seg1_length = seg1_length_m
        self.seg2_length = seg2_length_m

        self._state = STATE_STRAIGHT_1

        # Heading (for telemetry readout only — not used for steering)
        self._ref_heading  = 0.0
        self._gps_heading  = 0.0
        self._prev_gps_pos = None

        # GPS start point for current segment
        self._seg_start = None

        # Turn timer
        self._turn_start_time = None

        # Precompute PWM values once
        self._pwm_base = speed_to_pwm(BASE_SPEED)
        self._pwm_turn = speed_to_pwm(TURN_SPEED_MS)

        print(f"[PathFollower] Segment 1    : {seg1_length_m:.2f} m")
        print(f"[PathFollower] Segment 2    : {seg2_length_m:.2f} m")
        print(f"[PathFollower] Base PWM     : {self._pwm_base}  "
              f"({BASE_SPEED} m/s)")
        print(f"[PathFollower] Turn PWM     : {self._pwm_turn}  "
              f"({TURN_SPEED_MS} m/s)")
        print(f"[PathFollower] Turn duration: {TURN_DURATION_SEC} s")
        print(f"[PathFollower] Stop margin  : {STOP_MARGIN_M} m")

    # ------------------------------------------------------------------
    # Setup — call before each segment
    # ------------------------------------------------------------------

    def set_segment_start(self, pos: tuple):
        """Record GPS start point for the current segment."""
        self._seg_start    = pos
        self._prev_gps_pos = pos
        print(f"[PathFollower] Segment start: "
              f"E={pos[0]:.3f}  N={pos[1]:.3f}")

    def set_target_heading(self, fix1: tuple, fix2: tuple):
        """
        Lock reference heading from two nudge fixes.
        Used for telemetry display only — does not affect motor commands.
        """
        d = dist_m(fix1, fix2)
        if d < 0.10:
            print(f"[PathFollower] WARNING: nudge too close ({d:.3f} m).")
            return
        self._ref_heading = heading_from_points(fix1, fix2)
        self._gps_heading = self._ref_heading
        print(f"[PathFollower] Heading reference: "
              f"{math.degrees(self._ref_heading):.1f}° from East  "
              f"({heading_to_compass(self._ref_heading):.1f}° compass)")

    def is_finished(self) -> bool:
        return self._state == STATE_DONE

    def current_state(self) -> str:
        return self._state

    # ------------------------------------------------------------------
    # GPS heading (telemetry only)
    # ------------------------------------------------------------------

    def _update_gps_heading(self, pos: tuple):
        """Update displayed GPS heading when rover has moved enough."""
        if self._prev_gps_pos is None:
            return
        d = dist_m(self._prev_gps_pos, pos)
        if d < 0.25:
            return
        self._gps_heading  = heading_from_points(self._prev_gps_pos, pos)
        self._prev_gps_pos = pos

    # ------------------------------------------------------------------
    # Straight segment step
    # ------------------------------------------------------------------

    def _step_straight(self, pos: tuple,
                       seg_length: float,
                       next_state: str) -> dict:

        self._update_gps_heading(pos)

        gps_dist  = dist_m(self._seg_start, pos)
        remaining = seg_length - gps_dist

        # Transition check
        if gps_dist >= (seg_length - STOP_MARGIN_M):
            print(f"\n[PathFollower] {self._state} complete — "
                  f"{gps_dist:.2f} m travelled, target {seg_length:.2f} m")
            self._state = next_state

            if next_state == STATE_TURNING:
                self._turn_start_time = time.time()
                print(f"[PathFollower] Starting {TURN_DURATION_SEC}s "
                      f"right point turn ...")
            elif next_state == STATE_DONE:
                print("[PathFollower] All segments complete. DONE.")

            # Stop motors on transition
            return self._make_dict(pos, gps_dist, remaining,
                                   pwm_all=0, pwm_turn=0,
                                   paint=False, turn_pct=0.0)

        return self._make_dict(pos, gps_dist, remaining,
                               pwm_all=self._pwm_base, pwm_turn=0,
                               paint=True, turn_pct=0.0)

    # ------------------------------------------------------------------
    # Turn step
    # ------------------------------------------------------------------

    def _step_turning(self) -> dict:
        elapsed  = time.time() - self._turn_start_time
        progress = min(1.0, elapsed / TURN_DURATION_SEC)

        if elapsed >= TURN_DURATION_SEC:
            print(f"[PathFollower] Turn complete.")
            self._state = STATE_STRAIGHT_2
            return self._make_dict(None, 0, 0,
                                   pwm_all=0, pwm_turn=0,
                                   paint=False, turn_pct=100.0)

        return self._make_dict(None, 0, 0,
                               pwm_all=0, pwm_turn=self._pwm_turn,
                               paint=False, turn_pct=round(progress * 100, 1))

    # ------------------------------------------------------------------
    # Main step
    # ------------------------------------------------------------------

    def step(self, current_pos: tuple) -> dict:
        """
        Run one control cycle.

        Parameters
        ----------
        current_pos : (easting, northing) from gnss.get_position()

        Returns
        -------
        dict with keys:
            state           current state string
            pwm_all         PWM for all four wheels (straight only, else 0)
            pwm_turn        PWM magnitude for turn (turn only, else 0)
            paint_arm       bool
            status          "running" | "finished" | "transitioning"
            easting         float (0.0 during turn)
            northing        float (0.0 during turn)
            dist_from_start GPS distance from segment start (m)
            dist_remaining  metres left in current segment
            heading_deg     measured GPS heading (compass degrees)
            heading_ref_deg reference heading from nudge (compass degrees)
            turn_progress_pct 0–100 during turn, else 0.0

        Motor commands in main.py
        -------------------------
            if result["state"] == STATE_TURNING:
                send_turn_command(ser, result["pwm_turn"],
                                       result["pwm_turn"], False)
            else:
                send_motor_commands(ser, result["pwm_all"],
                                         result["paint_arm"])
        """
        if self._state == STATE_DONE:
            return self._make_dict(current_pos, 0, 0,
                                   pwm_all=0, pwm_turn=0,
                                   paint=False, turn_pct=0.0)

        if self._state == STATE_STRAIGHT_1:
            return self._step_straight(current_pos, self.seg1_length,
                                       STATE_TURNING)

        if self._state == STATE_TURNING:
            return self._step_turning()

        if self._state == STATE_STRAIGHT_2:
            return self._step_straight(current_pos, self.seg2_length,
                                       STATE_DONE)

        return self._make_dict(current_pos, 0, 0,
                               pwm_all=0, pwm_turn=0,
                               paint=False, turn_pct=0.0)

    # ------------------------------------------------------------------
    # Dict builder
    # ------------------------------------------------------------------

    def _make_dict(self, pos, dist_from_start, dist_remaining,
                   pwm_all, pwm_turn, paint, turn_pct) -> dict:

        status = ("finished"      if self._state == STATE_DONE
                  else "transitioning" if pwm_all == 0 and pwm_turn == 0
                  else "running")

        return {
            "state"            : self._state,
            "pwm_all"          : pwm_all,
            "pwm_turn"         : pwm_turn,
            "paint_arm"        : paint,
            "status"           : status,
            "easting"          : round(pos[0], 3) if pos else 0.0,
            "northing"         : round(pos[1], 3) if pos else 0.0,
            "dist_from_start"  : round(dist_from_start, 3),
            "dist_remaining"   : round(dist_remaining, 3),
            "heading_deg"      : round(heading_to_compass(self._gps_heading), 1),
            "heading_ref_deg"  : round(heading_to_compass(self._ref_heading), 1),
            "turn_progress_pct": turn_pct,
        }


# ---------------------------------------------------------------------------
# Smoke test
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import random
    random.seed(42)

    print("PathFollower v5 smoke test — single speed, GPS stop, timed turn\n")

    follower = PathFollower(seg1_length_m=10.0, seg2_length_m=8.0)
    follower.set_segment_start((500.0, 1000.0))
    follower.set_target_heading((500.0, 1000.0), (500.8, 1000.01))

    e, n   = 500.0, 1000.0
    step_n = 0
    seg2_ready = False

    print(f"{'#':>4}  {'State':<12}  {'E':>8}  {'N':>8}  "
          f"{'Dist':>5}  {'Rem':>5}  {'PWM':>4}  ARM  Extra")
    print("-" * 72)

    while not follower.is_finished():
        step_n += 1
        state = follower.current_state()

        if state == STATE_STRAIGHT_1:
            e += 0.4 + random.uniform(-0.01, 0.01)
            n += random.uniform(-0.02, 0.02)
        elif state == STATE_STRAIGHT_2:
            if not seg2_ready:
                follower.set_segment_start((e, n))
                follower.set_target_heading((e, n), (e + 0.02, n + 0.8))
                seg2_ready = True
            n += 0.4 + random.uniform(-0.01, 0.01)
            e += random.uniform(-0.02, 0.02)

        r = follower.step((e, n))

        pwm   = r["pwm_turn"] if state == STATE_TURNING else r["pwm_all"]
        arm   = "ON " if r["paint_arm"] else "OFF"
        extra = (f"turn {r['turn_progress_pct']:.0f}%"
                 if state == STATE_TURNING else "")

        print(f"{step_n:>4}  {r['state']:<12}  "
              f"{r['easting']:>8.3f}  {r['northing']:>8.3f}  "
              f"{r['dist_from_start']:>5.2f}  {r['dist_remaining']:>5.2f}  "
              f"{pwm:>4}  {arm}  {extra}")

        if step_n > 300:
            print("(step limit)")
            break
        time.sleep(0.01)

    print("\n[Smoke test] complete.")
