"""
encoder_stop.py

Dead-reckoning stop conditions using wheel encoder pulse counts.
Provides two independent methods — use either or both together.

Method A — Distance accumulation (simple geometry):
    Count pulses × metres_per_pulse until we reach run_length_m.
    Simple, reliable, drifts slightly with wheel slip.

Method B — Cross-product sign flip (Stanley-style):
    Project the rover's estimated position onto the path vector.
    Stop when the rover crosses the perpendicular plane through wp2.
    More robust against encoder drift for the stop condition specifically.

Calibration:
    PULSES_PER_REV and WHEEL_CIRCUMFERENCE_M are the only numbers you need
    to measure. Roll the rover exactly 1 metre, count the pulses, done.

Usage:
    from encoder_stop import EncoderStop

    stop = EncoderStop(wp1, wp2, run_length_m)

    # Call this every time your encoder ISR fires (or in your main loop
    # if you're polling):
    stop.update(left_pulses, right_pulses)

    # Check stop condition each control tick:
    if stop.should_stop():
        reason = stop.stop_reason
        print(f"Stopping: {reason}")
"""

import math


# ---------------------------------------------------------------------------
# Calibration — measure these on your rover
# ---------------------------------------------------------------------------

# Number of encoder pulses per full wheel revolution.
# Count by rolling the wheel by hand one full turn and watching your encoder.
PULSES_PER_REV = 20

# Wheel circumference in metres.
# Measure your wheel diameter in mm, divide by 1000, multiply by pi.
# e.g. 150mm wheel: 0.150 * pi = 0.471 m
WHEEL_CIRCUMFERENCE_M = 0.471

# Derived: metres per pulse (do not edit — computed from above)
METRES_PER_PULSE = WHEEL_CIRCUMFERENCE_M / PULSES_PER_REV

# Stop a little early to account for braking distance (metres).
# Rover won't stop instantly — tune this so it actually stops at wp2.
BRAKE_MARGIN_M = 0.15

# How far past wp2 to tolerate before triggering cross-product stop (metres).
# Keeps the stop from firing on a single noisy pulse.
CROSS_GUARD_M = 0.05


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _path_vec(wp1: tuple, wp2: tuple) -> tuple:
    """Unit vector from wp1 to wp2."""
    dx = wp2[0] - wp1[0]
    dy = wp2[1] - wp1[1]
    length = math.sqrt(dx**2 + dy**2)
    if length < 1e-9:
        return (0.0, 0.0)
    return (dx / length, dy / length)


def _dot_wp2_to_pos(wp2: tuple, ux: float, uy: float,
                    pos: tuple) -> float:
    """
    Dot product of the path unit vector with the vector (wp2 → pos).

    Positive  = pos is still short of wp2 (rover hasn't reached it yet)
    Zero      = pos is exactly at wp2
    Negative  = pos has passed wp2

    This is Method B's core quantity.
    """
    return ux * (wp2[0] - pos[0]) + uy * (wp2[1] - pos[1])


# ---------------------------------------------------------------------------
# EncoderStop
# ---------------------------------------------------------------------------

class EncoderStop:
    """
    Tracks encoder pulses and estimates when the rover has reached wp2.

    Two stop conditions run in parallel:
      A. dist_travelled >= (run_length_m - BRAKE_MARGIN_M)
      B. cross-product sign flips from + to ≤ 0

    Call update() every encoder tick (or poll it in your loop).
    Call should_stop() every control cycle.

    Parameters
    ----------
    wp1          : (easting, northing) start position
    wp2          : (easting, northing) target position
    run_length_m : intended run distance in metres (from waypoint_gen)
    """

    def __init__(self, wp1: tuple, wp2: tuple, run_length_m: float):
        self.wp1          = wp1
        self.wp2          = wp2
        self.run_length_m = run_length_m

        # Precompute path unit vector (constant for a straight run)
        self._ux, self._uy = _path_vec(wp1, wp2)

        # Dead-reckoning state
        self._dist_travelled = 0.0          # metres accumulated so far
        self._est_pos        = list(wp1)    # estimated position (easting, northing)
        self._prev_dot       = None         # previous cross-product value

        # Result
        self._stopped    = False
        self.stop_reason = ""

        # Cumulative pulse accumulators (for logging)
        self._total_left  = 0
        self._total_right = 0

    # ------------------------------------------------------------------
    # Pulse intake — call this whenever you have new encoder counts
    # ------------------------------------------------------------------

    def update(self, left_pulses: int, right_pulses: int):
        """
        Feed new encoder pulse counts since the last call.

        Parameters
        ----------
        left_pulses  : int  Pulses from left wheel encoder since last update
        right_pulses : int  Pulses from right wheel encoder since last update

        Both values should be non-negative incremental counts, not totals.
        """
        if self._stopped:
            return

        self._total_left  += left_pulses
        self._total_right += right_pulses

        # Average both sides — smooths out minor slip differences
        avg_pulses = (left_pulses + right_pulses) / 2.0
        dist_step  = avg_pulses * METRES_PER_PULSE

        self._dist_travelled += dist_step

        # Advance estimated position along path unit vector
        self._est_pos[0] += self._ux * dist_step
        self._est_pos[1] += self._uy * dist_step

        # --- Method B: cross-product sign flip ---
        dot = _dot_wp2_to_pos(
            self.wp2, self._ux, self._uy, tuple(self._est_pos)
        )

        if self._prev_dot is not None:
            # Positive → zero-or-negative = crossed the wp2 plane
            if self._prev_dot > CROSS_GUARD_M and dot <= 0:
                self._trigger(
                    f"cross-product sign flip "
                    f"(dist={self._dist_travelled:.3f} m)"
                )
                return

        self._prev_dot = dot

        # --- Method A: distance accumulation ---
        target = self.run_length_m - BRAKE_MARGIN_M
        if self._dist_travelled >= target:
            self._trigger(
                f"distance reached "
                f"({self._dist_travelled:.3f} m >= {target:.3f} m target)"
            )

    def _trigger(self, reason: str):
        self._stopped    = True
        self.stop_reason = reason

    # ------------------------------------------------------------------
    # Query — call each control tick
    # ------------------------------------------------------------------

    def should_stop(self) -> bool:
        """Return True when either stop condition has fired."""
        return self._stopped

    def distance_travelled(self) -> float:
        """Estimated metres travelled so far."""
        return self._dist_travelled

    def estimated_position(self) -> tuple:
        """Dead-reckoned (easting, northing) — best guess without GPS."""
        return tuple(self._est_pos)

    def progress(self) -> float:
        """Fraction of run_length_m completed [0.0 – 1.0+]."""
        if self.run_length_m < 1e-9:
            return 0.0
        return self._dist_travelled / self.run_length_m

    def status(self) -> dict:
        """Full status snapshot — useful for logging."""
        return {
            "dist_travelled_m" : round(self._dist_travelled, 4),
            "run_length_m"     : self.run_length_m,
            "progress"         : round(self.progress(), 4),
            "est_easting"      : round(self._est_pos[0], 4),
            "est_northing"     : round(self._est_pos[1], 4),
            "stopped"          : self._stopped,
            "stop_reason"      : self.stop_reason,
            "total_left_pulses": self._total_left,
            "total_right_pulses": self._total_right,
        }

    def reset(self):
        """Reset for a new run (keeps wp1/wp2/run_length)."""
        self.__init__(self.wp1, self.wp2, self.run_length_m)


# ---------------------------------------------------------------------------
# Calibration helper — run this once on the bench
# ---------------------------------------------------------------------------

def calibrate(measured_pulses: int, measured_distance_m: float):
    """
    Print the calibrated METRES_PER_PULSE for your rover.

    Run the rover a known distance, count pulses, call this.

    Example:
        calibrate(measured_pulses=847, measured_distance_m=10.0)
    """
    mpp = measured_distance_m / measured_pulses
    circ = mpp * PULSES_PER_REV
    print(f"\n--- Encoder calibration ---")
    print(f"  Pulses measured    : {measured_pulses}")
    print(f"  Distance measured  : {measured_distance_m:.4f} m")
    print(f"  Metres per pulse   : {mpp:.6f} m")
    print(f"  Wheel circumference: {circ:.4f} m")
    print(f"  Wheel diameter     : {circ / math.pi * 1000:.1f} mm")
    print(f"\nSet in encoder_stop.py:")
    print(f"  METRES_PER_PULSE      = {mpp:.6f}")
    print(f"  WHEEL_CIRCUMFERENCE_M = {circ:.4f}")
