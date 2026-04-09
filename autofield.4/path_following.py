"""
path_following.py

Simplified straight-line path follower for a skid-steer rover.

Controller:
    - Heading error correction  (turn to face the path direction)
    - Lateral error correction  (steer back onto the line)
    - Combined into a single steering angle → differential PWM split

Paint arm:
    - Activates only once heading error is within HEADING_ARM_THRESHOLD
    - Deactivates at end of run

Arduino serial format (unchanged):
    "L1:<v>,L2:<v>,R1:<v>,R2:<v>,ARM:<v>\\n"
    All values 0–255 for PWM, ARM is 0 or 1.
"""

import math
import serial


# ---------------------------------------------------------------------------
# Serial config
# ---------------------------------------------------------------------------
SERIAL_PORT = "COM15"
BAUD_RATE   = 9600


def open_serial() -> serial.Serial:
    return serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)


def send_motor_commands(ser, pwm_L: int, pwm_R: int, arm: bool):
    """
    Send PWM + arm command to Arduino.
    Format: "L1:<v>,L2:<v>,R1:<v>,R2:<v>,ARM:<v>\\n"
    """
    pwm_L   = int(max(0, min(255, pwm_L)))
    pwm_R   = int(max(0, min(255, pwm_R)))
    arm_val = 1 if arm else 0
    cmd = f"L1:{pwm_L},L2:{pwm_L},R1:{pwm_R},R2:{pwm_R},ARM:{arm_val}\n"
    ser.write(cmd.encode())


# ---------------------------------------------------------------------------
# Tuning constants
# ---------------------------------------------------------------------------
K_HEADING        = 1.2    # Gain on heading error (radians → steering correction)
K_LATERAL        = 0.6    # Gain on lateral error (metres → steering correction)
BASE_SPEED_PWM   = 180    # Nominal forward PWM (0–255) — tune to your rover
MAX_STEER_PWM    = 80     # Max PWM differential between left and right wheels
WHEELBASE        = 0.6    # Left-to-right wheel separation (metres)

# Rover must be within this heading error (degrees) before arm activates
HEADING_ARM_THRESHOLD_DEG = 10.0

# Rover is considered "at waypoint" when within this distance (metres)
WP_ACCEPT_RADIUS = 0.8


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _normalize_angle(angle: float) -> float:
    """Wrap angle to [-pi, pi]."""
    while angle >  math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def _heading_from_points(p1: tuple, p2: tuple) -> float:
    """Heading angle (radians, CCW from east) from two UTM positions."""
    return math.atan2(p2[1] - p1[1], p2[0] - p1[0])


def _distance(p1: tuple, p2: tuple) -> float:
    return math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)


def _lateral_error(wp1: tuple, wp2: tuple, P: tuple) -> float:
    """
    Signed perpendicular distance from P to the line wp1→wp2.
    Positive = rover is to the LEFT of the path direction.
    """
    dx = wp2[0] - wp1[0]
    dy = wp2[1] - wp1[1]
    path_len = math.sqrt(dx**2 + dy**2)
    if path_len < 1e-9:
        return 0.0
    # 2D cross product: (wp2-wp1) × (P-wp1) / |wp2-wp1|
    return ((dx * (P[1] - wp1[1])) - (dy * (P[0] - wp1[0]))) / path_len


# ---------------------------------------------------------------------------
# PathFollower
# ---------------------------------------------------------------------------

class PathFollower:
    """
    Drives the rover from wp1 to wp2 in a straight line.

    Call step() every control tick. When is_finished() returns True,
    command motors to stop.
    """

    def __init__(self, wp1: tuple, wp2: tuple, heading_rad: float):
        """
        Parameters
        ----------
        wp1         : (easting, northing) start waypoint
        wp2         : (easting, northing) target waypoint
        heading_rad : intended heading (radians), pre-computed from nudge fix
        """
        self.wp1         = wp1
        self.wp2         = wp2
        self.heading_rad = heading_rad   # intended heading of the path
        self._finished   = False

    def is_finished(self) -> bool:
        return self._finished

    # ------------------------------------------------------------------
    # Progress along path [0, 1]
    # ------------------------------------------------------------------
    def _progress_t(self, P: tuple) -> float:
        dx = self.wp2[0] - self.wp1[0]
        dy = self.wp2[1] - self.wp1[1]
        denom = dx**2 + dy**2
        if denom < 1e-9:
            return 0.0
        ax = P[0] - self.wp1[0]
        ay = P[1] - self.wp1[1]
        return (ax * dx + ay * dy) / denom

    # ------------------------------------------------------------------
    # Main control step
    # ------------------------------------------------------------------
    def step(self, P: tuple, prev_P: tuple, speed_ms: float = 0.5) -> dict:
        """
        Compute motor commands for the current position.

        Parameters
        ----------
        P        : (easting, northing) current GPS position
        prev_P   : (easting, northing) previous GPS position (for heading)
        speed_ms : current speed in m/s from GNSSReader (used for soft stop)

        Returns
        -------
        dict with keys:
            pwm_L, pwm_R    — motor PWM [0, 255]
            paint_arm       — bool
            heading_error   — degrees
            lateral_error   — metres (signed)
            t               — progress [0, 1]
            status          — "running" | "finished"
        """
        if self._finished:
            return _stopped_result()

        # --- Check arrival ---
        dist_to_wp2 = _distance(P, self.wp2)
        if dist_to_wp2 <= WP_ACCEPT_RADIUS:
            self._finished = True
            return _stopped_result()

        # --- Heading from recent GPS motion ---
        actual_heading = _heading_from_points(prev_P, P)

        # --- Errors ---
        head_err = _normalize_angle(self.heading_rad - actual_heading)
        lat_err  = _lateral_error(self.wp1, self.wp2, P)

        # --- Steering correction (radians) ---
        steer = K_HEADING * head_err + K_LATERAL * lat_err

        # --- Differential PWM ---
        # Positive steer → turn left → reduce left, increase right
        steer_pwm = int(max(-MAX_STEER_PWM, min(MAX_STEER_PWM, steer * (180 / math.pi))))
        pwm_L = BASE_SPEED_PWM - steer_pwm
        pwm_R = BASE_SPEED_PWM + steer_pwm
        pwm_L = int(max(0, min(255, pwm_L)))
        pwm_R = int(max(0, min(255, pwm_R)))

        # --- Paint arm — only when on-heading ---
        on_heading = abs(math.degrees(head_err)) <= HEADING_ARM_THRESHOLD_DEG
        paint_arm  = on_heading

        # --- Progress ---
        t = self._progress_t(P)

        return {
            "pwm_L"         : pwm_L,
            "pwm_R"         : pwm_R,
            "paint_arm"     : paint_arm,
            "heading_error" : math.degrees(head_err),
            "lateral_error" : lat_err,
            "t"             : t,
            "status"        : "running",
        }


def _stopped_result() -> dict:
    return {
        "pwm_L"         : 0,
        "pwm_R"         : 0,
        "paint_arm"     : False,
        "heading_error" : 0.0,
        "lateral_error" : 0.0,
        "t"             : 1.0,
        "status"        : "finished",
    }
