"""
path_following.py

Simplified straight-line path follower for a skid-steer rover.
All tuning constants and serial config come from config.yaml.

Arduino serial format (unchanged):
    "L1:<v>,L2:<v>,R1:<v>,R2:<v>,ARM:<v>\\n"
"""

import math
import serial

from config_loader import CFG

# ---------------------------------------------------------------------------
# Serial — from config
# ---------------------------------------------------------------------------
SERIAL_PORT = CFG["serial"]["arduino_port"]
BAUD_RATE   = CFG["serial"]["arduino_baud"]


def open_serial() -> serial.Serial:
    return serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)


def send_motor_commands(ser, pwm_L: int, pwm_R: int, arm: bool):
    """Send PWM + arm command to Arduino."""
    pwm_L   = int(max(0, min(255, pwm_L)))
    pwm_R   = int(max(0, min(255, pwm_R)))
    arm_val = 1 if arm else 0
    cmd = f"L1:{pwm_L},L2:{pwm_L},R1:{pwm_R},R2:{pwm_R},ARM:{arm_val}\n"
    ser.write(cmd.encode())


# ---------------------------------------------------------------------------
# Tuning — from config
# ---------------------------------------------------------------------------
K_HEADING             = CFG["follower"]["k_heading"]
K_LATERAL             = CFG["follower"]["k_lateral"]
BASE_SPEED_PWM        = CFG["follower"]["base_speed_pwm"]
MAX_STEER_PWM         = CFG["follower"]["max_steer_pwm"]
WHEELBASE             = CFG["follower"]["wheelbase_m"]
HEADING_ARM_THRESHOLD = CFG["follower"]["heading_arm_threshold"]   # degrees
WP_ACCEPT_RADIUS      = CFG["control"]["wp_accept_radius"]


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _normalize_angle(angle: float) -> float:
    while angle >  math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def _heading_from_points(p1: tuple, p2: tuple) -> float:
    return math.atan2(p2[1] - p1[1], p2[0] - p1[0])


def _distance(p1: tuple, p2: tuple) -> float:
    return math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)


def _lateral_error(wp1: tuple, wp2: tuple, P: tuple) -> float:
    """
    Signed perpendicular distance from P to line wp1→wp2.
    Positive = rover is to the LEFT of the path direction.
    """
    dx = wp2[0] - wp1[0]
    dy = wp2[1] - wp1[1]
    path_len = math.sqrt(dx**2 + dy**2)
    if path_len < 1e-9:
        return 0.0
    return ((dx * (P[1] - wp1[1])) - (dy * (P[0] - wp1[0]))) / path_len


# ---------------------------------------------------------------------------
# PathFollower
# ---------------------------------------------------------------------------

class PathFollower:
    def __init__(self, wp1: tuple, wp2: tuple, heading_rad: float):
        self.wp1         = wp1
        self.wp2         = wp2
        self.heading_rad = heading_rad
        self._finished   = False

    def is_finished(self) -> bool:
        return self._finished

    def _progress_t(self, P: tuple) -> float:
        dx = self.wp2[0] - self.wp1[0]
        dy = self.wp2[1] - self.wp1[1]
        denom = dx**2 + dy**2
        if denom < 1e-9:
            return 0.0
        return ((P[0] - self.wp1[0]) * dx + (P[1] - self.wp1[1]) * dy) / denom

    def step(self, P: tuple, prev_P: tuple, speed_ms: float = 0.5) -> dict:
        if self._finished:
            return _stopped_result()

        # Arrival check
        if _distance(P, self.wp2) <= WP_ACCEPT_RADIUS:
            self._finished = True
            return _stopped_result()

        # Heading from GPS motion
        actual_heading = _heading_from_points(prev_P, P)

        # Errors
        head_err = _normalize_angle(self.heading_rad - actual_heading)
        lat_err  = _lateral_error(self.wp1, self.wp2, P)

        # Steering correction
        steer = K_HEADING * head_err + K_LATERAL * lat_err

        # Differential PWM
        steer_pwm = int(max(-MAX_STEER_PWM,
                            min(MAX_STEER_PWM, steer * (180 / math.pi))))
        pwm_L = int(max(0, min(255, BASE_SPEED_PWM - steer_pwm)))
        pwm_R = int(max(0, min(255, BASE_SPEED_PWM + steer_pwm)))

        # Paint arm — only when on-heading
        paint_arm = abs(math.degrees(head_err)) <= HEADING_ARM_THRESHOLD

        t = self._progress_t(P)

        return {
            "pwm_L"        : pwm_L,
            "pwm_R"        : pwm_R,
            "paint_arm"    : paint_arm,
            "heading_error": math.degrees(head_err),
            "lateral_error": lat_err,
            "t"            : t,
            "status"       : "running",
        }


def _stopped_result() -> dict:
    return {
        "pwm_L"        : 0,
        "pwm_R"        : 0,
        "paint_arm"    : False,
        "heading_error": 0.0,
        "lateral_error": 0.0,
        "t"            : 1.0,
        "status"       : "finished",
    }
