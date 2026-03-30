"""
path_following.py

Stanley-style path following controller for a skid-steer rover.
Inputs:
    - Current position (easting, northing) from GNSS
    - Current speed (m/s) derived from consecutive GPS positions
    - Waypoint list from path_planner
Outputs (via serial to Arduino):
    - L1, L2 : front-left / rear-left motor PWM  (0–255)
    - R1, R2 : front-right / rear-right motor PWM (0–255)
    - ARM    : servo paint arm (0 = off, 1 = on)
    Serial format: "L1:<v>,L2:<v>,R1:<v>,R2:<v>,ARM:<v>\\n"

Motor direction is handled by the Arduino high/low direction pins.
Left-side and right-side motors always receive the same PWM value.
"""

import math


# ---------------------------------------------------------------------------
# Tuning constants — adjust these for your rover
# ---------------------------------------------------------------------------
K1 = 1.0          # Heading error gain
K2 = 0.5          # Lateral error gain
WHEELBASE = 0.6   # Distance between left and right wheels (metres)
BASE_SPEED = 0.5  # Nominal forward speed (m/s)
LOOK_AHEAD = 2.0  # Look-ahead offset along path (metres)
WP_ACCEPT_RADIUS = 1.0  # Distance (m) at which we consider a waypoint reached


# ---------------------------------------------------------------------------
# Serial output — adapt port/baud to match your motor controller
# ---------------------------------------------------------------------------
import serial

SERIAL_PORT = 'COM15'
BAUD_RATE = 9600

def open_serial():
    return serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)


def send_motor_commands(ser, pwm_L: int, pwm_R: int, arm: bool):
    """
    Send PWM commands to Arduino over serial.

    Format: "L1:<v>,L2:<v>,R1:<v>,R2:<v>,ARM:<v>\\n"
      - L1/L2 : front-left and rear-left motors  (0–255)
      - R1/R2 : front-right and rear-right motors (0–255)
      - ARM   : servo paint arm (0 or 1)

    Both left motors receive the same value; same for right.
    Motor direction (forward/reverse) is set by the Arduino
    direction pins — PWM here is always a positive magnitude.

    Parameters
    ----------
    pwm_L : int  Left-side PWM magnitude [0, 255]
    pwm_R : int  Right-side PWM magnitude [0, 255]
    arm   : bool True = paint arm down (servo ON)
    """
    pwm_L = int(max(0, min(255, pwm_L)))
    pwm_R = int(max(0, min(255, pwm_R)))
    arm_val = 1 if arm else 0
    cmd = f"L1:{pwm_L},L2:{pwm_L},R1:{pwm_R},R2:{pwm_R},ARM:{arm_val}\n"
    ser.write(cmd.encode())


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def vec2d(a, b):
    """Return vector from point a to point b."""
    return (b[0] - a[0], b[1] - a[1])


def dot2d(u, v):
    return u[0] * v[0] + u[1] * v[1]


def norm2d(v):
    return math.sqrt(v[0] ** 2 + v[1] ** 2)


def cross2d(u, v):
    """2D cross product (scalar z-component). Positive = v is to the left of u."""
    return u[0] * v[1] - u[1] * v[0]


def heading_from_points(p1, p2):
    """
    Compute heading angle (radians, measured CCW from East) from two
    consecutive (easting, northing) positions.
    """
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    return math.atan2(dy, dx)


def normalize_angle(angle):
    """Wrap angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


# ---------------------------------------------------------------------------
# Speed → PWM conversion
# ---------------------------------------------------------------------------

MAX_SPEED_MS = 1.5   # m/s that maps to PWM 255 — calibrate to your rover

def speed_to_pwm(speed_ms: float) -> int:
    """
    Linear mapping from m/s to Arduino PWM [0, 255].
    Negative speeds are clamped to 0 — direction is handled by the
    Arduino high/low direction pins, not the PWM magnitude.
    """
    speed_ms = max(0.0, speed_ms)          # clamp — no negative PWM
    pwm = (speed_ms / MAX_SPEED_MS) * 255.0
    return int(min(255, pwm))


# ---------------------------------------------------------------------------
# Core controller
# ---------------------------------------------------------------------------

class PathFollower:
    def __init__(self, waypoints: list):
        """
        Parameters
        ----------
        waypoints : list of (easting, northing) tuples
            Ordered list of waypoints from path_planner.
        """
        if len(waypoints) < 2:
            raise ValueError("Need at least 2 waypoints.")
        self.waypoints = waypoints
        self.wp_index = 0           # Index of current target waypoint (wp_b)
        self.prev_position = None   # Used to derive heading from GPS
        self.prev_IC = None         # Sign of cross product at previous step

    @property
    def wp_a(self):
        return self.waypoints[self.wp_index]

    @property
    def wp_b(self):
        return self.waypoints[self.wp_index + 1]

    def is_finished(self):
        """True when we have no more waypoint pairs to follow."""
        return self.wp_index >= len(self.waypoints) - 1

    # ------------------------------------------------------------------
    # 2.1  Projected distance along path
    # ------------------------------------------------------------------
    def compute_t(self, P):
        """
        t = scalar projection of (P - P1) onto the path vector (P2 - P1),
        normalised so t=0 at wp_a and t=1 at wp_b.
        Returns t (float).
        """
        a_to_b = vec2d(self.wp_a, self.wp_b)
        a_to_p = vec2d(self.wp_a, P)
        denom = dot2d(a_to_b, a_to_b)
        if denom < 1e-9:
            return 0.0
        return dot2d(a_to_p, a_to_b) / denom

    # ------------------------------------------------------------------
    # 2.2 / 2.3  Heading & lateral errors
    # ------------------------------------------------------------------
    def compute_errors(self, P, actual_heading):
        """
        Parameters
        ----------
        P               : (easting, northing) current position
        actual_heading  : float, radians — heading derived from GPS history

        Returns
        -------
        head_angle_error : float, radians
        lateral_error    : float, metres (signed — positive = rover is left of path)
        IC               : float, cross product used for waypoint switching
        """
        a_to_b = vec2d(self.wp_a, self.wp_b)

        # Intended heading from the current segment
        intend_head = math.atan2(a_to_b[1], a_to_b[0])

        # Heading error — difference between intended and actual
        head_angle_error = normalize_angle(intend_head - actual_heading)

        # Lateral error — signed perpendicular distance from path
        a_to_p = vec2d(self.wp_a, P)
        path_len = norm2d(a_to_b)
        if path_len < 1e-9:
            lateral_error = 0.0
            IC = 0.0
        else:
            # Positive when rover is to the left of the path direction
            lateral_error = cross2d(a_to_b, a_to_p) / path_len

            # Rover's heading as a unit vector
            rover_vec = (math.cos(actual_heading), math.sin(actual_heading))
            IC = cross2d(a_to_b, rover_vec)

        return head_angle_error, lateral_error, IC

    # ------------------------------------------------------------------
    # Steering — Stanley / guiding angle
    # ------------------------------------------------------------------
    def compute_steering(self, head_angle_error, lateral_error, speed):
        """
        Stanley controller steering angle (delta_t) and guiding angle.

        delta_t     = K1 * head_angle_error + arctan(K2 * lateral_error / v)
        guiding_angle = head_angle_error - arctan(lateral_error / look_ahead)
        """
        speed = max(speed, 0.1)   # avoid divide-by-zero at standstill

        delta_t = (K1 * head_angle_error +
                   math.atan2(K2 * lateral_error, speed))

        guiding_angle = (head_angle_error -
                         math.atan2(lateral_error, LOOK_AHEAD))

        return delta_t, guiding_angle

    # ------------------------------------------------------------------
    # Skid-steer velocity split
    # ------------------------------------------------------------------
    def compute_wheel_speeds(self, delta_t):
        """
        Convert steering correction angle to left/right PWM magnitudes.

        curvature = tan(delta_t) / WHEELBASE
        v_L = BASE_SPEED * (1 - (WHEELBASE/2) * curvature)
        v_R = BASE_SPEED * (1 + (WHEELBASE/2) * curvature)

        Speeds are kept as positive magnitudes [0, 255].
        The Arduino handles forward/reverse via its direction pins.

        Returns (pwm_L, pwm_R) as integers in [0, 255].
        """
        curvature = math.tan(delta_t) / WHEELBASE
        v_L_ms = BASE_SPEED * (1.0 - (WHEELBASE / 2.0) * curvature)
        v_R_ms = BASE_SPEED * (1.0 + (WHEELBASE / 2.0) * curvature)
        return speed_to_pwm(v_L_ms), speed_to_pwm(v_R_ms)

    # ------------------------------------------------------------------
    # Waypoint advancement
    # ------------------------------------------------------------------
    def check_and_advance_waypoint(self, IC):
        """
        When IC flips sign, the rover has passed wp_b.
        Advance wp_index so the next segment becomes active.
        Returns True if waypoints were advanced.
        """
        if self.prev_IC is not None and self.prev_IC * IC < 0:
            # Sign flip detected — rover crossed the wp_b threshold
            if self.wp_index < len(self.waypoints) - 2:
                self.wp_index += 1
                print(f"[PathFollower] Advanced to segment "
                      f"{self.wp_index} → {self.wp_index + 1}")
                return True
        return False

    # ------------------------------------------------------------------
    # Paint arm logic
    # ------------------------------------------------------------------
    def paint_arm_active(self, t):
        """
        Paint arm is ON while the rover is between wp_a and wp_b
        (0 <= t <= 1). Extend this with more nuanced logic as needed.
        """
        return 0.0 <= t <= 1.0

    # ------------------------------------------------------------------
    # Main step — call this at every control loop iteration
    # ------------------------------------------------------------------
    def step(self, P, prev_P, speed=None):
        """
        Run one control cycle.

        Parameters
        ----------
        P      : (easting, northing) — current GPS position
        prev_P : (easting, northing) — previous GPS position (for heading)
        speed  : float, optional — rover speed in m/s from gnss_reader.
                 If None, speed is estimated from consecutive GPS positions.

        Returns
        -------
        dict with keys: pwm_L, pwm_R, paint_arm, t, head_angle_error,
                        lateral_error, delta_t, guiding_angle
        """
        if self.is_finished():
            return {"pwm_L": 0, "pwm_R": 0, "paint_arm": False,
                    "status": "finished"}

        # Derive heading from consecutive GPS positions
        actual_heading = heading_from_points(prev_P, P)

        # Use gnss_reader speed if provided, otherwise estimate from dist_step
        if speed is None:
            speed = norm2d(vec2d(prev_P, P))  # rough fallback

        # Projected position along segment
        t = self.compute_t(P)

        # Errors
        head_angle_error, lateral_error, IC = self.compute_errors(
            P, actual_heading)

        # Waypoint switching
        self.check_and_advance_waypoint(IC)
        self.prev_IC = IC

        # Steering angle
        delta_t, guiding_angle = self.compute_steering(
            head_angle_error, lateral_error, speed)

        # Wheel speeds
        pwm_L, pwm_R = self.compute_wheel_speeds(delta_t)

        # Paint arm
        paint = self.paint_arm_active(t)

        return {
            "pwm_L": pwm_L,
            "pwm_R": pwm_R,
            "paint_arm": paint,
            "t": t,
            "head_angle_error": math.degrees(head_angle_error),
            "lateral_error": lateral_error,
            "delta_t": math.degrees(delta_t),
            "guiding_angle": math.degrees(guiding_angle),
            "status": "running",
        }


# ---------------------------------------------------------------------------
# Example control loop
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    # Placeholder waypoints — replace with output from path_planner
    waypoints = [
        (500.0, 1000.0),
        (520.0, 1000.0),
    ]

    follower = PathFollower(waypoints)
    ser = open_serial()

    # Seed with a starting position (first GPS fix)
    prev_P = waypoints[0]

    try:
        while not follower.is_finished():
            # ---- Replace this block with your gnss_reader call ----
            P = (501.0, 1000.3)   # example: current GPS fix
            # -------------------------------------------------------

            result = follower.step(P, prev_P)
            prev_P = P

            send_motor_commands(ser,
                                result["pwm_L"],
                                result["pwm_R"],
                                result["paint_arm"])

            print(f"PWM L={result['pwm_L']:3d}  R={result['pwm_R']:3d} | "
                  f"ARM={'ON ' if result['paint_arm'] else 'OFF'} | "
                  f"TE={result['lateral_error']:+.3f}m  "
                  f"HE={result['head_angle_error']:+.1f}°")

    except KeyboardInterrupt:
        send_motor_commands(ser, 0, 0, False)
        print("Stopped.")
    finally:
        ser.close()
