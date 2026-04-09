"""
dead_reckoning.py

PWM-velocity dead-reckoning odometry for the skid-steer rover.

Geometry (hard-coded from physical measurements):
    Wheel diameter : 8.375 in  → radius r = 0.10636 m
    Track width    : 27.25 in  → L        = 0.69215 m  (wheel centre to centre)
    Body length    : 24.0  in  → 0.6096 m  (reference only, not used in kinematics)

Math summary
────────────
1.  PWM  →  wheel linear speed
        duty  = PWM / 255
        ω     = duty × ω_max          # rad/s (calibrate ω_max — see below)
        v     = ω × r                 # m/s

2.  Differential-drive kinematics
        v   = (v_L + v_R) / 2         # body centre speed
        ω_b = (v_R − v_L) / L_eff     # yaw rate  (L_eff = slip_k × L)

3.  Pose integration  (x, y in metres; θ in radians, CCW from east)
        Δθ  = ω_b × dt
        if |ω_b| < ε  (straight):
            x += v · cos(θ) · dt
            y += v · sin(θ) · dt
        else (arc, exact):
            R  = v / ω_b
            x += R · (sin(θ + Δθ) − sin θ)
            y += R · (cos θ       − cos(θ + Δθ))
            θ += Δθ

Calibration procedure
─────────────────────
ω_max (run once per surface / battery charge):
    1. Place rover on the test surface.
    2. Send PWM 255 to both sides for exactly T seconds (e.g. T = 3 s).
    3. Measure the distance d travelled with a tape measure.
    4. v_measured  = d / T
    5. ω_max_calibrated = v_measured / WHEEL_RADIUS
    Store separately for left (OMG_MAX_L) and right (OMG_MAX_R) if the
    motors are mismatched.

SLIP_K (skid-steer track-width correction):
    Drive a square (4 × 90° turns) and check if the rover returns to origin.
    Tune SLIP_K until it does.  Typical range: 0.72 – 0.92.

PWM_DEADBAND:
    Lower the PWM from 255 until the wheel just stops.
    Set PWM_DEADBAND to that value + 5 for safety margin.

Usage
─────
    from dead_reckoning import DeadReckoning

    dr = DeadReckoning(initial_x=0.0, initial_y=0.0, initial_heading=0.0)

    # In your control loop:
    dr.update(pwm_L=180, pwm_R=180, dt=0.2)   # dt in seconds

    x, y, theta = dr.get_pose()
    vx, vy      = dr.get_velocity_vector()
    speed       = dr.get_speed()
"""

import math
import time


# ──────────────────────────────────────────────────────────────────────────────
# Physical constants  (DO NOT change without re-measuring the rover)
# ──────────────────────────────────────────────────────────────────────────────

WHEEL_DIAMETER_IN  = 8.375          # inches
WHEEL_DIAMETER_M   = WHEEL_DIAMETER_IN * 0.0254        # → 0.21273 m
WHEEL_RADIUS       = WHEEL_DIAMETER_M / 2.0             # → 0.10636 m
WHEEL_CIRCUMFERENCE = math.pi * WHEEL_DIAMETER_M       # → 0.66820 m

TRACK_WIDTH_IN     = 27.25          # inches, wheel centre to wheel centre
TRACK_WIDTH_M      = TRACK_WIDTH_IN * 0.0254            # → 0.69215 m

BODY_LENGTH_IN     = 24.0           # inches (widest length — front to back)
BODY_LENGTH_M      = BODY_LENGTH_IN * 0.0254            # → 0.60960 m


# ──────────────────────────────────────────────────────────────────────────────
# Calibration constants  (tune these experimentally — see docstring above)
# ──────────────────────────────────────────────────────────────────────────────

# Maximum wheel angular velocity at PWM = 255  (rad/s)
# Initial guess: assume ~1.2 m/s top speed → ω = 1.2 / 0.10636 ≈ 11.3 rad/s
# Replace with measured values after calibration.
OMG_MAX_L  = 11.3    # left side  (rad/s)
OMG_MAX_R  = 11.3    # right side (rad/s)

# Skid-steer effective track width correction factor.
# Accounts for tyre scrub / slip during turns.
# Start at 0.85, tune by driving squares.
SLIP_K     = 0.85
TRACK_EFF  = SLIP_K * TRACK_WIDTH_M   # effective track width used in ω_b calc

# PWM values below this threshold produce no wheel motion.
# Measure and set per your motors.
PWM_DEADBAND = 40    # [0, 255]  — set to 0 to disable

# Yaw-rate epsilon — below this magnitude we treat motion as straight-line
# (avoids divide-by-zero and floating-point arc errors at very small ω)
OMEGA_EPS = 1e-6    # rad/s


# ──────────────────────────────────────────────────────────────────────────────
# Core class
# ──────────────────────────────────────────────────────────────────────────────

class DeadReckoning:
    """
    Maintains the rover's (x, y, θ) pose estimate using PWM-based odometry.

    Coordinate frame
    ────────────────
    x      : metres east   (matches UTM easting if you seed with a GPS fix)
    y      : metres north  (matches UTM northing)
    theta  : radians, measured counter-clockwise from the +x (east) axis.
             θ = 0   → facing east
             θ = π/2 → facing north
             θ = π   → facing west
    """

    def __init__(self,
                 initial_x: float = 0.0,
                 initial_y: float = 0.0,
                 initial_heading: float = 0.0):
        """
        Parameters
        ----------
        initial_x, initial_y : float
            Starting position in metres.  Set to UTM easting/northing if you
            want dead-reckoning coordinates to stay in the same frame as GPS.
        initial_heading : float
            Starting heading in radians (CCW from east).
            If rover starts pointing north, pass math.pi / 2.
        """
        self.x      = float(initial_x)
        self.y      = float(initial_y)
        self.theta  = float(initial_heading)

        self._v     = 0.0    # body centre speed   (m/s)
        self._omega = 0.0    # yaw rate            (rad/s)
        self._v_L   = 0.0    # left wheel speed    (m/s)
        self._v_R   = 0.0    # right wheel speed   (m/s)

        self._dist_total = 0.0   # odometer (metres)

    # ──────────────────────────────────────────────────────────────────────
    # PWM → wheel speed conversion
    # ──────────────────────────────────────────────────────────────────────

    @staticmethod
    def _pwm_to_speed(pwm: int, omega_max: float) -> float:
        """
        Convert a PWM magnitude to wheel linear speed in m/s.

        Parameters
        ----------
        pwm       : int   [0, 255]  — always a positive magnitude.
                    Negative PWM (reverse) must be handled BEFORE calling this
                    (see update() — pass negative pwm for reverse motion).
        omega_max : float — calibrated max angular velocity for this side (rad/s).

        Returns
        -------
        float : signed linear speed in m/s.
                Positive = forward, negative = reverse.
        """
        # Apply deadband — below threshold the wheel is stationary
        if abs(pwm) < PWM_DEADBAND:
            return 0.0

        duty  = pwm / 255.0                    # normalised duty cycle [-1, 1]
        omega = duty * omega_max               # angular velocity (rad/s)
        return omega * WHEEL_RADIUS            # linear speed (m/s)

    # ──────────────────────────────────────────────────────────────────────
    # Main update — call every control tick
    # ──────────────────────────────────────────────────────────────────────

    def update(self, pwm_L: int, pwm_R: int, dt: float) -> None:
        """
        Integrate one timestep of dead-reckoning.

        Parameters
        ----------
        pwm_L : int  [-255, 255]
            Left-side PWM.  Positive = forward, negative = reverse.
            (path_following.py only outputs positive magnitudes; negate
            here if your Arduino uses separate direction pins and always
            sends positive PWM to this module.)
        pwm_R : int  [-255, 255]
            Right-side PWM.  Same sign convention as pwm_L.
        dt    : float
            Elapsed time since last call (seconds).  Use your loop timer
            (e.g. LOOP_DT from main.py, or measure wall-clock directly).

        Notes
        -----
        Clamps PWM to [-255, 255] before conversion.
        dt must be > 0; negative or zero dt is silently ignored.
        """
        if dt <= 0:
            return

        pwm_L = max(-255, min(255, int(pwm_L)))
        pwm_R = max(-255, min(255, int(pwm_R)))

        # 1. PWM → wheel speeds
        v_L = self._pwm_to_speed(pwm_L, OMG_MAX_L)
        v_R = self._pwm_to_speed(pwm_R, OMG_MAX_R)

        self._v_L = v_L
        self._v_R = v_R

        # 2. Differential-drive kinematics
        v     = (v_L + v_R) / 2.0             # body centre speed
        omega = (v_R - v_L) / TRACK_EFF       # yaw rate

        self._v     = v
        self._omega = omega

        # 3. Pose integration
        delta_theta = omega * dt

        if abs(omega) < OMEGA_EPS:
            # Straight-line approximation (avoids numerical issues at ω → 0)
            self.x     += v * math.cos(self.theta) * dt
            self.y     += v * math.sin(self.theta) * dt
            self.theta += delta_theta            # essentially 0, but keep consistent
        else:
            # Exact arc integration (Runge-Kutta / ICC method)
            R = v / omega                        # turning radius to ICC

            self.x     += R * (math.sin(self.theta + delta_theta) - math.sin(self.theta))
            self.y     += R * (math.cos(self.theta) - math.cos(self.theta + delta_theta))
            self.theta += delta_theta

        # Normalise heading to (-π, π]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Odometer
        self._dist_total += abs(v) * dt

    # ──────────────────────────────────────────────────────────────────────
    # Seeding — snap DR to a known GPS fix
    # ──────────────────────────────────────────────────────────────────────

    def seed(self, x: float, y: float, heading: float = None) -> None:
        """
        Reset (or correct) the pose estimate from an external source.

        Call this whenever a reliable GPS fix is available.  The dead-
        reckoning then continues from the corrected position.

        Parameters
        ----------
        x, y    : float  New position in metres (e.g. UTM easting/northing).
        heading : float, optional  New heading in radians.  If None, the
                  current heading estimate is kept.
        """
        self.x = float(x)
        self.y = float(y)
        if heading is not None:
            self.theta = float(heading)

    # ──────────────────────────────────────────────────────────────────────
    # Accessors
    # ──────────────────────────────────────────────────────────────────────

    def get_pose(self) -> tuple:
        """Return (x, y, theta) — position in metres, heading in radians."""
        return (self.x, self.y, self.theta)

    def get_position(self) -> tuple:
        """Return (x, y) — compatible with GNSSReader.get_position()."""
        return (self.x, self.y)

    def get_heading_deg(self) -> float:
        """Return heading in degrees, CCW from east."""
        return math.degrees(self.theta)

    def get_speed(self) -> float:
        """Return current body-centre speed in m/s."""
        return self._v

    def get_yaw_rate(self) -> float:
        """Return current yaw rate in rad/s (positive = turning left / CCW)."""
        return self._omega

    def get_velocity_vector(self) -> tuple:
        """Return (vx, vy) velocity components in m/s."""
        return (self._v * math.cos(self.theta),
                self._v * math.sin(self.theta))

    def get_wheel_speeds(self) -> tuple:
        """Return (v_L, v_R) individual wheel speeds in m/s."""
        return (self._v_L, self._v_R)

    def get_odometer(self) -> float:
        """Return total distance travelled in metres since construction."""
        return self._dist_total

    def reset_odometer(self) -> None:
        """Zero the odometer (does not affect pose)."""
        self._dist_total = 0.0

    def status_str(self) -> str:
        """One-line status for console logging."""
        return (f"x={self.x:+8.3f}m  y={self.y:+8.3f}m  "
                f"θ={math.degrees(self.theta):+6.1f}°  "
                f"v={self._v:+5.2f}m/s  ω={self._omega:+5.2f}rad/s  "
                f"odo={self._dist_total:.2f}m")


# ──────────────────────────────────────────────────────────────────────────────
# Calibration helpers
# ──────────────────────────────────────────────────────────────────────────────

def calibrate_omega_max(distance_m: float,
                        duration_s: float,
                        side: str = "both") -> float:
    """
    Compute ω_max from a straight-line calibration run.

    Parameters
    ----------
    distance_m : float  Measured distance travelled (tape measure, metres).
    duration_s : float  Duration of the run at PWM=255 (seconds).
    side       : str    "left", "right", or "both" — for logging only.

    Returns
    -------
    float : ω_max in rad/s.  Assign to OMG_MAX_L and/or OMG_MAX_R.

    Example
    -------
    Rover drove 2.85 m in 3 s at full throttle:
        om = calibrate_omega_max(2.85, 3.0)
        # → 8.93 rad/s
        # Set OMG_MAX_L = OMG_MAX_R = 8.93
    """
    v_meas = distance_m / duration_s
    omega  = v_meas / WHEEL_RADIUS
    print(f"[Calibration] {side}: v={v_meas:.4f} m/s  →  ω_max={omega:.4f} rad/s")
    print(f"  Set OMG_MAX_{'L = OMG_MAX_R' if side == 'both' else side.upper()} = {omega:.3f}")
    return omega


# ──────────────────────────────────────────────────────────────────────────────
# Integration with path_following.py
# ──────────────────────────────────────────────────────────────────────────────
#
# In main.py, replace the plain position tracker with:
#
#   from dead_reckoning import DeadReckoning
#
#   # Seed from first GPS fix so DR coordinates stay in UTM frame
#   dr = DeadReckoning(*gnss.get_position(), initial_heading=0.0)
#
#   prev_P = gnss.get_position()
#   loop_start = time.time()
#
#   while not follower.is_finished():
#       dt = time.time() - loop_start
#       loop_start = time.time()
#
#       # --- choose position source ---
#       if gnss.has_fix() and gnss.get_fix_info()["hdop"] < 3.0:
#           P = gnss.get_position()
#           dr.seed(*P)              # snap DR to GPS when fix is good
#       else:
#           dr.update(result["pwm_L"], result["pwm_R"], dt)
#           P = dr.get_position()   # fall back to dead-reckoning
#
#       speed  = dr.get_speed() or gnss.get_speed()
#       result = follower.step(P, prev_P, speed=speed)
#       prev_P = P
#
#       send_motor_commands(ser, result["pwm_L"], result["pwm_R"], result["paint_arm"])
#       print(dr.status_str())
#
# ──────────────────────────────────────────────────────────────────────────────


# ──────────────────────────────────────────────────────────────────────────────
# Standalone demo — simulates a square path
# ──────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    import time as _time

    print("Dead-reckoning demo — simulated square (5 m sides)")
    print(f"  Wheel radius  : {WHEEL_RADIUS:.5f} m")
    print(f"  Track width   : {TRACK_WIDTH_M:.5f} m  (eff: {TRACK_EFF:.5f} m)")
    print(f"  ω_max         : {OMG_MAX_L:.2f} rad/s")
    print(f"  PWM deadband  : {PWM_DEADBAND}\n")

    dr = DeadReckoning(0.0, 0.0, 0.0)
    DT = 0.05   # 20 Hz sim

    def drive_straight(pwm, seconds):
        steps = int(seconds / DT)
        for _ in range(steps):
            dr.update(pwm, pwm, DT)

    def turn_left_90():
        """
        In-place left turn: left wheels back, right wheels forward.
        Duration depends on your rover — tune to achieve ~90°.
        The formula for a pure spin turn:
            arc = (π/2) × (L/2)
            time = arc / v_wheel
        where v_wheel = (PWM/255) × ω_max × r
        """
        pwm_turn = 150
        v_w = (pwm_turn / 255.0) * OMG_MAX_R * WHEEL_RADIUS
        arc = (math.pi / 2) * (TRACK_EFF / 2)
        t_turn = arc / v_w
        steps = int(t_turn / DT)
        for _ in range(steps):
            dr.update(-pwm_turn, pwm_turn, DT)  # spin left

    STRAIGHT_PWM = 200
    STRAIGHT_T   = 5.0 / ((STRAIGHT_PWM / 255.0) * OMG_MAX_L * WHEEL_RADIUS)

    for leg in range(4):
        print(f"Leg {leg+1}: straight")
        drive_straight(STRAIGHT_PWM, STRAIGHT_T)
        print(f"  {dr.status_str()}")
        print(f"Leg {leg+1}: turn left 90°")
        turn_left_90()
        print(f"  {dr.status_str()}")

    x, y, th = dr.get_pose()
    print(f"\nFinal pose: x={x:+.3f} m  y={y:+.3f} m  θ={math.degrees(th):+.1f}°")
    print(f"Close-loop error: {math.sqrt(x**2+y**2):.3f} m from origin")
    print(f"Total odometer  : {dr.get_odometer():.2f} m")
