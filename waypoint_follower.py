# Consumes current position in meters (x, y)
# Consumes a waypoint list [(x1,y1), (x2,y2), ...]
# Computes:
#   distance error
#   heading error (needs heading source: IMU or course-over-ground when moving)
# Outputs a velocity command:
#   v (forward m/s)
#   omega (turn rad/s)  
#   OR directly (left_wheel_mps, right_wheel_mps)
#!/usr/bin/env python3
"""
Waypoint Follower using actual motor interface
Consumes waypoints from path planner and drives real motors
"""

import math
import time
from typing import List, Tuple
from motor_interface import MotorControl  # <-- your real motor class

Waypoint = Tuple[float, float, bool]  # (x, y, paint_on)

class WaypointFollower:
    def __init__(self, motors: MotorControl, tolerance: float = 0.1, lookahead: float = 0.5):
        """
        :param motors: instance of MotorControl
        :param tolerance: distance to waypoint to consider it reached
        :param lookahead: pure pursuit lookahead distance (m)
        """
        self.motors = motors
        self.tolerance = tolerance
        self.lookahead = lookahead
        self.waypoints: List[Waypoint] = []
        self.wp_idx = 0
        self.running = False

    def load_waypoints(self, waypoints: List[Waypoint]):
        """Load a list of waypoints (x, y, paint_on)"""
        self.waypoints = waypoints
        self.wp_idx = 0
        print(f"Loaded {len(waypoints)} waypoints")

    def start(self, get_pos_func):
        """
        Start following waypoints.
        get_pos_func() -> (x, y) returns current rover position in same frame as waypoints
        """
        self.running = True
        while self.running and self.wp_idx < len(self.waypoints):
            pos_x, pos_y = get_pos_func()
            self._follow_next(pos_x, pos_y)
            time.sleep(0.05)  # 20Hz update

    def _follow_next(self, pos_x: float, pos_y: float):
        if self.wp_idx >= len(self.waypoints):
            print("🎉 All waypoints reached!")
            self.motors.set_velocity(0, 0)
            self.motors.set_spray(False)
            self.running = False
            return

        wp_x, wp_y, paint = self.waypoints[self.wp_idx]

        dx = wp_x - pos_x
        dy = wp_y - pos_y
        dist = math.hypot(dx, dy)

        # Waypoint reached
        if dist < self.tolerance:
            self.wp_idx += 1
            return

        # Simple pure pursuit
        heading = math.atan2(dy, dx)
        v_fwd = min(0.8, dist)  # scale speed by distance

        # Differential drive: simple proportional turn
        wheelbase = 0.4  # meters
        k_heading = 1.25
        vL = v_fwd - k_heading * heading * wheelbase
        vR = v_fwd + k_heading * heading * wheelbase

        # Send commands to actual motors
        self.motors.set_velocity(vL, vR)
        self.motors.set_spray(paint)

import math
import time
from motor_interface import SerialMotorInterface

def wrap_pi(a):
    # wrap angle to [-pi, pi]
    while a > math.pi:
        a -= 2*math.pi
    while a < -math.pi:
        a += 2*math.pi
    return a

class WaypointFollower:
    def __init__(self, motors: SerialMotorInterface,
                 v_max_mps=0.5,
                 k_heading=1.5):
        self.motors = motors
        self.v_max = v_max_mps
        self.k_heading = k_heading

    def step(self, x, y, heading_rad, goal_x, goal_y):
        dx = goal_x - x
        dy = goal_y - y
        dist = math.hypot(dx, dy)

        desired_heading = math.atan2(dy, dx)
        heading_err = wrap_pi(desired_heading - heading_rad)

        # Simple controller (replace later):
        # - drive slower when heading error is large
        v = self.v_max * max(0.0, 1.0 - abs(heading_err)/math.pi)
        w = self.k_heading * heading_err  # rad/s-ish (conceptually)

        # Convert to normalized integers for Arduino interface
        # Decide your own scaling; this is a sane start:
        v_cmd = int(max(-1.0, min(1.0, v / self.v_max)) * 1000)
        w_cmd = int(max(-1.0, min(1.0, w / 2.0)) * 1000)  # assumes ~2 rad/s max desired

        self.motors.send_vw(v_cmd, w_cmd)

        return dist, heading_err, v_cmd, w_cmd

if __name__ == "__main__":
    motors = SerialMotorInterface(port="/dev/ttyACM0", baud=115200)
    wf = WaypointFollower(motors)

    # fake “pose” for quick test
    x, y, hdg = 0.0, 0.0, 0.0
    gx, gy = 5.0, 0.0

    try:
        while True:
            dist, herr, v_cmd, w_cmd = wf.step(x, y, hdg, gx, gy)
            print(f"dist={dist:.2f} herr={herr:.2f} v_cmd={v_cmd} w_cmd={w_cmd}")
            time.sleep(0.05)  # 20 Hz
    except KeyboardInterrupt:
        motors.stop()
