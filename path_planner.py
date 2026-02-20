#Takes field spec + origin point + optional heading reference
# Generates waypoint list in meters (local ENU)
# Saves:
#   mission.json (metadata)
#   waypoints.csv or waypoints.json

#!/usr/bin/env python3
"""
Dry-Run Soccer Field Path Planner
Forward-only, overshoot + 270° arc + return to corner
"""

import math
from typing import List, Tuple

Waypoint = Tuple[float, float, bool]  # (x, y, paint_on)

def generate_soccer_perimeter_270(L: float, W: float, R: float, ds: float = 0.1) -> List[Waypoint]:
    waypoints: List[Waypoint] = []

    def add_straight(x0, y0, x1, y1, paint: bool):
        dx = x1 - x0
        dy = y1 - y0
        dist = math.hypot(dx, dy)
        steps = max(1, int(dist / ds))
        for i in range(steps):
            x = x0 + dx * (i+1)/steps
            y = y0 + dy * (i+1)/steps
            waypoints.append((x, y, paint))

    def add_270_arc(center_x, center_y, radius, start_angle_deg, ccw=True):
        # 270 deg sweep, paint OFF
        steps = max(1, int(radius * 3 * math.pi / 2 / ds))  # arc length / ds
        for i in range(steps):
            fraction = (i+1)/steps
            angle_rad = math.radians(start_angle_deg)
            if ccw:
                angle_rad += 3/2 * math.pi * fraction  # 270° CCW
            else:
                angle_rad -= 3/2 * math.pi * fraction
            x = center_x + radius * math.cos(angle_rad)
            y = center_y + radius * math.sin(angle_rad)
            waypoints.append((x, y, False))

    # --- Bottom Edge ---
    add_straight(-L, 0, 0, 0, True)       # paint bottom edge
    add_straight(0, 0, R, 0, False)       # overshoot R

    # --- 270° Arc around (R, -R) ---
    add_270_arc(R, -R, R, 90, ccw=True)   # start angle 90°, CCW sweep

    # --- Return to corner (paint OFF) ---
    add_straight(0, -R, 0, 0, False)      # back to corner

    # --- Left Edge ---
    add_straight(0, 0, 0, W, True)        # paint left edge

    return waypoints

# Example usage
if __name__ == "__main__":
    main()
