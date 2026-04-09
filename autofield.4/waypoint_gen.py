"""
waypoint_gen.py

Two-fix startup sequence:
  1. Average several GPS fixes → wp1 (start position)
  2. User nudges rover forward, presses ENTER
  3. Average several GPS fixes → fix2 (heading reference only)
  4. Compute unit heading vector wp1 → fix2
  5. User enters run length → wp2 = wp1 + length * heading_vector

All sample counts and thresholds come from config.yaml.

Usage (called by main.py):
    from waypoint_gen import run_waypoint_setup
    wp1, wp2, heading_rad = run_waypoint_setup(gnss)
"""

import math
import time

from config_loader import CFG

NUM_SAMPLES      = CFG["gps"]["num_samples"]
MIN_NUDGE_DIST   = CFG["gps"]["min_nudge_dist"]


# ---------------------------------------------------------------------------
# Internal helper
# ---------------------------------------------------------------------------

def _average_fixes(gnss, num_samples: int, label: str) -> tuple:
    """Collect num_samples fresh GPS readings and return averaged (e, n)."""
    eastings  = []
    northings = []
    last_pos  = None

    print(f"\n[WaypointGen] Collecting {num_samples} samples for {label} ...")

    while len(eastings) < num_samples:
        pos = gnss.get_position()
        if pos != last_pos:
            eastings.append(pos[0])
            northings.append(pos[1])
            last_pos = pos
            print(f"  Sample {len(eastings)}/{num_samples} — "
                  f"E={pos[0]:.3f}  N={pos[1]:.3f}")
        time.sleep(0.1)

    avg_e = sum(eastings)  / len(eastings)
    avg_n = sum(northings) / len(northings)
    print(f"[WaypointGen] {label}: E={avg_e:.4f}  N={avg_n:.4f}")
    return avg_e, avg_n


# ---------------------------------------------------------------------------
# Public entry point
# ---------------------------------------------------------------------------

def run_waypoint_setup(gnss) -> tuple:
    """
    Full two-fix startup sequence.

    Returns
    -------
    wp1         : (easting, northing) — start position
    wp2         : (easting, northing) — target position
    heading_rad : float — heading in radians (CCW from east, UTM convention)
    """
    print("\n" + "=" * 55)
    print("  Waypoint Setup")
    print("=" * 55)

    # Fix 1
    print("\nStep 1: Place the rover at the START position and hold still.")
    input("        Press ENTER to record Fix 1 ...\n")
    wp1 = _average_fixes(gnss, NUM_SAMPLES, "Fix 1 (start)")

    # Fix 2 — nudge
    print("\nStep 2: Nudge the rover forward in your intended run direction.")
    input("        Press ENTER when nudge is complete to record Fix 2 ...\n")
    fix2 = _average_fixes(gnss, NUM_SAMPLES, "Fix 2 (heading reference)")

    # Heading vector
    dx = fix2[0] - wp1[0]
    dy = fix2[1] - wp1[1]
    nudge_dist = math.sqrt(dx**2 + dy**2)

    if nudge_dist < MIN_NUDGE_DIST:
        raise RuntimeError(
            f"Nudge distance too small ({nudge_dist:.3f} m). "
            f"Need at least {MIN_NUDGE_DIST} m. "
            "Move the rover further before pressing ENTER."
        )

    heading_rad = math.atan2(dy, dx)
    ux = dx / nudge_dist
    uy = dy / nudge_dist

    print(f"\n[WaypointGen] Heading: {math.degrees(heading_rad):.1f}° "
          f"(nudge dist: {nudge_dist:.3f} m)")

    # Run length → wp2
    print("\nStep 3: Enter the desired run length.")
    while True:
        try:
            run_length = float(input("  Run length in metres (e.g. 20): ").strip())
            if run_length <= 0:
                print("  Must be greater than 0. Try again.")
                continue
            break
        except ValueError:
            print("  Invalid input — enter a number.")

    wp2 = (wp1[0] + run_length * ux, wp1[1] + run_length * uy)

    print(f"\n[WaypointGen] wp1 : E={wp1[0]:.4f}  N={wp1[1]:.4f}")
    print(f"[WaypointGen] wp2 : E={wp2[0]:.4f}  N={wp2[1]:.4f}")
    print(f"[WaypointGen] Run : {run_length:.2f} m  "
          f"@ {math.degrees(heading_rad):.1f}°")

    return wp1, wp2, heading_rad
