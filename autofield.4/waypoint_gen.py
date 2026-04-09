"""
waypoint_gen.py

Handles the two-fix startup sequence:
  1. Average several GPS fixes → wp1 (start position)
  2. User nudges rover forward, presses ENTER
  3. Average several GPS fixes → fix2 (heading reference only)
  4. Compute unit heading vector from wp1 → fix2
  5. User enters run length → wp2 = wp1 + length * heading_vector

Returns wp1, wp2 as (easting, northing) tuples and the heading in radians.

Usage (called by main.py):
    from waypoint_gen import run_waypoint_setup
    wp1, wp2, heading_rad = run_waypoint_setup(gnss)
"""

import math
import time


# Number of GPS samples to average for each fix
NUM_SAMPLES = 5

# Minimum separation between wp1 and fix2 to trust the heading (metres)
MIN_NUDGE_DISTANCE = 0.3


# ---------------------------------------------------------------------------
# Internal helper — collect N averaged fixes from a live GNSSReader
# ---------------------------------------------------------------------------

def _average_fixes(gnss, num_samples: int, label: str) -> tuple:
    """
    Collect `num_samples` fresh GPS readings and return their average
    as (easting, northing).

    Blocks until enough samples are collected. Prints progress.
    """
    eastings  = []
    northings = []
    last_pos  = None

    print(f"\n[WaypointGen] Collecting {num_samples} samples for {label} ...")

    while len(eastings) < num_samples:
        pos = gnss.get_position()

        # Only accept a new sample when the GPS has updated
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

    Parameters
    ----------
    gnss : GNSSReader
        Already started GNSSReader instance (from main.py).

    Returns
    -------
    wp1         : (easting, northing) — start position
    wp2         : (easting, northing) — target position
    heading_rad : float — heading in radians (CCW from east, UTM convention)
    """
    print("\n" + "=" * 55)
    print("  Waypoint Setup")
    print("=" * 55)

    # ------------------------------------------------------------------
    # Fix 1 — rover at start position
    # ------------------------------------------------------------------
    print("\nStep 1: Place the rover at the START position and hold still.")
    input("        Press ENTER to record Fix 1 ...\n")

    wp1 = _average_fixes(gnss, NUM_SAMPLES, "Fix 1 (start)")

    # ------------------------------------------------------------------
    # Fix 2 — nudge rover forward to establish heading
    # ------------------------------------------------------------------
    print("\nStep 2: Nudge the rover forward in your intended run direction.")
    print("        Any distance works — just make it consistent.")
    input("        Press ENTER when nudge is complete to record Fix 2 ...\n")

    fix2 = _average_fixes(gnss, NUM_SAMPLES, "Fix 2 (heading reference)")

    # Compute heading vector
    dx = fix2[0] - wp1[0]
    dy = fix2[1] - wp1[1]
    nudge_dist = math.sqrt(dx**2 + dy**2)

    if nudge_dist < MIN_NUDGE_DISTANCE:
        raise RuntimeError(
            f"Nudge distance too small ({nudge_dist:.3f} m). "
            f"Need at least {MIN_NUDGE_DISTANCE} m. "
            "Move the rover further before pressing ENTER."
        )

    # Unit heading vector and angle
    heading_rad = math.atan2(dy, dx)
    ux = dx / nudge_dist
    uy = dy / nudge_dist

    print(f"\n[WaypointGen] Heading: {math.degrees(heading_rad):.1f}° "
          f"(nudge dist: {nudge_dist:.3f} m)")

    # ------------------------------------------------------------------
    # Run length → wp2
    # ------------------------------------------------------------------
    print("\nStep 3: Enter the desired run length.")
    while True:
        try:
            raw = input("  Run length in metres (e.g. 20): ").strip()
            run_length = float(raw)
            if run_length <= 0:
                print("  Must be greater than 0. Try again.")
                continue
            break
        except ValueError:
            print("  Invalid input — enter a number.")

    wp2 = (
        wp1[0] + run_length * ux,
        wp1[1] + run_length * uy,
    )

    print(f"\n[WaypointGen] wp1 : E={wp1[0]:.4f}  N={wp1[1]:.4f}")
    print(f"[WaypointGen] wp2 : E={wp2[0]:.4f}  N={wp2[1]:.4f}")
    print(f"[WaypointGen] Run : {run_length:.2f} m  "
          f"@ {math.degrees(heading_rad):.1f}°")

    return wp1, wp2, heading_rad
