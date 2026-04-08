"""
main.py

Single entry point for the rover on test day.

Usage:
    python main.py

Sequence:
    1. Connects to Arduino over serial (verifies comms)
    2. Starts GNSSReader background thread (waits for first fix)
    3. Records fix_1 (rover stationary at start)
    4. Operator nudges rover ~1 m forward → records fix_2 (locks heading)
    5. Operator enters line length → builds waypoints → PathPlanner
    6. PathFollower runs control loop until distance reached or Ctrl+C
"""

import math
import time
import sys

from gnss_reader   import GNSSReader
from path_planning import PathPlanner
from path_following import (PathFollower, open_serial, send_motor_commands,
                             read_encoders, dist_m)


# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------
LOOP_HZ           = 5
LOOP_DT           = 1.0 / LOOP_HZ
GNSS_PORT         = "COM9"
GNSS_BAUD         = 57600
ARDUINO_BOOT_WAIT = 2.0    # seconds after serial open before talking
NUDGE_SAMPLES     = 3      # GPS fixes to average for each nudge point


# ---------------------------------------------------------------------------
# Arduino comms check
# ---------------------------------------------------------------------------

def verify_arduino(ser, timeout=5.0) -> bool:
    """
    Wait for the Arduino READY signal, then send START to arm it.
    Returns True if armed successfully, False if timed out.
    """
    print("[Main] Waiting for Arduino READY signal ...")
    deadline = time.time() + timeout
    while time.time() < deadline:
        if ser.in_waiting:
            line = ser.readline().decode("ascii", errors="replace").strip()
            if line == "READY":
                ser.write(b"START\n")
                arm_deadline = time.time() + 2.0
                while time.time() < arm_deadline:
                    if ser.in_waiting:
                        ack = ser.readline().decode("ascii",
                                                    errors="replace").strip()
                        if ack == "ARMED":
                            print("[Main] Arduino armed and ready.")
                            return True
    print("[Main] WARNING: Could not arm Arduino — check connection.")
    return False


# ---------------------------------------------------------------------------
# Averaged GPS fix helper
# ---------------------------------------------------------------------------

def averaged_fix(gnss: GNSSReader, n: int = NUDGE_SAMPLES) -> tuple:
    """
    Sample get_position() n times with a short delay and return the centroid.
    Smooths out single-fix GPS jitter when recording nudge points.
    """
    eastings, northings = [], []
    for _ in range(n):
        e, no = gnss.get_position()
        eastings.append(e)
        northings.append(no)
        time.sleep(0.2)
    return sum(eastings) / n, sum(northings) / n


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    print("\n" + "=" * 55)
    print("  Rover — Test Day Startup")
    print("=" * 55 + "\n")

    # ------------------------------------------------------------------
    # Step 1: Open serial to Arduino
    # ------------------------------------------------------------------
    print("[Main] Connecting to Arduino ...")
    try:
        ser = open_serial()
    except Exception as e:
        print(f"[Main] ERROR: Could not open Arduino serial port.\n  {e}")
        sys.exit(1)

    time.sleep(ARDUINO_BOOT_WAIT)
    verify_arduino(ser)

    # ------------------------------------------------------------------
    # Step 2: Start GNSS reader (background thread)
    # ------------------------------------------------------------------
    gnss = GNSSReader(port=GNSS_PORT, baud=GNSS_BAUD)
    try:
        gnss.start()
    except RuntimeError as e:
        print(f"[Main] GPS ERROR: {e}")
        send_motor_commands(ser, 0.0, 0.0, False)
        ser.close()
        sys.exit(1)

    # ------------------------------------------------------------------
    # Step 3: Record fix_1 — rover stationary at start line
    # ------------------------------------------------------------------
    print("\n[Main] Stand the rover at the START position and hold still.")
    input("        Press ENTER to record fix_1 ...\n")
    fix1 = averaged_fix(gnss)
    print(f"  fix_1: E={fix1[0]:.3f}  N={fix1[1]:.3f}")

    # ------------------------------------------------------------------
    # Step 4: Nudge → fix_2 — locks heading
    # ------------------------------------------------------------------
    print("\n[Main] Nudge the rover ~1 m forward along the desired line.")
    print("        Keep it pointed straight, then stop.")
    input("        Press ENTER to record fix_2 ...\n")
    fix2 = averaged_fix(gnss)
    print(f"  fix_2: E={fix2[0]:.3f}  N={fix2[1]:.3f}")

    nudge_dist = dist_m(fix1, fix2)
    print(f"  Nudge distance: {nudge_dist:.3f} m")
    if nudge_dist < 0.3:
        print(f"[Main] WARNING: nudge only {nudge_dist:.2f} m — "
              f"heading may be unreliable. Consider re-nudging further.")
        input("  Press ENTER to continue anyway, or Ctrl+C to abort ...\n")

    # ------------------------------------------------------------------
    # Step 5: Line length → waypoints → planner
    # ------------------------------------------------------------------
    print("\n[Main] Enter the total run distance.")
    while True:
        try:
            line_length = float(input("  Line length in metres: ").strip())
            if line_length > 0:
                break
            print("  Must be > 0.")
        except ValueError:
            print("  Enter a number.")

    # Build target waypoint along the nudge heading at the requested distance
    heading_rad = math.atan2(fix2[1] - fix1[1], fix2[0] - fix1[0])
    wp2 = (fix1[0] + line_length * math.cos(heading_rad),
           fix1[1] + line_length * math.sin(heading_rad))
    waypoints = [fix1, wp2]

    planner = PathPlanner()
    planner.load_waypoints(waypoints)

    if not planner.validate():
        print("[Main] Path validation failed. Aborting.")
        send_motor_commands(ser, 0.0, 0.0, False)
        gnss.stop()
        ser.close()
        sys.exit(1)

    print(f"\n[Main] Path ready — {planner.get_total_distance():.2f} m total.")

    # ------------------------------------------------------------------
    # Step 6: Build follower and lock heading from nudge fixes
    # ------------------------------------------------------------------
    follower = PathFollower(planner.get_waypoints())
    follower.set_target_heading(fix1, fix2)

    input("\n[Main] Press ENTER to start the rover ...\n")

    # ------------------------------------------------------------------
    # Step 7: Control loop
    # ------------------------------------------------------------------
    print("[Main] Running. Press Ctrl+C to stop.\n")
    print(f"  {'E':>9}  {'N':>9}  {'Dist':>5}  {'Rem':>5}  "
          f"{'HdgGPS':>7}  {'HErr':>6}  "
          f"{'Drift':>6}  {'Corr':>7}  {'vL':>6}  {'vR':>6}  ARM")
    print("  " + "-" * 78)

    try:
        while not follower.is_finished():
            loop_start = time.time()

            # Encoders — read before GPS so both arrive in the same cycle
            enc = read_encoders(ser)
            ticks_L = enc[0] if enc else None
            ticks_R = enc[1] if enc else None

            # GPS position
            current_pos = gnss.get_position()

            # Controller
            result = follower.step(current_pos, ticks_L, ticks_R)

            if result["status"] == "finished":
                break

            # Motor commands — now takes m/s floats, not raw PWM ints
            send_motor_commands(ser,
                                result["v_L_ms"],
                                result["v_R_ms"],
                                result["paint_arm"])

            # Non-ENC serial lines from Arduino (errors, status)
            if ser.in_waiting:
                echo = ser.readline().decode("ascii", errors="replace").strip()
                if echo.startswith("ERR"):
                    print(f"  [Arduino] {echo}")

            # GPS fix quality warning
            info = gnss.get_fix_info()
            if info["hdop"] > 3.0:
                print(f"  [GPS] HDOP={info['hdop']:.1f} — fix quality poor")

            # Telemetry
            arm_str = "ON " if result["paint_arm"] else "OFF"
            enc_str = "" if result["have_enc"] else " (no enc)"
            print(f"  {result['easting']:>9.3f}  "
                  f"{result['northing']:>9.3f}  "
                  f"{result['dist_from_start']:>5.2f}  "
                  f"{result['dist_remaining']:>5.2f}  "
                  f"{result['heading_deg']:>7.1f}  "
                  f"{result['heading_err_deg']:>+6.2f}  "
                  f"{result['enc_drift']:>+6.0f}  "
                  f"{result['correction_ms']:>+7.4f}  "
                  f"{result['v_L_ms']:>6.4f}  "
                  f"{result['v_R_ms']:>6.4f}  "
                  f"{arm_str}{enc_str}")

            # Hold loop rate
            elapsed = time.time() - loop_start
            sleep_t = LOOP_DT - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    except KeyboardInterrupt:
        print("\n[Main] Interrupted by user.")

    finally:
        print("[Main] Stopping motors ...")
        send_motor_commands(ser, 0.0, 0.0, False)
        ser.write(b"STOP\n")
        gnss.stop()
        ser.close()
        print("[Main] Done.")


if __name__ == "__main__":
    main()
