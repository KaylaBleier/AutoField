"""
main.py

Single entry point for the rover on test day.

Usage:
    python main.py

Sequence:
    1. Connects to Arduino over serial (verifies comms)
    2. Starts GNSSReader background thread (waits for first fix)
    3. Runs manual_waypoint_gen  → user confirms GPS fix + types line length
    4. Runs path_planning        → builds coordinate + distance matrices
    5. Runs path_following       → control loop until path complete or Ctrl+C
"""

import time
import sys

from gnss_reader import GNSSReader
from manual_waypoint_gen import run_startup
from path_planning import PathPlanner
from path_following import PathFollower, open_serial, send_motor_commands


# ---------------------------------------------------------------------------
# Config — check ports with: ls /dev/ttyUSB*
# ---------------------------------------------------------------------------
LOOP_HZ   = 5               # control loop rate (cycles per second)
LOOP_DT   = 1.0 / LOOP_HZ
GNSS_PORT = "COM5"  # GPS rover module
GNSS_BAUD = 57600
ARDUINO_BOOT_WAIT = 2.0     # seconds to wait after opening serial


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
                # Send arming command
                ser.write(b"START\n")
                # Wait for ARMED confirmation
                arm_deadline = time.time() + 2.0
                while time.time() < arm_deadline:
                    if ser.in_waiting:
                        ack = ser.readline().decode("ascii", errors="replace").strip()
                        if ack == "ARMED":
                            print("[Main] Arduino armed and ready.")
                            return True
    print("[Main] WARNING: Could not arm Arduino — check connection.")
    return False


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
        send_motor_commands(ser, 0, 0, False)
        ser.close()
        sys.exit(1)

    # ------------------------------------------------------------------
    # Step 3: Waypoint generation
    # Passes the live gnss reader into run_startup so it reuses the
    # already-running serial connection instead of opening a second one.
    # ------------------------------------------------------------------
    try:
        waypoints = run_startup(gnss=gnss)
    except RuntimeError as e:
        print(f"[Main] Waypoint ERROR: {e}")
        send_motor_commands(ser, 0, 0, False)
        gnss.stop()
        ser.close()
        sys.exit(1)

    # ------------------------------------------------------------------
    # Step 4: Path planning
    # ------------------------------------------------------------------
    planner = PathPlanner()
    planner.load_waypoints(waypoints)

    if not planner.validate():
        print("[Main] Path validation failed. Aborting.")
        send_motor_commands(ser, 0, 0, False)
        gnss.stop()
        ser.close()
        sys.exit(1)

    print(f"\n[Main] Path ready — {planner.get_total_distance():.2f} m total.")
    input("[Main] Press ENTER to start the rover ...\n")

    # ------------------------------------------------------------------
    # Step 5: Path following — control loop
    # ------------------------------------------------------------------
    follower = PathFollower(planner.get_waypoints())
    prev_P   = gnss.get_position()   # live starting position

    print("[Main] Running. Press Ctrl+C to stop.\n")

    try:
        while not follower.is_finished():
            loop_start = time.time()

            # Current GPS position and speed from background thread
            P     = gnss.get_position()
            speed = gnss.get_speed()

            result = follower.step(P, prev_P, speed=speed)
            prev_P = P

            if result["status"] == "finished":
                break

            # Send to Arduino
            send_motor_commands(ser,
                                result["pwm_L"],
                                result["pwm_R"],
                                result["paint_arm"])

            # Read any echo / error back from Arduino (non-blocking)
            if ser.in_waiting:
                echo = ser.readline().decode("ascii", errors="replace").strip()
                if echo.startswith("ERR"):
                    print(f"[Arduino] {echo}")

            # Fix quality warning
            info = gnss.get_fix_info()
            if info["hdop"] > 3.0:
                print(f"  [GPS] HDOP={info['hdop']:.1f} — fix quality poor")

            # Status printout
            print(f"  L={result['pwm_L']:3d} R={result['pwm_R']:3d} | "
                  f"ARM={'ON ' if result['paint_arm'] else 'OFF'} | "
                  f"TE={result['lateral_error']:+.3f}m "
                  f"HE={result['head_angle_error']:+.1f}° "
                  f"t={result['t']:.2f} "
                  f"spd={speed:.2f}m/s")

            # Hold loop rate
            elapsed = time.time() - loop_start
            sleep_t = LOOP_DT - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    except KeyboardInterrupt:
        print("\n[Main] Interrupted by user.")

    finally:
        print("[Main] Stopping motors ...")
        send_motor_commands(ser, 0, 0, False)
        ser.write(b"STOP\n")   # disarm Arduino
        gnss.stop()
        ser.close()
        print("[Main] Done.")


if __name__ == "__main__":
    main()
