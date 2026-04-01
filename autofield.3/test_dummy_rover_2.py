"""
test_dummy_rover.py

Laptop drag-test harness for path_following.py.

The real rover and Arduino are replaced by:
  - A fake GNSSReader that reads your laptop's GPS (or lets you type
    coordinates manually if no GPS is available).
  - A fake send_motor_commands() that translates PWM + steering output
    into plain-English guidance printed to the terminal.

Run this instead of main.py when the rover hardware is unavailable.
Walk/drag your laptop along the intended path and watch the guidance
update in real time.

Usage:
    python test_dummy_rover.py

    Mode 1 — USB GPS on laptop, no Arduino.  Walk the path yourself.
    Mode 2 — Manual coordinate entry each loop tick.  No hardware needed.
    Mode 3 — QUICK RUN: real GPS + real Arduino, skips the full main.py
             ceremony.  Designed for fast field tests without you present.
             Edit QUICK_RUN_DEFAULTS below before handing off to the team.

────────────────────────────────────────────────────────────────────────
QUICK-RUN SETUP  (fill these in before test day)
────────────────────────────────────────────────────────────────────────
"""

import csv
import math
import os
import time
import sys
from datetime import datetime

from path_planning import PathPlanner
from path_following import PathFollower, heading_from_points, send_motor_commands


# ---------------------------------------------------------------------------
# ★  EDIT THESE BEFORE TEST DAY  ★
# ---------------------------------------------------------------------------
# Mode 3 will use these values as defaults.  The team can still override
# them interactively — just press Enter to accept the bracketed default.

QUICK_RUN_DEFAULTS = {
    "gnss_port"    : "COM5",    # GPS serial port  (e.g. "COM5" or "/dev/ttyUSB1")
    "gnss_baud"    : 57600,     # GPS baud rate
    "arduino_port" : "COM15",   # Arduino serial port (e.g. "COM15")
    "arduino_baud" : 9600,      # Arduino baud rate
    "line_length_m": 20.0,      # Default run distance in metres
    "loop_hz"      : 5,         # Control loop rate (should match main.py)
}
# ---------------------------------------------------------------------------


# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------
LOOP_HZ          = 2      # Update rate for modes 1 & 2 (walking speed)
LOOP_DT          = 1.0 / LOOP_HZ
WALKING_SPEED_MS = 0.8    # Assumed speed (m/s) used if GPS speed unavailable

# CSV log — timestamped so each run gets its own file
LOG_DIR = "track_logs"


# ---------------------------------------------------------------------------
# CSV track logger
# ---------------------------------------------------------------------------

class TrackLogger:
    """
    Writes one row per control tick to a UTM CSV file.

    Columns:
        timestamp       — ISO-8601 wall-clock time
        easting         — UTM easting (m)
        northing        — UTM northing (m)
        lateral_error   — signed off-course distance (m), +ve = right of path
        head_error_deg  — heading error (degrees)
        progress_t      — progress along current segment [0, 1]
        paint_arm       — 1 if paint arm would be active, else 0

    The file can be loaded directly into QGIS, Google Earth (with a
    UTM → WGS84 reproject step), or any mapping tool that accepts CSV
    with easting/northing columns.
    """

    COLUMNS = [
        "timestamp", "easting", "northing",
        "lateral_error_m", "head_error_deg", "progress_t", "paint_arm",
    ]

    def __init__(self):
        os.makedirs(LOG_DIR, exist_ok=True)
        stamp    = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.path = os.path.join(LOG_DIR, f"track_{stamp}.csv")
        self._f   = open(self.path, "w", newline="")
        self._w   = csv.DictWriter(self._f, fieldnames=self.COLUMNS)
        self._w.writeheader()
        print(f"[TrackLogger] Logging to: {self.path}")

    def log(self, P: tuple, result: dict):
        self._w.writerow({
            "timestamp"       : datetime.now().isoformat(timespec="milliseconds"),
            "easting"         : round(P[0], 4),
            "northing"        : round(P[1], 4),
            "lateral_error_m" : round(result["lateral_error"], 4),
            "head_error_deg"  : round(result["head_angle_error"], 2),
            "progress_t"      : round(result["t"], 4),
            "paint_arm"       : 1 if result["paint_arm"] else 0,
        })
        self._f.flush()   # write immediately so Ctrl+C doesn't lose data

    def close(self):
        self._f.close()
        print(f"[TrackLogger] Saved {self.path}")


# ---------------------------------------------------------------------------
# Fake motor output — translates PWM diff into human steering guidance
# ---------------------------------------------------------------------------

def print_steering_guidance(result: dict):
    """
    Convert path_following output into readable terminal guidance.

    Replaces send_motor_commands() for the laptop test.
    """
    lat_err   = result["lateral_error"]       # metres, +ve = left of path
    head_err  = result["head_angle_error"]    # degrees
    delta_t   = result["delta_t"]             # degrees, steering correction
    t         = result["t"]                   # progress along segment [0,1]
    pwm_L     = result["pwm_L"]
    pwm_R     = result["pwm_R"]
    paint     = result["paint_arm"]

    # --- Off-course direction ---
    if abs(lat_err) < 0.05:
        course_str = "  ON PATH ✓"
    elif lat_err > 0:
        course_str = f"  {abs(lat_err):.2f} m RIGHT of path  →  drift LEFT"
    else:
        course_str = f"  {abs(lat_err):.2f} m LEFT of path   →  drift RIGHT"

    # --- Heading correction ---
    if abs(head_err) < 1.0:
        head_str = "  Heading OK ✓"
    elif head_err > 0:
        head_str = f"  Heading {abs(head_err):.1f}° too far LEFT  →  turn RIGHT"
    else:
        head_str = f"  Heading {abs(head_err):.1f}° too far RIGHT →  turn LEFT"

    # --- Combined steering verdict ---
    if abs(delta_t) < 2.0:
        steer_str = "  STRAIGHT"
    elif delta_t > 0:
        steer_str = f"  TURN LEFT  (correction: {abs(delta_t):.1f}°)"
    else:
        steer_str = f"  TURN RIGHT (correction: {abs(delta_t):.1f}°)"

    # --- Progress bar ---
    bar_len  = 30
    filled   = int(max(0.0, min(1.0, t)) * bar_len)
    progress = "[" + "█" * filled + "░" * (bar_len - filled) + f"]  {t*100:.0f}%"

    # --- Paint arm ---
    paint_str = "PAINTING 🎨" if paint else "arm OFF"

    # --- PWM diff (sanity check the controller) ---
    pwm_diff = pwm_L - pwm_R
    if abs(pwm_diff) < 5:
        pwm_str = f"  L={pwm_L} R={pwm_R}  (balanced)"
    elif pwm_diff > 0:
        pwm_str = f"  L={pwm_L} R={pwm_R}  (left faster → curving right)"
    else:
        pwm_str = f"  L={pwm_L} R={pwm_R}  (right faster → curving left)"

    # --- Print block ---
    sep = "─" * 52
    print(f"\n{sep}")
    print(f"  Progress : {progress}")
    print(f"  Course   :{course_str}")
    print(f"  Heading  :{head_str}")
    print(f"  Steer    :{steer_str}")
    print(f"  Motors   :{pwm_str}")
    print(f"  Paint arm: {paint_str}")
    print(sep)


# ---------------------------------------------------------------------------
# Fake GNSS — GPS serial or manual keyboard entry
# ---------------------------------------------------------------------------

class DummyGNSS:
    """
    Provides the same get_position() / get_speed() interface as GNSSReader
    without opening a serial port.

    Two modes:
      'gps'    — reads from a real GPS serial port using the existing
                 GNSSReader (useful if your laptop has USB GPS).
      'manual' — prompts you to type easting/northing each loop tick.
    """

    def __init__(self, mode: str, gnss_reader=None):
        self.mode   = mode
        self._gnss  = gnss_reader   # real GNSSReader if mode == 'gps'
        self._pos   = None
        self._speed = WALKING_SPEED_MS

    def get_position(self):
        if self.mode == "gps":
            return self._gnss.get_position()

        # Manual mode — prompt user
        print("\n  Enter your current position (UTM metres).")
        print("  Tip: copy from a GPS app or Google Maps → right-click → UTM coords.")
        while True:
            try:
                raw = input("  Easting Northing (e.g.  500.3 1000.7): ").strip()
                parts = raw.split()
                if len(parts) != 2:
                    raise ValueError
                e, n = float(parts[0]), float(parts[1])
                self._pos = (e, n)
                return self._pos
            except ValueError:
                print("  Bad input — enter two numbers separated by a space.")

    def get_speed(self):
        if self.mode == "gps":
            return self._gnss.get_speed()
        return WALKING_SPEED_MS   # assume walking pace


# ---------------------------------------------------------------------------
# Startup — mirrors manual_waypoint_gen.run_startup() without serial GPS
# ---------------------------------------------------------------------------

def dummy_startup(gnss: DummyGNSS) -> list:
    """
    Collect wp_1 from the dummy GNSS and ask for a line length.
    Returns a waypoint list identical in format to run_startup().
    """
    print("\n" + "=" * 52)
    print("  Dummy Rover — Waypoint Setup")
    print("=" * 52)
    print("\nStep 1: Stand at your intended START position.")
    input("        Press ENTER when ready to record wp_1 ...\n")

    wp1 = gnss.get_position()
    print(f"  wp_1 recorded: E={wp1[0]:.3f}  N={wp1[1]:.3f}")

    print("\nStep 2: Enter the line length.")
    while True:
        try:
            line_length = float(input("  Line length in metres (e.g. 20): ").strip())
            if line_length <= 0:
                print("  Must be > 0.")
                continue
            break
        except ValueError:
            print("  Enter a number.")

    wp2 = (wp1[0] + line_length, wp1[1])   # +easting, same as real code
    waypoints = [wp1, wp2]

    print(f"\n  wp_1: E={wp1[0]:.3f}  N={wp1[1]:.3f}")
    print(f"  wp_2: E={wp2[0]:.3f}  N={wp2[1]:.3f}  ({line_length} m east)")
    return waypoints


# ---------------------------------------------------------------------------
# Helper — prompt with a pre-filled default (press Enter to accept)
# ---------------------------------------------------------------------------

def prompt_with_default(prompt: str, default):
    """
    Show a prompt with the default value in brackets.
    Pressing Enter alone accepts the default.
    Returns the typed value cast to the same type as default,
    or default if the user pressed Enter.
    """
    raw = input(f"  {prompt} [{default}]: ").strip()
    if raw == "":
        return default
    try:
        return type(default)(raw)
    except ValueError:
        print(f"  Invalid input — using default ({default})")
        return default


# ---------------------------------------------------------------------------
# Mode 3 — Quick-run on live hardware
# ---------------------------------------------------------------------------

def quick_run():
    """
    Minimal-ceremony live rover test.

    - Connects to the real GPS and waits for first fix (no averaging).
    - Optionally arms the Arduino (skips gracefully if unavailable).
    - Takes one GPS fix as wp_1, asks for a line length, and runs.
    - All prompts show pre-filled defaults from QUICK_RUN_DEFAULTS so
      the team can press Enter through the whole setup if ports are correct.

    Intended as a fast functional check, not a calibration run.
    """
    d = QUICK_RUN_DEFAULTS
    loop_dt = 1.0 / d["loop_hz"]

    print("\n" + "=" * 52)
    print("  QUICK RUN — live GPS + Arduino")
    print("  (skips full main.py startup sequence)")
    print("=" * 52)
    print("\n  Defaults are pre-filled — press Enter to accept each one.\n")

    # --- GPS ---
    gnss_port = prompt_with_default("GPS port",   d["gnss_port"])
    gnss_baud = prompt_with_default("GPS baud",   d["gnss_baud"])

    from gnss_reader import GNSSReader
    gnss = GNSSReader(port=gnss_port, baud=gnss_baud)
    try:
        gnss.start()   # blocks until first fix or timeout
    except RuntimeError as e:
        print(f"\n[QuickRun] GPS ERROR: {e}")
        print("           Check the port and antenna, then try again.")
        sys.exit(1)

    # --- Arduino (optional) ---
    arduino_port = prompt_with_default("Arduino port", d["arduino_port"])
    arduino_baud = prompt_with_default("Arduino baud", d["arduino_baud"])

    ser = None
    import serial as _serial
    try:
        ser = _serial.Serial(arduino_port, arduino_baud, timeout=1)
        time.sleep(2.0)   # wait for Arduino bootloader

        # Arming handshake — wait up to 5 s for READY, then send START
        print("[QuickRun] Waiting for Arduino READY ...")
        deadline = time.time() + 5.0
        armed = False
        while time.time() < deadline:
            if ser.in_waiting:
                line = ser.readline().decode("ascii", errors="replace").strip()
                if line == "READY":
                    ser.write(b"START\n")
                    # Wait for ARMED
                    arm_deadline = time.time() + 2.0
                    while time.time() < arm_deadline:
                        if ser.in_waiting:
                            ack = ser.readline().decode("ascii", errors="replace").strip()
                            if ack == "ARMED":
                                print("[QuickRun] Arduino armed.")
                                armed = True
                                break
                    break

        if not armed:
            print("[QuickRun] WARNING: Arduino did not confirm ARMED.")
            print("           Continuing — motor commands will be printed but not sent.")
            ser.close()
            ser = None

    except _serial.SerialException as e:
        print(f"[QuickRun] WARNING: Could not open Arduino port {arduino_port}: {e}")
        print("           Continuing in print-only mode.")
        ser = None

    # --- Waypoint setup ---
    print("\n  Point the rover along your intended run direction.")
    input("  Press ENTER to record wp_1 from the current GPS fix ...\n")

    wp1 = gnss.get_position()
    print(f"  wp_1: E={wp1[0]:.3f}  N={wp1[1]:.3f}")
    fix_info = gnss.get_fix_info()
    print(f"        sats={fix_info['num_sats']}  HDOP={fix_info['hdop']:.1f}  "
          f"quality={fix_info['fix_quality']}")

    if fix_info["hdop"] > 3.0:
        print("  WARNING: HDOP > 3.0 — GPS fix quality is poor. Consider waiting.")

    line_length = prompt_with_default("Line length (m)", d["line_length_m"])
    wp2 = (wp1[0] + line_length, wp1[1])

    print(f"\n  wp_1: E={wp1[0]:.3f}  N={wp1[1]:.3f}")
    print(f"  wp_2: E={wp2[0]:.3f}  N={wp2[1]:.3f}  ({line_length} m east)")

    # --- Path planning ---
    planner = PathPlanner()
    planner.load_waypoints([wp1, wp2])
    if not planner.validate():
        print("[QuickRun] Path validation failed. Aborting.")
        _cleanup(ser, gnss)
        sys.exit(1)

    print(f"\n[QuickRun] Path ready — {planner.get_total_distance():.2f} m total.")
    if ser:
        print("[QuickRun] Arduino is armed. Rover will move on ENTER.")
    else:
        print("[QuickRun] No Arduino — running in print-only mode.")
    input("[QuickRun] Press ENTER to start ...\n")

    # --- Control loop ---
    follower = PathFollower(planner.get_waypoints())
    prev_P   = gnss.get_position()
    logger   = TrackLogger()

    print("[QuickRun] Running. Press Ctrl+C to stop.\n")

    try:
        while not follower.is_finished():
            loop_start = time.time()

            P     = gnss.get_position()
            speed = gnss.get_speed()

            result = follower.step(P, prev_P, speed=speed)
            prev_P = P

            if result["status"] == "finished":
                logger.log(P, result)
                break

            logger.log(P, result)

            if ser:
                send_motor_commands(ser,
                                    result["pwm_L"],
                                    result["pwm_R"],
                                    result["paint_arm"])
                # Read any echo / error back from Arduino (non-blocking)
                if ser.in_waiting:
                    echo = ser.readline().decode("ascii", errors="replace").strip()
                    if echo.startswith("ERR") or echo.startswith("WATCHDOG"):
                        print(f"  [Arduino] {echo}")

            # Always print guidance so the team can see what's happening
            print_steering_guidance(result)

            # GPS quality warning
            info = gnss.get_fix_info()
            if info["hdop"] > 3.0:
                print(f"  [GPS] HDOP={info['hdop']:.1f} — fix quality poor")

            # Hold loop rate
            elapsed = time.time() - loop_start
            sleep_t = loop_dt - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    except KeyboardInterrupt:
        print("\n[QuickRun] Stopped by user.")

    finally:
        logger.close()
        _cleanup(ser, gnss)

    print("\n[QuickRun] Run complete.")
    print(f"           Track CSV saved in: {LOG_DIR}/\n")


def _cleanup(ser, gnss):
    """Safe teardown — always stop motors before closing ports."""
    if ser is not None:
        try:
            send_motor_commands(ser, 0, 0, False)
            ser.write(b"STOP\n")
            time.sleep(0.1)
            ser.close()
        except Exception:
            pass
    if gnss is not None:
        try:
            gnss.stop()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    print("\n" + "=" * 52)
    print("  TEST DUMMY ROVER — path_following harness")
    print("=" * 52 + "\n")

    print("Mode:")
    print("  [1] USB GPS on laptop, no Arduino  (walk the path yourself)")
    print("  [2] Manual coordinate entry         (no hardware needed)")
    print("  [3] QUICK RUN — real GPS + Arduino  (fast field test)")
    choice = input("\nChoice (1/2/3): ").strip()

    if choice == "3":
        quick_run()
        return

    # ---- Modes 1 & 2 (unchanged) ----------------------------------------

    if choice == "1":
        from gnss_reader import GNSSReader
        port = input("GPS serial port (e.g. COM3 or /dev/ttyUSB0): ").strip()
        baud = int(input("Baud rate [57600]: ").strip() or "57600")
        real_gnss = GNSSReader(port=port, baud=baud)
        real_gnss.start()
        gnss = DummyGNSS(mode="gps", gnss_reader=real_gnss)
        print("[DummyRover] Live GPS ready.\n")
    else:
        gnss = DummyGNSS(mode="manual")
        print("[DummyRover] Manual entry mode.\n")

    # --- Waypoint setup ---
    try:
        waypoints = dummy_startup(gnss)
    except (KeyboardInterrupt, EOFError):
        print("\nAborted during setup.")
        sys.exit(0)

    # --- Path planning ---
    planner = PathPlanner()
    planner.load_waypoints(waypoints)

    if not planner.validate():
        print("Path validation failed. Aborting.")
        sys.exit(1)

    print(f"\n[DummyRover] Path ready — {planner.get_total_distance():.2f} m total.")
    input("[DummyRover] Press ENTER to start walking ...\n")

    # --- Control loop ---
    follower = PathFollower(planner.get_waypoints())
    prev_P   = gnss.get_position()
    logger   = TrackLogger()

    print("[DummyRover] Running. Press Ctrl+C to stop.\n")
    print("  Watch the STEER line — it tells you which way the rover")
    print("  would turn and by how much. Walk to keep it straight.\n")

    try:
        while not follower.is_finished():
            loop_start = time.time()

            P     = gnss.get_position()
            speed = gnss.get_speed()

            result = follower.step(P, prev_P, speed=speed)
            prev_P = P

            if result["status"] == "finished":
                logger.log(P, result)
                break

            logger.log(P, result)
            print_steering_guidance(result)

            # Loop rate (less critical in manual mode but keeps GPS mode tidy)
            elapsed = time.time() - loop_start
            sleep_t = LOOP_DT - elapsed
            if sleep_t > 0 and choice == "1":
                time.sleep(sleep_t)

    except KeyboardInterrupt:
        print("\n[DummyRover] Stopped by user.")

    finally:
        logger.close()
        if choice == "1":
            try:
                gnss._gnss.stop()
            except Exception:
                pass

    print("\n[DummyRover] Path complete. Check output above for any")
    print("             persistent off-course or heading errors.")
    print(f"             Track CSV saved in: {LOG_DIR}/\n")


if __name__ == "__main__":
    main()