"""
main.py

Entry point for the rover on test day.

Sequence:
    1. Open Arduino serial connection
    2. Start GNSSReader background thread
    3. Wait for GPS fix → print "GPS FIXED" confirmation
    4. Run waypoint_gen → two-fix nudge flow → wp1, wp2, heading
    5. Run control loop → drive wp1 → wp2
    6. Stop motors, lift paint arm, close logs

Logs written to logs/ each run (timestamped):
    track_log_<timestamp>.csv   — position + errors every tick
    command_log_<timestamp>.csv — every Arduino command sent

Usage:
    python main.py
"""

import csv
import math
import os
import sys
import time
from datetime import datetime

from config_loader import CFG
from gnss_reader import GNSSReader
from waypoint_gen import run_waypoint_setup
from path_following import PathFollower, open_serial, send_motor_commands

# ---------------------------------------------------------------------------
# Config shortcuts
# ---------------------------------------------------------------------------
GNSS_PORT         = CFG["serial"]["gnss_port"]
GNSS_BAUD         = CFG["serial"]["gnss_baud"]
LOOP_HZ           = CFG["control"]["loop_hz"]
LOOP_DT           = 1.0 / LOOP_HZ
GPS_FIX_TIMEOUT   = CFG["gps"]["fix_timeout_s"]
MIN_FIX_QUALITY   = CFG["gps"]["min_fix_quality"]
HDOP_WARN         = CFG["gps"]["hdop_warn"]
LOG_DIR           = CFG["logging"]["log_dir"]
ARDUINO_BOOT_WAIT = 2.0


# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------

def _make_logger(name: str, fieldnames: list):
    os.makedirs(LOG_DIR, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    path  = os.path.join(LOG_DIR, f"{name}_{stamp}.csv")
    f     = open(path, "w", newline="")
    w     = csv.DictWriter(f, fieldnames=fieldnames)
    w.writeheader()
    print(f"[Log] {name} → {path}")
    return f, w, path


TRACK_FIELDS = [
    "timestamp", "easting", "northing",
    "heading_error_deg", "lateral_error_m", "progress_t", "paint_arm",
]

COMMAND_FIELDS = [
    "timestamp", "pwm_L", "pwm_R", "arm",
    "heading_error_deg", "lateral_error_m",
]

SESSION_FIELDS = [
    "key", "value",
]


def write_session_log(stamp: str,
                      wp1: tuple, wp2: tuple, heading_rad: float,
                      fix_info: dict):
    """
    Write a one-time session log capturing everything the rover was told:
      - wp1, wp2 coordinates
      - intended heading
      - GPS fix quality at run start
      - all config values used

    This lets you reconstruct post-run what the rover believed vs what
    it actually did (from track_log and command_log).
    """
    import math as _math
    os.makedirs(LOG_DIR, exist_ok=True)
    path = os.path.join(LOG_DIR, f"session_log_{stamp}.csv")
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=SESSION_FIELDS)
        w.writeheader()

        rows = [
            # Waypoints
            ("wp1_easting_m",         round(wp1[0], 4)),
            ("wp1_northing_m",        round(wp1[1], 4)),
            ("wp2_easting_m",         round(wp2[0], 4)),
            ("wp2_northing_m",        round(wp2[1], 4)),
            ("intended_heading_deg",  round(_math.degrees(heading_rad), 3)),
            ("intended_heading_rad",  round(heading_rad, 6)),
            ("run_length_m",          round(
                _math.sqrt((wp2[0]-wp1[0])**2 + (wp2[1]-wp1[1])**2), 4)),
            # GPS fix quality at run start
            ("fix_quality_at_start",  fix_info.get("fix_quality", "?")),
            ("num_sats_at_start",     fix_info.get("num_sats",    "?")),
            ("hdop_at_start",         fix_info.get("hdop",        "?")),
            # Config snapshot — so old logs stay interpretable after tuning
            ("cfg_gnss_port",         CFG["serial"]["gnss_port"]),
            ("cfg_gnss_baud",         CFG["serial"]["gnss_baud"]),
            ("cfg_arduino_port",      CFG["serial"]["arduino_port"]),
            ("cfg_arduino_baud",      CFG["serial"]["arduino_baud"]),
            ("cfg_loop_hz",           CFG["control"]["loop_hz"]),
            ("cfg_wp_accept_radius",  CFG["control"]["wp_accept_radius"]),
            ("cfg_base_speed_pwm",    CFG["follower"]["base_speed_pwm"]),
            ("cfg_max_steer_pwm",     CFG["follower"]["max_steer_pwm"]),
            ("cfg_k_heading",         CFG["follower"]["k_heading"]),
            ("cfg_k_lateral",         CFG["follower"]["k_lateral"]),
            ("cfg_heading_arm_threshold", CFG["follower"]["heading_arm_threshold"]),
            ("cfg_num_samples",       CFG["gps"]["num_samples"]),
            ("cfg_min_nudge_dist",    CFG["gps"]["min_nudge_dist"]),
        ]
        for key, value in rows:
            w.writerow({"key": key, "value": value})

    print(f"[Log] session_log  → {path}")
    return path


# ---------------------------------------------------------------------------
# GPS fix confirmation
# ---------------------------------------------------------------------------

def wait_for_fix(gnss: GNSSReader, timeout: float) -> bool:
    print(f"\n[GPS] Waiting for fix (timeout {timeout}s) ...")
    deadline = time.time() + timeout
    while time.time() < deadline:
        info = gnss.get_fix_info()
        if info["fix_quality"] >= MIN_FIX_QUALITY and info["position"] is not None:
            hdop = info["hdop"]
            sats = info["num_sats"]
            pos  = info["position"]
            print(f"\n{'='*45}")
            print(f"  GPS FIXED")
            print(f"  E={pos[0]:.3f}  N={pos[1]:.3f}")
            print(f"  Satellites: {sats}   HDOP: {hdop:.1f}")
            if hdop > HDOP_WARN:
                print(f"  WARNING: HDOP {hdop:.1f} is high — fix quality poor")
            print(f"{'='*45}\n")
            return True
        print(f"\r  sats={info['num_sats']}  HDOP={info['hdop']:.1f}  "
              f"quality={info['fix_quality']}  waiting...", end="", flush=True)
        time.sleep(0.5)
    print("\n[GPS] ERROR: Timed out waiting for fix.")
    return False


# ---------------------------------------------------------------------------
# Arduino handshake
# ---------------------------------------------------------------------------

def verify_arduino(ser, timeout: float = 5.0) -> bool:
    print("[Arduino] Waiting for READY ...")
    deadline = time.time() + timeout
    while time.time() < deadline:
        if ser.in_waiting:
            line = ser.readline().decode("ascii", errors="replace").strip()
            if line == "READY":
                ser.write(b"START\n")
                arm_deadline = time.time() + 2.0
                while time.time() < arm_deadline:
                    if ser.in_waiting:
                        ack = ser.readline().decode("ascii", errors="replace").strip()
                        if ack == "ARMED":
                            print("[Arduino] Armed and ready.")
                            return True
        time.sleep(0.05)
    print("[Arduino] WARNING: No READY/ARMED handshake received.")
    return False


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    print("\n" + "=" * 55)
    print("  Rover — Startup")
    print("=" * 55 + "\n")

    # ------------------------------------------------------------------
    # Step 1: Arduino serial
    # ------------------------------------------------------------------
    print("[Main] Connecting to Arduino ...")
    try:
        ser = open_serial()
    except Exception as e:
        print(f"[Main] ERROR: Cannot open Arduino serial port.\n  {e}")
        sys.exit(1)

    time.sleep(ARDUINO_BOOT_WAIT)
    verify_arduino(ser)

    # ------------------------------------------------------------------
    # Step 2: GNSS reader
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
    # Step 3: Confirm GPS fix quality
    # ------------------------------------------------------------------
    if not wait_for_fix(gnss, GPS_FIX_TIMEOUT):
        send_motor_commands(ser, 0, 0, False)
        gnss.stop()
        ser.close()
        sys.exit(1)

    # ------------------------------------------------------------------
    # Steps 4 & 5: Waypoint setup (two-fix nudge)
    # ------------------------------------------------------------------
    try:
        wp1, wp2, heading_rad = run_waypoint_setup(gnss)
    except RuntimeError as e:
        print(f"[Main] Waypoint ERROR: {e}")
        send_motor_commands(ser, 0, 0, False)
        gnss.stop()
        ser.close()
        sys.exit(1)

    # ------------------------------------------------------------------
    # Confirm before running
    # ------------------------------------------------------------------
    dist = math.sqrt((wp2[0]-wp1[0])**2 + (wp2[1]-wp1[1])**2)
    print(f"\n[Main] Ready to run.")
    print(f"       Start : E={wp1[0]:.3f}  N={wp1[1]:.3f}")
    print(f"       Target: E={wp2[0]:.3f}  N={wp2[1]:.3f}")
    print(f"       Distance: {dist:.2f} m  @ {math.degrees(heading_rad):.1f}°")
    input("\n[Main] Press ENTER to start the rover ...\n")

    # ------------------------------------------------------------------
    # Open logs — session log written before rover moves
    # ------------------------------------------------------------------
    stamp        = datetime.now().strftime("%Y%m%d_%H%M%S")
    session_path = write_session_log(stamp, wp1, wp2, heading_rad, gnss.get_fix_info())
    track_f,   track_w,   track_path   = _make_logger("track_log",   TRACK_FIELDS)
    command_f, command_w, command_path = _make_logger("command_log", COMMAND_FIELDS)

    # ------------------------------------------------------------------
    # Step 6: Control loop
    # ------------------------------------------------------------------
    follower = PathFollower(wp1, wp2, heading_rad)
    prev_P   = gnss.get_position()

    print("[Main] Rover running. Press Ctrl+C to e-stop.\n")

    try:
        while not follower.is_finished():
            loop_start = time.time()

            P     = gnss.get_position()
            speed = gnss.get_speed()

            result = follower.step(P, prev_P, speed_ms=speed)
            prev_P = P

            send_motor_commands(ser,
                                result["pwm_L"],
                                result["pwm_R"],
                                result["paint_arm"])

            now = datetime.now().isoformat(timespec="milliseconds")

            track_w.writerow({
                "timestamp"        : now,
                "easting"          : round(P[0], 4),
                "northing"         : round(P[1], 4),
                "heading_error_deg": round(result["heading_error"], 2),
                "lateral_error_m"  : round(result["lateral_error"], 4),
                "progress_t"       : round(result["t"], 4),
                "paint_arm"        : 1 if result["paint_arm"] else 0,
            })
            track_f.flush()

            command_w.writerow({
                "timestamp"        : now,
                "pwm_L"            : result["pwm_L"],
                "pwm_R"            : result["pwm_R"],
                "arm"              : 1 if result["paint_arm"] else 0,
                "heading_error_deg": round(result["heading_error"], 2),
                "lateral_error_m"  : round(result["lateral_error"], 4),
            })
            command_f.flush()

            if ser.in_waiting:
                echo = ser.readline().decode("ascii", errors="replace").strip()
                if echo.startswith("ERR"):
                    print(f"  [Arduino] {echo}")

            info = gnss.get_fix_info()
            if info["hdop"] > HDOP_WARN:
                print(f"  [GPS] HDOP={info['hdop']:.1f} — fix quality poor")

            arm_str = "ON " if result["paint_arm"] else "OFF"
            print(f"  L={result['pwm_L']:3d} R={result['pwm_R']:3d} | "
                  f"ARM={arm_str} | "
                  f"HE={result['heading_error']:+.1f}° "
                  f"LE={result['lateral_error']:+.3f}m "
                  f"t={result['t']:.2f} "
                  f"spd={speed:.2f}m/s")

            if result["status"] == "finished":
                reason = result.get("stop_reason", "")
                print(f"[Main] Run complete — {reason}")
                break

            elapsed = time.time() - loop_start
            sleep_t = LOOP_DT - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    except KeyboardInterrupt:
        print("\n[Main] E-STOP — interrupted by user.")

    finally:
        print("\n[Main] Stopping motors and lifting paint arm ...")
        send_motor_commands(ser, 0, 0, False)
        ser.write(b"STOP\n")
        gnss.stop()
        track_f.close()
        command_f.close()
        ser.close()
        print(f"[Main] Session log : {session_path}")
        print(f"[Main] Track log   : {track_path}")
        print(f"[Main] Command log : {command_path}")
        print("[Main] Done.\n")


if __name__ == "__main__":
    main()
