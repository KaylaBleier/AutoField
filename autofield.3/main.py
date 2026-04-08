"""
main.py  (v5 — simple path, track log + command log)

Logs two CSV files per run into the logs/ directory:
  track_YYYYMMDD_HHMMSS.csv   — GPS position + state every cycle
  command_YYYYMMDD_HHMMSS.csv — every motor command sent + why

Both files share the same timestamp so a run's two files are easy to match.
"""

import csv
import os
import sys
import time
from datetime import datetime

from gnss_reader          import GNSSReader
from simple_path_following import (SimplePathFollower, open_serial,
                                   all_stop, all_forward, turn_right,
                                   dist_m, STATE_TURN, STATE_DONE,
                                   STATE_SEG2, PWM_BASE, PWM_TURN,
                                   TURN_DURATION_SEC)


# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------
LOOP_HZ           = 5
LOOP_DT           = 1.0 / LOOP_HZ
GNSS_PORT         = "COM9"
GNSS_BAUD         = 57600
ARDUINO_BOOT_WAIT = 2.0
NUDGE_SAMPLES     = 3
LOG_DIR           = "logs"


# ---------------------------------------------------------------------------
# Loggers
# ---------------------------------------------------------------------------

class TrackLog:
    """
    One row per control cycle — where the rover was and what state it was in.

    Columns
    -------
    timestamp       ISO-8601 wall clock
    state           STRAIGHT_1 / TURNING / STRAIGHT_2 / DONE
    easting         UTM easting (m)
    northing        UTM northing (m)
    dist_from_start GPS distance from current segment start (m)
    dist_remaining  metres left in current segment
    turn_pct        0–100 during turn, else 0
    hdop            GPS fix quality (lower = better)
    """

    COLUMNS = ["timestamp", "state", "easting", "northing",
               "dist_from_start", "dist_remaining", "turn_pct", "hdop"]

    def __init__(self, stamp: str):
        os.makedirs(LOG_DIR, exist_ok=True)
        self.path = os.path.join(LOG_DIR, f"track_{stamp}.csv")
        self._f   = open(self.path, "w", newline="")
        self._w   = csv.DictWriter(self._f, fieldnames=self.COLUMNS)
        self._w.writeheader()
        print(f"[TrackLog]   {self.path}")

    def log(self, result: dict, hdop: float):
        self._w.writerow({
            "timestamp"      : datetime.now().isoformat(timespec="milliseconds"),
            "state"          : result["state"],
            "easting"        : result["easting"],
            "northing"       : result["northing"],
            "dist_from_start": result["dist"],
            "dist_remaining" : result["remaining"],
            "turn_pct"       : result["turn_pct"],
            "hdop"           : round(hdop, 2),
        })
        self._f.flush()

    def close(self):
        self._f.close()


class CommandLog:
    """
    One row every time a motor command is sent — what was sent and why.

    Columns
    -------
    timestamp       ISO-8601 wall clock
    state           rover state when command was sent
    command         "forward" | "turn" | "stop"
    pwm_L1          left  front PWM sent  (0–255, negative = reverse)
    pwm_L2          left  rear  PWM sent
    pwm_R1          right front PWM sent
    pwm_R2          right rear  PWM sent
    paint_arm       0 or 1
    reason          human-readable explanation of why this command was sent
    """

    COLUMNS = ["timestamp", "state", "command",
               "pwm_L1", "pwm_L2", "pwm_R1", "pwm_R2",
               "paint_arm", "reason"]

    def __init__(self, stamp: str):
        os.makedirs(LOG_DIR, exist_ok=True)
        self.path = os.path.join(LOG_DIR, f"command_{stamp}.csv")
        self._f   = open(self.path, "w", newline="")
        self._w   = csv.DictWriter(self._f, fieldnames=self.COLUMNS)
        self._w.writeheader()
        print(f"[CommandLog] {self.path}")

    def log(self, state: str, command: str,
            pwm_L: int, pwm_R: int, paint: int, reason: str):
        self._w.writerow({
            "timestamp": datetime.now().isoformat(timespec="milliseconds"),
            "state"    : state,
            "command"  : command,
            "pwm_L1"   : pwm_L,
            "pwm_L2"   : pwm_L,
            "pwm_R1"   : pwm_R,
            "pwm_R2"   : pwm_R,
            "paint_arm": paint,
            "reason"   : reason,
        })
        self._f.flush()

    def close(self):
        self._f.close()


# ---------------------------------------------------------------------------
# Arduino handshake
# ---------------------------------------------------------------------------

def verify_arduino(ser, timeout=5.0) -> bool:
    print("[Main] Waiting for Arduino READY ...")
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
                            print("[Main] Arduino armed.")
                            return True
    print("[Main] WARNING: Could not arm Arduino.")
    return False


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def averaged_fix(gnss: GNSSReader, n: int = NUDGE_SAMPLES) -> tuple:
    eastings, northings = [], []
    for _ in range(n):
        e, no = gnss.get_position()
        eastings.append(e)
        northings.append(no)
        time.sleep(0.2)
    return sum(eastings) / n, sum(northings) / n


def ask_length() -> float:
    while True:
        try:
            val = float(input("  Run length in metres: ").strip())
            if val > 0:
                return val
            print("  Must be > 0.")
        except ValueError:
            print("  Enter a number.")


def print_header():
    print(f"\n  {'State':<12}  {'E':>9}  {'N':>9}  "
          f"{'Dist':>5}  {'Rem':>5}  {'Cmd':<8}  Note")
    print("  " + "-" * 65)


def print_row(result: dict):
    note = f"turn {result['turn_pct']:.0f}%" if result["state"] == STATE_TURN else ""
    print(f"  {result['state']:<12}  "
          f"{result['easting']:>9.3f}  {result['northing']:>9.3f}  "
          f"{result['dist']:>5.2f}  {result['remaining']:>5.2f}  "
          f"{result['command']:<8}  {note}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    print("\n" + "=" * 55)
    print("  Rover — Simple Path (single speed + timed turn)")
    print("=" * 55 + "\n")

    # Shared timestamp for both log files
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    # ------------------------------------------------------------------
    # Step 1: Arduino
    # ------------------------------------------------------------------
    print("[Main] Connecting to Arduino ...")
    try:
        ser = open_serial()
    except Exception as e:
        print(f"[Main] ERROR: {e}")
        sys.exit(1)

    time.sleep(ARDUINO_BOOT_WAIT)
    verify_arduino(ser)

    # ------------------------------------------------------------------
    # Step 2: GNSS
    # ------------------------------------------------------------------
    gnss = GNSSReader(port=GNSS_PORT, baud=GNSS_BAUD)
    try:
        gnss.start()
    except RuntimeError as e:
        print(f"[Main] GPS ERROR: {e}")
        all_stop(ser)
        ser.close()
        sys.exit(1)

    # ------------------------------------------------------------------
    # Step 3: Length input
    # ------------------------------------------------------------------
    print("\n[Main] Enter run length.")
    length_m = ask_length()
    print(f"\n  Will drive {length_m:.2f} m  →  turn {TURN_DURATION_SEC}s  "
          f"→  drive {length_m:.2f} m")

    # ------------------------------------------------------------------
    # Step 4: Record start fix
    # ------------------------------------------------------------------
    print("\n[Main] Stand rover at start position.")
    input("  Press ENTER to record start fix ...\n")
    start_fix = averaged_fix(gnss)
    print(f"  Start: E={start_fix[0]:.3f}  N={start_fix[1]:.3f}")

    # ------------------------------------------------------------------
    # Step 5: Build follower and loggers
    # ------------------------------------------------------------------
    follower = SimplePathFollower(length_m)
    follower.set_start(start_fix)

    track_log   = TrackLog(stamp)
    command_log = CommandLog(stamp)

    input("\n[Main] Press ENTER to start ...\n")

    # ------------------------------------------------------------------
    # Step 6: Control loop
    # ------------------------------------------------------------------
    print_header()
    seg2_started = False
    prev_command = None   # track command changes for command log

    try:
        while not follower.is_done():
            loop_start = time.time()

            # Drain Arduino serial (no encoders — just discard)
            while ser.in_waiting:
                echo = ser.readline().decode("ascii", errors="replace").strip()
                if echo.startswith("ERR"):
                    print(f"  [Arduino] {echo}")

            current_pos = gnss.get_position()
            info        = gnss.get_fix_info()

            # -- Pause after turn to record seg 2 start ----------------
            if follower.current_state() == STATE_SEG2 and not seg2_started:
                all_stop(ser)
                command_log.log(STATE_SEG2, "stop",
                                0, 0, 0,
                                "Turn complete — waiting for seg 2 start fix")
                print("\n[Main] Turn complete. Hold rover still.")
                input("  Press ENTER to record seg 2 start fix ...\n")
                seg2_fix = averaged_fix(gnss)
                follower.set_start(seg2_fix)
                seg2_started = True
                print_header()
                input("[Main] Press ENTER to start segment 2 ...\n")
                continue

            # -- Controller --------------------------------------------
            result  = follower.step(current_pos)
            command = result["command"]
            state   = result["state"]

            # -- Motor commands ----------------------------------------
            if command == "forward":
                all_forward(ser, arm=True)
                pwm_L, pwm_R, paint = PWM_BASE, PWM_BASE, 1
                reason = (f"driving forward — "
                          f"{result['dist']:.2f}/{length_m:.2f} m")

            elif command == "turn":
                turn_right(ser)
                pwm_L, pwm_R, paint = PWM_TURN, -PWM_TURN, 0
                reason = (f"point turn right — "
                          f"{result['turn_elapsed']:.1f}/{TURN_DURATION_SEC}s "
                          f"({result['turn_pct']:.0f}%)")

            else:  # stop (transition or done)
                all_stop(ser)
                pwm_L, pwm_R, paint = 0, 0, 0
                reason = f"transition or segment complete — state={state}"

            # -- Log every cycle ---------------------------------------
            track_log.log(result, info["hdop"])

            # Log command on every cycle AND whenever it changes
            if command != prev_command:
                command_log.log(state, command, pwm_L, pwm_R, paint,
                                f"COMMAND CHANGE: {reason}")
            else:
                command_log.log(state, command, pwm_L, pwm_R, paint, reason)
            prev_command = command

            # -- GPS quality warning -----------------------------------
            if info["hdop"] > 3.0:
                print(f"  [GPS] HDOP={info['hdop']:.1f} — fix quality poor")

            print_row(result)

            # -- Loop rate ---------------------------------------------
            elapsed = time.time() - loop_start
            sleep_t = LOOP_DT - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    except KeyboardInterrupt:
        print("\n[Main] Interrupted by user.")
        command_log.log(follower.current_state(), "stop",
                        0, 0, 0, "KeyboardInterrupt — operator stopped run")

    finally:
        print("\n[Main] Stopping motors ...")
        all_stop(ser)
        command_log.log(follower.current_state(), "stop",
                        0, 0, 0, "Shutdown — motors zeroed")
        ser.write(b"STOP\n")
        gnss.stop()
        track_log.close()
        command_log.close()
        ser.close()
        print(f"\n[Main] Logs saved to '{LOG_DIR}/'")
        print(f"         track_{stamp}.csv")
        print(f"         command_{stamp}.csv")
        print("[Main] Done.")


if __name__ == "__main__":
    main()
