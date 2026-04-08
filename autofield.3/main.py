"""
main.py  (v3 — state machine: STRAIGHT_1 → TURNING → STRAIGHT_2 → DONE)

Startup sequence
----------------
1.  Open Arduino serial, wait for READY / send START.
2.  Start GNSSReader background thread, wait for first fix.
3.  Operator inputs segment 1 length and segment 2 length.
4.  Nudge fix for segment 1 → locks heading.
5.  Run segment 1 (straight, paint ON).
6.  Point turn (90° right, paint OFF). Rover waits after turn.
7.  Nudge fix for segment 2 → locks new heading.
8.  Run segment 2 (straight, paint ON).
9.  Stop, disarm Arduino.
"""

import math
import time
import sys

from gnss_reader    import GNSSReader
from path_following import (PathFollower, open_serial, send_motor_commands,
                             read_encoders, dist_m,
                             STATE_STRAIGHT_1, STATE_TURNING,
                             STATE_STRAIGHT_2, STATE_DONE)


# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------
LOOP_HZ           = 5
LOOP_DT           = 1.0 / LOOP_HZ
GNSS_PORT         = "COM9"
GNSS_BAUD         = 57600
ARDUINO_BOOT_WAIT = 2.0
NUDGE_SAMPLES     = 3      # GPS fixes averaged per nudge point


# ---------------------------------------------------------------------------
# Arduino handshake
# ---------------------------------------------------------------------------

def verify_arduino(ser, timeout=5.0) -> bool:
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
# Helpers
# ---------------------------------------------------------------------------

def averaged_fix(gnss: GNSSReader, n: int = NUDGE_SAMPLES) -> tuple:
    """Average n GPS fixes to reduce single-fix jitter."""
    eastings, northings = [], []
    for _ in range(n):
        e, no = gnss.get_position()
        eastings.append(e)
        northings.append(no)
        time.sleep(0.2)
    return sum(eastings) / n, sum(northings) / n


def ask_length(prompt: str) -> float:
    while True:
        try:
            val = float(input(prompt).strip())
            if val > 0:
                return val
            print("  Must be > 0.")
        except ValueError:
            print("  Enter a number.")


def print_header():
    print(f"\n  {'State':<12}  {'E':>9}  {'N':>9}  "
          f"{'Dist':>5}  {'Rem':>5}  "
          f"{'HdgGPS':>7}  {'HErr':>6}  "
          f"{'Drift':>6}  {'Corr':>7}  "
          f"{'vL':>6}  {'vR':>6}  ARM")
    print("  " + "-" * 90)


def print_telemetry(result: dict):
    arm_str  = "ON " if result["paint_arm"] else "OFF"
    state    = result["state"]

    if state == STATE_TURNING:
        prog = result.get("turn_progress_pct", 0.0)
        print(f"  {state:<12}  {'':>9}  {'':>9}  "
              f"{'':>5}  {'':>5}  "
              f"{'':>7}  {'':>6}  "
              f"{'':>6}  {'':>7}  "
              f"{'':>6}  {'':>6}  {arm_str}  "
              f"turn {prog:.0f}%")
    else:
        print(f"  {state:<12}  "
              f"{result['easting']:>9.3f}  {result['northing']:>9.3f}  "
              f"{result['dist_from_start']:>5.2f}  {result['dist_remaining']:>5.2f}  "
              f"{result['heading_deg']:>7.1f}  {result['heading_err_deg']:>+6.2f}  "
              f"{result['enc_drift']:>+6.0f}  {result['correction_ms']:>+7.4f}  "
              f"{result['v_L_ms']:>6.4f}  {result['v_R_ms']:>6.4f}  {arm_str}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    print("\n" + "=" * 55)
    print("  Rover v3 — Two-Segment Lawnmower")
    print("=" * 55 + "\n")

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
        send_motor_commands(ser, 0.0, 0.0, False)
        ser.close()
        sys.exit(1)

    # ------------------------------------------------------------------
    # Step 3: Segment lengths
    # ------------------------------------------------------------------
    print("\n[Main] Enter path dimensions.")
    seg1_length = ask_length("  Segment 1 length (metres): ")
    seg2_length = ask_length("  Segment 2 length (metres): ")
    print(f"\n  Seg 1: {seg1_length:.2f} m  →  90° right turn  →  "
          f"Seg 2: {seg2_length:.2f} m")

    # ------------------------------------------------------------------
    # Step 4: Build follower
    # ------------------------------------------------------------------
    follower = PathFollower(seg1_length, seg2_length)

    # ------------------------------------------------------------------
    # Step 5: Nudge fix for segment 1
    # ------------------------------------------------------------------
    print("\n[Main] SEGMENT 1 SETUP")
    print("  Stand rover at the start of segment 1 and hold still.")
    input("  Press ENTER to record fix_1 ...\n")
    fix1 = averaged_fix(gnss)
    print(f"  fix_1: E={fix1[0]:.3f}  N={fix1[1]:.3f}")

    print("\n  Nudge rover ~1 m forward along segment 1 direction, then stop.")
    input("  Press ENTER to record fix_2 ...\n")
    fix2 = averaged_fix(gnss)
    print(f"  fix_2: E={fix2[0]:.3f}  N={fix2[1]:.3f}")

    nudge_dist = dist_m(fix1, fix2)
    if nudge_dist < 0.3:
        print(f"  WARNING: nudge only {nudge_dist:.2f} m — heading may be "
              f"unreliable.")
        input("  Press ENTER to continue or Ctrl+C to abort ...\n")

    follower.set_segment_start(fix1)
    follower.set_target_heading(fix1, fix2)

    input("\n[Main] Press ENTER to start segment 1 ...\n")

    # ------------------------------------------------------------------
    # Control loop — handles all states
    # ------------------------------------------------------------------
    print_header()
    seg2_heading_set = False   # flag: nudge for seg 2 done yet?

    try:
        while not follower.is_finished():
            loop_start = time.time()

            # Encoders
            enc     = read_encoders(ser)
            ticks_L = enc[0] if enc else None
            ticks_R = enc[1] if enc else None

            # GPS
            current_pos = gnss.get_position()

            # ---- State-specific pre-step actions ----------------------

            # When we enter STRAIGHT_2 for the first time, pause and get
            # the nudge fix before letting the rover move.
            if follower.current_state() == STATE_STRAIGHT_2 \
                    and not seg2_heading_set:

                print("\n[Main] Turn complete. Rover is now facing segment 2.")
                print("  Hold rover still.")
                input("  Press ENTER to record seg-2 fix_1 ...\n")
                s2_fix1 = averaged_fix(gnss)
                print(f"  fix_1: E={s2_fix1[0]:.3f}  N={s2_fix1[1]:.3f}")

                print("\n  Nudge rover ~1 m forward along segment 2 direction.")
                input("  Press ENTER to record seg-2 fix_2 ...\n")
                s2_fix2 = averaged_fix(gnss)
                print(f"  fix_2: E={s2_fix2[0]:.3f}  N={s2_fix2[1]:.3f}")

                nudge2_dist = dist_m(s2_fix1, s2_fix2)
                if nudge2_dist < 0.3:
                    print(f"  WARNING: nudge only {nudge2_dist:.2f} m.")
                    input("  Press ENTER to continue or Ctrl+C to abort ...\n")

                follower.set_segment_start(s2_fix1)
                follower.set_target_heading(s2_fix1, s2_fix2)
                seg2_heading_set = True
                print_header()
                input("\n[Main] Press ENTER to start segment 2 ...\n")

            # ---- Controller ------------------------------------------
            result = follower.step(current_pos, ticks_L, ticks_R)

            # ---- Motor commands --------------------------------------
            send_motor_commands(ser,
                                result["v_L_ms"],
                                result["v_R_ms"],
                                result["paint_arm"])

            # ---- Arduino echo (errors, status) -----------------------
            if ser.in_waiting:
                echo = ser.readline().decode("ascii", errors="replace").strip()
                if echo.startswith("ERR"):
                    print(f"  [Arduino] {echo}")

            # ---- GPS quality warning ---------------------------------
            info = gnss.get_fix_info()
            if info["hdop"] > 3.0:
                print(f"  [GPS] HDOP={info['hdop']:.1f} — fix quality poor")

            # ---- Telemetry ------------------------------------------
            print_telemetry(result)

            # ---- Loop rate ------------------------------------------
            elapsed = time.time() - loop_start
            sleep_t = LOOP_DT - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    except KeyboardInterrupt:
        print("\n[Main] Interrupted by user.")

    finally:
        print("\n[Main] Stopping motors ...")
        send_motor_commands(ser, 0.0, 0.0, False)
        ser.write(b"STOP\n")
        gnss.stop()
        ser.close()
        print("[Main] Done.")


if __name__ == "__main__":
    main()
