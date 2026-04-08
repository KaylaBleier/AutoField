"""
main.py  (v5 — single speed, GPS stop, timed turn)

Startup sequence
----------------
1.  Open Arduino serial, wait for READY / send START.
2.  Start GNSSReader background thread, wait for first fix.
3.  Operator inputs segment 1 and segment 2 lengths.
4.  Nudge fix → locks display heading for segment 1.
5.  Run segment 1: all four wheels at BASE_SPEED, GPS stop trigger.
6.  Point turn: left forward, right reverse for TURN_DURATION_SEC.
7.  Nudge fix → locks display heading for segment 2.
8.  Run segment 2: all four wheels at BASE_SPEED, GPS stop trigger.
9.  Stop, disarm Arduino.
"""

import time
import sys

from gnss_reader    import GNSSReader
from path_following import (PathFollower, open_serial,
                             send_motor_commands, send_turn_command,
                             dist_m,
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
NUDGE_SAMPLES     = 3


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


def do_nudge(gnss: GNSSReader, label: str) -> tuple:
    """
    Walk operator through recording two fixes for a heading nudge.
    Returns (fix1, fix2).
    """
    print(f"\n[Main] {label} — stand rover at start, hold still.")
    input("  Press ENTER to record fix_1 ...\n")
    fix1 = averaged_fix(gnss)
    print(f"  fix_1: E={fix1[0]:.3f}  N={fix1[1]:.3f}")

    print("\n  Nudge rover ~1 m forward along desired direction, then stop.")
    input("  Press ENTER to record fix_2 ...\n")
    fix2 = averaged_fix(gnss)
    print(f"  fix_2: E={fix2[0]:.3f}  N={fix2[1]:.3f}")

    d = dist_m(fix1, fix2)
    if d < 0.3:
        print(f"  WARNING: nudge only {d:.2f} m — heading reference may be "
              f"unreliable. Consider nudging further.")
        input("  Press ENTER to continue or Ctrl+C to abort ...\n")

    return fix1, fix2


def print_header():
    print(f"\n  {'State':<12}  {'E':>9}  {'N':>9}  "
          f"{'Dist':>5}  {'Rem':>5}  "
          f"{'HdgGPS':>7}  {'HdgRef':>7}  "
          f"{'PWM':>4}  ARM  Note")
    print("  " + "-" * 80)


def print_telemetry(result: dict):
    arm  = "ON " if result["paint_arm"] else "OFF"
    state = result["state"]

    if state == STATE_TURNING:
        note = f"turn {result['turn_progress_pct']:.0f}%"
        pwm  = result["pwm_turn"]
    else:
        note = ""
        pwm  = result["pwm_all"]

    print(f"  {state:<12}  "
          f"{result['easting']:>9.3f}  {result['northing']:>9.3f}  "
          f"{result['dist_from_start']:>5.2f}  {result['dist_remaining']:>5.2f}  "
          f"{result['heading_deg']:>7.1f}  {result['heading_ref_deg']:>7.1f}  "
          f"{pwm:>4}  {arm}  {note}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    print("\n" + "=" * 55)
    print("  Rover v5 — Single Speed, GPS Stop, Timed Turn")
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
        send_motor_commands(ser, 0, False)
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
    # Step 5: Segment 1 nudge
    # ------------------------------------------------------------------
    fix1, fix2 = do_nudge(gnss, "SEGMENT 1 HEADING")
    follower.set_segment_start(fix1)
    follower.set_target_heading(fix1, fix2)

    input("\n[Main] Press ENTER to start segment 1 ...\n")

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------
    print_header()
    seg2_ready = False

    try:
        while not follower.is_finished():
            loop_start = time.time()

            # Drain any serial from Arduino (errors, status lines)
            # No encoder reads needed
            while ser.in_waiting:
                echo = ser.readline().decode("ascii", errors="replace").strip()
                if echo.startswith("ERR"):
                    print(f"  [Arduino] {echo}")

            current_pos = gnss.get_position()
            result      = follower.step(current_pos)
            state       = result["state"]

            # ----------------------------------------------------------
            # Pause after turn completes to collect seg 2 nudge
            # ----------------------------------------------------------
            if state == STATE_STRAIGHT_2 and not seg2_ready:
                # Motors already zeroed by transition — rover is stopped
                print("\n[Main] Turn complete. Rover facing segment 2.")
                fix1_s2, fix2_s2 = do_nudge(gnss, "SEGMENT 2 HEADING")
                follower.set_segment_start(fix1_s2)
                follower.set_target_heading(fix1_s2, fix2_s2)
                seg2_ready = True
                print_header()
                input("\n[Main] Press ENTER to start segment 2 ...\n")
                continue   # re-run step() now that heading is set

            # ----------------------------------------------------------
            # Motor commands
            # ----------------------------------------------------------
            if state == STATE_TURNING:
                send_turn_command(ser,
                                  result["pwm_turn"],
                                  result["pwm_turn"],
                                  False)
            else:
                send_motor_commands(ser,
                                    result["pwm_all"],
                                    result["paint_arm"])

            # GPS fix quality warning
            info = gnss.get_fix_info()
            if info["hdop"] > 3.0:
                print(f"  [GPS] HDOP={info['hdop']:.1f} — fix quality poor")

            print_telemetry(result)

            # Loop rate
            elapsed = time.time() - loop_start
            sleep_t = LOOP_DT - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    except KeyboardInterrupt:
        print("\n[Main] Interrupted by user.")

    finally:
        print("\n[Main] Stopping motors ...")
        send_motor_commands(ser, 0, False)
        ser.write(b"STOP\n")
        gnss.stop()
        ser.close()
        print("[Main] Done.")


if __name__ == "__main__":
    main()
