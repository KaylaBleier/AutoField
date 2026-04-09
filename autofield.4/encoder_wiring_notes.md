"""
HOW TO WIRE encoder_stop INTO main.py
======================================

This is not a standalone file — it shows the changes to make in main.py.
The GPS cross-product stop (path_following.py) and encoder stop run in
parallel. Whichever fires first wins.

1. Add this import near the top of main.py:

    from encoder_stop import EncoderStop

2. After waypoints are confirmed, create the encoder stop object:

    enc_stop = EncoderStop(wp1, wp2, run_length_m=dist)

3. Replace the control loop body with the version below.
   The key additions are:
     - read encoder pulses from Arduino echo line
     - call enc_stop.update() each tick
     - check enc_stop.should_stop() alongside follower.is_finished()


ARDUINO SIDE
============
Your Arduino needs to echo encoder pulse counts each loop so Python
can read them. Add a line like this to your Arduino sketch:

    Serial.print("ENC:");
    Serial.print(left_pulses_since_last);
    Serial.print(",");
    Serial.println(right_pulses_since_last);

Then reset your counters. Python reads this in the echo block below.


MODIFIED CONTROL LOOP (replace the while block in main.py)
===========================================================
"""

# --- paste this block into main.py, replacing the existing while loop ---

"""
    follower = PathFollower(wp1, wp2, heading_rad)
    enc_stop = EncoderStop(wp1, wp2, run_length_m=dist)
    prev_P   = gnss.get_position()

    print("[Main] Rover running. Press Ctrl+C to e-stop.\\n")

    try:
        while not follower.is_finished() and not enc_stop.should_stop():
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

            # --- Read Arduino echo (encoder pulses + any errors) ---
            left_pulses  = 0
            right_pulses = 0
            if ser.in_waiting:
                echo = ser.readline().decode("ascii", errors="replace").strip()
                if echo.startswith("ENC:"):
                    try:
                        parts = echo[4:].split(",")
                        left_pulses  = int(parts[0])
                        right_pulses = int(parts[1])
                    except (IndexError, ValueError):
                        pass
                elif echo.startswith("ERR"):
                    print(f"  [Arduino] {echo}")

            # Update encoder stop regardless of GPS
            enc_stop.update(left_pulses, right_pulses)
            enc_status = enc_stop.status()

            # Track log
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

            # Command log (now includes encoder data)
            command_w.writerow({
                "timestamp"        : now,
                "pwm_L"            : result["pwm_L"],
                "pwm_R"            : result["pwm_R"],
                "arm"              : 1 if result["paint_arm"] else 0,
                "heading_error_deg": round(result["heading_error"], 2),
                "lateral_error_m"  : round(result["lateral_error"], 4),
                "enc_dist_m"       : enc_status["dist_travelled_m"],
                "enc_progress"     : enc_status["progress"],
            })
            command_f.flush()

            # GPS fix quality warning
            info = gnss.get_fix_info()
            if info["hdop"] > HDOP_WARN:
                print(f"  [GPS] HDOP={info['hdop']:.1f} — fix quality poor")

            # Status line — shows both GPS progress and encoder progress
            arm_str = "ON " if result["paint_arm"] else "OFF"
            print(f"  L={result['pwm_L']:3d} R={result['pwm_R']:3d} | "
                  f"ARM={arm_str} | "
                  f"GPS_t={result['t']:.2f} "
                  f"ENC={enc_status['dist_travelled_m']:.2f}m "
                  f"({enc_status['progress']*100:.0f}%) "
                  f"HE={result['heading_error']:+.1f}°")

            if result["status"] == "finished":
                reason = result.get("stop_reason", "GPS cross-product")
                print(f"[Main] GPS stop fired — {reason}")
                break

            elapsed = time.time() - loop_start
            sleep_t = LOOP_DT - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

        # Check which stop condition fired
        if enc_stop.should_stop():
            print(f"[Main] Encoder stop fired — {enc_stop.stop_reason}")

    except KeyboardInterrupt:
        print("\\n[Main] E-STOP — interrupted by user.")
"""
