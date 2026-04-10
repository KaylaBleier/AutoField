import time
import math

from path_following import open_serial, send_motor_commands


# ========================
# CONSTANTS (FROM YOUR DATA)
# ========================

IN_TO_M = 0.0254

WHEEL_DIAMETER_M = 7.198 * IN_TO_M
TRACK_WIDTH_M    = 24.908 * IN_TO_M
SPEED_PWM100_MPS = 23.2 * IN_TO_M   # ≈ 0.589 m/s

# Motion commands
STRAIGHT_PWM = 100
TURN_PWM     = 100

# Targets
STRAIGHT_1_DIST = 5.0   # meters
STRAIGHT_2_DIST = 5.0
TURN_ANGLE_DEG  = 90


# ========================
# MODEL
# ========================

def pwm_to_speed(pwm):
    return SPEED_PWM100_MPS * (pwm / 100.0)


def straight_time(distance_m, pwm):
    v = pwm_to_speed(pwm)
    return distance_m / v


def turn_time(angle_deg, pwm):
    angle_rad = math.radians(angle_deg)
    v = pwm_to_speed(pwm)
    omega = v / TRACK_WIDTH_M
    return angle_rad / omega


# ========================
# MAIN
# ========================

def main():
    print("\n=== GEOMETRY-BASED RUN ===\n")

    ser = open_serial()
    time.sleep(2)
    ser.write(b"START\n")

    # Compute motion durations
    t1 = straight_time(STRAIGHT_1_DIST, STRAIGHT_PWM)
    tt = turn_time(TURN_ANGLE_DEG, TURN_PWM)
    t2 = straight_time(STRAIGHT_2_DIST, STRAIGHT_PWM)

    print(f"Straight1: {t1:.2f}s")
    print(f"Turn:      {tt:.2f}s")
    print(f"Straight2: {t2:.2f}s")

    input("\nPress ENTER to start...\n")

    try:
        # -----------------------
        # STRAIGHT 1
        # -----------------------
        print("[Run] Straight 1")
        t0 = time.time()
        while time.time() - t0 < t1:
            send_motor_commands(ser, STRAIGHT_PWM, STRAIGHT_PWM, False)
            time.sleep(0.02)

        # -----------------------
        # TURN
        # -----------------------
        print("[Run] Turning")
        t0 = time.time()
        while time.time() - t0 < tt:
            # pivot turn (left moves, right stopped)
            ser.write(f"L1:{TURN_PWM},L2:{TURN_PWM},R1:0,R2:0,ARM:0\n".encode())
            time.sleep(0.02)

        # -----------------------
        # STRAIGHT 2
        # -----------------------
        print("[Run] Straight 2")
        t0 = time.time()
        while time.time() - t0 < t2:
            send_motor_commands(ser, STRAIGHT_PWM, STRAIGHT_PWM, False)
            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\n[Main] Interrupted")

    finally:
        send_motor_commands(ser, 0, 0, False)
        ser.write(b"STOP\n")
        ser.close()
        print("[Main] Done")


if __name__ == "__main__":
    main()
