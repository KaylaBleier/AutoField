import csv
import time
from datetime import datetime

from config_loader import CFG
from gnss_reader import GNSSReader
from path_following import open_serial, send_motor_commands


# ========================
# CONFIG
# ========================

GNSS_PORT = CFG["serial"]["gnss_port"]
GNSS_BAUD = CFG["serial"]["gnss_baud"]

LOG_DIR = CFG["logging"]["log_dir"]

# Movement parameters (EDIT THESE)
STRAIGHT_SPEED = 140
TURN_SPEED = 140

STRAIGHT_1_TIME = 5.0   # seconds
TURN_TIME = 2.0         # seconds (adjust for ~90°)
STRAIGHT_2_TIME = 5.0


# ========================
# GPS LOGGER
# ========================

def start_gps_logger():
    import os
    os.makedirs(LOG_DIR, exist_ok=True)

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = f"{LOG_DIR}/gps_log_{stamp}.csv"

    f = open(path, "w", newline="")
    writer = csv.writer(f)
    writer.writerow(["timestamp", "easting", "northing", "speed", "hdop"])

    print(f"[GPS] Logging → {path}")

    return f, writer


def log_gps(writer, gnss):
    P = gnss.get_position()
    info = gnss.get_fix_info()

    writer.writerow([
        datetime.now().isoformat(timespec="milliseconds"),
        round(P[0], 4),
        round(P[1], 4),
        round(info["speed_ms"], 3),
        round(info["hdop"], 2)
    ])


# ========================
# MAIN
# ========================

def main():
    print("\n=== SIMPLE ROVER RUN ===\n")

    # --------------------
    # Arduino
    # --------------------
    print("[Main] Connecting to Arduino...")
    ser = open_serial()
    time.sleep(2)

    ser.write(b"START\n")
    print("[Main] Sent START")

    # --------------------
    # GPS
    # --------------------
    gnss = GNSSReader(GNSS_PORT, GNSS_BAUD)
    gnss.start()

    gps_file, gps_writer = start_gps_logger()

    input("\nPress ENTER to start movement...\n")

    try:
        # ========================
        # STRAIGHT 1
        # ========================
        print("[Run] Straight 1")
        t0 = time.time()
        while time.time() - t0 < STRAIGHT_1_TIME:
            send_motor_commands(ser, STRAIGHT_SPEED, STRAIGHT_SPEED, False)
            log_gps(gps_writer, gnss)
            gps_file.flush()
            time.sleep(0.1)

        # ========================
        # TURN (in-place)
        # ========================
        print("[Run] Turning")

        t0 = time.time()
        while time.time() - t0 < TURN_TIME:
            # LEFT forward, RIGHT backward
            ser.write(f"L1:{TURN_SPEED},L2:{TURN_SPEED},R1:0,R2:0,ARM:0\n".encode())
            log_gps(gps_writer, gnss)
            gps_file.flush()
            time.sleep(0.1)

        # ========================
        # STRAIGHT 2
        # ========================
        print("[Run] Straight 2")

        t0 = time.time()
        while time.time() - t0 < STRAIGHT_2_TIME:
            send_motor_commands(ser, STRAIGHT_SPEED, STRAIGHT_SPEED, False)
            log_gps(gps_writer, gnss)
            gps_file.flush()
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n[Main] Interrupted")

    finally:
        print("[Main] Stopping")

        send_motor_commands(ser, 0, 0, False)
        ser.write(b"STOP\n")

        gnss.stop()
        gps_file.close()
        ser.close()

        print("[Main] Done")


if __name__ == "__main__":
    main()
