"""
manual_waypoint_gen.py

Startup routine that:
  1. Reads an initial GPS fix from the GNSS receiver (Serial NMEA)
  2. Prompts the user for a line length (metres)
  3. Generates wp_1 (initial fix) and wp_2 (wp_1 + length along easting)
  4. Passes the waypoint list to PathPlanner

Coordinate convention:
    All positions in UTM (easting, northing) metres.
    wp_2 is offset along the +easting axis (UTM grid east).
    The user is responsible for physically orienting the rover
    along their desired "grid north" before confirming the fix.

Future expansion:
    Rectangle / lawnmower patterns can be added by extending
    generate_waypoints() — the interface to path_planning is unchanged.

Flow:
    manual_waypoint_gen  →  PathPlanner.load_waypoints()  →  PathFollower
"""

import serial
import time
import math

# ---------------------------------------------------------------------------
# GNSS serial config — match to your receiver
# ---------------------------------------------------------------------------
GNSS_PORT     = "COM5"   # adjust if GPS and Arduino share a hub
GNSS_BAUD     = 57600
GNSS_TIMEOUT  = 5                # seconds to wait for a valid fix

# Minimum number of consecutive fixes to average before accepting wp_1
NUM_FIX_AVERAGE = 5


# ---------------------------------------------------------------------------
# NMEA parsing
# ---------------------------------------------------------------------------

def _nmea_checksum_valid(sentence: str) -> bool:
    """Verify NMEA checksum (the *XX suffix)."""
    try:
        body, checksum = sentence.strip().lstrip("$").split("*")
        calc = 0
        for ch in body:
            calc ^= ord(ch)
        return calc == int(checksum, 16)
    except Exception:
        return False


def parse_gga(sentence: str):
    """
    Parse a $GPGGA or $GNGGA sentence.

    Returns (lat_dd, lon_dd, fix_quality, num_sats, hdop) or None if invalid.
    fix_quality: 0=no fix, 1=GPS, 2=DGPS, 4=RTK fixed, 5=RTK float
    """
    if not _nmea_checksum_valid(sentence):
        return None

    parts = sentence.strip().split(",")
    if len(parts) < 15:
        return None
    if not parts[0].endswith("GGA"):
        return None

    try:
        fix_quality = int(parts[6])
        if fix_quality == 0:
            return None                         # no fix

        # Latitude: DDMM.MMMM
        raw_lat  = parts[2]
        lat_dir  = parts[3]
        raw_lon  = parts[4]
        lon_dir  = parts[5]
        num_sats = int(parts[7])
        hdop     = float(parts[8]) if parts[8] else 99.0

        lat_deg = float(raw_lat[:2])
        lat_min = float(raw_lat[2:])
        lat_dd  = lat_deg + lat_min / 60.0
        if lat_dir == "S":
            lat_dd = -lat_dd

        lon_deg = float(raw_lon[:3])
        lon_min = float(raw_lon[3:])
        lon_dd  = lon_deg + lon_min / 60.0
        if lon_dir == "W":
            lon_dd = -lon_dd

        return lat_dd, lon_dd, fix_quality, num_sats, hdop

    except (ValueError, IndexError):
        return None


# ---------------------------------------------------------------------------
# WGS84 lat/lon  →  UTM (easting, northing)
# ---------------------------------------------------------------------------
# Minimal self-contained conversion — no pyproj dependency.
# Accurate to ~1 mm within a UTM zone, sufficient for field use.

def latlon_to_utm(lat_dd: float, lon_dd: float):
    """
    Convert WGS84 decimal degrees to UTM (easting, northing) in metres.

    Returns (easting, northing, zone_number, zone_letter).
    """
    a  = 6378137.0           # WGS84 semi-major axis
    f  = 1 / 298.257223563
    b  = a * (1 - f)
    e2 = 1 - (b / a) ** 2
    e  = math.sqrt(e2)
    n  = f / (2 - f)

    lat = math.radians(lat_dd)
    lon = math.radians(lon_dd)

    zone_number = int((lon_dd + 180) / 6) + 1
    lon0 = math.radians((zone_number - 1) * 6 - 180 + 3)   # central meridian

    N  = a / math.sqrt(1 - e2 * math.sin(lat) ** 2)
    T  = math.tan(lat) ** 2
    C  = (e2 / (1 - e2)) * math.cos(lat) ** 2
    A_ = math.cos(lat) * (lon - lon0)

    M = a * (
        (1 - e2/4 - 3*e2**2/64 - 5*e2**3/256) * lat
      - (3*e2/8 + 3*e2**2/32 + 45*e2**3/1024) * math.sin(2*lat)
      + (15*e2**2/256 + 45*e2**3/1024)         * math.sin(4*lat)
      - (35*e2**3/3072)                         * math.sin(6*lat)
    )

    k0 = 0.9996   # UTM scale factor
    easting  = k0 * N * (
        A_ + (1 - T + C) * A_**3 / 6
           + (5 - 18*T + T**2 + 72*C - 58*(e2/(1-e2))) * A_**5 / 120
    ) + 500000.0

    northing = k0 * (
        M + N * math.tan(lat) * (
            A_**2 / 2
          + (5 - T + 9*C + 4*C**2) * A_**4 / 24
          + (61 - 58*T + T**2 + 600*C - 330*(e2/(1-e2))) * A_**6 / 720
        )
    )
    if lat_dd < 0:
        northing += 10_000_000.0   # southern hemisphere offset

    # UTM zone letter
    letters = "CDEFGHJKLMNPQRSTUVWX"
    zone_letter = letters[int((lat_dd + 80) / 8)] if -80 <= lat_dd <= 84 else "?"

    return easting, northing, zone_number, zone_letter


# ---------------------------------------------------------------------------
# GNSS reader — averages multiple fixes for wp_1
# ---------------------------------------------------------------------------

def read_averaged_fix(port: str, baud: int, num_samples: int) -> tuple:
    """
    Open the GNSS serial port, collect `num_samples` valid GGA fixes,
    and return the averaged (easting, northing).

    Prints progress to terminal so the user can see fix quality.
    Raises RuntimeError if a valid fix cannot be obtained.
    """
    eastings  = []
    northings = []

    print(f"\n[GNSS] Opening {port} at {baud} baud ...")
    try:
        ser = serial.Serial(port, baud, timeout=GNSS_TIMEOUT)
    except serial.SerialException as e:
        raise RuntimeError(f"Cannot open GNSS port {port}: {e}")

    print(f"[GNSS] Waiting for {num_samples} valid fixes ...")

    try:
        while len(eastings) < num_samples:
            raw = ser.readline().decode("ascii", errors="replace").strip()
            if "GGA" not in raw:
                continue

            result = parse_gga(raw)
            if result is None:
                continue

            lat, lon, quality, sats, hdop = result
            e, n, zone_num, zone_let = latlon_to_utm(lat, lon)
            eastings.append(e)
            northings.append(n)

            print(f"  Fix {len(eastings):2d}/{num_samples} | "
                  f"UTM {zone_num}{zone_let} "
                  f"E={e:.3f} N={n:.3f} | "
                  f"Q={quality} sats={sats} HDOP={hdop:.1f}")
    finally:
        ser.close()

    avg_e = sum(eastings)  / len(eastings)
    avg_n = sum(northings) / len(northings)
    print(f"\n[GNSS] Averaged fix: E={avg_e:.3f}  N={avg_n:.3f}")
    return avg_e, avg_n


# ---------------------------------------------------------------------------
# Waypoint generation
# ---------------------------------------------------------------------------

def generate_waypoints(line_length_m: float,
                       wp1: tuple) -> list:
    """
    Generate a straight-line path along the +easting axis.

    Parameters
    ----------
    line_length_m : float
        Distance in metres from wp_1 to wp_2.
    wp1 : (easting, northing)
        Initial GPS fix in UTM metres.

    Returns
    -------
    list of (easting, northing) tuples : [wp_1, wp_2]

    Notes
    -----
    wp_2 is offset along +easting (UTM grid east).
    Orient the rover physically along the desired field direction
    before confirming the fix — this axis will later become the
    user-defined "grid north" when rectangle mode is added.

    Future extension
    ----------------
    For a rectangle, add width_m parameter and generate 4 corners:
        wp_3 = (wp1[0] + line_length_m, wp1[1] + width_m)
        wp_4 = (wp1[0],                 wp1[1] + width_m)
    """
    wp_1 = wp1
    wp_2 = (wp1[0] + line_length_m, wp1[1])   # +easting offset
    return [wp_1, wp_2]


# ---------------------------------------------------------------------------
# Startup routine — entry point
# ---------------------------------------------------------------------------

def run_startup(gnss=None) -> list:
    """
    Full startup sequence:
      1. Collect averaged GPS fix -> wp_1
      2. Prompt user for line length
      3. Generate and return waypoint list

    Parameters
    ----------
    gnss : GNSSReader instance, optional
        If provided (called from main.py), reuses the already-running GPS
        reader instead of opening a second serial connection.
        If None (run standalone), opens its own serial connection.

    Returns
    -------
    list of (easting, northing) tuples, ready for PathPlanner.load_waypoints()
    """
    print("=" * 55)
    print("  Rover Startup — Waypoint Generation")
    print("=" * 55)
    print("\nStep 1: Orient the rover along your desired line direction.")
    input("         Press ENTER when ready to record initial GPS fix ...\n")

    # Collect GPS fix — reuse live reader if available, else open own connection
    if gnss is not None:
        wp1 = gnss.get_position()
        print(f"[GNSS] Using live fix: E={wp1[0]:.3f}  N={wp1[1]:.3f}")
    else:
        wp1 = read_averaged_fix(GNSS_PORT, GNSS_BAUD, NUM_FIX_AVERAGE)

    print(f"\nStep 2: Enter the line length.")
    while True:
        try:
            length_str = input("  Line length in metres (e.g. 20): ").strip()
            line_length = float(length_str)
            if line_length <= 0:
                print("  Length must be greater than 0. Try again.")
                continue
            break
        except ValueError:
            print("  Invalid input — please enter a number.")

    waypoints = generate_waypoints(line_length, wp1)

    print(f"\n[WaypointGen] Generated {len(waypoints)} waypoints:")
    for i, wp in enumerate(waypoints, 1):
        print(f"  wp_{i}: E={wp[0]:.3f}  N={wp[1]:.3f}")

    return waypoints


# ---------------------------------------------------------------------------
# Example — full startup → path_planning → path_following handoff
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    from path_planning import PathPlanner

    waypoints = run_startup()

    planner = PathPlanner()
    planner.load_waypoints(waypoints)

    if planner.validate():
        print(f"\n[Startup] Total path: {planner.get_total_distance():.2f} m")
        print("[Startup] Handing off to PathFollower ...")

        # from path_following import PathFollower, open_serial, send_motor_commands
        # follower = PathFollower(planner.get_waypoints())
        # ser = open_serial()
        # prev_P = waypoints[0]
        # while not follower.is_finished():
        #     P = gnss_reader.get_position()
        #     result = follower.step(P, prev_P)
        #     prev_P = P
        #     send_motor_commands(ser, result["pwm_L"], result["pwm_R"], result["paint_arm"])
