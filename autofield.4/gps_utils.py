"""
gps_utils.py

Shared NMEA parsing and coordinate conversion utilities.
Imported by gnss_reader.py and waypoint_gen.py.

NOTE: gnss_reader.py has one import line to update:
    OLD: from manual_waypoint_gen import parse_gga, latlon_to_utm
    NEW: from gps_utils import parse_gga, latlon_to_utm
"""

import math


# ---------------------------------------------------------------------------
# NMEA checksum
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


# ---------------------------------------------------------------------------
# GGA parser
# ---------------------------------------------------------------------------

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
            return None  # no fix

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
# WGS84 lat/lon → UTM (easting, northing)
# ---------------------------------------------------------------------------

def latlon_to_utm(lat_dd: float, lon_dd: float):
    """
    Convert WGS84 decimal degrees to UTM (easting, northing) in metres.
    Returns (easting, northing, zone_number, zone_letter).
    Accurate to ~1 mm within a UTM zone — no external dependencies.
    """
    a  = 6378137.0
    f  = 1 / 298.257223563
    b  = a * (1 - f)
    e2 = 1 - (b / a) ** 2
    e  = math.sqrt(e2)
    n  = f / (2 - f)

    lat = math.radians(lat_dd)
    lon = math.radians(lon_dd)

    zone_number = int((lon_dd + 180) / 6) + 1
    lon0 = math.radians((zone_number - 1) * 6 - 180 + 3)

    N  = a / math.sqrt(1 - e2 * math.sin(lat) ** 2)
    T  = math.tan(lat) ** 2
    C  = (e2 / (1 - e2)) * math.cos(lat) ** 2
    A_ = math.cos(lat) * (lon - lon0)

    M = a * (
        (1 - e2/4 - 3*e2**2/64 - 5*e2**3/256)  * lat
      - (3*e2/8 + 3*e2**2/32 + 45*e2**3/1024)  * math.sin(2*lat)
      + (15*e2**2/256 + 45*e2**3/1024)          * math.sin(4*lat)
      - (35*e2**3/3072)                          * math.sin(6*lat)
    )

    k0 = 0.9996
    easting = k0 * N * (
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
        northing += 10_000_000.0

    letters = "CDEFGHJKLMNPQRSTUVWX"
    zone_letter = letters[int((lat_dd + 80) / 8)] if -80 <= lat_dd <= 84 else "?"

    return easting, northing, zone_number, zone_letter
