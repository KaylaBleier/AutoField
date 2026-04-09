"""
gnss_reader.py

Continuous NMEA GPS reader for the path-following control loop.

Runs a background thread that constantly reads GGA sentences from the
GPS serial port and caches the latest valid fix. The control loop calls
get_position() at any time and gets the most recent (easting, northing).

Usage:
    from gnss_reader import GNSSReader

    reader = GNSSReader(port="/dev/ttyUSB1", baud=9600)
    reader.start()

    easting, northing = reader.get_position()
    speed             = reader.get_speed()   # m/s, derived from GPS positions

    reader.stop()
"""

import serial
import threading
import time
import math


# ---------------------------------------------------------------------------
# Re-use the parsers already written in manual_waypoint_gen
# ---------------------------------------------------------------------------
from gps_utils import parse_gga, latlon_to_utm


# ---------------------------------------------------------------------------
# GNSSReader
# ---------------------------------------------------------------------------

class GNSSReader:
    def __init__(self, port: str, baud: int = 9600, timeout: float = 5.0):
        """
        Parameters
        ----------
        port    : serial port the GPS is on (e.g. "/dev/ttyUSB1")
        baud    : baud rate (default 9600 for most NMEA receivers)
        timeout : seconds before get_position() raises if no fix seen
        """
        self.port    = port
        self.baud    = baud
        self.timeout = timeout

        self._lock         = threading.Lock()
        self._latest       = None    # (easting, northing)
        self._prev         = None    # previous fix, for speed calc
        self._prev_time    = None    # timestamp of previous fix
        self._speed        = 0.0     # m/s derived from consecutive fixes
        self._fix_quality  = 0
        self._num_sats     = 0
        self._hdop         = 99.0
        self._running      = False
        self._thread       = None
        self._ser          = None

    # ------------------------------------------------------------------
    # Start / stop
    # ------------------------------------------------------------------

    def start(self):
        """Open serial port and begin background reading thread."""
        print(f"[GNSSReader] Opening {self.port} at {self.baud} baud ...")
        try:
            self._ser = serial.Serial(self.port, self.baud, timeout=1)
        except serial.SerialException as e:
            raise RuntimeError(f"Cannot open GPS port {self.port}: {e}")

        self._running = True
        self._thread  = threading.Thread(target=self._read_loop,
                                         daemon=True, name="gnss-reader")
        self._thread.start()

        # Block until we have at least one valid fix (up to self.timeout)
        print("[GNSSReader] Waiting for first fix ...")
        deadline = time.time() + self.timeout
        while time.time() < deadline:
            with self._lock:
                if self._latest is not None:
                    e, n = self._latest
                    print(f"[GNSSReader] First fix: E={e:.3f}  N={n:.3f}  "
                          f"sats={self._num_sats}  HDOP={self._hdop:.1f}")
                    return
            time.sleep(0.05)

        raise RuntimeError(
            "[GNSSReader] Timed out waiting for a valid GPS fix. "
            "Check cable, port, and antenna."
        )

    def stop(self):
        """Stop the background thread and close the serial port."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        if self._ser and self._ser.is_open:
            self._ser.close()
        print("[GNSSReader] Stopped.")

    # ------------------------------------------------------------------
    # Background read loop
    # ------------------------------------------------------------------

    def _read_loop(self):
        """Runs in a daemon thread — reads GGA sentences continuously."""
        while self._running:
            try:
                raw = self._ser.readline().decode("ascii", errors="replace")
            except serial.SerialException:
                break   # port closed or disconnected

            raw = raw.strip()
            if "GGA" not in raw:
                continue

            result = parse_gga(raw)
            if result is None:
                continue

            lat, lon, quality, sats, hdop = result
            easting, northing, _, _ = latlon_to_utm(lat, lon)
            now = time.time()

            with self._lock:
                # Speed from consecutive fixes
                if self._latest is not None and self._prev_time is not None:
                    dt = now - self._prev_time
                    if dt > 0:
                        prev_e, prev_n = self._latest
                        dist = math.sqrt((easting  - prev_e) ** 2 +
                                         (northing - prev_n) ** 2)
                        # Low-pass filter to smooth GPS noise
                        raw_speed    = dist / dt
                        self._speed  = 0.7 * self._speed + 0.3 * raw_speed

                self._prev         = self._latest
                self._prev_time    = now
                self._latest       = (easting, northing)
                self._fix_quality  = quality
                self._num_sats     = sats
                self._hdop         = hdop

    # ------------------------------------------------------------------
    # Public getters — safe to call from any thread
    # ------------------------------------------------------------------

    def get_position(self) -> tuple:
        """
        Return the latest (easting, northing) in UTM metres.
        Raises RuntimeError if no fix has been received yet.
        """
        with self._lock:
            if self._latest is None:
                raise RuntimeError("No GPS fix available yet.")
            return self._latest

    def get_speed(self) -> float:
        """Return estimated speed in m/s derived from consecutive fixes."""
        with self._lock:
            return self._speed

    def get_fix_info(self) -> dict:
        """Return latest fix metadata — useful for logging."""
        with self._lock:
            return {
                "fix_quality" : self._fix_quality,
                "num_sats"    : self._num_sats,
                "hdop"        : self._hdop,
                "speed_ms"    : self._speed,
                "position"    : self._latest,
            }

    def has_fix(self) -> bool:
        """True if at least one valid fix has been received."""
        with self._lock:
            return self._latest is not None
