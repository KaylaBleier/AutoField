# rover/gnss/ubx_reader.py
from __future__ import annotations
import time
from typing import Optional

import serial

try:
    from pyubx2 import UBXReader
except ImportError as e:
    raise ImportError(
        "pyubx2 is required for UBX parsing. Install with: pip install pyubx2"
    ) from e

from shared.datatypes import GNSSFix

def _rtk_from_flags(carr_soln: int) -> str:
    # carrSoln: 0 = no carrier solution, 1 = float, 2 = fixed
    if carr_soln == 2:
        return "fix"
    if carr_soln == 1:
        return "float"
    return "none"

class UBXNavPVTReader:
    """
    Reads UBX NAV-PVT messages from a ZED-F9P over serial and returns GNSSFix.
    """

    def __init__(self, port: str, baud: int = 115200, timeout_s: float = 1.0):
        self.port = port
        self.baud = baud
        self.timeout_s = timeout_s
        self._ser: Optional[serial.Serial] = None
        self._ubr: Optional[UBXReader] = None

    def open(self) -> None:
        self._ser = serial.Serial(self.port, self.baud, timeout=self.timeout_s)
        self._ubr = UBXReader(self._ser, protfilter=2)  # 2=UBX only

    def close(self) -> None:
        if self._ser:
            self._ser.close()
        self._ser = None
        self._ubr = None

    def read_fix_blocking(self) -> Optional[GNSSFix]:
        """
        Blocks until a NAV-PVT arrives or timeout. Returns None if nothing valid.
        """
        if not self._ubr:
            raise RuntimeError("UBX reader not open().")

        try:
            raw, parsed = self._ubr.read()
        except Exception:
            return None

        if not parsed:
            return None

        if getattr(parsed, "identity", "") != "NAV-PVT":
            return None

        # NAV-PVT lat/lon are in 1e-7 degrees
        lat = getattr(parsed, "lat", None)
        lon = getattr(parsed, "lon", None)
        if lat is None or lon is None:
            return None

        lat_deg = lat / 1e7
        lon_deg = lon / 1e7

        # hAcc is in mm
        hacc_mm = getattr(parsed, "hAcc", None)
        hacc_m = (hacc_mm / 1000.0) if isinstance(hacc_mm, (int, float)) else None

        # flags2 has carrSoln bits (carrier solution)
        flags2 = getattr(parsed, "flags2", 0)
        carr_soln = flags2 & 0b11
        rtk = _rtk_from_flags(carr_soln)

        num_sats = getattr(parsed, "numSV", None)

        return GNSSFix(
            lat_deg=lat_deg,
            lon_deg=lon_deg,
            h_acc_m=hacc_m,
            rtk=rtk,  # "none"|"float"|"fix"
            num_sats=int(num_sats) if num_sats is not None else None,
        )
