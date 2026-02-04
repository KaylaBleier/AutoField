# rover/comms/app_radio.py
from __future__ import annotations

import serial
import time
from typing import Optional, Tuple, Dict

from shared.protocol import decode_kv

class AppRadioReceiver:
    """
    Simple newline-delimited receiver for app-data radio (NOT RTCM).
    Use with a separate radio pair from RTK radios.
    """

    def __init__(self, port: str, baud: int = 115200, timeout_s: float = 0.1):
        self.ser = serial.Serial(port, baud, timeout=timeout_s)

    def read_message(self) -> Optional[Tuple[str, Dict[str, str]]]:
        """
        Returns (msg_type, kv_dict) or None if no full line.
        """
        try:
            line = self.ser.readline()
        except Exception:
            return None

        if not line:
            return None

        s = line.decode("ascii", errors="ignore").strip()
        if not s:
            return None

        mtype, kv = decode_kv(s)
        if not mtype:
            return None
        return mtype, kv

    def close(self) -> None:
        try:
            self.ser.close()
        except Exception:
            pass
