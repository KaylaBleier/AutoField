# rover/comms/radio_link.py
from __future__ import annotations
import queue
import threading
from typing import Optional

import serial

class SerialLineLink:
    """
    Newline-delimited line transport over a serial port (SiK/3DR appears as COM port / tty).
    """

    def __init__(self, port: str, baud: int = 115200, timeout_s: float = 0.1):
        self.port = port
        self.baud = baud
        self.timeout_s = timeout_s
        self._ser: Optional[serial.Serial] = None
        self._rx_thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._rxq: "queue.Queue[str]" = queue.Queue()

    def open(self) -> None:
        self._ser = serial.Serial(self.port, self.baud, timeout=self.timeout_s)
        self._stop.clear()
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

    def close(self) -> None:
        self._stop.set()
        if self._rx_thread:
            self._rx_thread.join(timeout=1.0)
        if self._ser:
            self._ser.close()
        self._ser = None

    def send_line(self, line: bytes) -> None:
        if not self._ser:
            raise RuntimeError("Link not open.")
        self._ser.write(line)

    def recv_line(self, timeout_s: float = 0.0) -> Optional[str]:
        try:
            return self._rxq.get(timeout=timeout_s)
        except queue.Empty:
            return None

    def _rx_loop(self) -> None:
        assert self._ser is not None
        buf = b""
        while not self._stop.is_set():
            try:
                chunk = self._ser.read(256)
            except Exception:
                continue
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                try:
                    self._rxq.put(line.decode("ascii", errors="ignore").strip())
                except Exception:
                    pass

