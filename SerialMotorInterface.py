# motor_interface.py (serial version)
# Requires: pip install pyserial

import serial
import time
from dataclasses import dataclass

SYNC1 = 0xAA
SYNC2 = 0x55
PAYLOAD_LEN = 0x08  # seq + v(2) + w(2) + flags + reserved + reserved = 8 bytes

CRC_POLY = 0x1021
CRC_INIT = 0xFFFF

def crc16_ccitt(data: bytes, init: int = CRC_INIT) -> int:
    crc = init
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ CRC_POLY) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc

def clamp(x: int, lo: int, hi: int) -> int:
    return lo if x < lo else hi if x > hi else x

@dataclass
class SerialMotorInterface:
    port: str = "/dev/ttyACM0"
    baud: int = 115200
    timeout: float = 0.02

    def __post_init__(self):
        self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
        self.seq = 0
        # give Arduino a moment after opening serial
        time.sleep(1.5)

    def send_vw(self, v_cmd: int, w_cmd: int, estop: bool = False):
        """
        v_cmd, w_cmd: int in [-1000, 1000]
        estop: sets flag bit0
        """
        v_cmd = clamp(int(v_cmd), -1000, 1000)
        w_cmd = clamp(int(w_cmd), -1000, 1000)

        flags = 0x01 if estop else 0x00
        reserved1 = 0x00
        reserved2 = 0x00

        seq = self.seq & 0xFF
        self.seq = (self.seq + 1) & 0xFF

        # body = [seq][v L][v H][w L][w H][flags][r1][r2]
        body = bytearray()
        body.append(seq)
        body += int(v_cmd).to_bytes(2, "little", signed=True)
        body += int(w_cmd).to_bytes(2, "little", signed=True)
        body.append(flags)
        body.append(reserved1)
        body.append(reserved2)

        assert len(body) == PAYLOAD_LEN

        length_byte = bytes([PAYLOAD_LEN])

        # CRC over [len] + body
        crc_input = length_byte + bytes(body)
        crc = crc16_ccitt(crc_input)

        pkt = bytearray([SYNC1, SYNC2, PAYLOAD_LEN]) + body
        pkt += crc.to_bytes(2, "little", signed=False)

        self.ser.write(pkt)

    def stop(self):
        self.send_vw(0, 0, estop=False)

    def estop(self):
        self.send_vw(0, 0, estop=True)
