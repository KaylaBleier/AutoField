# rover/main.py
from __future__ import annotations
import time
from typing import List, Optional

from rover.comms.radio_link import SerialLineLink
from rover.gnss.ubx_reader import UBXNavPVTReader
from shared.protocol import decode_kv, encode_status
from shared.datatypes import FieldDims, WaypointLLA, RoverStatus, GNSSFix

# ---- CONFIG (swap later to yaml) ----
RADIO_PORT = "COM8"      # rover radio appears as a serial port on laptop; later /dev/tty...
RADIO_BAUD = 115200

GNSS_PORT = "COM9"       # ZED-F9P serial/USB
GNSS_BAUD = 115200
# ------------------------------------

class PlanBuffer:
    def __init__(self) -> None:
        self.user_id: Optional[str] = None
        self.field: Optional[FieldDims] = None
        self.origin_lat: Optional[float] = None
        self.origin_lon: Optional[float] = None
        self.waypoints: List[WaypointLLA] = []
        self.expected_n: Optional[int] = None

    def complete(self) -> bool:
        return (
            self.user_id is not None
            and self.field is not None
            and self.origin_lat is not None
            and self.origin_lon is not None
            and self.expected_n is not None
            and len(self.waypoints) == self.expected_n
        )

def main() -> None:
    status = RoverStatus()

    # Start GNSS reader
    gnss = UBXNavPVTReader(GNSS_PORT, GNSS_BAUD)
    gnss.open()

    # Start radio link
    radio = SerialLineLink(RADIO_PORT, RADIO_BAUD)
    radio.open()

    plan = PlanBuffer()
    last_status_tx = 0.0

    print("Rover running. Waiting for plan...")

    try:
        while True:
            # GNSS update (non-busy)
            fix = gnss.read_fix_blocking()
            if fix:
                status.gnss = fix

            # Receive base messages
            line = radio.recv_line(timeout_s=0.01)
            if line:
                mtype, kv = decode_kv(line)

                if mtype == "FIELD":
                    plan.user_id = kv.get("user")
                    plan.field = FieldDims(float(kv["length"]), float(kv["width"]))
                    plan.waypoints = []
                    plan.expected_n = None
                    print("Got FIELD", plan.user_id, plan.field)

                elif mtype == "ORIGIN":
                    plan.origin_lat = float(kv["lat"])
                    plan.origin_lon = float(kv["lon"])
                    print("Got ORIGIN", plan.origin_lat, plan.origin_lon)

                elif mtype == "WP":
                    i = int(kv["i"])
                    lat = float(kv["lat"])
                    lon = float(kv["lon"])
                    plan.waypoints.append(WaypointLLA(lat, lon))
                    # optional: enforce i ordering later
                    # print(f"Got WP {i}")

                elif mtype == "PLAN_END":
                    plan.expected_n = int(kv["n"])
                    print("Got PLAN_END n=", plan.expected_n)
                    if plan.complete():
                        print("Plan complete ✅")
                    else:
                        print("Plan incomplete ❌ (waiting for missing fields/waypoints)")

                elif mtype == "START":
                    dry = kv.get("dry", "1") == "1"
                    if not plan.complete():
                        status.fault = "start_without_complete_plan"
                        print("FAULT: start without complete plan")
                    else:
                        status.fault = None
                        print(f"START received. dry={dry}. (Driving loop not implemented yet.)")
                        # TODO: hand plan to navigation + esp32 command stream

                elif mtype == "STOP":
                    print("STOP received. (TODO: command motors stop)")
                    # TODO: esp32_link.stop()

            # Periodically send STATUS back to base
            now = time.time()
            if now - last_status_tx > 0.2:
                radio.send_line(encode_status(status))
                last_status_tx = now

    finally:
        radio.close()
        gnss.close()

if __name__ == "__main__":
    main()
