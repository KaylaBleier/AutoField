# base_station/main.py
from __future__ import annotations
import time

from shared.protocol import (
    encode_plan_header,
    encode_origin,
    encode_waypoint,
    encode_plan_end,
    encode_start,
    decode_kv,
)
from shared.datatypes import MissionPlan, RoverStatus, GNSSFix
from base_station.ui.flow import run_base_input_flow, prompt_confirm
from base_station.comms.radio_link import SerialLineLink
from base_station.planning.waypoint_adapter import sanity_check_map_overlay, generate_waypoints_ll

# ---- CONFIG (swap later to yaml) ----
RADIO_PORT = "COM7"      # change for your machine
RADIO_BAUD = 115200
# ------------------------------------

def main() -> None:
    ui = run_base_input_flow()
    if ui is None:
        return

    # TODO: replace with real base GNSS fix.
    # For now, you can manually enter origin or set dummy.
    origin_lat = float(input("\nEnter base origin LAT (deg) (or dummy): ").strip())
    origin_lon = float(input("Enter base origin LON (deg) (or dummy): ").strip())

    # sanity check overlay
    if not sanity_check_map_overlay(origin_lat, origin_lon, ui.field):
        print("Map overlay sanity check failed (field hits obstacle). Aborting.")
        return

    print("\nGenerating waypoint plan (stub until partner code arrives)...")
    wps = generate_waypoints_ll(origin_lat, origin_lon, ui.field)

    print(f"Plan ready: {len(wps)} waypoints.")
    if not prompt_confirm("Proceed to transmit plan to rover?"):
        return

    link = SerialLineLink(RADIO_PORT, RADIO_BAUD)
    link.open()
    try:
        # Transmit plan
        link.send_line(encode_plan_header(ui.user_id, ui.field))
        link.send_line(encode_origin(origin_lat, origin_lon))
        for i, wp in enumerate(wps):
            link.send_line(encode_waypoint(i, wp))
        link.send_line(encode_plan_end(len(wps)))

        print("Plan sent. Waiting briefly for rover STATUS...")
        t0 = time.time()
        while time.time() - t0 < 5.0:
            line = link.recv_line(timeout_s=0.2)
            if not line:
                continue
            mtype, kv = decode_kv(line)
            if mtype == "STATUS":
                print("ROVER:", line)

        if prompt_confirm("\nReady for DRY RUN start?"):
            link.send_line(encode_start(dry_run=True))
            print("Sent START dry=1. (Rover should begin dry run when implemented.)")
        else:
            print("Not starting.")

        print("\n(You can add a live status loop here next.)")

    finally:
        link.close()

if __name__ == "__main__":
    main()
