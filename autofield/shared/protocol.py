# shared/protocol.py
from __future__ import annotations
from dataclasses import asdict
from typing import Any, Dict, List, Tuple

from shared.datatypes import FieldDims, WaypointLLA, MissionPlan, RoverStatus, GNSSFix

# Messages are newline-delimited ASCII:
#   TYPE key=value key=value ...
# Example:
#   FIELD user=03 length=100.0 width=60.0
#   ORIGIN lat=39.95 lon=-75.19
#   WP i=0 lat=... lon=...
#   START dry=1
#   STATUS rtk=fix hacc=0.018 lat=... lon=... idx=3 fault=none

def encode_kv(msg_type: str, kv: Dict[str, Any]) -> bytes:
    parts = [msg_type]
    for k, v in kv.items():
        parts.append(f"{k}={v}")
    return (" ".join(parts) + "\n").encode("ascii", errors="ignore")

def decode_kv(line: str) -> Tuple[str, Dict[str, str]]:
    line = line.strip()
    if not line:
        return "", {}
    tokens = line.split()
    msg_type = tokens[0]
    kv: Dict[str, str] = {}
    for t in tokens[1:]:
        if "=" in t:
            k, v = t.split("=", 1)
            kv[k] = v
    return msg_type, kv

# -------- Mission plan (Base -> Rover) --------

def encode_plan_header(user_id: str, field: FieldDims) -> bytes:
    return encode_kv("FIELD", {"user": user_id, "length": field.length_m, "width": field.width_m})

def encode_origin(lat_deg: float, lon_deg: float) -> bytes:
    return encode_kv("ORIGIN", {"lat": lat_deg, "lon": lon_deg})

def encode_waypoint(i: int, wp: WaypointLLA) -> bytes:
    kv = {"i": i, "lat": wp.lat_deg, "lon": wp.lon_deg}
    if wp.alt_m is not None:
        kv["alt"] = wp.alt_m
    return encode_kv("WP", kv)

def encode_plan_end(count: int) -> bytes:
    return encode_kv("PLAN_END", {"n": count})

def encode_start(dry_run: bool) -> bytes:
    return encode_kv("START", {"dry": 1 if dry_run else 0})

def encode_stop() -> bytes:
    return encode_kv("STOP", {})

# -------- Status (Rover -> Base) --------

def encode_status(status: RoverStatus) -> bytes:
    kv: Dict[str, Any] = {"idx": status.waypoint_idx, "fault": status.fault or "none"}
    if status.gnss:
        kv["lat"] = status.gnss.lat_deg
        kv["lon"] = status.gnss.lon_deg
        kv["rtk"] = status.gnss.rtk
        if status.gnss.h_acc_m is not None:
            kv["hacc"] = status.gnss.h_acc_m
        if status.gnss.num_sats is not None:
            kv["sats"] = status.gnss.num_sats
    return encode_kv("STATUS", kv)

