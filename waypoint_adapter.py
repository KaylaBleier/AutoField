# base_station/planning/waypoint_adapter.py
from __future__ import annotations
from typing import List, Tuple

from shared.datatypes import FieldDims, WaypointLLA

def sanity_check_map_overlay(origin_lat: float, origin_lon: float, field: FieldDims) -> bool:
    """
    Stub: later you’ll call GIS/building overlay check.
    For now, always returns True.
    """
    return True

def generate_waypoints_ll(origin_lat: float, origin_lon: float, field: FieldDims) -> List[WaypointLLA]:
    """
    Stub: partner code will create full waypoint list.
    For now, create a tiny “rectangle-ish” 4-point plan around origin.
    """
    # NOTE: This is NOT correct geodesy. It's a placeholder only.
    # Replace with mapping code soon.
    d = 0.00005  # ~5m-ish in lat
    return [
        WaypointLLA(origin_lat, origin_lon),
        WaypointLLA(origin_lat + d, origin_lon),
        WaypointLLA(origin_lat + d, origin_lon + d),
        WaypointLLA(origin_lat, origin_lon + d),
    ]
