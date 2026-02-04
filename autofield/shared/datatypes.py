# shared/datatypes.py
from __future__ import annotations
from dataclasses import dataclass
from typing import List, Optional, Literal

RtkStatus = Literal["none", "float", "fix"]

@dataclass
class FieldDims:
    length_m: float
    width_m: float

@dataclass
class WaypointLLA:
    lat_deg: float
    lon_deg: float
    alt_m: Optional[float] = None

@dataclass
class MissionPlan:
    user_id: str
    field: FieldDims
    origin_lat_deg: float
    origin_lon_deg: float
    waypoints: List[WaypointLLA]
    dry_run: bool = True

@dataclass
class GNSSFix:
    lat_deg: float
    lon_deg: float
    h_acc_m: Optional[float] = None
    rtk: RtkStatus = "none"
    num_sats: Optional[int] = None

@dataclass
class RoverStatus:
    gnss: Optional[GNSSFix] = None
    waypoint_idx: int = 0
    fault: Optional[str] = None
