# rover/mission_builder.py

from dataclasses import dataclass
from typing import List, Tuple
import uuid

from path_planner import generate_waypoints


@dataclass
class FieldSpec:
    field_type: str
    length_m: float
    width_m: float
    paint_boxes: bool = True
    paint_center_circle: bool = True


@dataclass
class Mission:
    mission_id: str
    field_spec: FieldSpec
    waypoints: List[Tuple[float, float]]
    total_length_m: float


def validate_field_spec(spec: FieldSpec):
    if spec.length_m <= 0 or spec.width_m <= 0:
        raise ValueError("Invalid field dimensions")


def compute_total_length(waypoints: List[Tuple[float, float]]) -> float:
    total = 0.0
    for i in range(1, len(waypoints)):
        dx = waypoints[i][0] - waypoints[i - 1][0]
        dy = waypoints[i][1] - waypoints[i - 1][1]
        total += (dx**2 + dy**2) ** 0.5
    return total


def build_mission(spec: FieldSpec) -> Mission:
    validate_field_spec(spec)

    waypoints = generate_waypoints(spec)
    total_length = compute_total_length(waypoints)

    mission_id = str(uuid.uuid4())[:8]

    return Mission(
        mission_id=mission_id,
        field_spec=spec,
        waypoints=waypoints,
        total_length_m=total_length,
    )
