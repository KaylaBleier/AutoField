# base_station/ui/flow.py
from __future__ import annotations
from dataclasses import dataclass
from typing import Optional

from shared.datatypes import FieldDims

@dataclass
class BaseUIResult:
    user_id: str
    field: FieldDims

def prompt_user_id() -> str:
    while True:
        raw = input("Enter 2-digit user ID (00-99) or 'N' for new: ").strip()
        if raw.lower() == "n":
            # new -> pick next available later; for now ask user
            raw = input("Enter new 2-digit ID to use (00-99): ").strip()
        if raw.isdigit() and len(raw) == 2:
            return raw
        print("Invalid. Must be two digits like 03, or N for new.")

def prompt_float(prompt: str, min_val: float = 0.1, max_val: float = 500.0) -> float:
    while True:
        raw = input(prompt).strip()
        try:
            v = float(raw)
            if not (min_val <= v <= max_val):
                raise ValueError
            return v
        except Exception:
            print(f"Invalid. Enter a number between {min_val} and {max_val}.")

def prompt_confirm(msg: str) -> bool:
    raw = input(f"{msg} (y/n): ").strip().lower()
    return raw.startswith("y")

def run_base_input_flow() -> Optional[BaseUIResult]:
    print("\n=== AutoField Base Station ===\n")
    user_id = prompt_user_id()

    length = prompt_float("Enter field LENGTH (m): ")
    width = prompt_float("Enter field WIDTH  (m): ")

    # Backend rule: shorter distance = width
    L = max(length, width)
    W = min(length, width)

    print(f"\nCaptured dims. Interpreting: length={L:.2f} m, width={W:.2f} m (width is shorter side).")

    ok = prompt_confirm(
        "\nPlace rover directly under base station and position both at the field origin (0,0). "
        "Field is assumed in +x,+y (1st quadrant). Confirm when ready"
    )
    if not ok:
        print("Cancelled by user.")
        return None

    return BaseUIResult(user_id=user_id, field=FieldDims(length_m=L, width_m=W))
