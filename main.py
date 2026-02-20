# main.py
import json
import os
import csv
import math
from datetime import datetime

from test_ui_input import get_field_spec_cli
from path_planner import generate_soccer_perimeter_270
from motor_interface import build_motor_interface


def load_config(path="config.yaml"):
    # Minimal YAML-free loader: if you already parse YAML elsewhere, swap this.
    # For now, we’ll default terminal mode even if config is missing.
    cfg = {
        "motor_mode": "terminal",
        "turn_gain": 1.0,
        "print_hz": 10,
    }
    if os.path.isfile(path):
        try:
            import yaml
            with open(path, "r", encoding="utf-8") as f:
                y = yaml.safe_load(f) or {}
            cfg.update(y)
        except Exception as e:
            print(f"[main] Config read warning ({path}): {e}")
    return cfg


def save_waypoints_csv(waypoints, out_path):
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["idx", "x_m", "y_m", "paint_on"])
        for i, (x, y, paint) in enumerate(waypoints):
            w.writerow([i, x, y, int(bool(paint))])


def save_mission_json(mission, out_path):
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(mission, f, indent=2)


def wrap_pi(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def dry_run_motor_outputs(motors, waypoints, v_cmd=450, k_turn=700, print_every=1):
    """
    Laptop-only: turn waypoint geometry into (v,w) commands and print via TerminalMotorInterface.
    This is a “fake follower” to prove integration. No GNSS required.

    v_cmd: constant forward command (0..1000)
    k_turn: gain mapping heading error (rad) -> w_cmd
    """
    if not waypoints:
        print("[dryrun] No waypoints.")
        return

    # Simulated pose starts at first waypoint
    x, y, _paint = waypoints[0]
    heading = 0.0

    print("\n[dryrun] Streaming motor commands from planned path...\n")
    for i in range(1, len(waypoints)):
        gx, gy, paint = waypoints[i]

        dx = gx - x
        dy = gy - y
        dist = math.hypot(dx, dy)

        # Skip tiny steps
        if dist < 1e-6:
            continue

        desired = math.atan2(dy, dx)
        err = wrap_pi(desired - heading)

        w_cmd = int(max(-1000, min(1000, k_turn * err)))  # proportional
        v_out = v_cmd

        # Slow down if we need to turn hard
        if abs(w_cmd) > 600:
            v_out = int(v_cmd * 0.4)
        elif abs(w_cmd) > 300:
            v_out = int(v_cmd * 0.7)

        # “Paint” flag not wired yet (fine for today)
        motors.send_vw(v_out, w_cmd)

        # crude simulate moving along this segment
        heading = desired
        x, y = gx, gy

        if print_every and (i % print_every == 0):
            print(f"[dryrun] wp={i}/{len(waypoints)-1} dist_step={dist:.2f} v={v_out} w={w_cmd} paint={paint}")

    motors.stop()
    print("\n[dryrun] Done. Motors stopped.\n")


def main():
    print("\n=== AutoField Main (Laptop Dry Run) ===\n")

    # 1) UI input -> JSON + returned dict
    field_spec = get_field_spec_cli()
    user_id = field_spec["user_id"]
    L = field_spec["converted"]["meters"]["length_m"]
    W = field_spec["converted"]["meters"]["width_m"]

    # 2) Path planning -> waypoint list
    R = 1.0     # turning radius meters (tune later)
    ds = 0.10   # waypoint spacing meters
    waypoints = generate_soccer_perimeter_270(L, W, R, ds=ds)  # (x,y,paint)

    print(f"\n[main] ✅ Path planned: {len(waypoints)} waypoints")
    print("[main] First 10 waypoints:")
    for wp in waypoints[:10]:
        print("  ", wp)

    # 3) Mission artifacts
    out_dir = "ui_output"
    way_csv = os.path.join(out_dir, f"waypoints_{user_id}.csv")
    mission_json = os.path.join(out_dir, f"mission_{user_id}.json")

    save_waypoints_csv(waypoints, way_csv)

    mission = {
        "mission_id": f"{user_id}_{datetime.now().strftime('%Y%m%d_%H%M%S')}",
        "user_id": user_id,
        "field_length_m": L,
        "field_width_m": W,
        "turn_radius_m": R,
        "ds_m": ds,
        "waypoints_csv": way_csv,
        "count_waypoints": len(waypoints),
    }
    save_mission_json(mission, mission_json)

    print(f"\n[main] ✅ Wrote: {way_csv}")
    print(f"[main] ✅ Wrote: {mission_json}")

    # 4) Motor interface (terminal mode for laptop)
    config = load_config("config.yaml")
    motors = build_motor_interface(config)

    # 5) Dry run: turn the path into motor commands and print them
    dry_run_motor_outputs(motors, waypoints, v_cmd=450, k_turn=700, print_every=25)


if __name__ == "__main__":
    main()
