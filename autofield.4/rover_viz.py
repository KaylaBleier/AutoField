"""
rover_viz.py

Visualizes rover run logs from a single run.
Paste your file paths in the CONFIG block below and run:

    python rover_viz.py

Requires:
    pip install matplotlib pandas

Outputs:
    - Two-panel plot window (path map + error timeline)
    - Saves a PNG next to this script
"""

import math
import os
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.lines as mlines
from matplotlib.gridspec import GridSpec

# =============================================================================
# CONFIG — paste your file paths here
# =============================================================================

SESSION_LOG = "logs/session_log_20250409_143201.csv"   # required
TRACK_LOG   = "logs/track_log_20250409_143201.csv"     # optional, set None to skip
COMMAND_LOG = "logs/command_log_20250409_143201.csv"   # optional, set None to skip

# =============================================================================
# END CONFIG
# =============================================================================


# -----------------------------------------------------------------------------
# Loaders
# -----------------------------------------------------------------------------

def load_session(path: str) -> dict:
    df = pd.read_csv(path)
    return dict(zip(df["key"], df["value"]))


def load_track(path: str) -> pd.DataFrame:
    df = pd.read_csv(path)
    df["easting"]          = pd.to_numeric(df["easting"],          errors="coerce")
    df["northing"]         = pd.to_numeric(df["northing"],         errors="coerce")
    df["heading_error_deg"]= pd.to_numeric(df["heading_error_deg"],errors="coerce")
    df["lateral_error_m"]  = pd.to_numeric(df["lateral_error_m"],  errors="coerce")
    df["progress_t"]       = pd.to_numeric(df["progress_t"],       errors="coerce")
    df["paint_arm"]        = pd.to_numeric(df["paint_arm"],        errors="coerce")
    return df.dropna(subset=["easting", "northing"])


def load_command(path: str) -> pd.DataFrame:
    df = pd.read_csv(path)
    df["pwm_L"]            = pd.to_numeric(df["pwm_L"],            errors="coerce")
    df["pwm_R"]            = pd.to_numeric(df["pwm_R"],            errors="coerce")
    df["heading_error_deg"]= pd.to_numeric(df["heading_error_deg"],errors="coerce")
    df["lateral_error_m"]  = pd.to_numeric(df["lateral_error_m"],  errors="coerce")
    return df


# -----------------------------------------------------------------------------
# Helpers
# -----------------------------------------------------------------------------

def _fv(sess: dict, key: str, fallback=None):
    """Safely get a float from session dict."""
    try:
        return float(sess[key])
    except (KeyError, ValueError, TypeError):
        return fallback


def _sv(sess: dict, key: str, fallback="—"):
    return sess.get(key, fallback)


def _heading_arrow(ax, e, n, heading_deg, length, color, label=None):
    """Draw an arrow showing intended heading from a point."""
    rad = math.radians(heading_deg)
    de  = math.cos(rad) * length
    dn  = math.sin(rad) * length
    ax.annotate(
        "", xy=(e + de, n + dn), xytext=(e, n),
        arrowprops=dict(arrowstyle="-|>", color=color, lw=1.5),
    )


# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------

def main():
    # --- Load files -----------------------------------------------------------
    if not os.path.exists(SESSION_LOG):
        raise FileNotFoundError(f"session_log not found: {SESSION_LOG}")

    sess    = load_session(SESSION_LOG)
    track   = load_track(TRACK_LOG)   if TRACK_LOG   and os.path.exists(TRACK_LOG)   else None
    command = load_command(COMMAND_LOG) if COMMAND_LOG and os.path.exists(COMMAND_LOG) else None

    if TRACK_LOG and not os.path.exists(TRACK_LOG):
        print(f"[warn] track_log not found, skipping: {TRACK_LOG}")
    if COMMAND_LOG and not os.path.exists(COMMAND_LOG):
        print(f"[warn] command_log not found, skipping: {COMMAND_LOG}")

    # --- Parse session values -------------------------------------------------
    wp1e    = _fv(sess, "wp1_easting_m")
    wp1n    = _fv(sess, "wp1_northing_m")
    wp2e    = _fv(sess, "wp2_easting_m")
    wp2n    = _fv(sess, "wp2_northing_m")
    heading = _fv(sess, "intended_heading_deg", 0.0)
    run_len = _fv(sess, "run_length_m", 0.0)
    hdop    = _fv(sess, "hdop_at_start", 0.0)
    sats    = _sv(sess, "num_sats_at_start")
    fix_q   = _sv(sess, "fix_quality_at_start")
    k_head  = _sv(sess, "cfg_k_heading")
    k_lat   = _sv(sess, "cfg_k_lateral")
    base_pw = _sv(sess, "cfg_base_speed_pwm")
    arm_thr = _sv(sess, "cfg_heading_arm_threshold")

    if wp1e is None or wp2e is None:
        raise ValueError("session_log is missing waypoint data — check the file.")

    # --- Figure layout --------------------------------------------------------
    has_errors = track is not None
    n_rows     = 2 if has_errors else 1

    fig = plt.figure(figsize=(13, 10 if has_errors else 6))
    fig.patch.set_facecolor("#F8F8F6")

    gs = GridSpec(
        n_rows, 1, figure=fig,
        height_ratios=[2.2, 1] if has_errors else [1],
        hspace=0.38,
    )

    ax_map = fig.add_subplot(gs[0])
    ax_err = fig.add_subplot(gs[1]) if has_errors else None

    # =========================================================================
    # Panel 1 — Path map
    # =========================================================================
    ax_map.set_facecolor("#F8F8F6")
    ax_map.set_aspect("equal")
    ax_map.grid(True, color="#DDDDD8", linewidth=0.5, zorder=0)
    ax_map.set_xlabel("easting (m)", fontsize=11, color="#5F5E5A")
    ax_map.set_ylabel("northing (m)", fontsize=11, color="#5F5E5A")
    ax_map.tick_params(colors="#888780", labelsize=9)
    for spine in ax_map.spines.values():
        spine.set_edgecolor("#D3D1C7")

    # Intended path — dashed green line
    ax_map.plot(
        [wp1e, wp2e], [wp1n, wp2n],
        color="#1D9E75", linewidth=2, linestyle="--",
        zorder=3, label="intended path",
    )

    # wp1 and wp2 markers
    ax_map.scatter([wp1e], [wp1n], color="#1D9E75", s=80, zorder=5,
                   marker="o", label="wp1 (start)")
    ax_map.scatter([wp2e], [wp2n], color="#0F6E56", s=100, zorder=5,
                   marker="D", label="wp2 (target)")

    # Heading arrow from wp1
    arrow_len = run_len * 0.12 if run_len else 1.0
    _heading_arrow(ax_map, wp1e, wp1n, heading, arrow_len, "#1D9E75")

    # Actual GPS track
    if track is not None and len(track) > 0:
        # Colour by paint arm state
        arm_on  = track[track["paint_arm"] == 1]
        arm_off = track[track["paint_arm"] == 0]

        if len(arm_off) > 0:
            ax_map.plot(arm_off["easting"], arm_off["northing"],
                        color="#888780", linewidth=1.2, zorder=2,
                        label="actual path (arm off)")
        if len(arm_on) > 0:
            ax_map.plot(arm_on["easting"], arm_on["northing"],
                        color="#378ADD", linewidth=2, zorder=2,
                        label="actual path (arm on)")

        # Start marker on track
        ax_map.scatter(
            [track["easting"].iloc[0]], [track["northing"].iloc[0]],
            color="#378ADD", s=60, zorder=6, marker="^",
        )

    # Legend
    legend_handles = [
        mlines.Line2D([], [], color="#1D9E75", linestyle="--", linewidth=2,
                      label="intended path"),
        mlines.Line2D([], [], color="#888780", linewidth=1.5,
                      label="actual path — arm off"),
        mlines.Line2D([], [], color="#378ADD", linewidth=2,
                      label="actual path — arm on"),
        mlines.Line2D([], [], marker="o", color="#1D9E75", linestyle="None",
                      markersize=7, label="wp1 start"),
        mlines.Line2D([], [], marker="D", color="#0F6E56", linestyle="None",
                      markersize=7, label="wp2 target"),
    ]
    ax_map.legend(handles=legend_handles, fontsize=9,
                  framealpha=0.92, edgecolor="#D3D1C7", loc="best")

    # Info box
    hdop_color = "#A32D2D" if hdop and hdop > 3.0 else "#1D9E75"
    info_lines = [
        f"run length : {run_len:.2f} m",
        f"heading    : {heading:.1f}°",
        f"HDOP       : {hdop:.1f}",
        f"satellites : {sats}",
        f"fix quality: {fix_q}",
        f"K_heading  : {k_head}",
        f"K_lateral  : {k_lat}",
        f"base PWM   : {base_pw}",
        f"arm thresh : {arm_thr}°",
    ]
    ax_map.text(
        0.015, 0.975, "\n".join(info_lines),
        transform=ax_map.transAxes,
        fontsize=8.5, verticalalignment="top",
        fontfamily="monospace",
        color="#444441",
        bbox=dict(boxstyle="round,pad=0.5", facecolor="#FFFFFF",
                  edgecolor="#D3D1C7", alpha=0.92),
    )

    ax_map.set_title("rover path — intended vs actual", fontsize=13,
                     fontweight="normal", color="#2C2C2A", pad=10)

    # =========================================================================
    # Panel 2 — Error timeline
    # =========================================================================
    if ax_err is not None and track is not None:
        ax_err.set_facecolor("#F8F8F6")
        ax_err.grid(True, color="#DDDDD8", linewidth=0.5, zorder=0)
        ax_err.set_xlabel("control tick", fontsize=11, color="#5F5E5A")
        ax_err.set_ylabel("heading error (°)", fontsize=11, color="#D85A30")
        ax_err.tick_params(axis="y", colors="#D85A30", labelsize=9)
        ax_err.tick_params(axis="x", colors="#888780", labelsize=9)
        for spine in ax_err.spines.values():
            spine.set_edgecolor("#D3D1C7")

        ticks = range(len(track))

        ax_err.plot(ticks, track["heading_error_deg"],
                    color="#D85A30", linewidth=1.5, label="heading error (°)")
        ax_err.axhline(0, color="#D85A30", linewidth=0.5, linestyle=":")

        ax_lat = ax_err.twinx()
        ax_lat.set_ylabel("lateral error (m)", fontsize=11, color="#378ADD")
        ax_lat.tick_params(colors="#378ADD", labelsize=9)
        ax_lat.plot(ticks, track["lateral_error_m"],
                    color="#378ADD", linewidth=1.5, label="lateral error (m)")
        ax_lat.axhline(0, color="#378ADD", linewidth=0.5, linestyle=":")

        # PWM diff overlay from command log
        if command is not None and len(command) == len(track):
            pwm_diff = command["pwm_L"] - command["pwm_R"]
            ax_err.plot(ticks, pwm_diff,
                        color="#BA7517", linewidth=1, linestyle="--",
                        label="PWM diff (L−R)", alpha=0.7)

        # Combined legend
        err_handles = [
            mlines.Line2D([], [], color="#D85A30", linewidth=1.5,
                          label="heading error (°)"),
            mlines.Line2D([], [], color="#378ADD", linewidth=1.5,
                          label="lateral error (m)"),
        ]
        if command is not None and len(command) == len(track):
            err_handles.append(
                mlines.Line2D([], [], color="#BA7517", linewidth=1,
                              linestyle="--", label="PWM diff L−R")
            )
        ax_err.legend(handles=err_handles, fontsize=9,
                      framealpha=0.92, edgecolor="#D3D1C7", loc="upper right")

        ax_err.set_title("error timeline", fontsize=12,
                         fontweight="normal", color="#2C2C2A", pad=8)

    # =========================================================================
    # Save + show
    # =========================================================================
    out_path = os.path.splitext(SESSION_LOG)[0] + "_viz.png"
    plt.savefig(out_path, dpi=150, bbox_inches="tight",
                facecolor=fig.get_facecolor())
    print(f"[rover_viz] Saved: {out_path}")
    plt.show()


if __name__ == "__main__":
    main()
