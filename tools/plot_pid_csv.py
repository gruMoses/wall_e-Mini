#!/usr/bin/env python3
"""
Plot PID tuning CSV from WALL-E.

Usage:
    python3 tools/plot_pid_csv.py                        # uses logs/pid_latest.csv
    python3 tools/plot_pid_csv.py --csv logs/pid_*.csv   # specific file
    python3 tools/plot_pid_csv.py --last 30              # last 30 seconds only

Produces a multi-panel PNG saved next to the CSV (and opens it if a display
is available).
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import pandas as pd

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.gridspec import GridSpec
except ImportError:
    print("matplotlib and pandas are required:  pip3 install matplotlib pandas")
    sys.exit(1)


def load(csv_path: Path, last_s: float | None = None) -> pd.DataFrame:
    df = pd.read_csv(csv_path)
    df["t_s"] = df["t_ms"] / 1000.0
    if last_s is not None and len(df) > 0:
        t_max = df["t_s"].iloc[-1]
        df = df[df["t_s"] >= t_max - last_s].copy()
        df["t_s"] = df["t_s"] - df["t_s"].iloc[0]
    return df


def _has_follow_me_data(df: pd.DataFrame) -> bool:
    if "fm_tracking" not in df.columns:
        return False
    return df["fm_tracking"].sum() > 0


def plot(df: pd.DataFrame, out_path: Path, title: str = "") -> None:
    has_fm = _has_follow_me_data(df)
    n_panels = 9 if has_fm else 6
    fig = plt.figure(figsize=(16, 2.4 * n_panels))
    gs = GridSpec(n_panels, 1, hspace=0.40, figure=fig)
    t = df["t_s"]

    # 1 — Heading & target
    ax1 = fig.add_subplot(gs[0])
    ax1.plot(t, df["heading_deg"], label="heading", linewidth=0.8)
    ax1.plot(t, df["target_deg"], label="target", linewidth=0.8, linestyle="--")
    ax1.set_ylabel("deg")
    ax1.set_title("Heading vs Target")
    ax1.legend(loc="upper right", fontsize=8)
    ax1.grid(True, alpha=0.3)

    # 2 — Error + yaw rate
    ax2 = fig.add_subplot(gs[1], sharex=ax1)
    ax2.plot(t, df["error_deg"], label="error", linewidth=0.8, color="tab:red")
    ax2r = ax2.twinx()
    ax2r.plot(t, df["yaw_rate_dps"], label="yaw rate", linewidth=0.6, color="tab:purple", alpha=0.6)
    ax2r.set_ylabel("deg/s", color="tab:purple")
    ax2.set_ylabel("deg")
    ax2.set_title("Heading Error & Yaw Rate")
    ax2.legend(loc="upper left", fontsize=8)
    ax2r.legend(loc="upper right", fontsize=8)
    ax2.grid(True, alpha=0.3)

    # 3 — PID terms
    ax3 = fig.add_subplot(gs[2], sharex=ax1)
    ax3.plot(t, df["p_term"], label="P", linewidth=0.8)
    ax3.plot(t, df["i_term"], label="I", linewidth=0.8)
    ax3.plot(t, df["d_term"], label="D", linewidth=0.8)
    ax3.set_ylabel("byte-equiv")
    ax3.set_title("PID Terms (P / I / D)")
    ax3.legend(loc="upper right", fontsize=8)
    ax3.grid(True, alpha=0.3)

    # 4 — Correction raw vs applied + integral accumulator
    ax4 = fig.add_subplot(gs[3], sharex=ax1)
    if "correction_raw" in df.columns:
        ax4.plot(t, pd.to_numeric(df["correction_raw"], errors="coerce"),
                 label="raw", linewidth=0.8)
    if "correction_applied" in df.columns:
        ax4.plot(t, pd.to_numeric(df["correction_applied"], errors="coerce"),
                 label="applied", linewidth=0.8, linestyle="--")
    ax4r = ax4.twinx()
    ax4r.plot(t, df["integral_accum"], label="integral accum", linewidth=0.6,
              color="tab:green", alpha=0.6)
    ax4r.set_ylabel("integral", color="tab:green")
    ax4.set_ylabel("correction")
    ax4.set_title("Correction & Integral Accumulator")
    ax4.legend(loc="upper left", fontsize=8)
    ax4r.legend(loc="upper right", fontsize=8)
    ax4.grid(True, alpha=0.3)

    # 5 — Motor outputs
    ax5 = fig.add_subplot(gs[4], sharex=ax1)
    ax5.plot(t, df["motor_l"], label="left", linewidth=0.8)
    ax5.plot(t, df["motor_r"], label="right", linewidth=0.8)
    ax5.axhline(126, color="gray", linestyle=":", linewidth=0.5)
    ax5.set_ylabel("byte (0-254)")
    ax5.set_title("Motor Commands")
    ax5.legend(loc="upper right", fontsize=8)
    ax5.grid(True, alpha=0.3)

    # 6 — Roll, pitch (terrain)
    ax6 = fig.add_subplot(gs[5], sharex=ax1)
    ax6.plot(t, df["roll_deg"], label="roll", linewidth=0.8)
    ax6.plot(t, df["pitch_deg"], label="pitch", linewidth=0.8)
    ax6.set_ylabel("deg")
    ax6.set_title("Roll & Pitch (terrain)")
    ax6.legend(loc="upper right", fontsize=8)
    ax6.grid(True, alpha=0.3)

    # Follow Me panels (only when data exists)
    if has_fm:
        fm_z = pd.to_numeric(df.get("fm_target_z_m", pd.Series(dtype=float)), errors="coerce")
        fm_x = pd.to_numeric(df.get("fm_target_x_m", pd.Series(dtype=float)), errors="coerce")
        fm_derr = pd.to_numeric(df.get("fm_dist_err_m", pd.Series(dtype=float)), errors="coerce")
        fm_spd = pd.to_numeric(df.get("fm_speed_offset", pd.Series(dtype=float)), errors="coerce")
        fm_str = pd.to_numeric(df.get("fm_steer_offset", pd.Series(dtype=float)), errors="coerce")
        fm_conf = pd.to_numeric(df.get("fm_confidence", pd.Series(dtype=float)), errors="coerce")

        # 7 — Follow Me target distance + lateral offset
        ax7 = fig.add_subplot(gs[6], sharex=ax1)
        ax7.plot(t, fm_z, label="target Z (depth)", linewidth=0.8, color="tab:blue")
        ax7.axhline(1.5, color="gray", linestyle=":", linewidth=0.5, label="follow dist")
        ax7r = ax7.twinx()
        ax7r.plot(t, fm_x, label="target X (lateral)", linewidth=0.8, color="tab:orange", alpha=0.7)
        ax7r.set_ylabel("lateral m", color="tab:orange")
        ax7.set_ylabel("depth m")
        ax7.set_title("Follow Me — Target Position")
        ax7.legend(loc="upper left", fontsize=8)
        ax7r.legend(loc="upper right", fontsize=8)
        ax7.grid(True, alpha=0.3)

        # 8 — Follow Me speed & steer offsets + distance error
        ax8 = fig.add_subplot(gs[7], sharex=ax1)
        ax8.plot(t, fm_spd, label="speed offset", linewidth=0.8, color="tab:green")
        ax8.plot(t, fm_str, label="steer offset", linewidth=0.8, color="tab:red")
        ax8r = ax8.twinx()
        ax8r.plot(t, fm_derr, label="dist error", linewidth=0.6, color="tab:cyan", alpha=0.6)
        ax8r.set_ylabel("dist err (m)", color="tab:cyan")
        ax8.set_ylabel("byte offset")
        ax8.set_title("Follow Me — Speed & Steering Offsets")
        ax8.legend(loc="upper left", fontsize=8)
        ax8r.legend(loc="upper right", fontsize=8)
        ax8.grid(True, alpha=0.3)

        # 9 — Detection count + confidence
        ax9 = fig.add_subplot(gs[8], sharex=ax1)
        fm_ndet = pd.to_numeric(df.get("fm_num_det", pd.Series(dtype=float)), errors="coerce")
        ax9.fill_between(t, 0, fm_ndet, alpha=0.3, label="# detections", step="post")
        ax9r = ax9.twinx()
        ax9r.plot(t, fm_conf, label="confidence", linewidth=0.8, color="tab:purple", alpha=0.7)
        ax9r.set_ylabel("confidence", color="tab:purple")
        ax9r.set_ylim(0, 1.05)
        ax9.set_ylabel("count")
        ax9.set_xlabel("time (s)")
        ax9.set_title("Follow Me — Detections & Confidence")
        ax9.legend(loc="upper left", fontsize=8)
        ax9r.legend(loc="upper right", fontsize=8)
        ax9.grid(True, alpha=0.3)
    else:
        ax6.set_xlabel("time (s)")

    if title:
        fig.suptitle(title, fontsize=12, y=0.99)

    fig.savefig(out_path, dpi=120, bbox_inches="tight")
    print(f"Saved plot: {out_path}")
    plt.close(fig)


def print_summary(df: pd.DataFrame) -> None:
    dur = df["t_s"].iloc[-1] - df["t_s"].iloc[0] if len(df) > 1 else 0
    hz = len(df) / dur if dur > 0 else 0
    err = df["error_deg"]
    print(f"\n--- PID Log Summary ---")
    print(f"  Duration:      {dur:.1f}s  ({len(df)} samples, {hz:.0f} Hz)")
    print(f"  Error RMS:     {(err**2).mean()**0.5:.2f} deg")
    print(f"  Error max:     {err.abs().max():.2f} deg")
    print(f"  Error mean:    {err.abs().mean():.2f} deg")
    integ = df["integral_accum"]
    print(f"  Integral range: [{integ.min():.1f}, {integ.max():.1f}]")
    yaw = df["yaw_rate_dps"]
    print(f"  Yaw rate std:  {yaw.std():.2f} deg/s")
    armed_pct = df["armed"].mean() * 100
    straight_pct = df["straight_intent"].mean() * 100
    print(f"  Armed:         {armed_pct:.0f}%")
    print(f"  Straight:      {straight_pct:.0f}%")
    raw = pd.to_numeric(df.get("correction_raw", pd.Series(dtype=float)), errors="coerce")
    app = pd.to_numeric(df.get("correction_applied", pd.Series(dtype=float)), errors="coerce")
    sat_count = ((raw.notna()) & (app.notna()) & (raw != app)).sum()
    active = raw.notna().sum()
    if active > 0:
        print(f"  Saturated:     {sat_count}/{active} ({100*sat_count/active:.1f}%)")

    # Follow Me summary
    if "fm_tracking" in df.columns and df["fm_tracking"].sum() > 0:
        fm = df[df["fm_tracking"] == 1]
        print(f"\n--- Follow Me Summary ---")
        print(f"  Tracking:      {len(fm)} samples ({100*len(fm)/len(df):.0f}% of run)")
        fm_z = pd.to_numeric(fm.get("fm_target_z_m", pd.Series(dtype=float)), errors="coerce").dropna()
        fm_x = pd.to_numeric(fm.get("fm_target_x_m", pd.Series(dtype=float)), errors="coerce").dropna()
        fm_derr = pd.to_numeric(fm.get("fm_dist_err_m", pd.Series(dtype=float)), errors="coerce").dropna()
        fm_spd = pd.to_numeric(fm.get("fm_speed_offset", pd.Series(dtype=float)), errors="coerce").dropna()
        fm_str = pd.to_numeric(fm.get("fm_steer_offset", pd.Series(dtype=float)), errors="coerce").dropna()
        if len(fm_z) > 0:
            print(f"  Target Z:      mean={fm_z.mean():.2f}m  range=[{fm_z.min():.2f}, {fm_z.max():.2f}]")
        if len(fm_x) > 0:
            print(f"  Target X:      mean={fm_x.mean():.3f}m  std={fm_x.std():.3f}m")
        if len(fm_derr) > 0:
            print(f"  Dist error:    mean={fm_derr.mean():.2f}m  RMS={((fm_derr**2).mean()**0.5):.2f}m")
        if len(fm_spd) > 0:
            print(f"  Speed offset:  mean={fm_spd.mean():.1f}  max={fm_spd.max():.1f}")
        if len(fm_str) > 0:
            print(f"  Steer offset:  std={fm_str.std():.1f}  max_abs={fm_str.abs().max():.1f}")
    print()


def main():
    root = Path(__file__).resolve().parents[1]
    default_csv = root / "logs" / "pid_latest.csv"

    parser = argparse.ArgumentParser(description="Plot WALL-E PID tuning CSV")
    parser.add_argument("--csv", type=str, default=str(default_csv))
    parser.add_argument("--last", type=float, default=None,
                        help="Only plot the last N seconds")
    parser.add_argument("--no-show", action="store_true",
                        help="Don't try to open the image")
    args = parser.parse_args()

    csv_path = Path(args.csv)
    if not csv_path.exists():
        print(f"CSV not found: {csv_path}")
        sys.exit(1)

    df = load(csv_path, last_s=args.last)
    if df.empty:
        print("No data in CSV")
        sys.exit(1)

    print_summary(df)

    out_png = csv_path.with_suffix(".png")
    title = csv_path.stem
    if args.last:
        title += f" (last {args.last:.0f}s)"
    plot(df, out_png, title=title)

    if not args.no_show:
        try:
            import subprocess
            subprocess.Popen(["xdg-open", str(out_png)],
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception:
            pass


if __name__ == "__main__":
    main()
