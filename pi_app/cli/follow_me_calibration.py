#!/usr/bin/env python3
"""
Follow Me Calibration Tool — Three-Phase System Identification & PD Tuning.

Phase 1: Open-loop plant characterization (manual RC driving)
         Measures the relationship between motor differential and actual turn
         rate, dead zone, latency, and left/right asymmetry.

Phase 2: Slow-speed Follow Me calibration
         Records person-tracking data with conservative gains, then uses the
         plant model from Phase 1 to compute optimal PD gains for a desired
         closed-loop settling time.

Phase 3: Incremental speed-up
         Repeats Phase 2 at progressively higher speeds, detects gain-schedule
         requirements, and produces a final recommended config.

Usage:
    python3 pi_app/cli/follow_me_calibration.py
"""

from __future__ import annotations

import json
import math
import os
import select
import signal
import sys
import threading
import time
from dataclasses import asdict, dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple

_PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_PROJECT_ROOT))

import numpy as np

from config import config

NEUTRAL = 126
LOG_DIR = _PROJECT_ROOT / "logs"
CALIBRATION_DIR = LOG_DIR / "calibration"
LATEST_LOG = LOG_DIR / "latest.log"

BANNER = r"""
 ╔═══════════════════════════════════════════════════════════╗
 ║   WALL-E Follow Me Calibration Tool                      ║
 ║   Three-Phase System Identification & PD Tuning           ║
 ╚═══════════════════════════════════════════════════════════╝
"""


# ---------------------------------------------------------------------------
# Data classes
# ---------------------------------------------------------------------------

@dataclass
class Sample:
    ts: float
    mode: str
    armed: bool
    motor_l: int
    motor_r: int
    yaw_rate_dps: float
    heading_deg: float = 0.0
    person_x_m: Optional[float] = None
    person_z_m: Optional[float] = None
    person_tracking: bool = False
    person_confidence: float = 0.0
    obstacle_scale: float = 1.0
    steer_offset: Optional[float] = None
    speed_offset: Optional[float] = None
    distance_error_m: Optional[float] = None
    ch1: int = 1500
    ch2: int = 1500

    @property
    def motor_diff(self) -> float:
        return (self.motor_l - self.motor_r) / 2.0

    @property
    def speed_component(self) -> float:
        return (self.motor_l + self.motor_r) / 2.0 - NEUTRAL


@dataclass
class PlantResult:
    ff_gain_dps_per_byte: float
    dead_zone_bytes: float
    latency_samples: int
    latency_ms: float
    max_turn_rate_dps: float
    left_ff: float
    right_ff: float
    asymmetry_pct: float
    num_samples: int
    sample_rate_hz: float


@dataclass
class PDFitResult:
    kp_recommended: float
    kd_recommended: float
    settling_time_target_s: float
    damping_ratio: float
    plant_gain_used: float
    scale_factor: float
    avg_z_m: float
    rms_x_m: float
    peak_x_m: float
    oscillation_count: int
    tracking_pct: float
    num_samples: int
    current_kp: float
    current_kd: float


# ---------------------------------------------------------------------------
# Log parser
# ---------------------------------------------------------------------------

def parse_log_line(raw: str) -> Optional[Sample]:
    try:
        d = json.loads(raw.strip())
    except (json.JSONDecodeError, ValueError):
        return None

    safety = d.get("safety", {})
    motor = d.get("motor", {})
    imu = d.get("imu", {})
    rc = d.get("rc", {})
    fm = d.get("follow_me", {})
    obs = d.get("obstacle", {})

    return Sample(
        ts=float(d.get("ts", 0)),
        mode=d.get("mode", ""),
        armed=bool(safety.get("armed", False)),
        motor_l=int(motor.get("L", NEUTRAL)),
        motor_r=int(motor.get("R", NEUTRAL)),
        yaw_rate_dps=float(imu.get("yaw_rate_dps", 0.0)),
        heading_deg=float(imu.get("heading_deg", 0.0)),
        person_x_m=fm.get("target_x_m"),
        person_z_m=fm.get("target_z_m"),
        person_tracking=bool(fm.get("tracking")),
        person_confidence=float(fm.get("confidence", 0.0)),
        obstacle_scale=float(obs.get("throttle_scale", 1.0)),
        steer_offset=fm.get("steer_offset"),
        speed_offset=fm.get("speed_offset"),
        distance_error_m=fm.get("distance_error_m"),
        ch1=int(rc.get("ch1", 1500)),
        ch2=int(rc.get("ch2", 1500)),
    )


# ---------------------------------------------------------------------------
# Live log tailer
# ---------------------------------------------------------------------------

class LogTailer:
    """Tails the live log file, collecting samples until stopped."""

    def __init__(self, log_path: Path, filter_fn=None):
        self._path = log_path
        self._filter = filter_fn or (lambda s: True)
        self._samples: List[Sample] = []
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._stats = {"total_lines": 0, "matched": 0, "start_time": 0.0}

    def start(self):
        self._stop.clear()
        self._samples.clear()
        self._stats = {"total_lines": 0, "matched": 0, "start_time": time.time()}
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> List[Sample]:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=3.0)
        with self._lock:
            return list(self._samples)

    @property
    def count(self) -> int:
        with self._lock:
            return len(self._samples)

    @property
    def elapsed(self) -> float:
        return time.time() - self._stats["start_time"]

    def last_sample(self) -> Optional[Sample]:
        with self._lock:
            return self._samples[-1] if self._samples else None

    def _run(self):
        try:
            with open(self._path, "r") as f:
                f.seek(0, 2)
                while not self._stop.is_set():
                    line = f.readline()
                    if not line:
                        time.sleep(0.02)
                        continue
                    self._stats["total_lines"] += 1
                    s = parse_log_line(line)
                    if s is not None and self._filter(s):
                        with self._lock:
                            self._samples.append(s)
                        self._stats["matched"] += 1
        except Exception as e:
            print(f"\n  [LogTailer error: {e}]")


# ---------------------------------------------------------------------------
# Session management
# ---------------------------------------------------------------------------

class CalibrationSession:
    def __init__(self, session_dir: Optional[Path] = None):
        if session_dir is None:
            self.session_id = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.session_dir = CALIBRATION_DIR / self.session_id
        else:
            self.session_dir = session_dir
            self.session_id = session_dir.name

        self.session_dir.mkdir(parents=True, exist_ok=True)
        self.plant_result: Optional[PlantResult] = None
        self.phase2_result: Optional[PDFitResult] = None
        self.phase3_runs: List[Tuple[int, PDFitResult]] = []

        self._load_existing()

    def _load_existing(self):
        p1 = self.session_dir / "phase1_result.json"
        if p1.exists():
            try:
                data = json.loads(p1.read_text())
                self.plant_result = PlantResult(**data)
                print(f"  Loaded Phase 1 result from previous run.")
            except Exception:
                pass
        p2 = self.session_dir / "phase2_result.json"
        if p2.exists():
            try:
                data = json.loads(p2.read_text())
                self.phase2_result = PDFitResult(**data)
                print(f"  Loaded Phase 2 result from previous run.")
            except Exception:
                pass

    def save_samples(self, name: str, samples: List[Sample]):
        path = self.session_dir / f"{name}.json"
        data = [asdict(s) for s in samples]
        path.write_text(json.dumps(data, indent=1))
        print(f"  Saved {len(samples)} samples → {path}")

    def save_result(self, name: str, result):
        path = self.session_dir / f"{name}.json"
        path.write_text(json.dumps(asdict(result), indent=2))

    def save_report(self, name: str, text: str):
        path = self.session_dir / f"{name}.md"
        path.write_text(text)
        print(f"  Report saved → {path}")


# ---------------------------------------------------------------------------
# Phase 1: Plant Characterization
# ---------------------------------------------------------------------------

def analyze_phase1(samples: List[Sample]) -> PlantResult:
    """Analyze manual driving data to characterize the steering plant."""

    if len(samples) < 20:
        raise ValueError(f"Need at least 20 samples, got {len(samples)}")

    diffs = np.array([s.motor_diff for s in samples])
    yaw_rates = np.array([s.yaw_rate_dps for s in samples])
    ts = np.array([s.ts for s in samples])

    dt_arr = np.diff(ts)
    dt_arr = dt_arr[dt_arr > 0]
    sample_rate = 1.0 / np.median(dt_arr) if len(dt_arr) > 0 else 10.0

    # FF gain: use samples where both diff and yaw_rate are significant
    active_mask = (np.abs(diffs) > 3.0) & (np.abs(yaw_rates) > 2.0)
    if np.sum(active_mask) < 10:
        active_mask = np.abs(diffs) > 1.0

    active_diffs = diffs[active_mask]
    active_yaw = yaw_rates[active_mask]

    if len(active_diffs) < 5:
        ff_gain = 0.0
    else:
        ff_gain = float(np.mean(np.abs(active_yaw)) / np.mean(np.abs(active_diffs)))

    # Dead zone: minimum |diff| where |yaw_rate| > 3 deg/s
    moving_mask = np.abs(yaw_rates) > 3.0
    if np.any(moving_mask):
        dead_zone = float(np.percentile(np.abs(diffs[moving_mask]), 5))
    else:
        dead_zone = 0.0

    # Latency via cross-correlation
    if len(active_diffs) > 20:
        d_norm = diffs - np.mean(diffs)
        y_norm = yaw_rates - np.mean(yaw_rates)
        corr = np.correlate(y_norm, d_norm, mode="full")
        mid = len(d_norm) - 1
        search_range = min(int(sample_rate * 0.5), len(corr) // 4)
        if search_range > 0:
            window = corr[mid:mid + search_range]
            lag = int(np.argmax(window))
        else:
            lag = 0
    else:
        lag = 0
    latency_ms = lag / sample_rate * 1000.0 if sample_rate > 0 else 0.0

    max_turn = float(np.max(np.abs(yaw_rates))) if len(yaw_rates) > 0 else 0.0

    # Left/right asymmetry
    left_mask = active_diffs < -1.0
    right_mask = active_diffs > 1.0
    left_ff = (
        float(np.mean(np.abs(active_yaw[left_mask])) / np.mean(np.abs(active_diffs[left_mask])))
        if np.sum(left_mask) > 3 else ff_gain
    )
    right_ff = (
        float(np.mean(np.abs(active_yaw[right_mask])) / np.mean(np.abs(active_diffs[right_mask])))
        if np.sum(right_mask) > 3 else ff_gain
    )
    avg_ff = (left_ff + right_ff) / 2.0
    asymmetry = abs(left_ff - right_ff) / avg_ff * 100.0 if avg_ff > 0 else 0.0

    return PlantResult(
        ff_gain_dps_per_byte=ff_gain,
        dead_zone_bytes=dead_zone,
        latency_samples=lag,
        latency_ms=latency_ms,
        max_turn_rate_dps=max_turn,
        left_ff=left_ff,
        right_ff=right_ff,
        asymmetry_pct=asymmetry,
        num_samples=len(samples),
        sample_rate_hz=sample_rate,
    )


def format_phase1_report(r: PlantResult) -> str:
    lines = [
        "# Phase 1: Plant Characterization Report",
        "",
        f"**Samples:** {r.num_samples} @ {r.sample_rate_hz:.1f} Hz",
        "",
        "## Steering Plant Model",
        "",
        f"| Parameter | Value |",
        f"|-----------|-------|",
        f"| Feed-forward gain | **{r.ff_gain_dps_per_byte:.3f} deg/s per byte** |",
        f"| Dead zone | {r.dead_zone_bytes:.1f} bytes |",
        f"| Latency | {r.latency_ms:.0f} ms ({r.latency_samples} samples) |",
        f"| Max observed turn rate | {r.max_turn_rate_dps:.1f} deg/s |",
        "",
        "## Left/Right Asymmetry",
        "",
        f"| Direction | FF Gain (deg/s/byte) |",
        f"|-----------|---------------------|",
        f"| Left turn | {r.left_ff:.3f} |",
        f"| Right turn | {r.right_ff:.3f} |",
        f"| Asymmetry | {r.asymmetry_pct:.1f}% |",
        "",
        "## Interpretation",
        "",
        f"A motor differential of 10 bytes produces ~{r.ff_gain_dps_per_byte * 10:.1f} deg/s of turn.",
        f"The tracks don't grip below ~{r.dead_zone_bytes:.0f} bytes of differential.",
        f"The steering response lags the motor command by ~{r.latency_ms:.0f} ms.",
    ]
    if r.asymmetry_pct > 15:
        lines.append(f"WARNING: {r.asymmetry_pct:.0f}% left/right asymmetry — check track tension.")
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Phase 2: Follow Me PD Gain Fitting
# ---------------------------------------------------------------------------

def analyze_phase2(
    samples: List[Sample],
    plant: Optional[PlantResult],
    settling_time_s: float = 1.0,
    damping_ratio: float = 0.9,
) -> PDFitResult:
    """Compute recommended PD gains from Follow Me tracking data + plant model."""

    tracking = [s for s in samples if s.person_tracking and s.person_x_m is not None]
    if len(tracking) < 20:
        raise ValueError(f"Need at least 20 tracking samples, got {len(tracking)}")

    x_arr = np.array([s.person_x_m for s in tracking])
    z_arr = np.array([s.person_z_m for s in tracking])
    ts_arr = np.array([s.ts for s in tracking])
    diff_arr = np.array([s.motor_diff for s in tracking])
    yaw_arr = np.array([s.yaw_rate_dps for s in tracking])

    tracking_pct = len(tracking) / len(samples) * 100.0 if samples else 0.0
    avg_z = float(np.mean(z_arr))
    rms_x = float(np.sqrt(np.mean(x_arr ** 2)))
    peak_x = float(np.max(np.abs(x_arr)))

    # Count oscillations: zero-crossings of x_m
    signs = np.sign(x_arr)
    sign_changes = np.sum(np.abs(np.diff(signs)) > 0)
    osc_count = int(sign_changes // 2)

    # Compute dx/dt from smoothed x_m
    alpha = config.follow_me.steering_ema_alpha
    smoothed_x = np.zeros_like(x_arr)
    smoothed_x[0] = x_arr[0]
    for i in range(1, len(x_arr)):
        smoothed_x[i] = alpha * x_arr[i] + (1 - alpha) * smoothed_x[i - 1]

    dt_arr = np.diff(ts_arr)
    dx_dt = np.zeros_like(x_arr)
    dx_dt[1:] = np.diff(smoothed_x) / np.where(dt_arr > 0.001, dt_arr, 0.05)

    scale = config.follow_me.max_follow_speed_byte / max(config.follow_me.max_distance_m, 0.1)

    # If we have a plant model, compute gains from control theory
    if plant is not None and plant.ff_gain_dps_per_byte > 0.01:
        K_plant = plant.ff_gain_dps_per_byte
        K_plant_rad = K_plant * math.pi / 180.0

        # Closed-loop: first-order system from x_m to dx_m/dt
        # dx_m/dt ≈ -K_plant_rad * z * steer_offset
        # steer_offset = Kp_cfg * scale * x_m + Kd_cfg * scale * dx_dt
        #
        # For desired settling time tau_s (time to reach 37% of initial error):
        # pole = -1/tau_s
        # Kp_cfg = 1 / (K_plant_rad * z * scale * tau_s)
        #
        # The D term adds a zero that improves disturbance rejection.
        # For damping_ratio zeta applied to a second-order approximation
        # (accounting for plant latency tau_L):
        # Kd_cfg = 2 * zeta * tau_L / (K_plant_rad * z * scale * tau_s)

        tau_L = (plant.latency_ms / 1000.0) if plant.latency_ms > 0 else 0.05

        kp_rec = 1.0 / (K_plant_rad * avg_z * scale * settling_time_s)
        kd_rec = 2.0 * damping_ratio * tau_L / (K_plant_rad * avg_z * scale * settling_time_s)

        # Sanity clamp
        kp_rec = max(0.01, min(5.0, kp_rec))
        kd_rec = max(0.0, min(3.0, kd_rec))
    else:
        # Fallback: least-squares fit from observed data
        A = np.column_stack([x_arr * scale, dx_dt * scale])
        b = diff_arr
        try:
            result, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
            kp_rec = float(result[0])
            kd_rec = float(result[1])
        except np.linalg.LinAlgError:
            kp_rec = config.follow_me.steering_gain
            kd_rec = config.follow_me.steering_derivative_gain

    return PDFitResult(
        kp_recommended=round(kp_rec, 4),
        kd_recommended=round(kd_rec, 4),
        settling_time_target_s=settling_time_s,
        damping_ratio=damping_ratio,
        plant_gain_used=plant.ff_gain_dps_per_byte if plant else 0.0,
        scale_factor=scale,
        avg_z_m=round(avg_z, 2),
        rms_x_m=round(rms_x, 4),
        peak_x_m=round(peak_x, 4),
        oscillation_count=osc_count,
        tracking_pct=round(tracking_pct, 1),
        num_samples=len(tracking),
        current_kp=config.follow_me.steering_gain,
        current_kd=config.follow_me.steering_derivative_gain,
    )


def format_phase2_report(r: PDFitResult) -> str:
    lines = [
        "# Phase 2: Follow Me PD Gain Calibration Report",
        "",
        f"**Tracking samples:** {r.num_samples} ({r.tracking_pct:.0f}% of session)",
        f"**Average follow distance:** {r.avg_z_m:.2f} m",
        "",
        "## Tracking Quality",
        "",
        f"| Metric | Value |",
        f"|--------|-------|",
        f"| RMS lateral error | {r.rms_x_m:.4f} m |",
        f"| Peak lateral error | {r.peak_x_m:.4f} m |",
        f"| Oscillation count | {r.oscillation_count} |",
        "",
        "## Gain Recommendation",
        "",
        f"Target settling time: {r.settling_time_target_s:.1f}s, "
        f"damping ratio: {r.damping_ratio:.1f}",
        "",
        f"| Parameter | Current | Recommended | Change |",
        f"|-----------|---------|-------------|--------|",
        f"| steering_gain | {r.current_kp:.4f} | **{r.kp_recommended:.4f}** | "
        f"{'↑' if r.kp_recommended > r.current_kp else '↓'} "
        f"{abs(r.kp_recommended - r.current_kp) / max(r.current_kp, 0.001) * 100:.0f}% |",
        f"| steering_derivative_gain | {r.current_kd:.4f} | **{r.kd_recommended:.4f}** | "
        f"{'↑' if r.kd_recommended > r.current_kd else '↓'} "
        f"{abs(r.kd_recommended - r.current_kd) / max(r.current_kd, 0.001) * 100:.0f}% |",
        "",
        "## Derivation",
        "",
        f"Plant gain: {r.plant_gain_used:.3f} deg/s per byte",
        f"Scale factor (max_speed/max_dist): {r.scale_factor:.1f}",
        f"Average z: {r.avg_z_m:.2f} m",
    ]
    if r.oscillation_count > 5:
        lines.extend([
            "",
            f"**WARNING:** {r.oscillation_count} oscillations detected. "
            "Current gains are likely too aggressive.",
        ])
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Phase 3: Speed-Up Analysis
# ---------------------------------------------------------------------------

def format_phase3_report(runs: List[Tuple[int, PDFitResult]]) -> str:
    if not runs:
        return "# Phase 3: No runs recorded yet."

    lines = [
        "# Phase 3: Speed-Up Gain Schedule Report",
        "",
        f"**Runs recorded:** {len(runs)}",
        "",
        "## Gains vs Speed",
        "",
        "| Speed (byte) | Kp | Kd | RMS x_m | Oscillations |",
        "|-------------|-----|-----|---------|-------------|",
    ]
    for speed, r in runs:
        lines.append(
            f"| {speed} | {r.kp_recommended:.4f} | {r.kd_recommended:.4f} "
            f"| {r.rms_x_m:.4f} | {r.oscillation_count} |"
        )

    if len(runs) >= 2:
        kps = [r.kp_recommended for _, r in runs]
        speeds = [s for s, _ in runs]
        variation = (max(kps) - min(kps)) / np.mean(kps) * 100 if np.mean(kps) > 0 else 0
        lines.extend([
            "",
            "## Gain Scheduling Assessment",
            "",
            f"Kp variation across speeds: {variation:.1f}%",
        ])
        if variation > 20:
            lines.append("**Gain scheduling recommended** — gains vary significantly with speed.")
        else:
            lines.append("Gains are stable across tested speeds — no scheduling needed.")

    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Interactive helpers
# ---------------------------------------------------------------------------

def wait_for_go(prompt: str) -> bool:
    """Block until user types 'go' or 'q' to abort."""
    print(f"\n  {prompt}")
    print("  Type 'go' and press Enter when ready (or 'q' to abort).")
    while True:
        try:
            ans = input("  > ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            return False
        if ans == "go":
            return True
        if ans in ("q", "quit", "abort", "exit"):
            return False
        print("  Type 'go' to start or 'q' to go back.")


def wait_for_stop(tailer: LogTailer, phase_label: str):
    """Show live stats and wait for Enter to stop recording."""
    print(f"\n  ┌─ RECORDING: {phase_label}")
    print(f"  │  Press Enter to stop recording.")
    print(f"  │")

    def status_loop():
        while not tailer._stop.is_set():
            last = tailer.last_sample()
            n = tailer.count
            elapsed = tailer.elapsed
            parts = [f"{elapsed:.0f}s", f"samples={n}"]
            if last:
                parts.append(f"motor=({last.motor_l},{last.motor_r})")
                parts.append(f"yaw={last.yaw_rate_dps:.1f}°/s")
                if last.person_x_m is not None:
                    parts.append(f"person=({last.person_x_m:.2f}m, {last.person_z_m:.2f}m)")
                if last.person_tracking:
                    parts.append("TRACKING")
            sys.stdout.write(f"\r  │  [{' | '.join(parts)}]" + " " * 10)
            sys.stdout.flush()
            time.sleep(0.5)

    status_thread = threading.Thread(target=status_loop, daemon=True)
    status_thread.start()

    try:
        input()
    except (EOFError, KeyboardInterrupt):
        pass
    print(f"\n  └─ Recording stopped. {tailer.count} samples in {tailer.elapsed:.1f}s.\n")


# ---------------------------------------------------------------------------
# Phase runners
# ---------------------------------------------------------------------------

def run_phase1(session: CalibrationSession):
    """Phase 1: Manual driving plant characterization."""
    print("\n" + "=" * 60)
    print("  PHASE 1: Plant Characterization")
    print("  Drive manually with RC, making turns at slow/medium speed.")
    print("  The IMU will measure actual turn rate vs motor differential.")
    print("=" * 60)

    if session.plant_result is not None:
        print(f"\n  NOTE: Phase 1 data already exists (FF={session.plant_result.ff_gain_dps_per_byte:.3f}).")
        ans = input("  Overwrite? (y/n): ").strip().lower()
        if ans != "y":
            return

    print("\n  INSTRUCTIONS:")
    print("  1. Arm the robot with the RC transmitter")
    print("  2. Drive forward at SLOW speed (~30-40% throttle)")
    print("  3. Make deliberate turns: left, right, gentle, sharp")
    print("  4. Do NOT spin in place — keep some forward speed")
    print("  5. Dirt surface: go slow to avoid wheel spin")
    print("  6. Aim for 15-30 seconds of turning data")

    if not wait_for_go("Arm the robot, get in position."):
        return

    def phase1_filter(s: Sample) -> bool:
        return s.armed and s.mode == "MANUAL"

    tailer = LogTailer(LATEST_LOG, phase1_filter)
    tailer.start()
    wait_for_stop(tailer, "Phase 1 — Manual Driving")
    samples = tailer.stop()

    if len(samples) < 20:
        print(f"  Only {len(samples)} samples — need at least 20. Try again.")
        return

    session.save_samples("phase1_raw", samples)

    try:
        result = analyze_phase1(samples)
    except ValueError as e:
        print(f"  Analysis error: {e}")
        return

    session.plant_result = result
    session.save_result("phase1_result", result)

    report = format_phase1_report(result)
    session.save_report("phase1_report", report)

    print("\n" + report)
    print("\n  Phase 1 complete. Plant model saved.")


def run_phase2(session: CalibrationSession):
    """Phase 2: Slow Follow Me calibration."""
    print("\n" + "=" * 60)
    print("  PHASE 2: Slow Follow Me Calibration")
    print("  Walk slowly in front of the robot in Follow Me mode.")
    print("=" * 60)

    if session.plant_result is None:
        print("\n  WARNING: No Phase 1 data. Gains will be estimated via regression")
        print("  instead of control-theory derivation. Phase 1 is recommended first.")
        ans = input("  Continue anyway? (y/n): ").strip().lower()
        if ans != "y":
            return

    print(f"\n  Current Follow Me config:")
    print(f"    steering_gain:            {config.follow_me.steering_gain}")
    print(f"    steering_derivative_gain: {config.follow_me.steering_derivative_gain}")
    print(f"    max_follow_speed_byte:    {config.follow_me.max_follow_speed_byte}")
    print(f"    follow_distance_m:        {config.follow_me.follow_distance_m}")

    print("\n  INSTRUCTIONS:")
    print("  1. Stand ~2m in front of the robot, facing away")
    print("  2. Switch to Follow Me mode (Ch4 switch or gesture)")
    print("  3. Walk SLOWLY in a mostly straight line")
    print("  4. Add small left/right deviations (1-2 steps sideways)")
    print("  5. After ~20-30s, walk back or switch to manual")
    print("  6. Dirt surface: keep it slow — we're measuring, not racing")

    settling_input = input("\n  Desired settling time in seconds [1.0]: ").strip()
    settling_time = float(settling_input) if settling_input else 1.0
    settling_time = max(0.3, min(5.0, settling_time))

    if not wait_for_go("Get in position in front of the robot."):
        return

    def phase2_filter(s: Sample) -> bool:
        return s.armed and s.mode == "FOLLOW_ME"

    tailer = LogTailer(LATEST_LOG, phase2_filter)
    tailer.start()
    wait_for_stop(tailer, "Phase 2 — Follow Me")
    samples = tailer.stop()

    if len(samples) < 20:
        print(f"  Only {len(samples)} samples — need at least 20. Try again.")
        return

    session.save_samples("phase2_raw", samples)

    try:
        result = analyze_phase2(samples, session.plant_result, settling_time)
    except ValueError as e:
        print(f"  Analysis error: {e}")
        return

    session.phase2_result = result
    session.save_result("phase2_result", result)

    report = format_phase2_report(result)
    session.save_report("phase2_report", report)

    print("\n" + report)
    print("\n  Phase 2 complete. Recommended gains saved.")


def run_phase3(session: CalibrationSession):
    """Phase 3: Speed-up runs."""
    print("\n" + "=" * 60)
    print("  PHASE 3: Speed-Up Runs")
    print("  Repeat Follow Me at progressively higher speeds.")
    print("=" * 60)

    if session.plant_result is None:
        print("\n  Phase 1 data required for Phase 3. Run Phase 1 first.")
        return

    current_speed = config.follow_me.max_follow_speed_byte
    print(f"\n  Current max_follow_speed_byte: {current_speed}")
    print(f"  Previous Phase 3 runs: {len(session.phase3_runs)}")

    speed_input = input(f"\n  Speed for this run [{current_speed}]: ").strip()
    speed = int(speed_input) if speed_input else current_speed
    speed = max(20, min(254, speed))

    settling_input = input("  Desired settling time [1.0]: ").strip()
    settling_time = float(settling_input) if settling_input else 1.0

    print(f"\n  Running at speed={speed}, settling_time={settling_time}s")
    print(f"  NOTE: If max_follow_speed_byte in config differs from {speed},")
    print(f"  update config.py BEFORE this run and restart the service.")

    if not wait_for_go("Get in position, Follow Me mode active."):
        return

    def phase3_filter(s: Sample) -> bool:
        return s.armed and s.mode == "FOLLOW_ME"

    tailer = LogTailer(LATEST_LOG, phase3_filter)
    tailer.start()
    wait_for_stop(tailer, f"Phase 3 — Speed {speed}")
    samples = tailer.stop()

    if len(samples) < 20:
        print(f"  Only {len(samples)} samples — need at least 20. Try again.")
        return

    run_idx = len(session.phase3_runs) + 1
    session.save_samples(f"phase3_run{run_idx}_speed{speed}_raw", samples)

    try:
        result = analyze_phase2(samples, session.plant_result, settling_time)
    except ValueError as e:
        print(f"  Analysis error: {e}")
        return

    session.phase3_runs.append((speed, result))
    session.save_result(f"phase3_run{run_idx}_speed{speed}_result", result)

    report = format_phase3_report(session.phase3_runs)
    session.save_report("phase3_report", report)

    print("\n" + report)
    print(f"\n  Phase 3 run {run_idx} complete.")


# ---------------------------------------------------------------------------
# Analyze existing log
# ---------------------------------------------------------------------------

def analyze_existing(session: CalibrationSession):
    """Re-analyze raw data from a previous session."""
    print("\n  Available raw data files:")
    raws = sorted(session.session_dir.glob("*_raw.json"))
    if not raws:
        print("  No raw data files found.")
        return
    for i, p in enumerate(raws):
        size = p.stat().st_size
        print(f"    {i + 1}) {p.name} ({size // 1024}KB)")

    choice = input("\n  Select file number (or 'all'): ").strip()
    if choice == "all":
        for p in raws:
            _reanalyze_file(session, p)
    else:
        try:
            idx = int(choice) - 1
            _reanalyze_file(session, raws[idx])
        except (ValueError, IndexError):
            print("  Invalid selection.")


def _reanalyze_file(session: CalibrationSession, path: Path):
    data = json.loads(path.read_text())
    samples = [Sample(**d) for d in data]
    name = path.stem.replace("_raw", "")

    if "phase1" in name:
        result = analyze_phase1(samples)
        session.plant_result = result
        session.save_result("phase1_result", result)
        report = format_phase1_report(result)
        session.save_report("phase1_report", report)
        print(f"\n{report}")
    elif "phase2" in name or "phase3" in name:
        settling_input = input("  Settling time target [1.0]: ").strip()
        settling = float(settling_input) if settling_input else 1.0
        result = analyze_phase2(samples, session.plant_result, settling)
        session.save_result(f"{name}_result", result)
        report = format_phase2_report(result)
        session.save_report(f"{name}_report", report)
        print(f"\n{report}")


# ---------------------------------------------------------------------------
# Show / Apply config
# ---------------------------------------------------------------------------

def show_config():
    print(f"\n  Current Follow Me Config:")
    print(f"    steering_gain:            {config.follow_me.steering_gain}")
    print(f"    steering_derivative_gain: {config.follow_me.steering_derivative_gain}")
    print(f"    steering_ema_alpha:       {config.follow_me.steering_ema_alpha}")
    print(f"    max_follow_speed_byte:    {config.follow_me.max_follow_speed_byte}")
    print(f"    follow_distance_m:        {config.follow_me.follow_distance_m}")
    print(f"    max_steer_delta_per_s:    {config.follow_me.max_steer_delta_per_s}")
    print(f"    max_steer_offset_byte:    {config.follow_me.max_steer_offset_byte}")
    print(f"    lost_target_timeout_s:    {config.follow_me.lost_target_timeout_s}")
    print(f"    lost_target_search_steer_pct: {config.follow_me.lost_target_search_steer_pct}")
    print(f"\n  Slew Limiter:")
    print(f"    follow_me_accel_bps:      {config.slew_limiter.follow_me_accel_bps}")
    print(f"    follow_me_decel_bps:      {config.slew_limiter.follow_me_decel_bps}")
    print(f"    snap_first_follow_me:     {config.slew_limiter.snap_first_follow_me}")


def apply_recommended(session: CalibrationSession):
    """Write recommended gains to config.py."""
    result = session.phase2_result
    if result is None and session.phase3_runs:
        _, result = session.phase3_runs[-1]
    if result is None:
        print("  No calibration results to apply. Run Phase 2 or 3 first.")
        return

    print(f"\n  Recommended changes to config.py:")
    print(f"    steering_gain:            {config.follow_me.steering_gain} → {result.kp_recommended}")
    print(f"    steering_derivative_gain: {config.follow_me.steering_derivative_gain} → {result.kd_recommended}")
    ans = input("\n  Apply these changes? (y/n): ").strip().lower()
    if ans != "y":
        return

    config_path = _PROJECT_ROOT / "config.py"
    text = config_path.read_text()

    import re
    text = re.sub(
        r"(steering_gain:\s*float\s*=\s*)[\d.]+",
        f"\\g<1>{result.kp_recommended}",
        text,
        count=1,
    )
    text = re.sub(
        r"(steering_derivative_gain:\s*float\s*=\s*)[\d.]+",
        f"\\g<1>{result.kd_recommended}",
        text,
        count=1,
    )
    config_path.write_text(text)
    print(f"  Config updated. Restart the service to apply:")
    print(f"    sudo systemctl restart wall-e.service")


# ---------------------------------------------------------------------------
# Session selector
# ---------------------------------------------------------------------------

def select_session() -> CalibrationSession:
    """Let user pick an existing session or create a new one."""
    CALIBRATION_DIR.mkdir(parents=True, exist_ok=True)
    existing = sorted(CALIBRATION_DIR.iterdir(), reverse=True)
    existing = [d for d in existing if d.is_dir()]

    if existing:
        print("\n  Existing sessions:")
        for i, d in enumerate(existing[:10]):
            files = list(d.glob("*.json"))
            print(f"    {i + 1}) {d.name}  ({len(files)} files)")
        print(f"    n) New session")
        choice = input("\n  Select session: ").strip().lower()
        if choice == "n" or not choice:
            return CalibrationSession()
        try:
            idx = int(choice) - 1
            print(f"  Resuming session {existing[idx].name}")
            return CalibrationSession(existing[idx])
        except (ValueError, IndexError):
            return CalibrationSession()
    else:
        return CalibrationSession()


# ---------------------------------------------------------------------------
# Main menu
# ---------------------------------------------------------------------------

def main():
    print(BANNER)
    session = select_session()
    print(f"  Session: {session.session_id}")
    print(f"  Dir:     {session.session_dir}")

    while True:
        print("\n  ┌─────────────────────────────────┐")
        print("  │   CALIBRATION MENU              │")
        print("  ├─────────────────────────────────┤")
        print("  │  1) Phase 1: Plant ID (RC)      │")
        print("  │  2) Phase 2: Slow Follow Me     │")
        print("  │  3) Phase 3: Speed-Up Runs      │")
        print("  │  4) Re-analyze existing data     │")
        print("  │  5) Show current config          │")
        print("  │  6) Apply recommended config     │")
        print("  │  q) Quit                         │")
        print("  └─────────────────────────────────┘")

        try:
            choice = input("  > ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            break

        if choice == "1":
            run_phase1(session)
        elif choice == "2":
            run_phase2(session)
        elif choice == "3":
            run_phase3(session)
        elif choice == "4":
            analyze_existing(session)
        elif choice == "5":
            show_config()
        elif choice == "6":
            apply_recommended(session)
        elif choice in ("q", "quit", "exit"):
            break
        else:
            print("  Invalid choice.")

    print("\n  Goodbye. Session data saved in:")
    print(f"  {session.session_dir}\n")


if __name__ == "__main__":
    main()
