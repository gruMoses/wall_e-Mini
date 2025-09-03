import re
import glob
import statistics
from pathlib import Path


def normalize_heading_error(target_deg: float, heading_deg: float) -> float:
	"""Return shortest signed error in degrees in [-180, 180]."""
	err = target_deg - heading_deg
	while err > 180.0:
		err -= 360.0
	while err < -180.0:
		err += 360.0
	return err


def analyze_latest_log(log_dir: str) -> None:
	logs = sorted(glob.glob(str(Path(log_dir) / "run_*.log")))
	if not logs:
		print("No logs found")
		return
	path = logs[-1]
	print(f"Analyzing {path}")

	# Example heartbeat line pattern (robust spacing):
	# RC ch1=1500 ch2=1500 ch3=2000 ch5=1000  L=126 R=126 ... IMU: 205°→207°
	pat = re.compile(
		 r"ch1=\s*(\d+)\b.*?ch2=\s*(\d+)\b.*?L=\s*(\d+)\b\s*R=\s*(\d+)\b.*?IMU:\s*(\d+)\D+?(\d+)",
		 re.IGNORECASE,
	)

	neutral_errs: list[float] = []
	neutral_corrs: list[float] = []
	all_corrs: list[float] = []

	with open(path, "r", encoding="utf-8", errors="ignore") as f:
		for line in f:
			m = pat.search(line)
			if not m:
				continue
			ch1 = int(m.group(1))
			ch2 = int(m.group(2))
			left = int(m.group(3))
			right = int(m.group(4))
			heading = float(m.group(5))
			target = float(m.group(6))

			# Correction applied to bytes (left minus right)/2
			corr_bytes = (left - right) / 2.0
			all_corrs.append(corr_bytes)

			# Neutral throttle/steering (RC sticks near centered)
			if abs(ch1 - 1500) <= 10 and abs(ch2 - 1500) <= 10:
				err_deg = normalize_heading_error(target, heading)
				neutral_errs.append(err_deg)
				neutral_corrs.append(corr_bytes)

	if not neutral_errs:
		print("No neutral samples found; please include some straight/neutral hold segments.")
		return

	# Estimate proportional gain by linear regression of corr ~ kp * err
	se = sum(e * c for e, c in zip(neutral_errs, neutral_corrs))
	see = sum(e * e for e in neutral_errs) or 1e-9
	kp_est = se / see

	mean_err = statistics.fmean(neutral_errs)
	mean_corr = statistics.fmean(neutral_corrs)
	max_err = max(abs(e) for e in neutral_errs)
	max_corr = max(abs(c) for c in neutral_corrs)
	p95_corr = sorted(abs(c) for c in all_corrs)[max(0, int(0.95 * len(all_corrs)) - 1)] if all_corrs else 0.0

	print(
		"\nSummary:\n"
		f"  samples_neutral = {len(neutral_errs)}\n"
		f"  kp_est         = {kp_est:.2f} bytes/deg\n"
		f"  mean_err       = {mean_err:.2f} deg\n"
		f"  mean_corr      = {mean_corr:.2f} bytes\n"
		f"  max_err        = {max_err:.1f} deg\n"
		f"  max_corr       = {max_corr:.1f} bytes\n"
		f"  p95_corr(all)  = {p95_corr:.1f} bytes\n"
	)

	# Suggest tunings: kp from estimate, deadband ~ 1-2 deg, max_correction ~ p95+margin
	kp_suggest = max(0.1, min(5.0, round(kp_est, 2)))
	deadband_deg = 2.0 if max_err >= 5 else 1.0
	max_correction = int(min(60, max(20, round(p95_corr + 5))))

	print("Suggested config updates:")
	print(f"  imu_steering.kp = {kp_suggest}")
	print(f"  imu_steering.deadband_deg = {deadband_deg}")
	print(f"  imu_steering.max_correction = {max_correction}")
	print("(Adjust ki/kd manually based on overshoot/oscillation; this pass focuses on P and bounds.)")


if __name__ == "__main__":
	analyze_latest_log("/home/pi/wall_e-Mini/logs")


