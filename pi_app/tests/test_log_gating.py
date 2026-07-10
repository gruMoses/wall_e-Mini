"""Tests for pi_app.app.log_gating: the arm-gated telemetry write decision
and the bulk-log cleanup/pruning helper.

pi_app.app.main itself cannot be imported in this test environment (it pulls
in hardware deps like pyserial/depthai at module load time), so the write
gating decision and cleanup logic were extracted into a small dependency-free
module (pi_app/app/log_gating.py) and are tested directly here.
"""
import os
import time

from pi_app.app.log_gating import should_log_tick, cleanup_old_logs


LOG_INTERVAL_S = 0.1  # matches main.py's armed 10 Hz rate
HEARTBEAT_S = 5.0      # matches config.log_disarmed_heartbeat_s default


def _tick(is_armed, now_ts, last_log_ts, **overrides):
    kwargs = dict(
        is_armed=is_armed,
        now_ts=now_ts,
        last_log_ts=last_log_ts,
        log_interval_s=LOG_INTERVAL_S,
        heartbeat_s=HEARTBEAT_S,
        has_event=False,
        charger_inhibit_changed=False,
        emergency_active=False,
        mode_changed=False,
    )
    kwargs.update(overrides)
    return should_log_tick(**kwargs)


# --- (a) armed: a line is written every tick ---------------------------------

def test_armed_writes_every_tick_at_full_rate():
    t = 100.0
    # Slightly more than one interval since last write (avoid float-boundary
    # flakiness right at the >= threshold).
    last_log_ts = 100.0 - LOG_INTERVAL_S - 0.001
    assert _tick(True, t, last_log_ts) is True


def test_armed_simulated_loop_always_logs_on_interval_boundary():
    # Simulate a run of armed ticks spaced slightly over the 10 Hz interval;
    # every one of them should be written (full-rate, unchanged behavior).
    last_log_ts = 0.0
    now = 0.0
    for _ in range(20):
        now += LOG_INTERVAL_S + 0.001
        assert _tick(True, now, last_log_ts) is True
        last_log_ts = now


def test_armed_faster_than_interval_still_gated_like_before():
    # Even when armed, sub-interval ticks are still throttled to log_interval_s
    # -- that part of the behavior is unchanged from before this feature.
    t = 100.0
    last_log_ts = 100.0 - (LOG_INTERVAL_S / 2.0)
    assert _tick(True, t, last_log_ts) is False


# --- (b) disarmed + no event + under heartbeat interval -> no line -----------

def test_disarmed_no_event_under_heartbeat_no_write():
    t = 100.0
    last_log_ts = 100.0 - 1.0  # only 1s since last write, heartbeat is 5s
    assert _tick(False, t, last_log_ts) is False


# --- (c) disarmed + heartbeat interval elapsed -> one line -------------------

def test_disarmed_heartbeat_elapsed_writes():
    t = 100.0
    last_log_ts = 100.0 - HEARTBEAT_S  # exactly heartbeat_s elapsed
    assert _tick(False, t, last_log_ts) is True


def test_disarmed_heartbeat_not_yet_elapsed_by_epsilon_no_write():
    t = 100.0
    last_log_ts = 100.0 - (HEARTBEAT_S - 0.01)
    assert _tick(False, t, last_log_ts) is False


# --- (d) disarmed + event -> immediate write regardless of heartbeat timing --

def test_disarmed_event_forces_immediate_write():
    t = 100.0
    last_log_ts = 100.0 - 0.01  # far under heartbeat
    assert _tick(False, t, last_log_ts, has_event=True) is True


def test_disarmed_charger_inhibit_flip_forces_immediate_write():
    t = 100.0
    last_log_ts = 100.0 - 0.01
    assert _tick(False, t, last_log_ts, charger_inhibit_changed=True) is True


def test_disarmed_emergency_active_forces_immediate_write():
    t = 100.0
    last_log_ts = 100.0 - 0.01
    assert _tick(False, t, last_log_ts, emergency_active=True) is True


def test_disarmed_mode_transition_forces_immediate_write():
    t = 100.0
    last_log_ts = 100.0 - 0.01
    assert _tick(False, t, last_log_ts, mode_changed=True) is True


def test_disarmed_no_reason_and_no_heartbeat_stays_quiet():
    # Sanity check: with everything false and heartbeat not elapsed, nothing
    # forces a write.
    t = 100.0
    last_log_ts = 100.0 - 0.5
    assert _tick(False, t, last_log_ts) is False


# --- (e) cleanup_old_logs prunes bulk logs but preserves tuning artifacts ----

def _age_file(path, seconds_old):
    old_time = time.time() - seconds_old
    os.utime(path, (old_time, old_time))


def test_cleanup_prunes_old_arm_log_and_pid_csv(tmp_path):
    old_arm = tmp_path / "arm_20200101_000000.log"
    old_arm.write_text("x")
    _age_file(old_arm, 8 * 24 * 3600)  # 8 days old, retention is 7

    old_pid_csv = tmp_path / "pid_20200101_000000.csv"
    old_pid_csv.write_text("x")
    _age_file(old_pid_csv, 8 * 24 * 3600)

    old_run = tmp_path / "run_20200101_000000.log"
    old_run.write_text("x")
    _age_file(old_run, 8 * 24 * 3600)

    cleanup_old_logs(tmp_path, days=7)

    assert not old_arm.exists()
    assert not old_pid_csv.exists()
    assert not old_run.exists()


def test_cleanup_preserves_old_tuning_artifacts(tmp_path):
    keepers = [
        tmp_path / "pid_20200101_000000.json",
        tmp_path / "pid_20200101_000000.ndjson",
        tmp_path / "tuned_pid.txt",
        tmp_path / "bias_tune_20200101.json",
        tmp_path / "pid_latest.png",
    ]
    for f in keepers:
        f.write_text("x")
        _age_file(f, 30 * 24 * 3600)  # very old, still must survive

    cleanup_old_logs(tmp_path, days=7)

    for f in keepers:
        assert f.exists(), f"{f.name} should not have been pruned"


def test_cleanup_preserves_recent_bulk_logs(tmp_path):
    recent_arm = tmp_path / "arm_20990101_000000.log"
    recent_arm.write_text("x")  # fresh mtime, well within retention

    recent_pid_csv = tmp_path / "pid_20990101_000000.csv"
    recent_pid_csv.write_text("x")

    cleanup_old_logs(tmp_path, days=7)

    assert recent_arm.exists()
    assert recent_pid_csv.exists()
