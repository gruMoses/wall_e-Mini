"""
VESC CAN motor driver with telemetry reception and a low-voltage watchdog.

Design:
- TX path: set_tracks(left_byte, right_byte) converts byte offsets to RPM and
  sends CAN_PACKET_SET_RPM (extended frame 0x300 + can_id).  While the pack-low latch is
  engaged, set_tracks ramps each track to zero over pack_low_ramp_s and holds it there.
- RX path: background daemon thread reads VESC status broadcasts and updates
  thread-safe telemetry state.
- Pack-low latch (motor cutoff ONLY — no OS shutdown; contract of 2026-11-10): the pack
  voltage is the MINIMUM of the FRESH (< 0.5 s) per-VESC readings; none fresh, or two
  that disagree by > 1.5 V, is untrusted and never latches.  Trusted voltage in
  [floor, threshold) for the delay latches; so does the BMS lowest cell <= 3.30 V on two
  fresh polls (set_bms_cell_reading).  Release: trusted voltage >= threshold +
  hysteresis with zero throttle in MANUAL, or a disarm (set_drive_context).  Optional
  limp home after a disarm (VescConfig.pack_low_limp_*).  Readings below the
  plausibility floor are treated as "pack switched off" (bench event) and never latch.

VESC CAN status frame layout (all big-endian, extended IDs):
  arbitration_id = (packet_id << 8) | vesc_can_id

  CAN_PACKET_STATUS (9):
    [0:4]  int32   RPM
    [4:6]  int16   motor current × 10  → A
    [6:8]  int16   duty cycle × 1000   → fraction

  CAN_PACKET_STATUS_4 (16):
    [0:2]  int16   temp_fet    × 10  → °C
    [2:4]  int16   temp_motor  × 10  → °C
    [4:6]  int16   current_in  × 10  → A
    [6:8]  int16   pid_pos_err × 50  (not used)

  CAN_PACKET_STATUS_5 (27):
    [0:4]  int32   tachometer (not used)
    [4:6]  int16   input_voltage × 10 → V
    [6:8]  int16   (reserved / SOC — not used)

  Note: observed firmware sends 8-byte frames; voltage is at [4:6].
"""

from __future__ import annotations

import logging
import os
import struct
import threading
import time
from dataclasses import dataclass
from typing import Optional

from pi_app.control.mapping import CENTER_OUTPUT_VALUE, MAX_OUTPUT, MIN_OUTPUT

logger = logging.getLogger(__name__)

# VESC CAN packet IDs for status broadcast frames
_CAN_PACKET_STATUS = 9
_CAN_PACKET_STATUS_4 = 16
_CAN_PACKET_STATUS_5 = 27

# Defaults for the pack-low latch when vesc_cfg lacks a field (tests pass a
# minimal config object). Production values and their reasons are in
# config.VescConfig; keep these equal to them.
_PACK_DEFAULTS = {
    "pack_voltage_max_age_s": 0.5,
    "pack_voltage_max_disagree_v": 1.5,
    "pack_low_release_hysteresis_v": 2.0,
    "pack_low_sustain_grace_s": 2.0,
    "pack_voltage_right_alone_offset_v": 1.0,
    "pack_voltage_untrusted_limit_s": 10.0,
    "pack_voltage_trust_confirm_s": 2.0,
    "pack_voltage_frozen_s": 20.0,
    "pack_voltage_frozen_other_moved_s": 2.0,
    "pack_low_seed_on_start": True,
    "pack_low_seed_confirm_s": 2.0,
    "pack_low_ramp_s": 1.0,
    "pack_low_limp_enabled": True,   # owner's decision 2026-11-10
    "pack_low_limp_max_byte_offset": 32,
    "pack_low_limp_floor_v": 42.0,
    "pack_low_limp_delay_s": 5.0,
    "bms_cell_low_trip_v": 3.30,
    "bms_cell_limp_floor_v": 3.15,
    "bms_cell_max_age_s": 20.0,
    "bms_cell_low_consecutive_polls": 2,
    "bms_cell_plausible_min_v": 2.5,
}

# Latch reasons. A voltage reason is released by voltage recovery; a cell
# reason only by a disarm.
_REASON_PACK_V = "pack_voltage"
_REASON_LIMP_FLOOR = "limp_floor"
_REASON_BMS_CELL = "bms_cell"
_VOLTAGE_REASONS = frozenset({_REASON_PACK_V, _REASON_LIMP_FLOOR})

# If the control thread has not called set_tracks()/stop() for this long when
# the latch trips, the RX thread sends RPM 0 itself: nobody is ramping, and
# the VESC ~1 s command timeout would stop the motors anyway.
_CONTROL_SILENT_S = 0.5

# A bus voltage / input current older than this reads as None in get_telemetry().
# Same 0.5 s as the controller's STATUS(9) staleness cut-off; STATUS_4 and
# STATUS_5 have their own frame times, so they get their own check.
_BUS_STALE_S = 0.5


@dataclass
class _MotorTelemetry:
    """Mutable telemetry state for one motor (access only under VescCanDriver._telem_lock)."""
    rpm: Optional[int] = None
    current_a: Optional[float] = None
    duty_cycle: Optional[float] = None
    temp_fet_c: Optional[float] = None
    temp_motor_c: Optional[float] = None
    current_in_a: Optional[float] = None
    voltage_v: Optional[float] = None
    last_status_s: float = 0.0
    last_status4_s: float = 0.0
    last_status5_s: float = 0.0
    v_last_change_s: float = 0.0   # when voltage_v last took a NEW value
    # Peak-hold since the last take_bus_peaks(): the highest bus voltage and the
    # most negative input current (regen into the pack) in ANY frame, so a spike
    # shorter than one log line still shows. None = no frame since the last take.
    bus_v_max: Optional[float] = None
    bus_in_a_min: Optional[float] = None


@dataclass
class VescTelemetry:
    """Combined telemetry snapshot returned to the control loop.

    Constructed by VescCanDriver.get_telemetry() and consumed by Controller
    for closed-loop speed control and slip detection.
    """
    left_rpm: Optional[int] = None
    right_rpm: Optional[int] = None
    left_current_a: Optional[float] = None
    right_current_a: Optional[float] = None
    left_temp_c: Optional[float] = None       # MOSFET / FET (°C)
    right_temp_c: Optional[float] = None
    left_motor_temp_c: Optional[float] = None  # winding / motor (°C)
    right_motor_temp_c: Optional[float] = None
    # Duty cycle (fraction, -1.0..1.0) from STATUS(9). Parsed since the original
    # STATUS decoder but previously dropped here, so nothing downstream could see
    # it. It is the cheapest available load proxy: duty high while eRPM stays low
    # means the motor is loaded or stalling. Needed as a plausibility signal for
    # the velocity-PID re-enable (see docs/gearing_memo.md §e item 3).
    left_duty_cycle: Optional[float] = None
    right_duty_cycle: Optional[float] = None
    voltage_v: Optional[float] = None
    # Per-VESC bus input voltage (STATUS_5, V) and input current (STATUS_4, A;
    # negative = regen into the pack). None when never received or older than
    # _BUS_STALE_S. voltage_v above is the pack voltage the watchdog uses: the
    # MINIMUM of the fresh readings, None while untrusted (pack_voltage_state).
    left_voltage_v: Optional[float] = None
    right_voltage_v: Optional[float] = None
    left_current_in_a: Optional[float] = None
    right_current_in_a: Optional[float] = None
    timestamp: float = 0.0
    can_send_alert: bool = False
    can_send_fail_count: int = 0
    can_rx_frame_count: int = 0
    can_rx_parse_error_count: int = 0
    can_rx_recv_error_count: int = 0
    can_rx_reopen_count: int = 0
    can_rx_last_frame_age_s: Optional[float] = None
    # Per-motor STATUS(9)/RPM frame age — lets a consumer detect ONE motor
    # going silent even while the other keeps transmitting (see get_telemetry()).
    left_status_age_s: Optional[float] = None
    right_status_age_s: Optional[float] = None
    # False if the background RX thread has died (e.g. an unhandled exception
    # escaped _rx_loop). Telemetry can look "fresh" for a while after that
    # because get_telemetry() only reports frame age, not thread liveness.
    rx_thread_alive: Optional[bool] = None
    # True while the recoverable pack-low latch is engaged. set_tracks() ramps
    # the motors to zero and holds them there. Released by a disarm, or by the
    # trusted pack at threshold + hysteresis with zero throttle in MANUAL (a
    # bms_cell reason only by a disarm). It never shuts the Pi down.
    pack_low_latched: bool = False
    # Why: comma-joined sorted reasons ("pack_voltage", "limp_floor",
    # "bms_cell"), None when not latched.
    pack_low_reason: Optional[str] = None
    # "trusted_both" / "trusted_left" / "trusted_right" (one fresh reading) /
    # "stale" (no fresh reading) / "disagree" (fresh readings differ by more
    # than pack_voltage_max_disagree_v) / "no_data" (never evaluated).
    pack_voltage_state: str = "no_data"
    pack_voltage_trusted: bool = False
    # True from the disarm that clears a latch until the pack recovers
    # (threshold + hysteresis, zero throttle): MANUAL only in that session.
    pack_post_latch_active: bool = False
    # True while the pack voltage has been untrusted too long (RESTRICTED;
    # not the latch): MANUAL capped at the limp speed, autonomous ramps to zero.
    pack_restricted: bool = False
    # True while that session also limps (pack_low_limp_enabled only).
    pack_limp_active: bool = False


class VescCanDriver:
    def __init__(
        self,
        channel: str = "can0",
        left_id: int = 2,
        right_id: int = 1,
        max_rpm: int = 3000,
        vesc_cfg=None,  # VescConfig instance; kept as Any to avoid import-time coupling
    ) -> None:
        self._channel = channel
        self._left_id = left_id
        self._right_id = right_id
        self._max_rpm = max_rpm
        self._cfg = vesc_cfg
        self._bus = None  # Lazy-init python-can Bus
        self._bus_lock = threading.Lock()
        # Orders motor commands: held across "decide RPM" + "CAN write" in
        # set_tracks()/stop() and around the RX thread's own RPM 0, so the last
        # frame on the bus is always the latest decision. Lock order:
        # _tx_lock -> _telem_lock / _bus_lock (never the reverse).
        self._tx_lock = threading.Lock()

        # Telemetry state — read/write only under _telem_lock
        self._telem_lock = threading.Lock()
        self._left_telem = _MotorTelemetry()
        self._right_telem = _MotorTelemetry()

        # Background RX thread state
        self._stop_event: Optional[threading.Event] = None
        self._rx_thread: Optional[threading.Thread] = None

        # Pack-low latch state (MOTOR CUTOFF only, never an OS shutdown).
        # Thread ownership:
        #   RX thread only (no lock): _below_floor_active (de-dupes the
        #     below-floor "pack switched off" log).
        #   Shared, read and written ONLY under _telem_lock:
        #     _low_voltage_since    start of the trusted in-band sustain
        #                           (RX thread writes; a disarm on the control
        #                           thread resets it)
        #     _out_of_band_since    first sample of the current run of samples
        #                           that are NOT trusted-in-band (RX thread; a
        #                           disarm resets it). The countdown resets only
        #                           after pack_low_sustain_grace_s of them.
        #     _pack_low_reasons     set of latch reasons; latched = non-empty
        #                           (RX thread: voltage trip/release; control
        #                           thread: BMS cell trip, disarm clear)
        #     _pack_v_used / _pack_v_state  last evaluation (RX thread)
        #     _post_latch_active    post-latch session: MANUAL only, plus limp
        #                           when enabled (control thread sets it on the
        #                           disarm that clears a latch; RX thread ends
        #                           it on recovery)
        #     _ctx_armed / _ctx_manual      drive context (control thread)
        #     _req_neutral, _last_out_rpm, _last_tracks_mono, _ramp_*
        #                           (control thread in set_tracks()/stop(); the
        #                           RX thread zeroes _last_out_rpm when it sends
        #                           RPM 0 itself)
        #     _bms_*                BMS cell trigger (control thread)
        # _pack_low_latched is a property over _pack_low_reasons (kept for the
        # existing tests and callers).
        self._low_voltage_since: Optional[float] = None
        self._out_of_band_since: Optional[float] = None
        self._pack_low_reasons: set = set()
        self._below_floor_active: bool = False
        self._pack_v_used: Optional[float] = None
        self._pack_v_state: str = "no_data"
        self._post_latch_active: bool = False
        self._ctx_armed: Optional[bool] = None
        # False until the controller says otherwise: without a context push a
        # voltage release cannot happen (fail-safe; the controller pushes every
        # tick).
        self._ctx_manual: bool = False
        self._req_neutral: bool = True
        self._last_out_rpm: list = [0, 0]  # [left, right] last RPM sent by the control thread
        self._last_tracks_mono: float = 0.0
        self._req_from_ctx: bool = False   # True once set_drive_context() supplies request_neutral
        self._ramp_start_s: Optional[float] = None   # limit envelope active since
        self._ramp_last_s: float = 0.0
        self._ramp_env: list = [0, 0]
        self._ramp_sign: list = [0, 0]
        # RESTRICTED state (RX thread sets/ends it; a disarm with trusted data
        # ends it on the control thread): the pack voltage has been untrusted
        # for pack_voltage_untrusted_limit_s. No low-voltage claim: MANUAL at
        # the limp cap, autonomous modes ramp to zero.
        self._restricted_active: bool = False
        self._untrusted_since: Optional[float] = None
        self._trusted_since: Optional[float] = None
        # Seeded countdown after a service start (RX thread): armed until the
        # first trusted sample; see pack_low_seed_on_start.
        self._seed_pending: bool = True
        # Frozen-reader detection (RX thread; values under _telem_lock):
        # "left" / "right" when that fresh reader is excluded, else None.
        self._frozen_side: Optional[str] = None
        self._bms_cell_v: Optional[float] = None
        self._bms_cell_state: str = "no_data"
        self._bms_last_poll_id = None
        self._bms_low_count: int = 0

        # CAN send failure tracking (thread-safe via _telem_lock)
        self._send_fail_count: int = 0
        self._send_fail_consecutive: int = 0
        self._send_fail_first_ts: float = 0.0
        self._send_fail_last_ts: float = 0.0
        self._send_fail_alert: bool = False  # True when N consecutive failures within T seconds
        _SEND_FAIL_THRESHOLD = 5        # consecutive failures to trigger alert
        _SEND_FAIL_WINDOW_S = 2.0       # time window for consecutive failure detection
        self._SEND_FAIL_THRESHOLD = _SEND_FAIL_THRESHOLD
        self._SEND_FAIL_WINDOW_S = _SEND_FAIL_WINDOW_S

        # CAN RX health counters — used for live diagnostics and telemetry.
        self._rx_frame_count: int = 0
        # Per-status-type RX counters — the aggregate rx_frame_count can't tell
        # which status packet types are arriving, a blind spot that previously
        # caused a misdiagnosis ("rpm stuck at 0"). These prove ERPM/STATUS(9)
        # packet flow independently of STATUS_4/STATUS_5.
        self._rx_status_count: int = 0   # CAN_PACKET_STATUS   (9)
        self._rx_status4_count: int = 0  # CAN_PACKET_STATUS_4 (16)
        self._rx_status5_count: int = 0  # CAN_PACKET_STATUS_5 (27)
        self._rx_parse_error_count: int = 0
        self._rx_recv_error_count: int = 0
        self._rx_reopen_count: int = 0
        self._rx_last_frame_s: float = 0.0
        self._rx_last_reopen_s: float = 0.0

    # ──────────────────────────────────────────────────────────────────────────
    # Lifecycle
    # ──────────────────────────────────────────────────────────────────────────

    @staticmethod
    def detect(channel: str = "can0") -> bool:
        """Lightweight check for CAN network device presence."""
        return os.path.exists(f"/sys/class/net/{channel}")

    def start(self) -> None:
        """Start background CAN telemetry RX thread."""
        self._stop_event = threading.Event()
        t = threading.Thread(target=self._rx_loop, daemon=True, name="vesc-can-rx")
        t.start()
        self._rx_thread = t
        logger.info("VESC CAN RX thread started (left_id=%d right_id=%d)", self._left_id, self._right_id)

    def stop(self) -> None:
        """Command both motors to zero RPM without stopping telemetry RX.

        A stop is a forced zero, not the operator's zero throttle: it does not
        set _req_neutral (so a charger-inhibit or disarm stop cannot release
        the latch), and it collapses any running ramp envelope to zero so the
        next set_tracks() cannot climb back to the pre-stop speed.
        """
        with self._tx_lock:
            with self._telem_lock:
                self._last_out_rpm = [0, 0]
                self._ramp_env = [0, 0]
                self._last_tracks_mono = time.monotonic()
            self._send_rpm(self._left_id, 0)
            self._send_rpm(self._right_id, 0)
        with self._telem_lock:
            self._send_fail_consecutive = 0

    def shutdown(self) -> None:
        """Terminate background RX thread and close CAN bus."""
        self.stop()
        if self._stop_event is not None:
            self._stop_event.set()
        if self._rx_thread is not None:
            self._rx_thread.join(timeout=2.0)
            self._rx_thread = None
        self._close_bus()

    # ──────────────────────────────────────────────────────────────────────────
    # MotorDriver API
    # ──────────────────────────────────────────────────────────────────────────

    def _limp_on_locked(self) -> bool:
        """Limp floors and speed cap apply (caller holds _telem_lock)."""
        return self._post_latch_active and bool(self._pcfg("pack_low_limp_enabled"))

    def _drive_limit_bytes_locked(self) -> Optional[int]:
        """Largest allowed |byte - centre| now, or None (no limit).

        Caller holds _telem_lock.
          * latched                                           -> 0
          * post-latch session or restricted, not in MANUAL   -> 0
          * restricted, or limp, in MANUAL                    -> limp cap
        """
        if self._pack_low_reasons:
            return 0
        session = self._post_latch_active or self._restricted_active
        if session and not self._ctx_manual:
            return 0
        if self._restricted_active or self._limp_on_locked():
            return max(0, int(self._pcfg("pack_low_limp_max_byte_offset")))
        return None

    def get_drive_limit_bytes(self) -> Optional[int]:
        """Control thread: the pack guard's speed limit in bytes off centre.

        0 = ramp to zero and hold; N = cap at N; None = no limit. The
        controller applies the same limit through its slew state, at the
        pack_low_ramp_s rate, so its output (and its release) never steps;
        set_tracks() enforces it again as the last gate.
        """
        with self._telem_lock:
            return self._drive_limit_bytes_locked()

    def _autonomous_blocked_locked(self) -> bool:
        """An autonomous mode inside a limp / post-latch / restricted session.

        Caller holds _telem_lock. Autonomous modes never move there: the
        output is zero AT ONCE (no ramp; a crawl is at most the 32-byte cap),
        unlike a latch trip, which ramps. Grok 01a1c89a.
        """
        if self._pack_low_reasons:
            return False
        return (self._post_latch_active or self._restricted_active) and not self._ctx_manual

    def get_drive_limit_immediate(self) -> bool:
        """Control thread: True when the limit must apply at once (see
        _autonomous_blocked_locked), False when it ramps."""
        with self._telem_lock:
            return self._autonomous_blocked_locked()

    def set_tracks(self, left_byte: int, right_byte: int) -> None:
        """Send one drive command (control thread, every tick).

        The pack guard is enforced here, the last gate before the CAN bus.
        While a limit is active (see _drive_limit_bytes_locked) each track has
        an envelope that starts at the RPM it had when the limit began and
        shrinks at full speed per pack_low_ramp_s down to the limit (0 or the
        limp cap). A request inside the envelope passes, so a stop is never
        delayed; a track cannot speed up past its envelope, and while the
        envelope is above the limit it cannot reverse beyond the limit.
        Called at ~30 Hz, so the ramp also refreshes the VESC ~1 s timeout.
        _tx_lock keeps the decision and the CAN write together, so an RX-thread
        RPM 0 can never be overtaken by an older, larger command.
        """
        req = [
            self._byte_to_rpm(left_byte, self._max_rpm),
            self._byte_to_rpm(right_byte, self._max_rpm),
        ]
        ramp_s = float(self._pcfg("pack_low_ramp_s"))
        full = abs(self._byte_to_rpm(MAX_OUTPUT, self._max_rpm))
        with self._tx_lock:
            now = time.monotonic()
            with self._telem_lock:
                self._last_tracks_mono = now
                if not self._req_from_ctx:
                    self._req_neutral = req[0] == 0 and req[1] == 0
                limit_b = self._drive_limit_bytes_locked()
                if limit_b is None:
                    self._ramp_start_s = None
                    out = list(req)
                elif self._autonomous_blocked_locked():
                    # Autonomous in a limp / restricted session: zero at once,
                    # and the envelope collapses so a return to MANUAL climbs
                    # from zero.
                    self._ramp_start_s = now
                    self._ramp_last_s = now
                    self._ramp_env = [0, 0]
                    self._ramp_sign = [0, 0]
                    out = [0, 0]
                else:
                    limit = abs(self._byte_to_rpm(CENTER_OUTPUT_VALUE + limit_b, self._max_rpm))
                    if self._ramp_start_s is None:
                        self._ramp_start_s = now
                        self._ramp_last_s = now
                        for i in (0, 1):
                            last = int(self._last_out_rpm[i])
                            self._ramp_env[i] = abs(last)
                            self._ramp_sign[i] = (last > 0) - (last < 0)
                    dt = max(0.0, now - self._ramp_last_s)
                    self._ramp_last_s = now
                    dec = (full / ramp_s) * dt if ramp_s > 0.0 else float("inf")
                    out = [0, 0]
                    for i in (0, 1):
                        prev = self._ramp_env[i]
                        env = max(limit, prev - dec) if prev > limit else limit
                        sign = self._ramp_sign[i]
                        if env > limit and sign > 0:
                            lo, hi = -limit, env
                        elif env > limit and sign < 0:
                            lo, hi = -env, limit
                        else:
                            lo, hi = -limit, limit
                        o = max(lo, min(hi, req[i]))
                        self._ramp_env[i] = min(env, max(abs(o), limit))
                        if o != 0:
                            self._ramp_sign[i] = (o > 0) - (o < 0)
                        out[i] = o
                self._last_out_rpm = list(out)
            self._send_rpm(self._left_id, out[0])
            self._send_rpm(self._right_id, out[1])

    def set_drive_context(
        self,
        armed: bool,
        manual: bool,
        request_neutral: Optional[bool] = None,
    ) -> None:
        """Control thread, every tick: arm state, MANUAL or not, and whether
        the operator's request (before the pack guard) is zero throttle.

        ``request_neutral`` is what "zero commanded throttle" means for the
        releases: the controller holds its output at neutral during a limit,
        so the bytes sent to set_tracks() cannot tell. Once a caller passes it,
        set_tracks() stops deriving it from its own bytes.

        The armed -> disarmed edge clears every latch reason (the operator's
        deliberate act; a latch must never need a restart). It also restarts
        the voltage countdown and the BMS cell count, and starts the
        POST-LATCH session: MANUAL only (autonomous modes ramp to zero) until
        the trusted pack is back at threshold + hysteresis with zero throttle.
        With pack_low_limp_enabled the session also caps the speed and uses
        the limp floors; without it the next MANUAL drive gets a fresh
        voltage_shutdown_delay_s below the 44 V threshold before it latches.
        The same edge ends the RESTRICTED state when the voltage is trusted.
        """
        armed = bool(armed)
        manual = bool(manual)
        msgs = []
        with self._telem_lock:
            prev = self._ctx_armed
            self._ctx_armed = armed
            self._ctx_manual = manual
            if request_neutral is not None:
                self._req_from_ctx = True
                self._req_neutral = bool(request_neutral)
            disarm_edge = prev is True and not armed
            if disarm_edge and self._restricted_active and self._pack_v_state == "trusted_both":
                self._restricted_active = False
                self._ramp_start_s = None
                msgs.append((
                    "VESC: pack voltage RESTRICTED state ended by disarm (trusted %s V, %s)",
                    (self._fmt_v(self._pack_v_used), self._pack_v_state),
                ))
            # The limp floor is a HARD floor: a latch tripped while already
            # limping is NOT cleared by a disarm while its evidence still holds
            # (the 42 V countdown still running, or the cell count still at
            # the limit). Another crawl needs the resting pack back above the
            # floor for the sustain grace (which clears the countdown), then a
            # disarm; charging to 46 V also releases it. Not cleared = no tick
            # of motion under the floor (Grok 01a1c89a).
            hard_floor = self._post_latch_active and self._limp_on_locked()
            need_cells = max(1, int(self._pcfg("bms_cell_low_consecutive_polls")))
            floor_evidence = hard_floor and (
                (_REASON_LIMP_FLOOR in self._pack_low_reasons and self._low_voltage_since is not None)
                or (_REASON_BMS_CELL in self._pack_low_reasons and self._bms_low_count >= need_cells)
            )
            if disarm_edge and self._pack_low_reasons and floor_evidence:
                msgs.append((
                    "VESC: pack-low latch (%s) KEPT through disarm: still below the limp floor "
                    "(pack %s V, %s); rest the pack above %.1f V or charge",
                    (",".join(sorted(self._pack_low_reasons)), self._fmt_v(self._pack_v_used),
                     self._pack_v_state, float(self._pcfg("pack_low_limp_floor_v"))),
                ))
            elif disarm_edge and self._pack_low_reasons:
                cleared = ",".join(sorted(self._pack_low_reasons))
                self._pack_low_reasons = set()
                self._ramp_start_s = None
                # The crawl (or the next drive) starts from zero.
                self._last_out_rpm = [0, 0]
                self._ramp_env = [0, 0]
                self._low_voltage_since = None
                self._out_of_band_since = None
                self._bms_low_count = 0
                self._post_latch_active = True
                if self._limp_on_locked():
                    msgs.append((
                        "VESC: pack-low latch (%s) cleared by disarm; LIMP HOME active: "
                        "MANUAL only, speed cap %d bytes, floors %.1f V / cell %.2f V "
                        "(pack %s V, %s)",
                        (cleared, int(self._pcfg("pack_low_limp_max_byte_offset")),
                         float(self._pcfg("pack_low_limp_floor_v")),
                         float(self._pcfg("bms_cell_limp_floor_v")),
                         self._fmt_v(self._pack_v_used), self._pack_v_state),
                    ))
                else:
                    msgs.append((
                        "VESC: pack-low latch (%s) cleared by disarm; MANUAL only until the "
                        "pack is back at %.1f V; a new countdown starts (pack %s V, %s)",
                        (cleared,
                         float(self._cfg.voltage_shutdown_threshold_v)
                         + float(self._pcfg("pack_low_release_hysteresis_v")) if self._cfg else 0.0,
                         self._fmt_v(self._pack_v_used), self._pack_v_state),
                    ))
        for fmt, args in msgs:
            logger.warning(fmt, *args)

    def set_bms_cell_reading(
        self,
        cell_min_v: Optional[float],
        poll_id,
        age_s: Optional[float],
    ) -> None:
        """Control thread, every tick: the BMS lowest cell, its poll id and age.

        A sample older than bms_cell_max_age_s (or none) is ignored and never
        stops the robot. Each NEW fresh poll at or below the limit (the limp
        floor while limp is active) counts; bms_cell_low_consecutive_polls in a
        row latch with reason bms_cell, released only by a disarm.
        """
        if self._cfg is None:
            return
        max_age = float(self._pcfg("bms_cell_max_age_s"))
        need = max(1, int(self._pcfg("bms_cell_low_consecutive_polls")))
        plausible = float(self._pcfg("bms_cell_plausible_min_v"))
        fresh = (
            cell_min_v is not None
            and age_s is not None
            and 0.0 <= float(age_s) <= max_age
            and float(cell_min_v) >= plausible
        )
        msgs = []
        with self._telem_lock:
            if cell_min_v is None or age_s is None:
                self._bms_cell_state = "no_data"
            elif fresh:
                self._bms_cell_state = "fresh"
            elif float(cell_min_v) < plausible:
                self._bms_cell_state = "implausible"
            else:
                self._bms_cell_state = "stale"
            self._bms_cell_v = float(cell_min_v) if cell_min_v is not None else None
            if not fresh:
                # A gap (dark link, old or implausible sample) breaks the run:
                # "consecutive" means consecutive FRESH polls.
                self._bms_low_count = 0
                return
            if poll_id is None or poll_id == self._bms_last_poll_id:
                return
            self._bms_last_poll_id = poll_id
            limit = float(self._pcfg(
                "bms_cell_limp_floor_v" if self._limp_on_locked() else "bms_cell_low_trip_v"
            ))
            if float(cell_min_v) <= limit:
                self._bms_low_count += 1
            else:
                self._bms_low_count = 0
            if self._bms_low_count >= need and _REASON_BMS_CELL not in self._pack_low_reasons:
                was = bool(self._pack_low_reasons)
                self._pack_low_reasons.add(_REASON_BMS_CELL)
                msgs.append((
                    "VESC: pack-low latch ENGAGED (bms_cell%s): lowest cell %.3f V <= %.2f V "
                    "on %d fresh BMS polls (age %.1f s); ramping motors to zero over %.1f s; "
                    "a disarm clears it",
                    (" added" if was else "", float(cell_min_v), limit, self._bms_low_count,
                     float(age_s), float(self._pcfg("pack_low_ramp_s"))),
                ))
        for fmt, args in msgs:
            logger.warning(fmt, *args)

    # ──────────────────────────────────────────────────────────────────────────
    # Telemetry API
    # ──────────────────────────────────────────────────────────────────────────

    def get_voltage(self) -> Optional[float]:
        """Return the trusted pack voltage (V): min of fresh readings, or None."""
        return self._get_pack_voltage()

    def get_pack_low_latched(self) -> bool:
        """True while the recoverable pack-low latch is engaged."""
        with self._telem_lock:
            return bool(self._pack_low_reasons)

    def get_pack_guard_status(self) -> dict:
        """Snapshot of the pack-low latch for telemetry and the run log."""
        with self._telem_lock:
            return {
                "latched": bool(self._pack_low_reasons),
                "reason": ",".join(sorted(self._pack_low_reasons)) or None,
                "voltage_v": self._pack_v_used,
                "voltage_state": self._pack_v_state,
                "trusted": self._pack_v_state.startswith("trusted"),
                "post_latch": self._post_latch_active,   # MANUAL only until recovery
                "limp_active": self._limp_on_locked(),
                # RESTRICTED: voltage untrusted too long. NOT the latch (no
                # low-voltage claim): MANUAL capped, autonomous ramps to zero.
                "restricted": self._restricted_active,
                "unchecked_for_s": (   # not cross-checked (untrusted or one reader) for
                    round(time.monotonic() - self._untrusted_since, 1)
                    if self._untrusted_since is not None else None
                ),
                "frozen_reader": self._frozen_side,
                "drive_limit_bytes": self._drive_limit_bytes_locked(),
                "ramp_active": self._ramp_start_s is not None,
                "low_since_s": (
                    round(time.monotonic() - self._low_voltage_since, 1)
                    if self._low_voltage_since is not None else None
                ),
                "bms_cell_min_v": self._bms_cell_v,
                "bms_cell_state": self._bms_cell_state,
                "bms_cell_low_count": self._bms_low_count,
            }

    @property
    def _pack_low_latched(self) -> bool:
        # Unlocked read for the RX-thread paths and tests; a single bool() of a
        # set is atomic enough here. Locked callers use _pack_low_reasons.
        return bool(self._pack_low_reasons)

    def _pcfg(self, name: str):
        return getattr(self._cfg, name, _PACK_DEFAULTS[name]) if self._cfg is not None else _PACK_DEFAULTS[name]

    @staticmethod
    def _fmt_v(v: Optional[float]) -> str:
        return "%.1f" % v if v is not None else "none"

    def get_rpm(self, motor: str) -> Optional[int]:
        """Return actual electrical RPM for 'left' or 'right' motor, or None."""
        with self._telem_lock:
            return self._telem_for(motor).rpm

    def get_current(self, motor: str) -> Optional[float]:
        """Return motor phase current (A) for 'left' or 'right', or None."""
        with self._telem_lock:
            return self._telem_for(motor).current_a

    def get_temperature(self, motor: str) -> Optional[float]:
        """Return MOSFET temperature (°C) for 'left' or 'right', or None."""
        with self._telem_lock:
            return self._telem_for(motor).temp_fet_c

    def get_telemetry(self) -> Optional[VescTelemetry]:
        """Return a combined telemetry snapshot for the control loop.

        Returns None if no STATUS frames have been received yet (RPM is the
        primary signal; until at least one motor reports RPM the snapshot is
        not useful for closed-loop control).
        """
        with self._telem_lock:
            l = self._left_telem
            r = self._right_telem
            if l.rpm is None and r.rpm is None:
                return None
            # The watchdog's last trusted value (min of fresh readings), None
            # while untrusted. Was max() of possibly stale readings.
            voltage = self._pack_v_used
            now = time.monotonic()

            def _fresh(value: Optional[float], last_s: float) -> Optional[float]:
                return value if (last_s > 0.0 and now - last_s <= _BUS_STALE_S) else None

            telem = VescTelemetry(
                left_rpm=l.rpm,
                right_rpm=r.rpm,
                left_current_a=l.current_a,
                right_current_a=r.current_a,
                left_temp_c=l.temp_fet_c,
                right_temp_c=r.temp_fet_c,
                left_motor_temp_c=l.temp_motor_c,
                right_motor_temp_c=r.temp_motor_c,
                left_duty_cycle=l.duty_cycle,
                right_duty_cycle=r.duty_cycle,
                voltage_v=voltage,
                left_voltage_v=_fresh(l.voltage_v, l.last_status5_s),
                right_voltage_v=_fresh(r.voltage_v, r.last_status5_s),
                left_current_in_a=_fresh(l.current_in_a, l.last_status4_s),
                right_current_in_a=_fresh(r.current_in_a, r.last_status4_s),
                timestamp=max(l.last_status_s, r.last_status_s),
                left_status_age_s=(now - l.last_status_s) if l.last_status_s > 0.0 else None,
                right_status_age_s=(now - r.last_status_s) if r.last_status_s > 0.0 else None,
            )
            telem.can_send_alert = self._send_fail_alert
            telem.can_send_fail_count = self._send_fail_count
            telem.can_rx_frame_count = self._rx_frame_count
            telem.can_rx_parse_error_count = self._rx_parse_error_count
            telem.can_rx_recv_error_count = self._rx_recv_error_count
            telem.can_rx_reopen_count = self._rx_reopen_count
            telem.can_rx_last_frame_age_s = (
                max(0.0, time.monotonic() - self._rx_last_frame_s)
                if self._rx_last_frame_s > 0.0
                else None
            )
            telem.pack_low_latched = bool(self._pack_low_reasons)
            telem.pack_low_reason = ",".join(sorted(self._pack_low_reasons)) or None
            telem.pack_voltage_state = self._pack_v_state
            telem.pack_voltage_trusted = self._pack_v_state.startswith("trusted")
            telem.pack_limp_active = self._limp_on_locked()
            telem.pack_post_latch_active = self._post_latch_active
            telem.pack_restricted = self._restricted_active
        # Thread liveness is independent of _telem_lock (it's a plain
        # threading.Thread reference, not telemetry state).
        _rx_thread = self._rx_thread
        telem.rx_thread_alive = _rx_thread.is_alive() if _rx_thread is not None else None
        return telem

    def take_bus_peaks(self) -> dict:
        """Return the bus peaks since the previous call, then reset them.

        The RX thread sees every STATUS frame (~48 Hz per VESC), so a spike that
        falls between two log lines still lands here. Per side: the highest bus
        input voltage (V) and the most negative input current (A, regen into the
        pack). A value is None when no frame of that kind arrived since the last
        call. Call it from the log writer only: each call starts a new window.
        """
        with self._telem_lock:
            l = self._left_telem
            r = self._right_telem
            peaks = {
                "left_bus_v_max": l.bus_v_max,
                "right_bus_v_max": r.bus_v_max,
                "left_bus_a_min": l.bus_in_a_min,
                "right_bus_a_min": r.bus_in_a_min,
            }
            l.bus_v_max = l.bus_in_a_min = None
            r.bus_v_max = r.bus_in_a_min = None
        return peaks

    def get_rx_health(self) -> dict:
        """Return CAN RX health counters for diagnostics/telemetry UI."""
        with self._telem_lock:
            health = {
                "rx_frame_count": int(self._rx_frame_count),
                "rx_status_count": int(self._rx_status_count),
                "rx_status4_count": int(self._rx_status4_count),
                "rx_status5_count": int(self._rx_status5_count),
                "rx_parse_error_count": int(self._rx_parse_error_count),
                "rx_recv_error_count": int(self._rx_recv_error_count),
                "rx_reopen_count": int(self._rx_reopen_count),
                "rx_last_frame_age_s": (
                    max(0.0, time.monotonic() - self._rx_last_frame_s)
                    if self._rx_last_frame_s > 0.0
                    else None
                ),
            }
        _rx_thread = self._rx_thread
        health["rx_thread_alive"] = _rx_thread.is_alive() if _rx_thread is not None else None
        return health

    # ──────────────────────────────────────────────────────────────────────────
    # Background CAN RX loop
    # ──────────────────────────────────────────────────────────────────────────

    def _rx_loop(self) -> None:
        stop_event = self._stop_event
        while stop_event is not None and not stop_event.is_set():
            try:
                with self._bus_lock:
                    self._ensure_bus()
            except RuntimeError:
                # python-can unavailable.
                time.sleep(1.0)
                continue
            except Exception as exc:
                # SocketCAN open/init can fail transiently; force reopen path.
                logger.warning("VESC CAN open failed on %s: %s", self._channel, exc)
                self._close_bus()
                with self._telem_lock:
                    self._rx_reopen_count += 1
                self._check_voltage_guarded()   # readings age out to "stale"
                time.sleep(1.0)
                continue

            try:
                with self._bus_lock:
                    msg = self._bus.recv(timeout=0.1)  # type: ignore[union-attr]
            except Exception as exc:
                logger.warning("VESC CAN recv error on %s: %s; reopening bus", self._channel, exc)
                with self._telem_lock:
                    self._rx_recv_error_count += 1
                    self._rx_reopen_count += 1
                self._close_bus()
                time.sleep(0.05)
                continue

            if msg is None:
                try:
                    now = time.monotonic()
                    with self._telem_lock:
                        last_frame_s = self._rx_last_frame_s
                        last_reopen_s = self._rx_last_reopen_s
                    if (
                        last_frame_s > 0.0
                        and (now - last_frame_s) > 1.0
                        and (now - last_reopen_s) > 1.0
                    ):
                        logger.warning(
                            "VESC CAN RX stale for %.2fs on %s; reopening bus",
                            now - last_frame_s,
                            self._channel,
                        )
                        with self._telem_lock:
                            self._rx_reopen_count += 1
                            self._rx_last_reopen_s = now
                        self._close_bus()
                except Exception as exc:
                    # This block must never be allowed to kill the daemon RX
                    # thread silently (see rx_thread_alive below) — contain it
                    # the same way every sibling block in this loop is contained.
                    logger.warning("VESC CAN RX stale-check error on %s: %s", self._channel, exc)
                    with self._telem_lock:
                        self._rx_reopen_count += 1
                    self._close_bus()
                # No frame for 0.1 s: still evaluate the pack watchdog so a
                # silent bus turns the pack voltage "stale" (untrusted) instead
                # of freezing the last value.
                self._check_voltage_guarded()
                continue

            if not msg.is_extended_id:
                continue

            vesc_id = msg.arbitration_id & 0xFF
            packet_id = (msg.arbitration_id >> 8) & 0xFF

            if vesc_id not in (self._left_id, self._right_id):
                continue

            motor = "left" if vesc_id == self._left_id else "right"
            now = time.monotonic()

            try:
                if packet_id == _CAN_PACKET_STATUS:
                    with self._telem_lock:
                        self._rx_frame_count += 1
                        self._rx_status_count += 1
                        self._rx_last_frame_s = now
                    self._parse_status(motor, msg.data, now)
                elif packet_id == _CAN_PACKET_STATUS_4:
                    with self._telem_lock:
                        self._rx_frame_count += 1
                        self._rx_status4_count += 1
                        self._rx_last_frame_s = now
                    self._parse_status4(motor, msg.data, now)
                elif packet_id == _CAN_PACKET_STATUS_5:
                    with self._telem_lock:
                        self._rx_frame_count += 1
                        self._rx_status5_count += 1
                        self._rx_last_frame_s = now
                    self._parse_status5(motor, msg.data, now)
            except Exception as exc:
                with self._telem_lock:
                    self._rx_parse_error_count += 1
                logger.debug("VESC frame parse error (pkt=%d motor=%s): %s", packet_id, motor, exc)

            self._check_voltage_guarded()

    def _check_voltage_guarded(self) -> None:
        """Run the pack watchdog; an error must never kill the RX thread."""
        try:
            self._check_voltage_shutdown()
        except Exception as exc:
            logger.warning("VESC voltage monitor error: %s; reopening bus", exc)
            with self._telem_lock:
                self._rx_parse_error_count += 1
                self._rx_reopen_count += 1
            self._close_bus()

    # ──────────────────────────────────────────────────────────────────────────
    # Frame parsers
    # ──────────────────────────────────────────────────────────────────────────

    def _parse_status(self, motor: str, data: bytes, now: float) -> None:
        """CAN_PACKET_STATUS (9): RPM, motor current, duty cycle."""
        if len(data) < 8:
            return
        rpm = struct.unpack_from(">i", data, 0)[0]
        current_raw = struct.unpack_from(">h", data, 4)[0]
        duty_raw = struct.unpack_from(">h", data, 6)[0]
        with self._telem_lock:
            t = self._telem_for(motor)
            t.rpm = rpm
            t.current_a = current_raw / 10.0
            t.duty_cycle = duty_raw / 1000.0
            t.last_status_s = now

    def _parse_status4(self, motor: str, data: bytes, now: float) -> None:
        """CAN_PACKET_STATUS_4 (16): MOSFET temp, motor temp, input current."""
        if len(data) < 8:
            return
        temp_fet_raw = struct.unpack_from(">h", data, 0)[0]
        temp_motor_raw = struct.unpack_from(">h", data, 2)[0]
        current_in_raw = struct.unpack_from(">h", data, 4)[0]
        with self._telem_lock:
            t = self._telem_for(motor)
            t.temp_fet_c = temp_fet_raw / 10.0
            t.temp_motor_c = temp_motor_raw / 10.0
            t.current_in_a = current_in_raw / 10.0
            if t.bus_in_a_min is None or t.current_in_a < t.bus_in_a_min:
                t.bus_in_a_min = t.current_in_a
            t.last_status4_s = now

    def _parse_status5(self, motor: str, data: bytes, now: float) -> None:
        """CAN_PACKET_STATUS_5 (27): tachometer (ignored), input voltage.

        Observed VESC firmware sends 8-byte frames with voltage at bytes [4:6].
        """
        if len(data) < 6:
            return
        voltage_raw = struct.unpack_from(">h", data, 4)[0]
        with self._telem_lock:
            t = self._telem_for(motor)
            new_v = voltage_raw / 10.0
            if t.voltage_v is None or new_v != t.voltage_v:
                t.v_last_change_s = now      # frozen-reader detection
            t.voltage_v = new_v
            if t.bus_v_max is None or t.voltage_v > t.bus_v_max:
                t.bus_v_max = t.voltage_v
            t.last_status5_s = now

    # ──────────────────────────────────────────────────────────────────────────
    # Low-voltage watchdog — MOTOR CUTOFF only (no OS shutdown)
    # ──────────────────────────────────────────────────────────────────────────

    def _read_pack_voltage(self, now: float):
        """Return (voltage_or_None, state, left_v, right_v) from FRESH readings.

        Both VESCs are on the same pack. A reading counts only when its
        STATUS_5 frame is newer than pack_voltage_max_age_s (a reading that
        stopped updating is not used). The voltage is the MINIMUM of the fresh
        readings: the right VESC reads about 0.9 V high, and the low reader is
        the one that agrees with the BMS. One fresh reading is used alone; the
        right one alone minus pack_voltage_right_alone_offset_v (it reads
        high, so uncorrected it could hold a low pack above the threshold).
        No fresh reading, or two that differ by more than
        pack_voltage_max_disagree_v, is UNTRUSTED (voltage None).

        Physically impossible readers (2026-11-10 follow-up):
          * FROZEN: a fresh reader whose value has not changed for
            pack_voltage_frozen_s while the other reader changed within the
            last pack_voltage_frozen_other_moved_s is excluded (the other is
            then used alone). Both still at rest is normal and not frozen.
          * BELOW FLOOR: a selected value below voltage_shutdown_floor_v while
            frames still arrive is state "below_floor" and untrusted: never a
            latch, never a release, and it counts toward the restricted state.
        Returns (voltage_or_None, state, left_v, right_v, frozen_side).
        """
        max_age = float(self._pcfg("pack_voltage_max_age_s"))
        max_dis = float(self._pcfg("pack_voltage_max_disagree_v"))
        frozen_s = float(self._pcfg("pack_voltage_frozen_s"))
        moved_s = float(self._pcfg("pack_voltage_frozen_other_moved_s"))
        with self._telem_lock:
            l = self._left_telem
            r = self._right_telem
            lv = l.voltage_v if (l.voltage_v is not None and l.last_status5_s > 0.0
                                 and now - l.last_status5_s <= max_age) else None
            rv = r.voltage_v if (r.voltage_v is not None and r.last_status5_s > 0.0
                                 and now - r.last_status5_s <= max_age) else None
            l_chg = l.v_last_change_s
            r_chg = r.v_last_change_s
        frozen = None
        if lv is not None and rv is not None and frozen_s > 0.0:
            if now - l_chg >= frozen_s and now - r_chg <= moved_s:
                frozen = "left"
            elif now - r_chg >= frozen_s and now - l_chg <= moved_s:
                frozen = "right"
        ulv = None if frozen == "left" else lv
        urv = None if frozen == "right" else rv
        if ulv is not None and urv is not None:
            if abs(ulv - urv) > max_dis:
                return None, "disagree", lv, rv, frozen
            v, st = min(ulv, urv), "trusted_both"
        elif ulv is not None:
            v, st = ulv, "trusted_left"
        elif urv is not None:
            v, st = urv - float(self._pcfg("pack_voltage_right_alone_offset_v")), "trusted_right"
        else:
            return None, "stale", lv, rv, frozen
        if self._cfg is not None and v < float(self._cfg.voltage_shutdown_floor_v):
            return None, "below_floor", lv, rv, frozen
        return v, st, lv, rv, frozen

    def _get_pack_voltage(self) -> Optional[float]:
        """Trusted pack voltage now (min of fresh readings), or None."""
        return self._read_pack_voltage(time.monotonic())[0]

    def _check_voltage_shutdown(self) -> None:
        """RX thread: after every frame and every empty 0.1 s receive.

        MOTOR-CUTOFF watchdog. It does NOT shut the Pi down: the Pi runs on its
        own UPS. Regimes on the TRUSTED pack voltage V (floor <= threshold):
          * untrusted (stale / disagree) -> no latch, no release; WARNING once
                                     per episode.
          * V < floor              -> pack switched off / garbage. Stop motors
                                     once, log once, no latch.
          * floor <= V < threshold -> start/continue the sustain countdown;
                                     after the delay, latch (set_tracks() ramps
                                     the tracks to 0).
          * V >= threshold         -> out of band.
        Every sample that is not trusted-in-band PAUSES the countdown; it
        resets after pack_low_sustain_grace_s of such samples in a row.
        Release of a voltage reason: V >= voltage_shutdown_threshold_v +
        pack_low_release_hysteresis_v AND zero commanded throttle AND MANUAL
        (autonomous modes release only by a disarm). The same condition ends
        the post-latch session. With limp on, that session uses
        pack_low_limp_floor_v / pack_low_limp_delay_s instead.
        Runs under _tx_lock so its own RPM 0 is ordered with set_tracks().
        """
        if self._cfg is None:
            return
        with self._tx_lock:
            self._check_voltage_locked_tx()

    def _check_voltage_locked_tx(self) -> None:
        """Body of _check_voltage_shutdown(); the caller holds _tx_lock."""
        now = time.monotonic()
        voltage, state, lv, rv, frozen = self._read_pack_voltage(now)

        base_threshold = float(self._cfg.voltage_shutdown_threshold_v)
        floor = float(self._cfg.voltage_shutdown_floor_v)
        release_v = base_threshold + float(self._pcfg("pack_low_release_hysteresis_v"))
        grace = float(self._pcfg("pack_low_sustain_grace_s"))
        untrusted_limit = float(self._pcfg("pack_voltage_untrusted_limit_s"))
        confirm = float(self._pcfg("pack_voltage_trust_confirm_s"))
        msgs = []   # (level, fmt, args), logged after the lock is released
        send_zero = False
        with self._telem_lock:
            prev_state = self._pack_v_state
            self._pack_v_state = state
            self._pack_v_used = voltage
            limp = self._limp_on_locked()
            if limp:
                threshold = float(self._pcfg("pack_low_limp_floor_v"))
                delay = float(self._pcfg("pack_low_limp_delay_s"))
            else:
                threshold = base_threshold
                delay = float(self._cfg.voltage_shutdown_delay_s)

            # ── Frozen-reader edges ───────────────────────────────────────────
            if frozen != self._frozen_side:
                if frozen is not None:
                    msgs.append((logging.WARNING,
                                 "VESC: %s pack-voltage reading FROZEN (no change for %.0f s while "
                                 "the other moves; left %s V, right %s V) — not used",
                                 (frozen, float(self._pcfg("pack_voltage_frozen_s")),
                                  self._fmt_v(lv), self._fmt_v(rv))))
                else:
                    msgs.append((logging.WARNING,
                                 "VESC: %s pack-voltage reading moving again — used again",
                                 (self._frozen_side,)))
                self._frozen_side = frozen

            # ── Trust edges (WARNING once per episode) ────────────────────────
            if voltage is None:
                if prev_state != state:
                    msgs.append((logging.WARNING,
                                 "VESC: pack voltage UNTRUSTED (%s: left %s V, right %s V, "
                                 "max age %.1f s, max difference %.1f V); no pack-low latch on it",
                                 (state, self._fmt_v(lv), self._fmt_v(rv),
                                  float(self._pcfg("pack_voltage_max_age_s")),
                                  float(self._pcfg("pack_voltage_max_disagree_v")))))
            elif not prev_state.startswith("trusted") and prev_state != "no_data":
                msgs.append((logging.WARNING,
                             "VESC: pack voltage trusted again (%s, %.1f V)", (state, voltage)))

            # ── Below the plausibility floor: pack switched off ───────────────
            if state == "below_floor":
                if not self._below_floor_active:
                    self._below_floor_active = True
                    # RPM 0 only when the control loop is silent: while it is
                    # alive a zero here would be a one-tick brake pulse and the
                    # next tick would restore the full command (Grok 01a1c86b).
                    # A real pack-off leaves the motors unpowered anyway.
                    if now - self._last_tracks_mono > _CONTROL_SILENT_S:
                        send_zero = True
                        self._last_out_rpm = [0, 0]
                        self._ramp_env = [0, 0]
                    msgs.append((logging.WARNING,
                                 "VESC: pack voltage below plausibility floor %.1fV (left %s, right %s) — "
                                 "treating as pack switched off, NOT shutting down",
                                 (floor, self._fmt_v(lv), self._fmt_v(rv))))
            elif voltage is not None:
                self._below_floor_active = False

            # ── RESTRICTED state (no low-voltage claim) ───────────────────────
            # Not cross-checked for pack_voltage_untrusted_limit_s -> restricted:
            # MANUAL at the limp cap, autonomous modes ramp to zero. "Not
            # cross-checked" = untrusted (stale, disagree, below floor) OR only
            # one usable reader (the other VESC silent or frozen): a lone
            # reader stuck high cannot be caught, so autonomy must not depend
            # on it. A lone reader still latches. It ends after
            # pack_voltage_trust_confirm_s of trusted_both readings with zero
            # throttle, or on a disarm with trusted_both data.
            if state != "trusted_both":
                self._trusted_since = None
                if self._untrusted_since is None:
                    self._untrusted_since = now
                if (not self._restricted_active
                        and now - self._untrusted_since >= untrusted_limit):
                    self._restricted_active = True
                    msgs.append((logging.WARNING,
                                 "VESC: pack voltage not cross-checked for %.0f s (%s: left %s V, right %s V) — "
                                 "RESTRICTED: MANUAL capped at %d bytes, autonomous modes ramp to zero "
                                 "(no low-voltage claim)",
                                 (now - self._untrusted_since, state, self._fmt_v(lv), self._fmt_v(rv),
                                  int(self._pcfg("pack_low_limp_max_byte_offset")))))
            else:
                if self._trusted_since is None:
                    self._trusted_since = now
                # The not-cross-checked time ACCUMULATES across short good
                # gaps: it is forgotten only after pack_voltage_trust_confirm_s
                # of cross-checked readings in a row, so 9.9 s bad / 0.1 s good
                # cannot keep autonomy unrestricted (Grok 01a1c86b).
                if now - self._trusted_since >= confirm:
                    self._untrusted_since = None
                if (self._restricted_active and self._req_neutral
                        and now - self._trusted_since >= confirm):
                    self._restricted_active = False
                    self._ramp_start_s = None if not self._pack_low_reasons else self._ramp_start_s
                    msgs.append((logging.WARNING,
                                 "VESC: pack voltage cross-checked for %.1f s (%s, %.1f V) with zero throttle "
                                 "— RESTRICTED state ended",
                                 (now - self._trusted_since, state, voltage)))

            # ── Seeded countdown after a service start ────────────────────────
            # The first trusted sample decides, and only a CROSS-CHECKED
            # (trusted_both) one can seed: below the
            # threshold, the countdown starts already run down to
            # pack_low_seed_confirm_s, so a restart cannot hand out a fresh
            # voltage_shutdown_delay_s of full drive on a low pack. At or
            # above it, no seed. A lone reader cannot seed (a single bad first
            # reader cannot latch a healthy pack); the normal rules apply to it.
            seeded = False
            if self._seed_pending and voltage is not None:
                self._seed_pending = False     # decided once, on the first trusted sample
                if (state == "trusted_both" and bool(self._pcfg("pack_low_seed_on_start"))
                        and floor <= voltage < threshold and self._low_voltage_since is None):
                    seed_confirm = float(self._pcfg("pack_low_seed_confirm_s"))
                    self._low_voltage_since = now - max(0.0, delay - seed_confirm)
                    seeded = True
                    msgs.append((logging.WARNING,
                                 "VESC: pack voltage %.1fV below threshold %.1fV at service start — "
                                 "countdown seeded: pack-low latch in %.0fs if sustained",
                                 (voltage, threshold, min(delay, seed_confirm))))

            # ── Sustain countdown ─────────────────────────────────────────────
            # A low sample starts, continues or completes it: a TRUSTED reading
            # in [floor, threshold), or a disagreement where even the HIGHER
            # reading is in that band (the pack is low whichever VESC is right;
            # this is the only use of disagreeing data, and it can only latch,
            # never release).
            # Only a TRUSTED reading at/above the threshold can clear it, and
            # only after pack_low_sustain_grace_s of them in a row. Untrusted
            # samples (stale, below floor, a disagreement that straddles the
            # threshold) neither latch nor clear: the countdown keeps its start
            # time, so the next low sample may latch at once (fail-safe). A
            # lasting untrusted state is handled by RESTRICTED above.
            band_v = voltage
            if voltage is None and state == "disagree" and lv is not None and rv is not None:
                hi = max(lv, rv)
                if floor <= min(lv, rv) and hi < threshold:
                    band_v = hi
            in_band = band_v is not None and floor <= band_v < threshold
            trusted_out = voltage is not None and not in_band
            if in_band:
                self._out_of_band_since = None
                if self._low_voltage_since is None:
                    self._low_voltage_since = now
                    msgs.append((logging.WARNING,
                                 "VESC: pack voltage %.1fV below %s %.1fV (left %s, right %s) — "
                                 "pack-low latch in %.0fs if sustained",
                                 (band_v, "limp floor" if limp else "threshold", threshold,
                                  self._fmt_v(lv), self._fmt_v(rv), delay)))
                elif now - self._low_voltage_since >= delay and not seeded:
                    reason = _REASON_LIMP_FLOOR if limp else _REASON_PACK_V
                    if reason not in self._pack_low_reasons:
                        was = bool(self._pack_low_reasons)
                        self._pack_low_reasons.add(reason)
                        if not was and now - self._last_tracks_mono > _CONTROL_SILENT_S:
                            # Nobody is driving (control loop silent): no ramp
                            # to run, so zero the motors here.
                            send_zero = True
                            self._last_out_rpm = [0, 0]
                        msgs.append((logging.WARNING,
                                     "VESC: pack-low latch ENGAGED (%s): pack %.1fV < %.1fV for "
                                     "%.0fs (left %s, right %s, %s); ramping motors to zero over "
                                     "%.1fs; release at >= %.1fV with zero throttle in MANUAL, "
                                     "or a disarm",
                                     (reason, band_v, threshold, delay, self._fmt_v(lv),
                                      self._fmt_v(rv), state,
                                      float(self._pcfg("pack_low_ramp_s")), release_v)))
            elif trusted_out:
                if self._out_of_band_since is None:
                    self._out_of_band_since = now
                if (self._low_voltage_since is not None
                        and now - self._out_of_band_since >= grace):
                    msgs.append((logging.INFO,
                                 "VESC: pack voltage out of the low band for %.1fs (%s, %s V) — "
                                 "low-voltage countdown cleared",
                                 (now - self._out_of_band_since, state, self._fmt_v(voltage))))
                    self._low_voltage_since = None

            # ── Release (trusted only) ────────────────────────────────────────
            if (voltage is not None and voltage >= release_v
                    and self._req_neutral and self._ctx_manual):
                voltage_reasons = self._pack_low_reasons & _VOLTAGE_REASONS
                if voltage_reasons:
                    self._pack_low_reasons -= voltage_reasons
                    msgs.append((logging.WARNING,
                                 "VESC: pack voltage recovered to %.1fV (>= %.1fV, zero throttle) "
                                 "— pack-low latch (%s) released%s",
                                 (voltage, release_v, ",".join(sorted(voltage_reasons)),
                                  ("; still latched: " + ",".join(sorted(self._pack_low_reasons)))
                                  if self._pack_low_reasons else "")))
                if self._post_latch_active and not self._pack_low_reasons:
                    self._post_latch_active = False
                    self._low_voltage_since = None
                    msgs.append((logging.WARNING,
                                 "VESC: pack voltage %.1fV >= %.1fV with zero throttle — "
                                 "post-latch session (MANUAL only%s) ended",
                                 (voltage, release_v, ", limp home" if limp else "")))
        for level, fmt, args in msgs:
            logger.log(level, fmt, *args)
        if send_zero:
            self._send_rpm(self._left_id, 0)
            self._send_rpm(self._right_id, 0)

    def _trigger_low_voltage_shutdown(self, voltage: float) -> None:
        """Engage the voltage pack-low latch — NO OS shutdown (tests and tools).

        The sudo shutdown path is intentionally GONE: the Pi is UPS-powered.
        set_tracks() ramps to zero while latched. Motors are zeroed here only
        when the control thread is silent (nothing to ramp).
        """
        with self._tx_lock:
            send_zero = False
            with self._telem_lock:
                already = bool(self._pack_low_reasons)
                self._pack_low_reasons.add(_REASON_PACK_V)
                if not already and time.monotonic() - self._last_tracks_mono > _CONTROL_SILENT_S:
                    send_zero = True
                    self._last_out_rpm = [0, 0]
            if not already:
                logger.warning(
                    "VESC: pack-low latch ENGAGED (pack_voltage): %.1fV < %.1fV for %.0fs",
                    voltage,
                    self._cfg.voltage_shutdown_threshold_v,  # type: ignore[union-attr]
                    self._cfg.voltage_shutdown_delay_s,      # type: ignore[union-attr]
                )
            if send_zero:
                self._send_rpm(self._left_id, 0)
                self._send_rpm(self._right_id, 0)

    # ──────────────────────────────────────────────────────────────────────────
    # Internal helpers
    # ──────────────────────────────────────────────────────────────────────────

    def _telem_for(self, motor: str) -> _MotorTelemetry:
        """Return the telemetry object for 'left' or 'right' (caller holds lock)."""
        return self._left_telem if motor == "left" else self._right_telem

    def _ensure_bus(self) -> None:
        if self._bus is not None:
            return
        try:
            import can  # type: ignore
        except Exception as exc:
            raise RuntimeError(
                "python-can is required for VESC CAN control. "
                "Install with: pip install python-can"
            ) from exc
        self._bus = can.interface.Bus(channel=self._channel, bustype="socketcan")

    def _close_bus(self) -> None:
        with self._bus_lock:
            bus = self._bus
            self._bus = None
        if bus is None:
            return
        try:
            shutdown = getattr(bus, "shutdown", None)
            if callable(shutdown):
                shutdown()
        except Exception:
            pass

    def _send_rpm(self, can_id: int, rpm: int) -> None:
        try:
            with self._bus_lock:
                self._ensure_bus()
        except Exception as exc:
            # RuntimeError (python-can missing) or a SocketCAN OSError/CanError
            # from Bus() construction — either way this must never escape to
            # the caller (set_tracks -> Controller.process -> the unguarded
            # main loop). Close/reset the bus and count it like a send failure
            # so callers see it in can_send_alert/can_send_fail_count.
            now = time.monotonic()
            with self._telem_lock:
                self._send_fail_count += 1
                self._send_fail_consecutive += 1
                if self._send_fail_first_ts == 0.0:
                    self._send_fail_first_ts = now
                self._send_fail_last_ts = now
                if (self._send_fail_consecutive >= self._SEND_FAIL_THRESHOLD
                        and (now - self._send_fail_first_ts) <= self._SEND_FAIL_WINDOW_S):
                    if not self._send_fail_alert:
                        self._send_fail_alert = True
                        logger.warning(
                            'VESC CAN send alert: %d consecutive failures in %.1fs - %s',
                            self._send_fail_consecutive,
                            now - self._send_fail_first_ts,
                            exc,
                        )
            self._close_bus()
            return

        try:
            import can  # type: ignore
        except Exception:
            return

        arbitration_id = 0x300 + can_id
        data = struct.pack(">i", int(rpm))
        msg = can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=True)
        close_bus = False
        try:
            with self._bus_lock:
                assert self._bus is not None
                self._bus.send(msg)
            with self._telem_lock:
                self._send_fail_consecutive = 0
                if self._send_fail_alert:
                    logger.info('VESC CAN send recovered after %d failures', self._send_fail_count)
                    self._send_fail_alert = False
        except Exception as exc:
            now = time.monotonic()
            with self._telem_lock:
                self._send_fail_count += 1
                self._send_fail_consecutive += 1
                if self._send_fail_first_ts == 0.0:
                    self._send_fail_first_ts = now
                self._send_fail_last_ts = now
                if (self._send_fail_consecutive >= self._SEND_FAIL_THRESHOLD
                        and (now - self._send_fail_first_ts) <= self._SEND_FAIL_WINDOW_S):
                    if not self._send_fail_alert:
                        self._send_fail_alert = True
                        logger.warning(
                            'VESC CAN send alert: %d consecutive failures in %.1fs - %s',
                            self._send_fail_consecutive,
                            now - self._send_fail_first_ts,
                            exc,
                        )
                close_bus = True
        if close_bus:
            self._close_bus()

    @staticmethod
    def _byte_to_rpm(byte_value: int, max_rpm: Optional[int] = None) -> int:
        max_rpm_val = 3000 if max_rpm is None else max_rpm
        clamped = min(MAX_OUTPUT, max(MIN_OUTPUT, byte_value))
        if clamped == CENTER_OUTPUT_VALUE:
            return 0
        if clamped > CENTER_OUTPUT_VALUE:
            span_up = MAX_OUTPUT - CENTER_OUTPUT_VALUE
            normalized = (clamped - CENTER_OUTPUT_VALUE) / span_up
            return int(round(normalized * max_rpm_val))
        else:
            normalized = (CENTER_OUTPUT_VALUE - clamped) / CENTER_OUTPUT_VALUE
            return -int(round(normalized * max_rpm_val))
