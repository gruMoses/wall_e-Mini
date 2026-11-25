"""Charger plug policy: when WALL-E's charger gets mains power.

Pure decision logic, no I/O, no threads (the ChargeManager owns those). One
``decide()`` per poll. docs/charger_plug.md has the operating notes and
docs/charger_plug_research_2026-11-16.md the measurements behind the numbers.

Modes (operator-chosen, persisted by the manager):

* ``hold``  keep the main pack near 50 %. The trigger is the RESTED CELL
            VOLTAGE, not the BMS percentage: the BMS undercounts the idle
            drain by about 16 %, so a hold on its percentage walks the pack
            down by several points a day (measured 2026-11-15/16). Plug on
            when the rested cell voltage is at or under ``hold_on_cell_v``;
            the top-up is sized in amp-hours from the pack's own rest curve
            (``ocv_table``) and counted from the BMS current, which is
            accurate at charger currents.
* ``full``  charge to full once (Kevin's word): on until the pack is full
            and the current has tapered, then off; a top-up when the rested
            cells fall to ``full_retop_cell_v``; back to ``hold`` by itself
            once the pack has been high and is rested under
            ``full_return_cell_v`` (a drive). A full charge is also the only
            time the BMS balances (balancer start 4000 mV per cell).
* ``on`` / ``off``  manual.

Health (Kevin, 2026-11-16: do not charge an unhealthy pack):

* hard faults LATCH and force the plug off in every mode but ``off`` until
  the operator clears them: BMS hardware flags, a BMS over-voltage or charge
  over-current trip, a deeply discharged cell, a cell spread far above this
  pack's normal (at rest or under charge), the wrong cell count, BMS voltages
  that disagree, over temperature, a temperature rise during a charge, a
  charge that overruns its time or amp-hour budget, amp-hours that go in
  with no voltage rise.
* soft inhibits block ``hold`` and ``full`` while they last: pack too cold
  or too warm, a cell near the BMS over-voltage trip, a BMS temperature
  trip, the charge FET open.
* the cell spread is flagged against this pack's normal (about 30 mV in the
  middle, under 10 mV at the top); a fixed small threshold would cry wolf.

Armed (Kevin, 2026-11-25, audit N1): no charge starts while the robot is
armed. A hold or full charge that is already running keeps going under all
its normal stop rules, so the charger inhibit stays on while it runs (the
manager reports it to the main loop). "Running" means the BMS showed charge
current on ``armed_keep_charge_polls`` polls of the session while the robot
was disarmed. Anything else goes off at the arm: a plug that is on with no
current (the lead is not on the robot), a dark-BMS wake or rescue pulse (no
current reading), a plug switched on from the Kasa app while armed.

BMS dark (no fresh reading): the plug goes off after ``soc_unknown_off_s``.
Then, only while the VESCs give an independent pack voltage and temperature
and the last BMS reading was healthy: a few short wake pulses (a charger
above the pack wakes a sleeping Daly BMS), and below ``rescue_voltage_v`` a
bounded top-up so the pack never runs flat. Kevin chose both on 2026-11-16.
Both stop for good when the pack voltage does not answer the charger, and
the rescue stops when the pack does not keep what it was given (an
independent review asked for these checks: blind charging must not feed a
failing cell).

Nothing here touches motion.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

MODES = ("hold", "full", "on", "off")

# Daly 0x98 labels (pi_app/hardware/bms.py). Warnings are not listed: the
# pack shows "Cell OVP Warning" and "Pack OVP Warning" at every full charge.
# A BMS over-voltage trip is a hard fault here: the charger ends at 4.20 V per
# cell and the BMS trips at 4.25 V, so a trip means one cell ran ahead.
HARD_FLAGS = frozenset({
    "Charge FET Failure", "Discharge FET Failure", "Cell Mismatch",
    "Cell Voltage Diff Trip", "Temp Sensor Diff Trip",
    "Cell OVP Trip", "Pack OVP Trip", "Charge OCP Trip",
})
SOFT_FLAGS = frozenset({"Charge OTP Trip", "Charge UTP Trip", "SOC High Trip"})


@dataclass(frozen=True)
class ChargeInputs:
    soc_pct: Optional[float] = None
    pack_v: Optional[float] = None
    pack_a: Optional[float] = None       # positive = charging
    bms_fresh: bool = False              # link up and the sample recent
    plug_on: Optional[bool] = None       # None = plug not reachable
    cell_min_mv: Optional[float] = None
    cell_max_mv: Optional[float] = None
    temp_min_c: Optional[float] = None
    temp_max_c: Optional[float] = None
    charge_fet_on: Optional[bool] = None
    error_flags: Tuple[str, ...] = ()
    num_cells: Optional[int] = None
    vesc_v: Optional[float] = None       # pack voltage from the VESCs, independent of the BMS
    vesc_temp_c: Optional[float] = None  # VESC heatsink temperature, the ambient proxy in the dark
    armed: bool = False


@dataclass(frozen=True)
class ChargeDecision:
    want_on: Optional[bool]              # None = leave the plug as it is
    reason: str
    mode: str                            # mode after this tick
    phase: str = ""
    alerts: Tuple[Dict[str, str], ...] = ()
    full_completed: bool = False
    info: Dict[str, Any] = field(default_factory=dict)
    # A hold or full charge is running (see the module docstring, "Armed"):
    # it goes on when the robot arms, and the charger inhibit follows it.
    charge_running: bool = False


@dataclass
class _Session:
    kind: str                            # hold / full / manual / wake / rescue
    t0: float
    last_t: float
    ah: float = 0.0
    target_ah: Optional[float] = None
    start_tmax: Optional[float] = None
    start_cell_v: Optional[float] = None    # first cell voltage under charge current
    charge_polls: int = 0                   # polls with charge current while disarmed


def _num(x: Any, lo: float, hi: float) -> Optional[float]:
    if x is None or isinstance(x, bool):
        return None
    try:
        f = float(x)
    except (TypeError, ValueError):
        return None
    if f != f or f < lo or f > hi:
        return None
    return f


def soc_from_cell_v(cell_v: float, table: Tuple[Tuple[float, float], ...]) -> float:
    """Rested cell voltage -> approximate state of charge (%), from the pack's own curve."""
    if not table:
        return 0.0
    if cell_v <= table[0][0]:
        return float(table[0][1])
    for (v0, s0), (v1, s1) in zip(table, table[1:]):
        if cell_v <= v1:
            if v1 <= v0:
                return float(s1)
            return float(s0 + (s1 - s0) * (cell_v - v0) / (v1 - v0))
    return float(table[-1][1])


class ChargePolicy:
    def __init__(self, cfg, mode: str = "hold") -> None:
        self.cfg = cfg
        self.mode = mode if mode in MODES else "hold"
        self.fault: Optional[Dict[str, str]] = None
        self.topped = False
        self._full_seen_high = False
        self._done_since: Optional[float] = None
        self._session: Optional[_Session] = None
        self._pending: Tuple[Optional[str], Optional[float]] = (None, None)
        self._first_tick_t: Optional[float] = None
        self._last_known_on: Optional[bool] = None
        self._last_plug_off_t: Optional[float] = None
        self._last_load_t: Optional[float] = None
        self._no_current_retry_t = 0.0
        self._offplug_charge_since: Optional[float] = None
        self._last_hold_start_t: Optional[float] = None
        # A hard-fault condition seen on fewer than fault_confirm_polls polls in a
        # row: the plug goes off at once, the latch waits. The BMS reader does not
        # check frame checksums and one garbled frame (281.5 A, 2026-11-16) must
        # not hold the charger off until a person comes.
        self._suspects: Dict[str, Tuple[int, str]] = {}
        self._suspect_seen: set = set()
        self._vesc_diff_count = 0
        self._mode_age0 = 0.0                 # age of the mode when this process started
        self._mode_t0: Optional[float] = None
        # dark handling
        self._unknown_since: Optional[float] = None
        self._dark_action: Optional[Tuple[str, float, Optional[float]]] = None
        self._dark_blocked: Optional[str] = None
        self._last_fresh_inhibit: Optional[str] = None
        self._wake_times: List[float] = []
        self._next_wake_t: Optional[float] = None
        self._rescue_times: List[float] = []
        self._last_rescue_t: Optional[float] = None
        self._last_rescue_end_t: Optional[float] = None
        self._last_rescue_v0: Optional[float] = None
        self._spread_charge_since: Optional[float] = None
        # cell spread
        self._spread_cand: Optional[str] = None
        self._spread_cand_since = 0.0
        self._spread_level = "ok"
        self._spread_snapshot: Optional[Tuple[float, float, str]] = None

    # -- operator ---------------------------------------------------------

    def set_mode(self, mode: str, now: Optional[float] = None) -> None:
        if mode not in MODES:
            raise ValueError(f"unknown mode {mode!r}")
        self.mode = mode
        self.topped = False
        self._full_seen_high = False
        self._done_since = None
        self._pending = (None, None)
        self._no_current_retry_t = 0.0
        self._dark_action = None
        self._last_hold_start_t = None
        self._mode_age0 = 0.0
        self._mode_t0 = now
        if self._session is not None:
            # The plug is on: the new mode owns the charge from here.
            t = self._session.last_t if now is None else now
            self._session = _Session(kind=self._kind_for_mode(), t0=t, last_t=t,
                                     start_tmax=self._session.start_tmax,
                                     charge_polls=self._session.charge_polls)

    def clear_fault(self) -> bool:
        had = self.fault is not None
        self.fault = None
        self._suspects.clear()
        self._spread_level = "ok"
        self._spread_cand = None
        return had

    def note_plug(self, on: bool) -> None:
        """The manager switched the plug: remember it for polls where the plug
        does not answer (an unreachable plug keeps its last known state)."""
        self._last_known_on = bool(on)

    def restore_mode_age(self, age_s: Any) -> None:
        """How long the mode had been set before this process started (the
        48 h limit on ``full`` must survive deploy restarts)."""
        a = _num(age_s, 0.0, 10 * 365 * 86400.0)
        self._mode_age0 = a if a is not None else 0.0
        self._mode_t0 = None

    def restore_fault(self, fault: Optional[Dict[str, str]]) -> None:
        if isinstance(fault, dict) and fault.get("code") and fault.get("text"):
            self.fault = {"code": str(fault["code"]), "text": str(fault["text"])}

    # -- helpers ----------------------------------------------------------

    def _kind_for_mode(self) -> str:
        return {"hold": "hold", "full": "full", "on": "manual"}.get(self.mode, "manual")

    def _charge_running(self) -> bool:
        """A hold or full charge the BMS saw charging while the robot was
        disarmed: the one charge that goes on when the robot arms."""
        s = self._session
        return bool(s is not None and s.kind in ("hold", "full")
                    and s.charge_polls >= self.cfg.armed_keep_charge_polls)

    def _latch(self, code: str, text: str) -> None:
        if self.fault is None:
            self.fault = {"code": code, "text": text}

    def _suspect(self, code: str, text: str) -> None:
        """A hard-fault condition on this poll: latch after fault_confirm_polls in a row."""
        n = self._suspects.get(code, (0, ""))[0] + 1
        self._suspects[code] = (n, text)
        self._suspect_seen.add(code)
        if n >= self.cfg.fault_confirm_polls:
            self._latch(code, text)

    def _session_limit_s(self, s: _Session) -> Optional[float]:
        cfg = self.cfg
        if s.kind == "hold":
            target = s.target_ah if s.target_ah is not None else cfg.hold_charge_default_ah
            return cfg.charge_time_margin_s + target / cfg.charge_rate_floor_a * 3600.0
        if s.kind == "full":
            return cfg.full_max_charge_s
        if s.kind == "wake":
            return cfg.wake_pulse_s
        if s.kind == "rescue":
            return cfg.rescue_charge_s
        return None   # manual

    def _track_session(self, inp: ChargeInputs, now: float, fresh: bool, a: Optional[float]) -> bool:
        """Follow the plug: start, integrate and end the charge session.
        Returns True while the plug counts as on."""
        if inp.plug_on is not None:
            self._last_known_on = bool(inp.plug_on)
        on = bool(self._last_known_on)
        if on:
            if self._session is None:
                kind, target = self._pending
                self._session = _Session(kind=kind or self._kind_for_mode(), t0=now, last_t=now,
                                         target_ah=target, start_tmax=_num(inp.temp_max_c, -40, 120) if fresh else None)
                if self._session.kind == "hold":
                    self._last_hold_start_t = now
            else:
                s = self._session
                dt = max(0.0, min(now - s.last_t, 60.0))
                if fresh and a is not None and a > 0.0:
                    s.ah += a * dt / 3600.0
                s.last_t = now
                if (s.start_cell_v is None and fresh and a is not None
                        and a >= self.cfg.rest_load_current_a):
                    cmin = _num(inp.cell_min_mv, 1500.0, 4500.0)
                    cmax = _num(inp.cell_max_mv, 1500.0, 4500.0)
                    if cmin is not None and cmax is not None:
                        s.start_cell_v = (cmin + cmax) / 2000.0
                if s.start_tmax is None and fresh:
                    s.start_tmax = _num(inp.temp_max_c, -40, 120)
            if (fresh and a is not None and a >= self.cfg.rest_load_current_a
                    and not inp.armed):
                # Counted only while disarmed: a charge that first shows
                # current while armed never counts as running.
                self._session.charge_polls += 1
            self._pending = (None, None)
        elif self._session is not None:
            self._session = None
            self._last_plug_off_t = now
        return on

    # -- one poll ---------------------------------------------------------

    def decide(self, inp: ChargeInputs, now: float) -> ChargeDecision:
        cfg = self.cfg
        if self._first_tick_t is None:
            self._first_tick_t = now
            if self._last_plug_off_t is None:
                self._last_plug_off_t = now      # unknown history: settle before the first reading
        if self._mode_t0 is None:
            self._mode_t0 = now

        soc = _num(inp.soc_pct, 0.0, 100.0)
        v = _num(inp.pack_v, 20.0, 70.0)
        cmin = _num(inp.cell_min_mv, 1500.0, 4500.0)
        cmax = _num(inp.cell_max_mv, 1500.0, 4500.0)
        fresh = (bool(inp.bms_fresh) and v is not None and cmin is not None
                 and cmax is not None and cmax >= cmin)
        # The charger gives 3.6 A. More charge current than max_charge_current_a
        # while parked is a garbled frame: the sample's current is not used.
        a = _num(inp.pack_a, -cfg.max_discharge_current_a, cfg.max_charge_current_a) if fresh else None
        if fresh and a is not None and abs(a) > cfg.rest_load_current_a:
            self._last_load_t = now

        on = self._track_session(inp, now, fresh, a)
        s = self._session
        cell_mid = (cmin + cmax) / 2000.0 if fresh else None
        spread = (cmax - cmin) if fresh else None
        since_off = now - self._last_plug_off_t if self._last_plug_off_t is not None else 1e9
        since_load = now - self._last_load_t if self._last_load_t is not None else 1e9
        rested = bool(fresh and not on and a is not None and abs(a) <= cfg.rest_max_current_a
                      and since_off >= cfg.rest_after_charge_s and since_load >= cfg.rest_after_load_s)

        alerts: List[Dict[str, str]] = []
        # Charge current with the plug off: the charger sits on another outlet,
        # outside this manager's control.
        if fresh and not on and a is not None and a >= cfg.rest_load_current_a:
            if self._offplug_charge_since is None:
                self._offplug_charge_since = now
            if now - self._offplug_charge_since >= cfg.rest_after_load_s:
                alerts.append({"level": "warning", "code": "charge_plug_off",
                               "text": f"the pack is charging at {a:.1f} A with the plug off: the charger is on "
                                       "another outlet, or the plug's relay is stuck. This manager cannot stop it"})
        elif fresh:
            self._offplug_charge_since = None
        if fresh:
            self._suspect_seen = set()
            charging = bool(on and a is not None and a >= cfg.rest_load_current_a)
            self._update_spread(rested, charging, cell_mid, spread, now)
            self._check_hard_faults(inp, a, v, cmin, cmax)
            self._check_vesc_agreement(inp, rested, v, alerts)
        self._check_session_faults(inp, now, fresh, cell_mid)
        if fresh:
            for code in [c for c in self._suspects if c not in self._suspect_seen]:
                del self._suspects[code]        # not seen on this poll: the count starts again

        want, reason, phase, full_completed = self._decide_mode(
            inp, now, fresh, on, rested, soc, v, a, cmin, cmax, cell_mid, alerts)
        running = self._charge_running()
        if inp.armed and running and want is not False:
            reason = "robot armed, charge already running (no drive until it ends): " + reason

        if self.fault is not None:
            alerts.insert(0, {"level": "fault", "code": self.fault["code"], "text": self.fault["text"]})
        if self._spread_level in ("notice", "warning") and self._spread_snapshot is not None:
            sp, cv, zone = self._spread_snapshot
            base = cfg.spread_baseline_mv[1] if zone == "top" else cfg.spread_baseline_mv[0]
            extra = "well above" if self._spread_level == "warning" else "above"
            alerts.append({"level": self._spread_level, "code": "cell_spread",
                           "text": f"cell spread {sp:.0f} mV at {cv:.2f} V per cell, {extra} this pack's "
                                   f"normal of about {base:.0f} mV: do a full charge to balance"})

        info: Dict[str, Any] = {
            "cell_mid_v": round(cell_mid, 3) if cell_mid is not None else None,
            "cell_spread_mv": spread,
            "rested": rested,
            "soc_by_voltage": (round(soc_from_cell_v(cell_mid, cfg.ocv_table))
                               if (cell_mid is not None and rested) else None),
            "topped": self.topped,
            "mode_age_s": round(self._mode_age0 + (now - self._mode_t0)),
            "dark_blocked": self._dark_blocked,
            "wake_pulses_24h": len(self._wake_times),
            "rescues_24h": len(self._rescue_times),
            "session": ({"kind": s.kind, "ah": round(s.ah, 3), "age_s": round(now - s.t0),
                         "target_ah": s.target_ah} if s is not None else None),
        }
        return ChargeDecision(want, reason, self.mode, phase, tuple(alerts), full_completed, info,
                              charge_running=running)

    def _decide_mode(self, inp, now, fresh, on, rested, soc, v, a, cmin, cmax, cell_mid, alerts):
        mode = self.mode
        if mode == "off":
            return False, "mode off", "manual", False
        if self.fault is not None:
            return False, f"FAULT: {self.fault['text']}", "fault", False
        if mode == "on":
            return True, "mode on (manual)", "manual", False
        if inp.armed and not self._charge_running():
            # No charge starts while armed, and a plug that is on without a
            # running charge goes off: lead off the robot, charger not started
            # yet, a dark-BMS pulse, a switch from the Kasa app.
            self._pending = (None, None)
            return False, "robot armed: no charging", "armed", False
        if not fresh:
            return self._decide_dark(inp, now, on, alerts) + (False,)
        self._dark_reset()
        if self._session is not None and self._session.kind in ("wake", "rescue"):
            # The reading is back mid-pulse: the mode owns the charge from here.
            self._session.kind = self._kind_for_mode()
            self._session.t0 = now
            self._session.target_ah = None

        inhibit = self._soft_inhibit(inp, mode, v, cmin, cmax)
        self._last_fresh_inhibit = inhibit[1] if inhibit is not None else None
        if inhibit is not None:
            code, text = inhibit
            if mode == "full" and code == "cell_high":
                # One cell reached the stop before the pack was full. That is
                # the top for this charge: no retry loop (it would chatter the
                # relay). The spread check says whether the pack needs help.
                self.topped = True
                self._done_since = None
            alerts.append({"level": "warning", "code": "inhibit", "text": f"charging inhibited: {text}"})
            return False, f"inhibited: {text}", "inhibited", False

        if mode == "full":
            if cell_mid >= self.cfg.full_seen_cell_v:
                self._full_seen_high = True
            used = rested and self._full_seen_high and cell_mid <= self.cfg.full_return_cell_v
            expired = self._mode_age0 + (now - self._mode_t0) >= self.cfg.full_expire_s
            if used or expired:
                # Used after the full charge, or nobody drove for 48 h: back to
                # holding 50 % (a pack kept full ages faster).
                self.mode = "hold"
                self.topped = False
                self._full_seen_high = False
                self._done_since = None
                self._mode_age0 = 0.0
                self._mode_t0 = now
                if self._session is not None:
                    self._session = _Session(kind="hold", t0=now, last_t=now,
                                             start_tmax=self._session.start_tmax,
                                             charge_polls=self._session.charge_polls)
            else:
                return self._decide_full(inp, now, on, rested, soc, v, a, cmax, cell_mid, alerts)
        return self._decide_hold(now, on, rested, v, a, cmin, cell_mid, alerts) + (False,)

    # -- hold -------------------------------------------------------------

    def _no_current(self, now, a, cell_mid, alerts) -> bool:
        cfg = self.cfg
        s = self._session
        if (s is not None and now - s.t0 >= cfg.no_current_after_s and a is not None
                and a < cfg.no_current_a):
            return True
        return False

    def _waiting_for_charger(self, now, alerts) -> Optional[str]:
        if now < self._no_current_retry_t:
            mins = (self._no_current_retry_t - now) / 60.0
            text = f"the charger gave no current: is it plugged into the robot? Next try in {mins:.0f} min"
            alerts.append({"level": "notice", "code": "no_current", "text": text})
            return text
        return None

    def _decide_hold(self, now, on, rested, v, a, cmin, cell_mid, alerts):
        cfg = self.cfg
        s = self._session
        if on and s is not None:
            target = s.target_ah if s.target_ah is not None else cfg.hold_charge_default_ah
            if s.ah >= target:
                return False, f"hold: top-up complete (+{s.ah:.2f} Ah)", "idle"
            if cell_mid >= cfg.hold_stop_cell_v_charging:
                return False, f"hold: {cell_mid:.2f} V per cell under charge, top of the band", "idle"
            limit = self._session_limit_s(s)
            if limit is not None and now - s.t0 >= limit:
                return False, f"hold: time limit after +{s.ah:.2f} Ah", "idle"
            if self._no_current(now, a, cell_mid, alerts):
                self._no_current_retry_t = now + cfg.no_current_retry_s
                return False, "hold: " + (self._waiting_for_charger(now, alerts) or ""), "waiting"
            return True, f"hold: charging, +{s.ah:.2f} of {target:.2f} Ah", "charging"
        if v > cfg.hold_max_voltage_v:
            return False, f"hold: pack {v:.1f} V, above the band, charger off", "idle"
        waiting = self._waiting_for_charger(now, alerts)
        if waiting is not None:
            return False, "hold: " + waiting, "waiting"
        if not rested:
            return False, "hold: settling before the next rested reading", "settling"
        est = soc_from_cell_v(cell_mid, cfg.ocv_table)
        if cell_mid <= cfg.hold_on_cell_v or cmin <= cfg.hold_on_cell_min_mv:
            if (self._last_hold_start_t is not None
                    and now - self._last_hold_start_t < cfg.hold_min_interval_s):
                wait = (cfg.hold_min_interval_s - (now - self._last_hold_start_t)) / 60.0
                return False, (f"hold: {cell_mid:.3f} V per cell rested (about {est:.0f} %), "
                               f"next charge in {wait:.0f} min"), "settling"
            need = (cfg.hold_target_soc_pct - est) / 100.0 * cfg.capacity_ah
            need = max(cfg.hold_charge_min_ah, min(cfg.hold_charge_max_ah, need))
            self._pending = ("hold", need)
            return True, (f"hold: {cell_mid:.3f} V per cell rested (about {est:.0f} %), "
                          f"charging {need:.2f} Ah"), "charging"
        return False, f"hold: {cell_mid:.3f} V per cell rested (about {est:.0f} %), in the band", "idle"

    # -- full -------------------------------------------------------------

    def _decide_full(self, inp, now, on, rested, soc, v, a, cmax, cell_mid, alerts):
        cfg = self.cfg
        s = self._session
        # Full by voltage only: the BMS percentage drifts and can read 100 on a
        # half-empty pack. Tapered means a small CHARGE current: the idle load
        # (-0.3 A with the charger lead off) is not a taper.
        at_full = v >= cfg.full_done_voltage_v or cmax >= cfg.full_done_cell_max_mv
        tapered = a is not None and -0.05 <= a <= cfg.full_done_current_a + 1e-6
        if on and at_full and tapered:
            if self._done_since is None:
                self._done_since = now
            held = now - self._done_since
            if held >= cfg.full_done_hold_s:
                self.topped = True
                self._done_since = None
                return False, f"full: charge complete ({cell_mid:.2f} V per cell)", "idle", True
            return True, f"full: finishing ({held:.0f} s of {cfg.full_done_hold_s:.0f})", "charging", False
        self._done_since = None
        if on and s is not None:
            limit = self._session_limit_s(s)
            if s.kind == "full" and limit is not None and now - s.t0 >= limit:
                self._latch("charge_overrun", f"the full charge did not finish in {limit / 3600.0:.0f} h")
                return False, f"FAULT: {self.fault['text']}", "fault", False
            if self._no_current(now, a, cell_mid, alerts):
                if cell_mid >= cfg.full_retop_cell_v:
                    # Many chargers do not start on a pack that is nearly full.
                    # Call it the top; the top-up rule tries again lower down.
                    self.topped = True
                    return False, (f"full: the charger does not start at {cell_mid:.2f} V per cell; "
                                   "the pack is near full"), "idle", False
                self._no_current_retry_t = now + cfg.no_current_retry_s
                return False, "full: " + (self._waiting_for_charger(now, alerts) or ""), "waiting", False
            if a is not None and a < cfg.no_current_a:
                return True, f"full: plug on, waiting for the charger to start ({cell_mid:.2f} V per cell)", "charging", False
            return True, f"full: charging ({cell_mid:.2f} V per cell, {a if a is not None else 0.0:.1f} A)", "charging", False
        if self.topped:
            if rested and cell_mid <= cfg.full_retop_cell_v:
                self.topped = False
                self._pending = ("full", None)
                return True, f"full: top-up from {cell_mid:.2f} V per cell", "charging", False
            return False, f"full: complete, resting at {cell_mid:.2f} V per cell", "idle", False
        waiting = self._waiting_for_charger(now, alerts)
        if waiting is not None:
            return False, "full: " + waiting, "waiting", False
        self._pending = ("full", None)
        return True, f"full: charging from {cell_mid:.2f} V per cell", "charging", False

    # -- health -----------------------------------------------------------

    def _soft_inhibit(self, inp, mode, v, cmin, cmax) -> Optional[Tuple[str, str]]:
        cfg = self.cfg
        hit = SOFT_FLAGS & set(inp.error_flags)
        if hit:
            return "bms_trip", "the BMS reports " + ", ".join(sorted(hit))
        if inp.charge_fet_on is False:
            return "fet_open", "the BMS charge FET is open"
        tmin = _num(inp.temp_min_c, -40, 120)
        tmax = _num(inp.temp_max_c, -40, 120)
        if tmin is None and tmax is None:
            return "no_temp", "no pack temperature reading"
        lo_t = tmin if tmin is not None else tmax
        hi_t = tmax if tmax is not None else tmin
        min_c = cfg.full_min_temp_c if mode == "full" else cfg.charge_min_temp_c
        if lo_t < min_c:
            return "cold", f"pack {lo_t:.0f} C, too cold to charge (limit {min_c:.0f} C)"
        if hi_t > cfg.charge_max_temp_c:
            return "warm", f"pack {hi_t:.0f} C, too warm to charge (limit {cfg.charge_max_temp_c:.0f} C)"
        if cmax >= cfg.cell_max_stop_mv:
            return "cell_high", f"highest cell {cmax / 1000.0:.3f} V"
        if self._suspects:
            code = sorted(self._suspects)[0]
            n, text = self._suspects[code]
            return "suspect", f"{text} (poll {n} of {cfg.fault_confirm_polls} before it latches)"
        return None

    def _check_hard_faults(self, inp, a, v, cmin, cmax) -> None:
        cfg = self.cfg
        hit = HARD_FLAGS & set(inp.error_flags)
        if hit:
            self._suspect("bms_flag", "the BMS reports " + ", ".join(sorted(hit)))
        if inp.num_cells is not None and int(inp.num_cells) != int(cfg.expected_cells):
            self._suspect("cell_count", f"the BMS reports {inp.num_cells} cells, expected {cfg.expected_cells}")
        at_rest = a is not None and abs(a) <= cfg.rest_load_current_a
        if cmin < cfg.fault_cell_min_any_mv or (cmin < cfg.fault_cell_min_mv and at_rest):
            self._suspect("cell_low", f"lowest cell {cmin / 1000.0:.2f} V: deeply discharged. Charge it by hand "
                                      "(charger straight into the wall) while you watch, then clear this fault")
        n = cfg.expected_cells
        if not (n * cmin / 1000.0 - 1.0 <= v <= n * cmax / 1000.0 + 1.0):
            self._suspect("bms_data", f"BMS voltages disagree (pack {v:.1f} V, cells "
                                      f"{cmin / 1000.0:.2f} to {cmax / 1000.0:.2f} V)")
        tmax = _num(inp.temp_max_c, -40, 120)
        if tmax is not None and tmax >= cfg.fault_temp_max_c:
            self._suspect("over_temp", f"pack at {tmax:.0f} C")
        if self._spread_level == "fault" and self._spread_snapshot is not None:
            sp, cv, _ = self._spread_snapshot      # already held for minutes: latch now
            self._latch("cell_spread", f"cell spread {sp:.0f} mV at {cv:.2f} V per cell: a cell group is failing")

    def _check_vesc_agreement(self, inp, rested, v, alerts) -> None:
        """BMS and VESC pack voltage should agree at rest: a warning, no latch
        (which of the two is wrong is not known)."""
        cfg = self.cfg
        vesc = _num(inp.vesc_v, 30.0, 60.0)
        if not rested or vesc is None:
            return
        if abs(vesc - v) > cfg.vesc_bms_max_diff_v:
            self._vesc_diff_count += 1
        else:
            self._vesc_diff_count = 0
        if self._vesc_diff_count >= cfg.implausible_polls:
            alerts.append({"level": "warning", "code": "voltage_mismatch",
                           "text": f"the BMS reads the pack at {v:.1f} V and the VESC at {vesc:.1f} V"})

    def _check_session_faults(self, inp, now, fresh, cell_mid) -> None:
        cfg = self.cfg
        s = self._session
        if s is None or s.kind == "manual":
            return
        if (s.kind == "hold" and s.start_cell_v is not None and cell_mid is not None
                and s.ah >= cfg.charge_rise_check_ah
                and cell_mid - s.start_cell_v < cfg.charge_rise_min_v):
            self._suspect("charge_no_rise",
                          f"{s.ah:.2f} Ah went in and the cells did not rise "
                          f"({s.start_cell_v:.3f} to {cell_mid:.3f} V)")
        limit = self._session_limit_s(s)
        if limit is not None and s.kind != "full" and now - s.t0 >= limit + cfg.overrun_grace_s:
            self._latch("charge_overrun",
                        f"the plug stayed on {((now - s.t0) / 60.0):.0f} min in a {s.kind} charge "
                        f"(limit {limit / 60.0:.0f} min): it does not switch off")
        if s.ah > cfg.max_session_ah:
            self._latch("charge_overfill", f"{s.ah:.1f} Ah went in during one charge (pack {cfg.capacity_ah:.0f} Ah)")
        tmax = _num(inp.temp_max_c, -40, 120) if fresh else None
        if tmax is not None and s.start_tmax is not None and tmax - s.start_tmax >= cfg.fault_temp_rise_c:
            self._suspect("temp_rise", f"pack warmed {tmax - s.start_tmax:.0f} C during the charge "
                                       f"({s.start_tmax:.0f} to {tmax:.0f} C)")

    def _update_spread(self, rested, charging, cell_mid, spread, now) -> None:
        cfg = self.cfg
        if cell_mid is None or spread is None:
            return
        if cell_mid >= cfg.spread_top_zone_cell_v:
            zone, thr = "top", cfg.spread_top_mv
        elif cell_mid >= cfg.spread_mid_zone_cell_v:
            zone, thr = "mid", cfg.spread_mid_mv
        else:
            self._spread_charge_since = None
            return      # under the knee the weak group falls away: not evaluated
        if charging:
            # Under charge only the fault level counts: a cell that pulls far
            # away from the others at 3.6 A has high resistance.
            if spread >= thr[2]:
                if self._spread_charge_since is None:
                    self._spread_charge_since = now
                if now - self._spread_charge_since >= cfg.spread_charge_debounce_s:
                    self._spread_level = "fault"
                    self._spread_snapshot = (float(spread), float(cell_mid), zone)
            else:
                self._spread_charge_since = None
            return
        self._spread_charge_since = None
        if not rested:
            return
        cand = ("fault" if spread >= thr[2] else "warning" if spread >= thr[1]
                else "notice" if spread >= thr[0] else "ok")
        if cand != self._spread_cand:
            self._spread_cand = cand
            self._spread_cand_since = now
        if now - self._spread_cand_since >= cfg.spread_debounce_s:
            self._spread_level = cand
            self._spread_snapshot = (float(spread), float(cell_mid), zone)

    # -- BMS dark ---------------------------------------------------------

    def _dark_reset(self) -> None:
        self._unknown_since = None
        self._dark_action = None
        self._dark_blocked = None
        self._next_wake_t = None
        self._last_rescue_end_t = None
        self._last_rescue_v0 = None

    def _dark_gate(self, inp) -> Optional[str]:
        """Why no charge may start in the dark, or None."""
        cfg = self.cfg
        if self._dark_blocked is not None:
            return self._dark_blocked
        if self._last_fresh_inhibit is not None:
            return f"the last BMS reading said: {self._last_fresh_inhibit}"
        amb = _num(inp.vesc_temp_c, -40, 120)
        if amb is None:
            return "no VESC temperature"
        if amb < cfg.dark_min_vesc_temp_c:
            return f"{amb:.0f} C at the VESC, too cold to charge blind (limit {cfg.dark_min_vesc_temp_c:.0f} C)"
        if amb > cfg.dark_max_vesc_temp_c:
            return f"{amb:.0f} C at the VESC, too warm to charge blind"
        return None

    def _decide_dark(self, inp, now, on, alerts):
        cfg = self.cfg
        if self._unknown_since is None:
            self._unknown_since = now
            self._next_wake_t = now + cfg.wake_first_after_s
        waited = now - self._unknown_since
        if waited < cfg.soc_unknown_off_s:
            return None, f"waiting for a BMS reading ({waited:.0f} s)", "dark"
        vesc = _num(inp.vesc_v, 30.0, 60.0)
        self._wake_times = [t for t in self._wake_times if now - t < 86400.0]
        self._rescue_times = [t for t in self._rescue_times if now - t < 86400.0]
        mins = waited / 60.0

        def say(text, want, phase):
            alerts.append({"level": "warning", "code": "bms_dark", "text": text})
            return want, text, phase

        if self._dark_action is not None:
            kind, t0, v0 = self._dark_action
            limit = cfg.wake_pulse_s if kind == "wake" else cfg.rescue_charge_s
            age = now - t0
            keep = vesc is not None and vesc < cfg.dark_stop_voltage_v and age < limit
            if (keep and age >= cfg.dark_response_after_s and v0 is not None
                    and vesc - v0 < cfg.dark_response_min_v):
                # The charger is not charging this pack, or the reading is stuck.
                self._dark_blocked = (f"the pack voltage did not rise with the plug on ({v0:.1f} to "
                                      f"{vesc:.1f} V by the VESC): charger not connected, or the reading is stuck")
                keep = False
            if keep:
                if kind == "wake":
                    return say(f"BMS dark {mins:.0f} min: wake pulse {len(self._wake_times)} of "
                               f"{cfg.wake_max_per_day} ({vesc:.1f} V by the VESC)", True, "wake")
                return say(f"BMS dark {mins:.0f} min and pack low: rescue top-up "
                           f"({vesc:.1f} V by the VESC)", True, "rescue")
            self._dark_action = None
            if kind == "rescue":
                self._last_rescue_end_t = now

        if inp.armed:
            # Here only with a charge running at the arm: the 2 min without a
            # reading are over, so it ends, and no pulse or rescue starts.
            return False, f"no BMS reading for {mins:.0f} min, robot armed: charger off", "armed"
        if vesc is None:
            return say(f"no BMS reading for {mins:.0f} min and no VESC voltage: charger off", False, "dark")
        gate = self._dark_gate(inp)
        if gate is not None:
            return say(f"no BMS reading for {mins:.0f} min ({vesc:.1f} V by the VESC): charger off. {gate}",
                       False, "dark")

        if cfg.rescue_enabled and vesc <= cfg.rescue_voltage_v:
            if (self._last_rescue_end_t is not None and self._last_rescue_v0 is not None
                    and now - self._last_rescue_end_t < cfg.rescue_min_hold_s
                    and vesc < self._last_rescue_v0 + cfg.rescue_min_gain_v):
                # A healthy pack keeps a rescue for about 3 h. This one is back
                # where it was: do not keep feeding it blind.
                self._dark_blocked = (f"the pack is back at {vesc:.1f} V "
                                      f"{(now - self._last_rescue_end_t) / 60.0:.0f} min after a rescue top-up "
                                      f"from {self._last_rescue_v0:.1f} V: it does not hold the charge")
                return say(f"no BMS reading for {mins:.0f} min: charger off. {self._dark_blocked}", False, "dark")
            if len(self._rescue_times) >= cfg.rescue_max_per_day:
                return say(f"no BMS reading for {mins:.0f} min ({vesc:.1f} V by the VESC): "
                           f"{cfg.rescue_max_per_day} rescue top-ups in 24 h, no more", False, "dark")
            if self._last_rescue_t is None or now - self._last_rescue_t >= cfg.rescue_interval_s:
                self._rescue_times.append(now)
                self._last_rescue_t = now
                self._last_rescue_v0 = vesc
                self._dark_action = ("rescue", now, vesc)
                self._pending = ("rescue", None)
                # A rescue is also the longest wake attempt there is: no pulse right after it.
                self._next_wake_t = now + cfg.rescue_charge_s + cfg.wake_backoff_s[0]
                return say(f"BMS dark {mins:.0f} min and pack at {vesc:.1f} V by the VESC: rescue top-up "
                           f"of {cfg.rescue_charge_s / 60.0:.0f} min", True, "rescue")

        can_wake = (cfg.wake_enabled and vesc <= cfg.wake_max_voltage_v
                    and len(self._wake_times) < cfg.wake_max_per_day)
        if can_wake and self._next_wake_t is not None and now >= self._next_wake_t:
            self._wake_times.append(now)
            n = len(self._wake_times)
            backoff = cfg.wake_backoff_s[min(n - 1, len(cfg.wake_backoff_s) - 1)]
            self._next_wake_t = now + cfg.wake_pulse_s + backoff
            self._dark_action = ("wake", now, vesc)
            self._pending = ("wake", None)
            return say(f"BMS dark {mins:.0f} min: wake pulse {n} of {cfg.wake_max_per_day} "
                       f"({vesc:.1f} V by the VESC)", True, "wake")
        if can_wake and self._next_wake_t is not None:
            wait = max(0.0, self._next_wake_t - now) / 60.0
            return say(f"no BMS reading for {mins:.0f} min ({vesc:.1f} V by the VESC): "
                       f"next wake pulse in {wait:.0f} min", False, "dark")
        return say(f"no BMS reading for {mins:.0f} min ({vesc:.1f} V by the VESC): charger off", False, "dark")
