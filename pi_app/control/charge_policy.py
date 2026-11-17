"""Charger plug policy: when WALL-E's charger gets mains power.

Pure decision logic, no I/O, no threads (the ChargeManager owns those).
One ``decide()`` per poll. Modes (operator-chosen, persisted by the manager):

* ``hold``  keep the main pack near 50 %: plug on at or under ``hold_on_pct``,
            off at or over ``hold_off_pct``. Never on while the pack is above
            ``hold_max_voltage_v`` (guards a drifted BMS state of charge).
* ``full``  charge to full once: plug on until the BMS reports full (SOC at
            ``full_done_soc_pct`` or pack at ``full_done_voltage_v``) with the
            current tapered under ``full_done_current_a`` for ``full_done_hold_s``,
            then off; a top-up when SOC falls to ``full_retop_pct``. After the
            pack has been used (SOC seen at or above ``full_return_pct`` and
            then under it) the mode returns to ``hold`` by itself.
* ``on`` / ``off``  manual; no automation, no BMS needed.

``hold`` and ``full`` act only on a fresh BMS reading. Without one for
``soc_unknown_off_s`` the plug goes off and stays off until a reading returns
(Kevin, 2026-11-16: no charging without a battery reading).

Nothing here touches motion. The plug powers the charger; the robot's own
charger inhibit (BmsService.is_charging) still blocks drive while charging.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

MODES = ("hold", "full", "on", "off")


@dataclass(frozen=True)
class ChargeInputs:
    soc_pct: Optional[float]
    pack_v: Optional[float]
    pack_a: Optional[float]      # positive = charging
    bms_fresh: bool              # link up and the sample recent
    plug_on: Optional[bool]      # None = plug not reachable


@dataclass(frozen=True)
class ChargeDecision:
    want_on: Optional[bool]      # None = leave the plug as it is
    reason: str
    mode: str                    # mode after this tick


def _valid_soc(soc: Optional[float]) -> Optional[float]:
    if soc is None:
        return None
    try:
        s = float(soc)
    except (TypeError, ValueError):
        return None
    if s != s or s < 0.0 or s > 100.0:
        return None
    return s


def _valid_v(v: Optional[float]) -> Optional[float]:
    if v is None:
        return None
    try:
        f = float(v)
    except (TypeError, ValueError):
        return None
    if f != f or f < 20.0 or f > 70.0:   # a 13S pack lives in 39-55 V
        return None
    return f


class ChargePolicy:
    def __init__(self, cfg, mode: str = "hold") -> None:
        self.cfg = cfg
        if mode not in MODES:
            mode = "hold"
        self.mode = mode
        self.topped = False               # full: charge complete, waiting for a top-up need
        self.seen_above_return = False    # full: SOC has been at/above full_return_pct
        self._unknown_since: Optional[float] = None
        self._done_since: Optional[float] = None

    # -- operator ---------------------------------------------------------

    def set_mode(self, mode: str) -> None:
        if mode not in MODES:
            raise ValueError(f"unknown mode {mode!r}")
        self.mode = mode
        self.topped = False
        self.seen_above_return = False
        self._done_since = None

    # -- one poll ---------------------------------------------------------

    def decide(self, inp: ChargeInputs, now: float) -> ChargeDecision:
        cfg = self.cfg
        mode = self.mode
        if mode == "off":
            return ChargeDecision(False, "mode off", mode)
        if mode == "on":
            return ChargeDecision(True, "mode on (manual)", mode)

        soc = _valid_soc(inp.soc_pct)
        v = _valid_v(inp.pack_v)
        a = inp.pack_a
        fresh = bool(inp.bms_fresh) and soc is not None

        if not fresh:
            if self._unknown_since is None:
                self._unknown_since = now
            waited = now - self._unknown_since
            if waited >= cfg.soc_unknown_off_s:
                return ChargeDecision(False, f"no BMS reading for {waited:.0f} s", mode)
            return ChargeDecision(None, f"waiting for a BMS reading ({waited:.0f} s)", mode)
        self._unknown_since = None
        assert soc is not None

        if mode == "full":
            if soc >= cfg.full_return_pct:
                self.seen_above_return = True
            elif self.seen_above_return:
                # The pack was used after the full charge: back to holding 50 %.
                self.mode = mode = "hold"
                self.topped = False
                self.seen_above_return = False
                self._done_since = None

        if mode == "full":
            return self._decide_full(inp, soc, v, a, now)
        return self._decide_hold(soc, v)

    def _decide_full(self, inp: ChargeInputs, soc: float, v: Optional[float],
                     a: Optional[float], now: float) -> ChargeDecision:
        cfg = self.cfg
        at_full = soc >= cfg.full_done_soc_pct or (v is not None and v >= cfg.full_done_voltage_v)
        tapered = a is not None and abs(float(a)) <= cfg.full_done_current_a
        if inp.plug_on and at_full and tapered:
            if self._done_since is None:
                self._done_since = now
            held = now - self._done_since
            if held >= cfg.full_done_hold_s:
                self.topped = True
                self._done_since = None
                return ChargeDecision(False, f"full: charge complete ({soc:.0f} %)", "full")
            return ChargeDecision(True, f"full: finishing ({held:.0f} s of {cfg.full_done_hold_s:.0f})", "full")
        self._done_since = None
        if self.topped:
            if soc <= cfg.full_retop_pct:
                self.topped = False
                return ChargeDecision(True, f"full: top-up ({soc:.0f} %)", "full")
            return ChargeDecision(False, f"full: topped ({soc:.0f} %)", "full")
        return ChargeDecision(True, f"full: charging ({soc:.0f} %)", "full")

    def _decide_hold(self, soc: float, v: Optional[float]) -> ChargeDecision:
        cfg = self.cfg
        if v is not None and v > cfg.hold_max_voltage_v:
            return ChargeDecision(False, f"hold: pack {v:.1f} V above the {cfg.hold_max_voltage_v:.1f} V guard", "hold")
        if soc <= cfg.hold_on_pct:
            return ChargeDecision(True, f"hold: {soc:.0f} % at or under {cfg.hold_on_pct:.0f} %", "hold")
        if soc >= cfg.hold_off_pct:
            return ChargeDecision(False, f"hold: {soc:.0f} % at or over {cfg.hold_off_pct:.0f} %", "hold")
        return ChargeDecision(None, f"hold: {soc:.0f} % in band", "hold")
