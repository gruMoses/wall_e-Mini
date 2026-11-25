"""ChargeManager: runs the charger plug policy on its own thread.

Every ``poll_s`` it reads the main-pack BMS (BmsService), the pack voltage
from the VESCs (an independent reading), the armed state and the Kasa plug,
runs ``ChargePolicy.decide`` and switches the plug when the decision differs
from the plug. It persists what must survive the service restarts that every
deploy causes, in a small JSON file under ``logs/``: the operator mode, a
latched fault, the time of the last full charge and the plug's address.

Switch pacing: OFF is never delayed. An automatic ON waits until
``min_switch_interval_s`` after the last switch (no chatter); an operator
mode change applies at once.

Dead-man: while the plug is on in ``hold`` or ``full`` the manager keeps a
countdown rule in the plug itself ("off in ``plug_deadman_s``") and re-arms
it every ``plug_deadman_refresh_s``. If the Pi halts, hangs or loses the
Wi-Fi link, the plug ends the charge without the robot. Manual ``on`` has no
countdown.

Stop: on ``stop()`` the plug is switched off when the system itself is
shutting down (e-stop halt, main switch off, ``poweroff``) and the mode is an
automatic one; a plain service restart (deploy) leaves the plug alone and the
next process decides again within a poll or two.

Charger inhibit (audit N1, Kevin 2026-11-25): a charge already running goes
on when the robot arms, and ``holds_plug_on()`` tells the main loop while
the plug is on in such a charge; the main loop adds it to the BMS current
inhibit. The BMS term alone releases in the taper under 2.0 A with the
charger still powered. The policy keeps a charge while armed only while
this term holds. Each poll that sees the plug sets the term again; when
the polls stop seeing it (plug unreachable, thread hung) it runs out by
itself after ``plug_deadman_s`` + ``plug_deadman_refresh_s``, when the
plug's own countdown has ended the charge (see ``_update_plug_hold``).

This thread never touches motion. The main control loop does not wait on it;
it only tells it the armed state (``note_armed``, one attribute write) and
reads ``holds_plug_on()`` (one attribute read and a clock).
"""

from __future__ import annotations

import json
import logging
import os
import subprocess
import threading
import time
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

from pi_app.control.charge_policy import MODES, ChargeInputs, ChargePolicy

logger = logging.getLogger(__name__)

_PLUG_FAILS_FOR_ALERT = 3


def system_is_stopping() -> bool:
    """True when systemd is shutting the machine down (halt, poweroff, reboot).

    Any failure to tell counts as "stopping" so the conservative branch
    (plug off) wins.
    """
    try:
        out = subprocess.run(
            ["systemctl", "is-system-running"],
            capture_output=True, text=True, timeout=2.0, check=False,
        ).stdout.strip()
    except Exception:
        return True
    if not out:
        return True
    return out == "stopping"


def vesc_heatsink_temp(driver) -> Optional[float]:
    """Lowest VESC heatsink temperature (C): the ambient proxy for a dark BMS."""
    try:
        t = driver.get_telemetry()
    except Exception:
        return None
    if t is None:
        return None
    vals = [x for x in (getattr(t, "left_temp_c", None), getattr(t, "right_temp_c", None))
            if isinstance(x, (int, float)) and not isinstance(x, bool)]
    return float(min(vals)) if vals else None


class ChargeManager:
    def __init__(self, cfg, bms_service, plug, state_path: Path,
                 system_stopping: Optional[Callable[[], bool]] = None,
                 clock: Callable[[], float] = time.monotonic,
                 vesc_voltage_fn: Optional[Callable[[], Optional[float]]] = None,
                 wall_clock: Callable[[], float] = time.time,
                 vesc_temp_fn: Optional[Callable[[], Optional[float]]] = None,
                 drive_inhibit_enabled: bool = True) -> None:
        self._cfg = cfg
        # BmsConfig.charger_inhibit_enabled. Off: holds_plug_on() is always
        # False, so the policy also stops a charge at the arm, as before.
        self._drive_inhibit_enabled = bool(drive_inhibit_enabled)
        self._bms = bms_service
        self._plug = plug
        self._state_path = Path(state_path)
        self._system_stopping = system_stopping or system_is_stopping
        self._clock = clock
        self._wall = wall_clock
        self._vesc_voltage_fn = vesc_voltage_fn
        self._vesc_temp_fn = vesc_temp_fn
        self._lock = threading.Lock()
        self._wake = threading.Event()
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._armed = False

        saved = self._load_state()
        default = getattr(cfg, "default_mode", "hold")
        mode = saved.get("mode") if saved.get("mode") in MODES else (default if default in MODES else "hold")
        self._policy = ChargePolicy(cfg, mode)
        self._policy.restore_fault(saved.get("fault"))
        me = saved.get("mode_epoch")
        if isinstance(me, (int, float)) and not isinstance(me, bool):
            self._mode_epoch: float = float(me)
            self._policy.restore_mode_age(self._wall() - self._mode_epoch)
        else:
            self._mode_epoch = self._wall()
        self._fault_epoch: Optional[float] = saved.get("fault_epoch") if self._policy.fault else None
        self._last_fault_code: Optional[str] = self._policy.fault["code"] if self._policy.fault else None
        lf = saved.get("last_full_epoch")
        # Unknown history counts from now: the 30-day "full charge due" notice
        # must not fire on the first day.
        self._last_full_epoch: float = float(lf) if isinstance(lf, (int, float)) else self._wall()
        saved_host = saved.get("plug_host")
        if isinstance(saved_host, str) and saved_host and getattr(plug, "host", None) not in (None, saved_host):
            try:
                plug.host = saved_host       # the MAC check corrects a stale address
            except Exception:
                pass
        setter = getattr(plug, "set_host_change_callback", None)
        if callable(setter):
            setter(self._on_plug_host_change)

        self._last_switch_t: Optional[float] = None
        self._operator_change = False
        self._plug_fails = 0
        self._alert_codes: set = set()
        self._deadman_last_t: Optional[float] = None
        self._deadman_armed = False
        self._deadman_fails = 0
        # Charger inhibit term: until this clock time the plug counts as on in
        # a running charge. None = no. Read by the main loop, written by tick().
        self._plug_hold_until: Optional[float] = None
        self._prev_charge_running = False
        self._status: Dict[str, Any] = {
            "mode": self._policy.mode,
            "plug_on": None,
            "want_on": None,
            "reason": "not started",
            "phase": "",
            "soc_pct": None, "pack_v": None, "pack_a": None, "bms_fresh": False,
            "cell_min_mv": None, "cell_max_mv": None, "cell_spread_mv": None, "cell_mid_v": None,
            "soc_by_voltage": None, "temp_min_c": None, "temp_max_c": None,
            "vesc_v": None, "armed": False,
            "alerts": [], "fault": self._fault_dict(), "session": None, "topped": False,
            "last_full_epoch": self._last_full_epoch, "days_since_full": None,
            "plug_error": None, "last_switch_epoch": None, "switch_count": 0,
            "deadman": "off", "holds_plug_on": False, "updated_epoch": None,
            "host": getattr(plug, "host", getattr(cfg, "host", "")),
            "mac": getattr(cfg, "mac", ""),
        }

    # -- persistence ------------------------------------------------------

    def _load_state(self) -> Dict[str, Any]:
        try:
            with open(self._state_path, "r", encoding="utf-8") as fh:
                data = json.load(fh)
            return data if isinstance(data, dict) else {}
        except FileNotFoundError:
            return {}
        except Exception as e:  # unreadable file: fall back, say so
            logger.warning("charger state file unreadable (%s); using defaults", e)
            return {}

    def _save_state(self) -> None:
        try:
            self._state_path.parent.mkdir(parents=True, exist_ok=True)
            tmp = self._state_path.with_suffix(".tmp")
            data = {
                "mode": self._policy.mode,
                "mode_epoch": self._mode_epoch,
                "fault": self._policy.fault,
                "fault_epoch": self._fault_epoch,
                "last_full_epoch": self._last_full_epoch,
                "plug_host": getattr(self._plug, "host", None),
                "updated_epoch": self._wall(),
            }
            with open(tmp, "w", encoding="utf-8") as fh:
                json.dump(data, fh)
            os.replace(tmp, self._state_path)
        except Exception as e:
            logger.warning("charger state file not written: %s", e)

    def _fault_dict(self) -> Optional[Dict[str, Any]]:
        f = self._policy.fault
        if f is None:
            return None
        return {"code": f["code"], "text": f["text"], "epoch": self._fault_epoch}

    def _on_plug_host_change(self, host: str) -> None:
        logger.warning("Charger plug found by MAC at a new address: %s", host)
        with self._lock:
            self._status["host"] = host
        self._save_state()

    # -- lifecycle --------------------------------------------------------

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._run, name="ChargeManager", daemon=True)
        self._thread.start()
        logger.warning("Charger plug manager started: plug %s (MAC %s), mode %s%s",
                       getattr(self._plug, "host", "?"), getattr(self._cfg, "mac", "?") or "none",
                       self._policy.mode,
                       f", FAULT latched: {self._policy.fault['text']}" if self._policy.fault else "")

    def stop(self) -> None:
        self._running = False
        self._wake.set()
        t = self._thread
        if t is not None and t.is_alive():
            t.join(timeout=5.0)
        mode = self._policy.mode
        if mode in ("hold", "full") and getattr(self._cfg, "plug_off_on_system_stop", True):
            if self._system_stopping():
                ok = self._switch(False, "system shutdown: no charging without the robot watching")
                logger.warning("Charger plug off at shutdown: %s", "done" if ok else "FAILED")

    # -- operator / main loop ---------------------------------------------

    def note_armed(self, armed: bool) -> None:
        """Called by the main loop every tick; one attribute write."""
        self._armed = bool(armed)

    def holds_plug_on(self, now: Optional[float] = None) -> bool:
        """True while the plug is on in a running hold or full charge: the
        charger is powered and its lead is on the robot. The main loop adds
        this to the charger inhibit. Called every control tick: no lock, no I/O.
        """
        until = self._plug_hold_until
        if until is None or not self._drive_inhibit_enabled:
            return False
        return (self._clock() if now is None else now) < until

    def set_mode(self, mode: Any) -> "tuple[bool, str]":
        if not isinstance(mode, str) or mode not in MODES:
            return False, f"mode must be one of {', '.join(MODES)}"
        with self._lock:
            self._policy.set_mode(mode, self._clock())
            self._operator_change = True
            self._status["mode"] = mode
            self._mode_epoch = self._wall()
        self._save_state()
        logger.warning("Charger plug mode set to %s (operator)", mode)
        self._wake.set()
        return True, f"mode {mode}"

    def clear_fault(self) -> "tuple[bool, str]":
        with self._lock:
            had = self._policy.clear_fault()
            self._fault_epoch = None
            self._last_fault_code = None
            self._status["fault"] = None
        if not had:
            return False, "no fault is latched"
        self._save_state()
        logger.warning("Charger fault cleared (operator)")
        self._wake.set()
        return True, "fault cleared; it latches again if the condition is still there"

    def get_status(self) -> Dict[str, Any]:
        with self._lock:
            return dict(self._status)

    # -- the loop ---------------------------------------------------------

    def _run(self) -> None:
        poll_s = max(2.0, float(getattr(self._cfg, "poll_s", 10.0)))
        while self._running:
            try:
                self.tick()
            except Exception as e:
                logger.warning("charger tick failed: %s", e)
            self._wake.wait(timeout=poll_s)
            self._wake.clear()

    def _read_inputs(self, plug_on: Optional[bool], now: float) -> ChargeInputs:
        kw: Dict[str, Any] = {"plug_on": plug_on, "armed": bool(self._armed),
                              "plug_inhibit": self.holds_plug_on(now)}
        try:
            st = self._bms.get_state()
            link = self._bms.get_link_health()
            max_age = float(getattr(self._cfg, "bms_max_age_s", 30.0))
            kw.update(
                soc_pct=st.soc_pct, pack_v=st.pack_voltage_v, pack_a=st.pack_current_a,
                cell_min_mv=getattr(st, "cell_min_mv", None), cell_max_mv=getattr(st, "cell_max_mv", None),
                temp_min_c=getattr(st, "temp_min_c", None), temp_max_c=getattr(st, "temp_max_c", None),
                charge_fet_on=getattr(st, "charge_fet_on", None),
                error_flags=tuple(getattr(st, "error_flags", None) or ()),
                num_cells=getattr(st, "num_cells", None),
                bms_fresh=(link.state == "up" and link.data_age_s is not None
                           and link.data_age_s <= max_age and bool(st.connected)),
            )
        except Exception as e:
            logger.debug("charger: BMS read failed: %s", e)
        if self._vesc_voltage_fn is not None:
            try:
                kw["vesc_v"] = self._vesc_voltage_fn()
            except Exception:
                kw["vesc_v"] = None
        if self._vesc_temp_fn is not None:
            try:
                kw["vesc_temp_c"] = self._vesc_temp_fn()
            except Exception:
                kw["vesc_temp_c"] = None
        return ChargeInputs(**kw)

    def _read_plug(self) -> "tuple[Optional[bool], Optional[str]]":
        try:
            state = self._plug.get_state()
            self._plug_fails = 0
            return state, None
        except Exception as e:
            self._plug_fails += 1
            return None, str(e)

    def _switch(self, on: bool, reason: str) -> bool:
        try:
            self._plug.set_state(on)
        except Exception as e:
            logger.warning("Charger plug %s FAILED (%s): %s", "ON" if on else "OFF", reason, e)
            with self._lock:
                self._status["plug_error"] = str(e)
            return False
        now = self._clock()
        with self._lock:
            self._policy.note_plug(on)
            self._last_switch_t = now
            self._status["plug_on"] = on
            self._status["plug_error"] = None
            self._status["last_switch_epoch"] = self._wall()
            self._status["switch_count"] = int(self._status.get("switch_count", 0)) + 1
        logger.warning("Charger plug %s: %s", "ON" if on else "OFF", reason)
        return True

    def _service_deadman(self, plug_on: Optional[bool], mode: str, now: float,
                         want_on: Optional[bool] = None) -> None:
        """Keep the plug's own off-countdown armed while it is on in hold/full;
        remove it otherwise (a leftover would end a manual "on" after minutes).
        Not re-armed while the policy wants the plug off: when the OFF switch
        failed, the countdown already running must end the charge."""
        arm = getattr(self._plug, "arm_off_timer", None)
        secs = float(getattr(self._cfg, "plug_deadman_s", 0.0) or 0.0)
        if not callable(arm) or secs <= 0.0:
            return
        if plug_on and mode in ("hold", "full"):
            if want_on is False:
                return
            refresh = float(getattr(self._cfg, "plug_deadman_refresh_s", 60.0))
            if (self._deadman_last_t is None or self._deadman_fails
                    or now - self._deadman_last_t >= refresh):
                try:
                    arm(secs)
                    self._deadman_last_t = now
                    self._deadman_armed = True
                    self._deadman_fails = 0
                except Exception as e:
                    self._deadman_fails += 1
                    if self._deadman_fails == 1:
                        logger.warning("Charger plug off-timer not armed: %s", e)
        elif self._deadman_armed:
            clear = getattr(self._plug, "clear_off_timer", None)
            try:
                if callable(clear):
                    clear()
                self._deadman_armed = False
                self._deadman_last_t = None
                self._deadman_fails = 0
            except Exception:
                pass    # the plug is unreachable: try again next poll

    def _update_plug_hold(self, plug_on: Optional[bool], plug_err: Optional[str],
                          charge_running: bool, now: float) -> None:
        """Set the charger-inhibit term that holds_plug_on() reads.

        The plug answered: the term follows it (on in a running charge, after
        this poll's switch). The plug did not answer: the term is never
        refreshed, except once on the poll where the charge becomes running
        (the policy goes by the last known plug state, so a charge can become
        running with no answer at all). Either way it runs out by itself
        plug_deadman_s + plug_deadman_refresh_s after the last setting: by
        then the plug's own countdown, re-armed at most at that poll (and the
        re-arm may still be in flight), has ended the charge. Then it fails
        open like the BMS term: a plug outage (8.4 h on 2026-11-24) must not
        hold the robot for hours.
        """
        cfg = self._cfg
        hold_s = max(float(getattr(cfg, "plug_deadman_s", 0.0) or 0.0)
                     + float(getattr(cfg, "plug_deadman_refresh_s", 0.0) or 0.0),
                     3.0 * float(getattr(cfg, "poll_s", 10.0)))
        if plug_err is None:
            self._plug_hold_until = now + hold_s if (plug_on and charge_running) else None
        elif (charge_running and not self._prev_charge_running
              and self._plug_hold_until is None):
            # Seeded once; a hold that has run out is never set again
            # without an answer from the plug.
            self._plug_hold_until = now + hold_s
        self._prev_charge_running = bool(charge_running)

    def tick(self, now: Optional[float] = None) -> Dict[str, Any]:
        """One poll. Public for tests; the thread calls it every poll_s."""
        if now is None:
            now = self._clock()
        plug_on, plug_err = self._read_plug()
        inp = self._read_inputs(plug_on, now)

        with self._lock:
            prev_mode = self._status.get("mode")
            dec = self._policy.decide(inp, now)
            operator = self._operator_change
            self._operator_change = False
            last_switch = self._last_switch_t
            fault = self._policy.fault

        save = False
        if dec.mode != prev_mode:
            save = True
            self._mode_epoch = self._wall()
            logger.warning("Charger plug mode %s -> %s (%s)", prev_mode, dec.mode, dec.reason)
        if dec.full_completed:
            self._last_full_epoch = self._wall()
            save = True
        code = fault["code"] if fault else None
        if code != self._last_fault_code:
            if fault is not None:
                self._fault_epoch = self._wall()
                logger.warning("Charger FAULT latched (%s): %s. Plug held off until it is cleared.",
                               fault["code"], fault["text"])
            self._last_fault_code = code
            save = True
        if save:
            self._save_state()

        reason = dec.reason
        if plug_err is not None:
            reason = f"{reason}; plug unreachable: {plug_err}"
        elif dec.want_on is not None and dec.want_on != plug_on:
            min_gap = float(getattr(self._cfg, "min_switch_interval_s", 60.0))
            if (dec.want_on and not operator and last_switch is not None
                    and now - last_switch < min_gap):
                reason = f"{reason}; switch in {min_gap - (now - last_switch):.0f} s"
            elif self._switch(dec.want_on, dec.reason):
                plug_on = dec.want_on
            else:
                reason = f"{reason}; switch FAILED"

        # The inhibit term BEFORE the countdown re-arm below: that call can
        # block for the plug timeout, and the term must already cover the
        # countdown it is about to restart.
        self._update_plug_hold(plug_on, plug_err, dec.charge_running, now)
        if plug_err is None:
            self._service_deadman(plug_on, dec.mode, now, dec.want_on)

        alerts: List[Dict[str, str]] = list(dec.alerts)
        if self._deadman_fails >= _PLUG_FAILS_FOR_ALERT:
            alerts.append({"level": "warning", "code": "deadman_unarmed",
                           "text": "the plug's own off-timer could not be set: a dead robot would "
                                   "leave the charger on"})
        if plug_err is not None and self._plug_fails >= _PLUG_FAILS_FOR_ALERT:
            alerts.append({"level": "warning", "code": "plug_unreachable",
                           "text": f"the plug does not answer ({plug_err}); it keeps its last state"})
        days = (self._wall() - self._last_full_epoch) / 86400.0
        if days >= float(getattr(self._cfg, "full_due_days", 30.0)):
            alerts.append({"level": "notice", "code": "full_due",
                           "text": f"no full charge for {days:.0f} days: the BMS only balances "
                                   "above 4.0 V per cell, so do one soon"})
        codes = {a["code"] for a in alerts}
        for a in alerts:
            if a["code"] not in self._alert_codes:
                logger.warning("Charger alert (%s): %s", a["level"], a["text"])
        for gone in self._alert_codes - codes:
            logger.warning("Charger alert cleared: %s", gone)
        self._alert_codes = codes

        with self._lock:
            self._status.update({
                "mode": dec.mode,
                "plug_on": plug_on,
                "want_on": dec.want_on,
                "reason": reason,
                "phase": dec.phase,
                "soc_pct": inp.soc_pct, "pack_v": inp.pack_v, "pack_a": inp.pack_a,
                "bms_fresh": inp.bms_fresh,
                "cell_min_mv": inp.cell_min_mv, "cell_max_mv": inp.cell_max_mv,
                "cell_spread_mv": dec.info.get("cell_spread_mv"),
                "cell_mid_v": dec.info.get("cell_mid_v"),
                "soc_by_voltage": dec.info.get("soc_by_voltage"),
                "temp_min_c": inp.temp_min_c, "temp_max_c": inp.temp_max_c,
                "vesc_v": inp.vesc_v, "vesc_temp_c": inp.vesc_temp_c, "armed": inp.armed,
                "deadman": "armed" if self._deadman_armed else "off",
                "holds_plug_on": self.holds_plug_on(now),
                "alerts": alerts, "fault": self._fault_dict(),
                "session": dec.info.get("session"), "topped": dec.info.get("topped"),
                "last_full_epoch": self._last_full_epoch, "days_since_full": round(days, 1),
                "plug_error": plug_err,
                "updated_epoch": self._wall(),
                "host": getattr(self._plug, "host", self._status.get("host")),
            })
            return dict(self._status)
