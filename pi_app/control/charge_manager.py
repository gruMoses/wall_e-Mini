"""ChargeManager: runs the charger plug policy on its own thread.

Every ``poll_s`` it reads the main-pack BMS (BmsService), asks the Kasa plug
for its relay state, runs ``ChargePolicy.decide`` and switches the plug when
the decision differs from the plug. The operator mode (hold / full / on /
off) is persisted in a small JSON file under ``logs/`` so it survives the
service restarts that every deploy causes.

Switch pacing: automatic switches are at least ``min_switch_interval_s``
apart (no chatter on a noisy SOC). An operator mode change applies at once.

Stop: on ``stop()`` the plug is switched off when the system itself is
shutting down (e-stop halt, main switch off, ``poweroff``) and the mode is an
automatic one; a plain service restart (deploy) leaves the plug alone and the
next process decides again within a poll or two. Kevin's rule (2026-11-16):
no charging without a battery reading.

This thread never touches motion. The main control loop does not wait on it.
"""

from __future__ import annotations

import json
import logging
import os
import subprocess
import threading
import time
from pathlib import Path
from typing import Any, Callable, Dict, Optional

from pi_app.control.charge_policy import MODES, ChargeInputs, ChargePolicy

logger = logging.getLogger(__name__)


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


class ChargeManager:
    def __init__(self, cfg, bms_service, plug, state_path: Path,
                 system_stopping: Optional[Callable[[], bool]] = None,
                 clock: Callable[[], float] = time.monotonic) -> None:
        self._cfg = cfg
        self._bms = bms_service
        self._plug = plug
        self._state_path = Path(state_path)
        self._system_stopping = system_stopping or system_is_stopping
        self._clock = clock
        self._lock = threading.Lock()
        self._wake = threading.Event()
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._policy = ChargePolicy(cfg, self._load_mode())
        self._last_switch_t: Optional[float] = None
        self._operator_change = False
        self._status: Dict[str, Any] = {
            "mode": self._policy.mode,
            "plug_on": None,
            "want_on": None,
            "reason": "not started",
            "soc_pct": None,
            "pack_v": None,
            "pack_a": None,
            "bms_fresh": False,
            "plug_error": None,
            "last_switch_epoch": None,
            "switch_count": 0,
            "updated_epoch": None,
            "host": getattr(cfg, "host", ""),
        }

    # -- persistence ------------------------------------------------------

    def _load_mode(self) -> str:
        default = getattr(self._cfg, "default_mode", "hold")
        if default not in MODES:
            default = "hold"
        try:
            with open(self._state_path, "r", encoding="utf-8") as fh:
                data = json.load(fh)
            mode = data.get("mode")
            if mode in MODES:
                return mode
        except FileNotFoundError:
            pass
        except Exception as e:  # unreadable file: fall back, say so
            logger.warning("charger state file unreadable (%s); mode %s", e, default)
        return default

    def _save_mode(self, mode: str) -> None:
        try:
            self._state_path.parent.mkdir(parents=True, exist_ok=True)
            tmp = self._state_path.with_suffix(".tmp")
            with open(tmp, "w", encoding="utf-8") as fh:
                json.dump({"mode": mode, "updated_epoch": time.time()}, fh)
            os.replace(tmp, self._state_path)
        except Exception as e:
            logger.warning("charger state file not written: %s", e)

    # -- lifecycle --------------------------------------------------------

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._run, name="ChargeManager", daemon=True)
        self._thread.start()
        logger.warning("Charger plug manager started: host %s, mode %s",
                       getattr(self._cfg, "host", "?"), self._policy.mode)

    def stop(self) -> None:
        self._running = False
        self._wake.set()
        t = self._thread
        if t is not None and t.is_alive():
            t.join(timeout=5.0)
        mode = self._policy.mode
        if mode in ("hold", "full") and getattr(self._cfg, "plug_off_on_system_stop", True):
            if self._system_stopping():
                ok = self._switch(False, "system shutdown: no charging without a battery reading")
                logger.warning("Charger plug off at shutdown: %s", "done" if ok else "FAILED")

    # -- operator ---------------------------------------------------------

    def set_mode(self, mode: Any) -> "tuple[bool, str]":
        if not isinstance(mode, str) or mode not in MODES:
            return False, f"mode must be one of {', '.join(MODES)}"
        with self._lock:
            self._policy.set_mode(mode)
            self._operator_change = True
            self._status["mode"] = mode
        self._save_mode(mode)
        logger.warning("Charger plug mode set to %s (operator)", mode)
        self._wake.set()
        return True, f"mode {mode}"

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

    def _read_bms(self) -> ChargeInputs:
        soc = v = a = None
        fresh = False
        try:
            st = self._bms.get_state()
            link = self._bms.get_link_health()
            soc, v, a = st.soc_pct, st.pack_voltage_v, st.pack_current_a
            max_age = float(getattr(self._cfg, "bms_max_age_s", 30.0))
            fresh = (link.state == "up" and link.data_age_s is not None
                     and link.data_age_s <= max_age and bool(st.connected))
        except Exception as e:
            logger.debug("charger: BMS read failed: %s", e)
        return ChargeInputs(soc_pct=soc, pack_v=v, pack_a=a, bms_fresh=fresh, plug_on=None)

    def _read_plug(self) -> "tuple[Optional[bool], Optional[str]]":
        try:
            return self._plug.get_state(), None
        except Exception as e:
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
            self._last_switch_t = now
            self._status["plug_on"] = on
            self._status["plug_error"] = None
            self._status["last_switch_epoch"] = time.time()
            self._status["switch_count"] = int(self._status.get("switch_count", 0)) + 1
        logger.warning("Charger plug %s: %s", "ON" if on else "OFF", reason)
        return True

    def tick(self, now: Optional[float] = None) -> Dict[str, Any]:
        """One poll. Public for tests; the thread calls it every poll_s."""
        if now is None:
            now = self._clock()
        bms = self._read_bms()
        plug_on, plug_err = self._read_plug()
        inp = ChargeInputs(bms.soc_pct, bms.pack_v, bms.pack_a, bms.bms_fresh, plug_on)

        with self._lock:
            dec = self._policy.decide(inp, now)
            prev_mode = self._status.get("mode")
            operator = self._operator_change
            self._operator_change = False
            last_switch = self._last_switch_t
        if dec.mode != prev_mode:
            self._save_mode(dec.mode)
            logger.warning("Charger plug mode %s -> %s (%s)", prev_mode, dec.mode, dec.reason)

        reason = dec.reason
        if plug_err is not None:
            reason = f"{reason}; plug unreachable: {plug_err}"
        elif dec.want_on is not None and dec.want_on != plug_on:
            min_gap = float(getattr(self._cfg, "min_switch_interval_s", 60.0))
            if (not operator and last_switch is not None and now - last_switch < min_gap):
                reason = f"{reason}; switch in {min_gap - (now - last_switch):.0f} s"
            else:
                if self._switch(dec.want_on, dec.reason):
                    plug_on = dec.want_on
                else:
                    reason = f"{reason}; switch FAILED"

        with self._lock:
            self._status.update({
                "mode": dec.mode,
                "plug_on": plug_on,
                "want_on": dec.want_on,
                "reason": reason,
                "soc_pct": inp.soc_pct,
                "pack_v": inp.pack_v,
                "pack_a": inp.pack_a,
                "bms_fresh": inp.bms_fresh,
                "plug_error": plug_err,
                "updated_epoch": time.time(),
            })
            return dict(self._status)
