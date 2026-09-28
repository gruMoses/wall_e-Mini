#!/usr/bin/env python3
"""Wi-Fi reconnect watchdog for the robot's Pi (NetworkManager 1.42).

WHY THIS EXISTS (2026-09-27 17:35-18:13): at the edge of Wi-Fi range,
NetworkManager failed three association attempts, decided the password
was wrong ("failed (reason 'no-secrets')"), and blocked autoconnect for
that profile. The Pi stayed up the whole time, came back into range at
18:05, and still stayed off the network until a power cycle at 18:13.
Wi-Fi is the robot's only IPv4 path, so nobody could reach it.

NM 1.42 clears a no-secrets block only when a secret agent registers,
networking restarts, the profile reaches IP config, or the profile is
updated with secrets (src/core/nm-policy.c). Its 300 s retry timer does
not. connection.auth-retries 0 covers association timeouts only: a drop
during the WPA 4-way handshake always asks for new secrets, and with no
agent that ends in the same block.

WHAT THIS SCRIPT DOES, every --interval seconds (default 15):
  1. Reads wlan0's NM device state.
  2. NM idle: wlan0 has been "disconnected" (state 30) for --offline-s
     seconds (default 120), so NM has given up for now. Or stuck: wlan0
     has not been connected, in any state, for --stuck-s seconds (default
     600) -- a profile with infinite auth retries loops in config/need-auth
     and never reaches 30. Then it rescans, takes the saved autoconnect
     Wi-Fi profiles that NM lists as available on wlan0 (NM checks the SSID
     AND the profile's band), and runs
         nmcli --wait 150 connection up uuid <uuid> ifname wlan0
     for the best one: highest autoconnect priority, then strongest signal.
     nmcli registers its own secret agent for the call, which clears every
     no-secrets block. It never interrupts an activation NM is running,
     except through the --stuck-s safety net.
  3. A profile that failed in the last --fail-backoff-s seconds (default
     600) goes behind the fresh ones; among failed ones the one tried
     longest ago goes first, so two failing profiles take turns.
  4. Tries are at least --retry-s seconds apart (default 60), counted from
     the END of the previous try (a failed `up` can take 150 s).
  5. Connected, but the default gateway has not answered ping or ARP for
     --gateway-dark-s seconds (default 180) AND has never answered since
     this connection came up -- a profile on the wrong subnet. The active
     profile counts as failed and step 2 runs. A gateway that answered at
     least once on this connection and then stops (a router restart) is
     logged and left alone: bouncing a working link cannot fix a router.

It never takes down a link whose gateway has answered, only acts on saved
autoconnect Wi-Fi profiles, and never reads, enters or prints a password.
See docs/wifi_watchdog.md.
"""

from __future__ import annotations

import argparse
import logging
import re
import signal
import subprocess
import time
from dataclasses import dataclass, field
from typing import Callable, Optional

LOG = logging.getLogger("wifi_watchdog")

CONNECTED = 100
DISCONNECTED = 30
WIFI_TYPE = "802-11-wireless"
_UUID_RE = re.compile(r"[0-9a-fA-F]{8}-[0-9a-fA-F]{4}-[0-9a-fA-F]{4}-[0-9a-fA-F]{4}-[0-9a-fA-F]{12}")


def split_terse(line: str) -> list[str]:
    """Split one `nmcli -t` line on unescaped ':' and unescape '\\:' / '\\\\'."""
    fields, cur, i = [], [], 0
    while i < len(line):
        ch = line[i]
        if ch == "\\" and i + 1 < len(line):
            cur.append(line[i + 1])
            i += 2
            continue
        if ch == ":":
            fields.append("".join(cur))
            cur = []
        else:
            cur.append(ch)
        i += 1
    fields.append("".join(cur))
    return fields


@dataclass
class Profile:
    name: str
    uuid: str
    priority: int
    ssid: str


def default_runner(cmd: list[str], timeout: float) -> tuple[int, str]:
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        return 124, ""
    except OSError as exc:
        return 127, str(exc)
    return proc.returncode, proc.stdout + (("\n" + proc.stderr) if proc.returncode else "")


class Nm:
    """The few nmcli / ip / ping calls the watchdog needs."""

    def __init__(self, runner: Callable[[list[str], float], tuple[int, str]] = default_runner,
                 iface: str = "wlan0") -> None:
        self.run = runner
        self.iface = iface

    def device_state(self) -> Optional[int]:
        rc, out = self.run(["nmcli", "-g", "GENERAL.STATE", "device", "show", self.iface], 15)
        if rc != 0 or not out.strip():
            return None
        try:
            return int(out.strip().split()[0])
        except ValueError:
            return None

    def active_uuid(self) -> Optional[str]:
        rc, out = self.run(["nmcli", "-g", "GENERAL.CON-UUID", "device", "show", self.iface], 15)
        uuid = out.strip().lower() if rc == 0 else ""
        return uuid or None

    def available_uuids(self) -> set[str]:
        """Profiles NM itself lists as available on the device (SSID and band checked)."""
        rc, out = self.run(["nmcli", "-g", "CONNECTIONS.AVAILABLE-CONNECTIONS", "device", "show",
                            self.iface], 15)
        if rc != 0:
            return set()
        return {m.lower() for m in _UUID_RE.findall(out)}

    def profiles(self) -> list[Profile]:
        rc, out = self.run(["nmcli", "-t", "-f", "NAME,UUID,TYPE,AUTOCONNECT,AUTOCONNECT-PRIORITY",
                            "connection", "show"], 15)
        if rc != 0:
            return []
        result = []
        for line in out.splitlines():
            parts = split_terse(line)
            if len(parts) != 5 or parts[2] != WIFI_TYPE or parts[3] != "yes":
                continue
            rc2, ssid = self.run(["nmcli", "-g", "802-11-wireless.ssid", "connection", "show",
                                  "uuid", parts[1]], 15)
            try:
                prio = int(parts[4])
            except ValueError:
                prio = 0
            result.append(Profile(name=parts[0], uuid=parts[1].lower(), priority=prio,
                                  ssid=ssid.strip() if rc2 == 0 else ""))
        return result

    def visible(self) -> dict[str, int]:
        """SSID -> best signal (0-100): a fresh scan, else NM's cached list."""
        cmd = ["nmcli", "-t", "-f", "SSID,SIGNAL", "device", "wifi", "list", "ifname", self.iface,
               "--rescan"]
        rc, out = self.run(cmd + ["yes"], 45)
        if rc != 0:  # e.g. a scan refused right after another scan
            rc, out = self.run(cmd + ["no"], 15)
        seen: dict[str, int] = {}
        if rc != 0:
            return seen
        for line in out.splitlines():
            parts = split_terse(line)
            if len(parts) != 2 or not parts[0]:
                continue
            try:
                sig = int(parts[1])
            except ValueError:
                continue
            seen[parts[0]] = max(sig, seen.get(parts[0], -1))
        return seen

    def up(self, uuid: str) -> tuple[bool, str]:
        rc, out = self.run(["nmcli", "--wait", "150", "connection", "up", "uuid", uuid,
                            "ifname", self.iface], 170)
        return rc == 0, " ".join(out.split())[:300]

    def gateway(self) -> Optional[str]:
        rc, out = self.run(["ip", "-4", "route", "show", "default", "dev", self.iface], 5)
        if rc != 0:
            return None
        for line in out.splitlines():
            parts = line.split()
            if "via" in parts:
                idx = parts.index("via")
                if idx + 1 < len(parts):
                    return parts[idx + 1]
        return None

    def gateway_answers(self, gw: str) -> bool:
        rc, _ = self.run(["ping", "-c", "1", "-W", "2", "-I", self.iface, gw], 5)
        if rc == 0:
            return True
        rc, out = self.run(["ip", "neigh", "show", gw, "dev", self.iface], 5)
        text = out.upper()
        return rc == 0 and "LLADDR" in text and not any(s in text for s in ("FAILED", "INCOMPLETE"))


@dataclass
class WatchdogConfig:
    offline_s: float = 120.0
    stuck_s: float = 600.0
    retry_s: float = 60.0
    fail_backoff_s: float = 600.0
    gateway_interval_s: float = 30.0
    gateway_dark_s: float = 180.0


@dataclass
class Watchdog:
    nm: Nm
    cfg: WatchdogConfig = field(default_factory=WatchdogConfig)
    clock: Callable[[], float] = time.monotonic
    offline_since: Optional[float] = None
    idle_since: Optional[float] = None
    last_attempt_end: Optional[float] = None
    link_uuid: Optional[str] = None
    gateway_answered: bool = False
    last_gateway_check: Optional[float] = None
    gateway_dark_since: Optional[float] = None
    outage_logged: bool = False
    failed_at: dict = field(default_factory=dict)
    no_candidate_logged: bool = False

    def tick(self, now: float) -> None:
        state = self.nm.device_state()
        if state == CONNECTED:
            self._connected(now)
            return
        # Not connected: forget the link, track idle and offline time.
        self.link_uuid = None
        self.gateway_answered = False
        self.last_gateway_check = None
        self.gateway_dark_since = None
        self.outage_logged = False
        if self.offline_since is None:
            self.offline_since = now
            LOG.warning("Wi-Fi not connected (NM device state %s)", state)
        if state == DISCONNECTED:
            if self.idle_since is None:
                self.idle_since = now
        else:
            self.idle_since = None
        idle = self.idle_since is not None and now - self.idle_since >= self.cfg.offline_s
        stuck = now - self.offline_since >= self.cfg.stuck_s
        if idle or stuck:
            self._try_reconnect(now, reason="NM idle" if idle else "stuck")

    def _connected(self, now: float) -> None:
        if self.offline_since is not None:
            LOG.warning("Wi-Fi back after %.0f s offline", now - self.offline_since)
            self.offline_since = None
            self.no_candidate_logged = False
        self.idle_since = None
        uuid = self.nm.active_uuid()
        if uuid != self.link_uuid:
            # A new connection: its gateway has not answered yet.
            self.link_uuid = uuid
            self.gateway_answered = False
            self.last_gateway_check = None
            self.gateway_dark_since = None
            self.outage_logged = False
        if self.last_gateway_check is not None and now - self.last_gateway_check < self.cfg.gateway_interval_s:
            return
        self.last_gateway_check = now
        gw = self.nm.gateway()
        if gw is None or self.nm.gateway_answers(gw):
            if gw is not None:
                self.gateway_answered = True
            self.gateway_dark_since = None
            self.outage_logged = False
            return
        if self.gateway_dark_since is None:
            self.gateway_dark_since = now
        if self.gateway_answered:
            if not self.outage_logged:
                LOG.warning("Wi-Fi gateway %s stopped answering; it answered earlier on this "
                            "connection, so this is not a Wi-Fi fault; leaving the link up", gw)
                self.outage_logged = True
            return
        if now - self.gateway_dark_since >= self.cfg.gateway_dark_s:
            LOG.warning("Wi-Fi connected, but gateway %s has not answered since the connection came "
                        "up (%.0f s); re-activating", gw, now - self.gateway_dark_since)
            if uuid:
                self.failed_at[uuid] = now
            self._try_reconnect(now, reason="dark gateway")

    def _try_reconnect(self, now: float, reason: str) -> None:
        if self.last_attempt_end is not None and now - self.last_attempt_end < self.cfg.retry_s:
            return
        profiles = self.nm.profiles()
        seen = self.nm.visible()  # rescan first so NM's available list is current
        available = self.nm.available_uuids()
        candidates = [p for p in profiles if p.uuid in available]
        if not candidates:
            if not self.no_candidate_logged:
                LOG.warning("No saved Wi-Fi profile available (%d saved, %d SSIDs visible); waiting",
                            len(profiles), len(seen))
                self.no_candidate_logged = True
            self.last_attempt_end = self.clock()
            return
        self.no_candidate_logged = False

        def order(p: Profile) -> tuple:
            t = self.failed_at.get(p.uuid)
            if t is not None and now - t < self.cfg.fail_backoff_s:
                return (1, t, -p.priority, -seen.get(p.ssid, 0))
            return (0, 0.0, -p.priority, -seen.get(p.ssid, 0))

        candidates.sort(key=order)
        pick = candidates[0]
        LOG.warning("Re-activating Wi-Fi profile '%s' (SSID '%s', signal %s; %s)",
                    pick.name, pick.ssid, seen.get(pick.ssid, "?"), reason)
        ok, detail = self.nm.up(pick.uuid)
        self.last_attempt_end = self.clock()
        if ok:
            LOG.warning("Wi-Fi profile '%s' activated", pick.name)
            self.failed_at.pop(pick.uuid, None)
            # A fresh activation: its gateway gets the full dark window again.
            self.link_uuid = None
        else:
            LOG.warning("Wi-Fi profile '%s' did not activate: %s", pick.name, detail)
            self.failed_at[pick.uuid] = self.last_attempt_end


class _Shutdown:
    def __init__(self) -> None:
        self.requested = False

    def handle(self, signum, frame) -> None:
        self.requested = True


def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Re-activate Wi-Fi when NetworkManager gives up.")
    p.add_argument("--iface", default="wlan0")
    p.add_argument("--interval", type=float, default=15.0)
    p.add_argument("--offline-s", type=float, default=120.0)
    p.add_argument("--stuck-s", type=float, default=600.0)
    p.add_argument("--retry-s", type=float, default=60.0)
    p.add_argument("--fail-backoff-s", type=float, default=600.0)
    p.add_argument("--gateway-interval", type=float, default=30.0)
    p.add_argument("--gateway-dark-s", type=float, default=180.0)
    return p


def main(argv: Optional[list[str]] = None) -> int:
    args = build_arg_parser().parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
    cfg = WatchdogConfig(offline_s=args.offline_s, stuck_s=args.stuck_s, retry_s=args.retry_s,
                         fail_backoff_s=args.fail_backoff_s,
                         gateway_interval_s=args.gateway_interval,
                         gateway_dark_s=args.gateway_dark_s)
    dog = Watchdog(nm=Nm(iface=args.iface), cfg=cfg)
    shutdown = _Shutdown()
    signal.signal(signal.SIGTERM, shutdown.handle)
    signal.signal(signal.SIGINT, shutdown.handle)
    LOG.warning("Wi-Fi watchdog started on %s (idle %.0f s, stuck %.0f s, gateway dark %.0f s)",
                args.iface, cfg.offline_s, cfg.stuck_s, cfg.gateway_dark_s)
    while not shutdown.requested:
        try:
            dog.tick(time.monotonic())
        except Exception:  # never let one bad nmcli parse kill the watchdog
            LOG.exception("Wi-Fi watchdog tick failed")
        slept = 0.0
        while slept < args.interval and not shutdown.requested:
            time.sleep(1.0)
            slept += 1.0
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
