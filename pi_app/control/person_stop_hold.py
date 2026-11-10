"""Memory for the YOLO person/animal hard stop (pure, hardware-free).

The defect
----------
``OakDepthReader._apply_safety_tier_override`` decides the 0.8 m stop again
at every depth poll and has no memory. Inside about 0.4 m the person's box is
clipped by the frame edge, the depth sample fails (``no_support``,
``far_veto``, ``ambiguous``) or reads the background through the clipped box,
the box is narrower than the ``safety_stop_bbox_width`` backstop, and the
corridor ignores pixels at or below ``min_depth_mm``. One such frame released
the stop (2026-11-08 14:27:40, FOLLOW_ME: about 0.66 s with Kevin 0.3-0.9 m
away; the clipped box sampled the background at 2.6-2.7 m).

While the hold is active the published obstacle distance is 0.0. The
controller scales only the forward part of the command with it, so a pivot
in place and reverse still pass (Kevin, 2026-11-08).

Modes
-----
* FOLLOW_ME / WAYPOINT_NAV ("autonomous"): the hold ENGAGES on a stateless
  trigger, or by carry-in: every trigger is time-stamped in every mode, and
  the first poll that sees an autonomous mode engages when that trigger is at
  most ``carry_in_s`` old (the chain is seeded with the triggering box at its
  REAL sighting time).
* MANUAL, ``in_manual`` False (the default, Kevin's 2026-09-27 rule): no hold
  engages. A hold that is active when the mode becomes MANUAL (the operator's
  exit, or the controller's automatic vision-stale exit) stays applied until
  ``NeutralTimer`` says no drive command was given for its hold time, or an
  evidence or operator release.
* MANUAL, ``in_manual`` True (not the default; needs Kevin's word): a hold
  also engages in MANUAL, and a hold ENGAGED in MANUAL may release on the
  corridor without a resolved chain (and ignores ``operator_only``) while
  the mode is MANUAL. See ``_manual_relaxed``.

The chain (the only object memory; O(1), no list that can grow)
--------------------------------------------------------------
On engage the chain is the box that forced the stop (for a carry-in, the box
of the stored trigger). It keeps: group (person, or animal for every other
stop-tier class; YOLO calls one goat horse, sheep or cow), last box centre x,
a range reference, an area reference, last sighting time, ``beyond_streak``
and ``resolved``. Besides the chain the hold keeps the evidence counter,
``operator_only`` and one bool for rule 3.

A box is SEEN-BEYOND when its depth status is "ok", its range is at or
beyond ``release_distance_m``, and it is at least ``edge_margin`` from the
left and right frame edge. A box within ``edge_margin`` of a side is CLIPPED:
its range is never trusted for anything and its area is only a lower bound
of the object's apparent area. A range is TRUSTED only from an unclipped
"ok" box.

On each fresh poll (states: CHAIN PRESENT / CHAIN MISSING / CHAIN LOST):

1. Candidates for the chain: stop-tier boxes of the chain's group whose
   centre is within ``chain_gate`` of the chain's last centre, seen within
   ``chain_gap_s`` of the last sighting (``carry_in_s`` for an unused
   carry-in seed), and physically plausible (``_plausible``).
   Label flicker: YOLO can call a close, clipped person "horse" for a
   packet (2026-11-08 14:27:41). When the poll has exactly ONE stop-tier
   box, and its centre is within ``FLICKER_GATE`` of the chain's last
   centre, the group is not compared: one box in the same place with a
   plausible size is the same object, whatever its label. With two or more
   boxes in view the group rule stays.
   - exactly one: it continues the chain (CHAIN PRESENT);
   - none: CHAIN MISSING; past the gap limit the chain is LOST:
     ``operator_only`` ("lost");
   - two or more: ``operator_only`` ("ambiguous"), the chain is missing.
2. Every other stop-tier box: one that is not seen-beyond on 2 consecutive
   fresh polls is a second unresolved object: ``operator_only``
   ("second_object"). Two polls, not one: a one-frame YOLO false positive
   (a post or a shadow called "person" for one packet) must not turn every
   hold into an operator-only one; two polls are ~0.13 s at 15 fps, and the
   rule is held across polls by one bool (ANY such box on both polls), so
   it cannot grow and errs toward holding. A seen-beyond box whose centre
   is inside the gate while the chain is missing could be mistaken for the
   chain: ``operator_only`` ("impostor_in_gate"). Any other box is ignored.
3. A stateless trigger whose box is not the chain's continuation (a second
   object got close): ``operator_only`` ("second_trigger").
4. ``beyond_streak`` counts consecutive fresh polls with the chain present
   and its box seen-beyond; any other fresh poll resets it. ``resolved``
   is ``beyond_streak >= resolve_polls`` on THIS poll; a poll without the
   chain clears it.

Releases (evidence counts only on a fresh poll: a poll without a new
detection packet neither counts nor resets; time alone never releases):

* seen_beyond (autonomous, and a hold carried into MANUAL): only when NOT
  ``operator_only``, ``release_polls`` consecutive fresh polls with the
  chain present and resolved, and every stop-tier box in view seen-beyond.
  This is the ONLY evidence release in those modes. The corridor is not
  evidence there (safety review 4, 2026-11-10): on 2026-11-08 it read
  2.9-4.7 m with the person at 0.3-0.9 m, and five empty polls fit inside
  the gap limit, so a person who was seen beyond and came back in undetected
  would have been released on it. An object that leaves the view before it
  has been seen beyond for ``release_polls`` polls makes the chain LOST.
* corridor_clear: ``in_manual`` True only, for a hold engaged in MANUAL
  (``_manual_relaxed``): no stop-tier box in view and the corridor measured
  (not rejected), at least ``min_valid_pct`` valid pixels, raw near distance
  at or beyond ``release_distance_m``.
* operator: the arm switch low on a live RC link (control loop). A trigger
  on the same poll in an autonomous mode engages a new hold at once.
* manual_neutral: a hold in MANUAL that was not engaged there, after
  ``NeutralTimer`` (both commanded track bytes near centre for its hold
  time; reverse and pivot are drive commands; unknown is not neutral).

``operator_only`` clears only when the hold ends (operator or manual_neutral).

Known and ACCEPTED consequences (owner's call; do not engineer around them):
a second person or animal near the chain, a chain lost for more than
``chain_gap_s`` (a detection gap or a stream stall), two candidates, or a
range or size jump beyond the plausibility bounds make the hold
operator-only. The operator ends it with the arm switch, by toggling
FOLLOW_ME, or with stop then GO on the waypoint page (both go through MANUAL
with the sticks neutral). The bounds assume the robot does not drive the
range itself: in FOLLOW_ME and WAYPOINT_NAV the hold leaves only pivots
(neither mode commands reverse); an operator who reverses during a hold
carried into MANUAL can break the chain, and letting go of the stick ends
that hold anyway. A later version can use real identity from an on-device
object tracker.

Threading: one writer. ``OakDepthReader._poll_depth`` (vision worker
thread) is the only caller of ``update``; the reader publishes
``snapshot()`` under its own lock.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class PersonStopHoldSnapshot:
    active: bool = False
    # What engaged it ("trigger", "carry_in"), or how it ended
    # ("seen_beyond", "corridor_clear", "operator", "manual_neutral");
    # "" before the first event.
    reason: str = ""
    evidence_polls: int = 0
    # True: no evidence release; only the operator ends this hold.
    operator_only: bool = False
    # Why ("lost", "ambiguous", "second_object", "impostor_in_gate",
    # "second_trigger"); "" when not operator_only.
    operator_only_reason: str = ""
    chain: str = ""            # "person" / "animal" / "" (no hold)
    chain_present: bool = False
    beyond_streak: int = 0
    resolved: bool = False
    manual_carry: bool = False

    def as_dict(self) -> dict:
        return {
            "active": self.active,
            "reason": self.reason,
            "evidence_polls": self.evidence_polls,
            "operator_only": self.operator_only,
            "operator_only_reason": self.operator_only_reason,
            "chain": self.chain,
            "chain_present": self.chain_present,
            "beyond_streak": self.beyond_streak,
            "resolved": self.resolved,
            "manual_carry": self.manual_carry,
        }


def _num(v) -> float | None:
    """A finite float, else None. Never raises."""
    if isinstance(v, bool):
        return None
    try:
        f = float(v)
    except (TypeError, ValueError):
        return None
    return f if math.isfinite(f) else None


def _bbox(bb):
    """A box sequence as (x1, y1, x2, y2) finite floats, else None. Never raises."""
    try:
        if bb is None or len(bb) < 4:
            return None
        vals = tuple(_num(bb[i]) for i in range(4))
    except Exception:
        return None
    if any(v is None for v in vals):
        return None
    return vals


def side_clipped(bbox, edge_margin: float) -> bool:
    """True when a normalized box is within ``edge_margin`` of the left or
    right edge, or is not a usable box."""
    bb = _bbox(bbox)
    if bb is None:
        return True
    return bb[0] < edge_margin or bb[2] > 1.0 - edge_margin


def rc_link_live(last_update_mono_s, now_mono: float, stale_s: float) -> bool:
    """RC link live for the operator release (main.py).

    ``last_update_mono_s``: ``RCInputs.last_update_mono_s``, the monotonic
    time the reader got the last frame; 0.0 means no frame yet. A future
    stamp (negative age) or a non-number is not live.
    """
    stamp = _num(last_update_mono_s)
    now = _num(now_mono)
    if stamp is None or now is None or stamp <= 0.0:
        return False
    age = now - stamp
    return 0.0 <= age <= float(stale_s)


class NeutralTimer:
    """MANUAL "no drive command at all" for ``hold_s`` (control loop only).

    Neutral: MANUAL, and both commanded track bytes (before the obstacle
    layer, RC or phone) within ``band_byte`` of 126. Reverse and a pivot are
    drive commands. Unknown bytes (an early-return tick) are not neutral and
    restart the timer. A stick swept through centre is inside the band for
    one or two ticks only.
    """

    CENTER = 126

    def __init__(self, band_byte: float = 6.0, hold_s: float = 0.25) -> None:
        self._band = float(band_byte)
        self._hold_s = float(hold_s)
        self._since: float | None = None

    def update(self, now: float, manual: bool, left, right) -> bool:
        lv, rv = _num(left), _num(right)
        ok = (
            manual
            and lv is not None and rv is not None
            and abs(lv - self.CENTER) <= self._band
            and abs(rv - self.CENTER) <= self._band
        )
        if not ok:
            self._since = None
            return False
        if self._since is None:
            self._since = now
        return (now - self._since) >= self._hold_s


@dataclass(frozen=True)
class _Box:
    """One stop-tier detection, parsed once per poll (never raises)."""
    det: object
    group: str
    cx: float | None          # None: no usable box
    area: float
    clipped: bool             # within edge_margin of a side, or no box
    z: float | None           # trusted range (unclipped "ok"), else None
    beyond: bool              # seen-beyond


@dataclass
class _Chain:
    group: str
    cx: float
    last_seen: float
    seed: bool                # unused carry-in seed: gap limit carry_in_s
    z_ref: float              # last trusted range, or the stop radius
    z_ref_ts: float
    z_trusted: bool           # False: z_ref is the stop radius (upper bound)
    area_ref: float           # area of the last unclipped box (or of the
    area_ref_ts: float        # engage box, as a lower bound when clipped)
    area_exact: bool          # area_ref from an unclipped box
    beyond_streak: int = 0
    resolved: bool = False


class PersonStopHold:
    # Label flicker (see the module docstring): with exactly one stop-tier
    # box in view, a box of the other group continues the chain when its
    # centre is this close to the chain's last centre (normalized x). 0.10
    # is a third of chain_gate: a clipped person relabelled "horse" keeps its
    # place, and the plausibility test still has to pass.
    FLICKER_GATE = 0.10

    def __init__(
        self,
        release_distance_m: float = 1.0,
        release_polls: int = 5,
        min_valid_pct: float = 20.0,
        edge_margin: float = 0.05,
        in_manual: bool = False,
        carry_in_s: float = 1.0,
        chain_gap_s: float = 0.6,
        chain_gate: float = 0.30,
        resolve_polls: int = 3,
        stop_radius_m: float = 0.8,
        plausible_speed_mps: float = 1.5,
        plausible_margin_m: float = 0.25,
        plausible_area_factor: float = 1.5,
        plausible_min_range_m: float = 0.2,
    ) -> None:
        self._release_mm = float(release_distance_m) * 1000.0
        self._release_m = float(release_distance_m)
        self._release_polls = max(1, int(release_polls))
        self._min_valid_pct = float(min_valid_pct)
        self._edge_margin = float(edge_margin)
        self._in_manual = bool(in_manual)
        self._carry_in_s = float(carry_in_s)
        self._gap_s = float(chain_gap_s)
        self._gate = float(chain_gate)
        self._resolve_polls = max(1, int(resolve_polls))
        self._stop_radius_m = float(stop_radius_m)
        self._v_max = float(plausible_speed_mps)
        self._z_margin = float(plausible_margin_m)
        self._area_factor = max(1.0, float(plausible_area_factor))
        self._z_min = max(1e-3, float(plausible_min_range_m))
        # Hold state (all scalars + one chain).
        self._active = False
        self._reason = ""
        self._evidence = 0
        self._chain: _Chain | None = None
        self._chain_present = False
        self._operator_only = False
        self._operator_only_reason = ""
        self._other_prev = False
        self._manual_carry = False
        self._engaged_in_manual = False
        # Mode edge and the last trigger (every mode), for carry-in.
        self._prev_engage_mode = False
        self._last_trigger_ts: float | None = None
        self._last_trigger_box: _Box | None = None

    # ------------------------------------------------------------------
    @property
    def active(self) -> bool:
        return self._active

    def snapshot(self) -> PersonStopHoldSnapshot:
        c = self._chain
        return PersonStopHoldSnapshot(
            active=self._active,
            reason=self._reason,
            evidence_polls=self._evidence,
            operator_only=self._operator_only,
            operator_only_reason=self._operator_only_reason,
            chain=c.group if (c is not None and self._active) else "",
            chain_present=bool(self._active and self._chain_present),
            beyond_streak=c.beyond_streak if (c is not None and self._active) else 0,
            resolved=bool(c is not None and self._active and c.resolved),
            manual_carry=self._manual_carry,
        )

    def seen_beyond(self, det) -> bool:
        return self._parse(det).beyond

    def force_release(self, reason: str) -> None:
        """End the hold without touching any detection (cannot raise).

        The reader calls this when ``update`` raised on a poll that also
        carries an operator release, so a defect in the hold can never need
        a process restart to clear.
        """
        self._release(reason)

    # ------------------------------------------------------------------
    def _parse(self, det) -> _Box:
        name = getattr(det, "label_name", "")
        group = "person" if name == "person" else "animal"
        bb = _bbox(getattr(det, "bbox", None))
        if bb is None:
            return _Box(det, group, None, 0.0, True, None, False)
        x1, y1, x2, y2 = bb
        cx = (x1 + x2) / 2.0
        area = max(0.0, x2 - x1) * max(0.0, y2 - y1)
        clipped = x1 < self._edge_margin or x2 > 1.0 - self._edge_margin
        z = _num(getattr(det, "z_m", None))
        ok = getattr(det, "depth_status", "ok") == "ok"
        trusted = (z if (ok and not clipped and z is not None and z > 0.0) else None)
        beyond = trusted is not None and trusted >= self._release_m
        return _Box(det, group, cx, area, clipped, trusted, beyond)

    def _reach(self, dt: float) -> float:
        """How far the range can change in ``dt`` seconds."""
        return self._v_max * max(0.0, dt) + self._z_margin

    def _plausible(self, c: _Chain, b: _Box, now: float) -> bool:
        """Can box ``b`` be the chain's object (centre and gap checked first)?

        Range, only when the box's range is trusted: with a trusted chain
        range |z - z_ref| <= reach(now - z_ref_ts); with only the implied
        reference (the chain never had a trusted range; the trigger put the
        object inside the stop radius) z <= stop radius + reach. The time
        runs from the reference, so a chain of clipped or unmeasured boxes
        widens the bound only with real time and never "refreshes" it.

        Area (apparent area goes as 1/z^2; ``area_factor`` absorbs YOLO box
        jitter), with z0 = the chain's range reference and r = reach(now -
        area_ref_ts):
        * shrink, unclipped boxes only (a clipped box shows a fragment):
          area >= area_ref * (z0 / (z0 + r))^2 / area_factor;
        * growth, only against an exact (unclipped) reference (a clipped
          box's visible area is a lower bound, so this stays valid for it):
          area <= area_ref * (z0 / max(z0 - r, z_min))^2 * area_factor.
        The area reference is set only by unclipped boxes (the engage box
        is the exception, as a lower bound), so a clipped fragment's small
        area never opens the shrink test.
        """
        if b.z is not None:
            reach = self._reach(now - c.z_ref_ts)
            if c.z_trusted:
                if abs(b.z - c.z_ref) > reach:
                    return False
            elif b.z > c.z_ref + reach:
                return False
        if c.area_ref > 0.0:
            z0 = max(c.z_ref, self._z_min)
            r = self._reach(now - c.area_ref_ts)
            ratio = b.area / c.area_ref
            if not b.clipped:
                lo = (z0 / (z0 + r)) ** 2 / self._area_factor
                if ratio < lo:
                    return False
            if c.area_exact:
                hi = (z0 / max(z0 - r, self._z_min)) ** 2 * self._area_factor
                if ratio > hi:
                    return False
        return True

    def _new_chain(self, b: _Box, seen_ts: float, seed: bool) -> _Chain | None:
        if b.cx is None:
            return None
        trusted = b.z is not None
        return _Chain(
            group=b.group, cx=b.cx, last_seen=seen_ts, seed=seed,
            z_ref=b.z if trusted else self._stop_radius_m, z_ref_ts=seen_ts,
            z_trusted=trusted,
            area_ref=b.area, area_ref_ts=seen_ts, area_exact=not b.clipped,
            beyond_streak=1 if b.beyond else 0,
            resolved=(1 if b.beyond else 0) >= self._resolve_polls,
        )

    def _continue(self, c: _Chain, b: _Box, now: float) -> None:
        c.cx = b.cx
        c.last_seen = now
        c.seed = False
        if b.z is not None:
            c.z_ref, c.z_ref_ts, c.z_trusted = b.z, now, True
        if not b.clipped:
            c.area_ref, c.area_ref_ts, c.area_exact = b.area, now, True
        c.beyond_streak = c.beyond_streak + 1 if b.beyond else 0
        c.resolved = c.beyond_streak >= self._resolve_polls

    def _latch(self, why: str) -> None:
        if not self._operator_only:
            self._operator_only = True
            self._operator_only_reason = why
        self._evidence = 0

    def _release(self, reason: str) -> None:
        self._active = False
        self._reason = reason
        self._evidence = 0
        self._chain = None
        self._chain_present = False
        self._operator_only = False
        self._operator_only_reason = ""
        self._other_prev = False
        self._manual_carry = False
        self._engaged_in_manual = False

    def _engage(self, reason: str, box: _Box | None, seen_ts: float, seed: bool,
                in_manual_mode: bool) -> None:
        self._active = True
        self._reason = reason
        self._evidence = 0
        self._operator_only = False
        self._operator_only_reason = ""
        self._other_prev = False
        self._manual_carry = False
        self._engaged_in_manual = in_manual_mode
        self._chain = self._new_chain(box, seen_ts, seed) if box is not None else None
        self._chain_present = False
        if self._chain is None:
            # A trigger box with no usable geometry: nothing to attribute
            # evidence to.
            self._latch("lost")

    def _manual_relaxed(self, autonomous: bool) -> bool:
        """in_manual True only: a hold ENGAGED in MANUAL, while in MANUAL,
        may release on the corridor without a resolved chain. The operator
        stops the robot by letting go of the stick (Kevin, 2026-09-27)."""
        return self._in_manual and not autonomous and self._engaged_in_manual

    # ------------------------------------------------------------------
    def update(
        self,
        *,
        now: float,
        triggered: bool,
        detections,
        fresh_detections: bool,
        corridor_mm: float,
        corridor_rejected: bool,
        corridor_valid_pct: float,
        stop_det=None,
        trigger_dets=None,
        operator_release: bool = False,
        autonomous: bool = True,
        forward_neutral: bool = False,
    ) -> bool:
        """Advance one depth poll. Returns True while the hold is applied.

        ``triggered``: the stateless override forced the stop on this poll;
        ``stop_det`` is the box it returned (the first one). ``trigger_dets``:
        every stop-tier box that alone forces the stateless stop (None: just
        ``stop_det``). ``detections``: the list the override read (all
        classes). ``fresh_detections``: that list came from a packet newer
        than the one the previous poll saw, and the stream is not stale.
        ``corridor_mm``: the corridor near distance before persistence and
        before the override (inf when rejected). ``autonomous``: FOLLOW_ME or
        WAYPOINT_NAV. ``forward_neutral``: MANUAL and no drive command for
        the neutral hold time (``NeutralTimer``; False when unknown).
        """
        stop_dets = [d for d in (detections or ())
                     if getattr(d, "safety_tier", "") == "stop"]
        if trigger_dets is None:
            trigger_dets = [stop_det] if stop_det is not None else []
        engage_mode = bool(autonomous) or self._in_manual
        entering = engage_mode and not self._prev_engage_mode
        self._prev_engage_mode = engage_mode
        trig_box = None
        if triggered:
            trig_box = self._parse(stop_det) if stop_det is not None else None
            self._last_trigger_ts = now
            self._last_trigger_box = trig_box

        if operator_release and self._active:
            self._release("operator")

        engaged_now = False
        if engage_mode:
            if triggered and not self._active:
                self._engage("trigger", trig_box, now, False, not autonomous)
                engaged_now = True
            elif (
                not self._active
                and entering
                and self._last_trigger_ts is not None
                and now - self._last_trigger_ts <= self._carry_in_s
            ):
                self._engage("carry_in", self._last_trigger_box,
                             self._last_trigger_ts, True, not autonomous)
            if self._active and autonomous:
                self._manual_carry = False

        if not self._active:
            return False
        if triggered:
            self._evidence = 0

        if not engage_mode:
            # A hold carried into MANUAL: applied until the stick is let go.
            self._manual_carry = True
            if forward_neutral:
                self._release("manual_neutral")
                return False

        if not fresh_detections:
            return True

        boxes = [self._parse(d) for d in stop_dets]
        c = self._chain
        cont: _Box | None = None
        if engaged_now:
            # The trigger box is the chain on its own engage poll.
            cont = next((b for b in boxes if b.det is stop_det), None)
        elif c is not None:
            gap = self._carry_in_s if c.seed else self._gap_s
            if now - c.last_seen <= gap:
                # Label flicker: one box in view, in the chain's place. Then
                # the group is not compared (see the module docstring).
                only_box = len(boxes) == 1
                cands = [
                    b for b in boxes
                    if b.cx is not None
                    and (
                        b.group == c.group
                        or (only_box and abs(b.cx - c.cx) <= self.FLICKER_GATE)
                    )
                    and abs(b.cx - c.cx) <= self._gate
                    and self._plausible(c, b, now)
                ]
                if len(cands) == 1:
                    cont = cands[0]
                elif len(cands) > 1:
                    self._latch("ambiguous")
            if cont is not None:
                self._continue(c, cont, now)
            else:
                c.beyond_streak = 0
                c.resolved = False
                if now - c.last_seen > gap:
                    self._latch("lost")
        self._chain_present = cont is not None

        others = [b for b in boxes if b is not cont]
        other_unresolved = any(not b.beyond for b in others)
        if other_unresolved and self._other_prev:
            self._latch("second_object")
        self._other_prev = other_unresolved
        if cont is None and c is not None and any(
            b.beyond and abs(b.cx - c.cx) <= self._gate for b in others
        ):
            self._latch("impostor_in_gate")
        if any(cont is None or d is not cont.det for d in trigger_dets):
            self._latch("second_trigger")

        # _num drops NaN and inf: the reader passes inf only for a rejected
        # corridor, and an unmeasured corridor is never clear.
        corridor_ok = (
            not corridor_rejected
            and _num(corridor_valid_pct) is not None
            and corridor_valid_pct >= self._min_valid_pct
            and _num(corridor_mm) is not None
            and corridor_mm >= self._release_mm
        )
        relaxed = self._manual_relaxed(autonomous)
        if self._operator_only and not relaxed:
            evidence, kind = False, ""
        elif boxes:
            evidence = (cont is not None and c is not None and c.resolved
                        and all(b.beyond for b in boxes))
            kind = "seen_beyond"
        else:
            # No stop-tier box in view. In the autonomous modes, and for a
            # hold carried into MANUAL, the corridor is NOT evidence: it read
            # 2.9-4.7 m with the person at 0.3-0.9 m (2026-11-08), and a
            # person who was seen beyond can come back in undetected. Only
            # the MANUAL relaxation (in_manual True) releases on it.
            evidence = corridor_ok and relaxed
            kind = "corridor_clear"
        self._evidence = self._evidence + 1 if evidence else 0
        if self._evidence >= self._release_polls:
            self._release(kind)
            return False
        return True
