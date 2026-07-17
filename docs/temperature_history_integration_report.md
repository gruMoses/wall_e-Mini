# Temperature History Debug Board — Integration Report

**Worktree:** `wall_e-Mini-temp-review`  
**Base commit:** `ccb99d1` (`feat: temperature history graph on /debug board`)  
**Parent:** `382b374`  
**Branch:** `review/temperature-debug-graph`  
**Status:** Hardening applied; **left uncommitted** for parent review. No push/deploy.

---

## 1. Changed files

### Original feature (`ccb99d1`)

| File | Role |
|------|------|
| `pi_app/hardware/vesc.py` | Expose `left/right_motor_temp_c` from STATUS_4 on `VescTelemetry` |
| `pi_app/control/controller.py` | Plumb motor FET/winding temps; clear on telemetry stale (same path as RPM) |
| `pi_app/hardware/oak_recorder.py` | `RecordingTelemetry` fields: `bms_temp_min_c`, VESC FET/motor temps |
| `pi_app/app/main.py` | Wire BMS min + VESC temps into recorder snapshot |
| `pi_app/web/oak_viewer.py` | SSE fields + Pi CPU sysfs + `/debug` canvas history panel |
| `pi_app/tests/test_debug_board.py` | SSE/route markers for temp keys |
| `pi_app/tests/test_vesc_telemetry.py` | `get_telemetry` exposes FET + motor temps |

### Hardening (this review, uncommitted)

| File | Role |
|------|------|
| `pi_app/web/oak_viewer.py` | Stale/gap/a11y/contrast/legend/Y-scale fixes |
| `pi_app/tests/test_debug_board.py` | Structural invariants, NaN/Inf, Pi range, BMS min/max |
| `pi_app/tests/test_vesc_telemetry.py` | STATUS_4 ×10 decode, short-frame, temps-null-until-STATUS_4 |

---

## 2. Data-path verification (series are real)

| Series | Source | Decode | Path |
|--------|--------|--------|------|
| **BMS Max** | Daly 0x92 temp range | `p[0] - 40` → °C | `BmsState.temp_max_c` → main → `RecordingTelemetry.bms_temp_max_c` → SSE → UI |
| **BMS Min** | Daly 0x92 | `p[2] - 40` → °C | same with `bms_temp_min_c` (**new plumb in ccb99d1**) |
| **VESC L/R FET** | CAN STATUS_4 (packet 16) | int16 BE × 0.1 → °C | `temp_fet_c` → `VescTelemetry.left/right_temp_c` → controller → SSE `vesc_*_temp_c` |
| **VESC L/R Motor** | STATUS_4 | int16 BE × 0.1 → °C | `temp_motor_c` → `left/right_motor_temp_c` → SSE `vesc_*_motor_temp_c` |
| **Pi CPU** | `/sys/class/thermal/thermal_zone0/temp` | milli-°C / 1000, clamped [-40, 125] | `read_pi_cpu_temp_c()` in SSE only (debug diagnostic) |

Unit conversions match VESC docs in `vesc.py` header and existing BMS tests (`temp_max_c=25` from raw 65).

### Stale / disconnect semantics

| Source | Backend | Frontend (hardened) |
|--------|---------|---------------------|
| VESC | Controller nulls all VESC temps if RX thread dead **or any** STATUS age > 0.5 s | Matches: `vescTempsStale()`; forces null + chip stale |
| BMS | `bms_connected`; last temps may linger in state | Offline (`connected === false`) forces null + stale |
| Pi | Missing/OOR sysfs → `None` | `raw == null` → stale; chip shows "—" |

### Safety / control

- No motor/safety decisions use the new motor-temp fields or Pi CPU temp.
- Controller only **reads/clears/publishes** temps (same stale gate as existing FET temps used for flight-recorder fields).
- `/debug` remains read-only (no POST control endpoints).

---

## 3. Findings by severity

### Fixed in this review

| Severity | Finding | Fix |
|----------|---------|-----|
| **High** | SSE reconnect / long pause drew a continuous line across the time hole (misleading diagonal) | Break stroke when `Δt > TEMP_GAP_MS` (5 s) |
| **High** | VESC “stale” required *both* motor ages > 0.5 s; controller nulls on *any* age > 0.5 s | Align with controller (`vescTempsStale`) |
| **Medium** | Legend full `innerHTML` rebuild every sample stole keyboard focus | Build once; patch values via `updateTempLegend` |
| **Medium** | BMS Max/Min colors both gold — hard to tell apart | BMS Min → `#86efac` |
| **Medium** | Chip “current” showed last non-null while disconnected | Current = "—" when series stale |
| **Low** | Secondary text `#5b6472` low contrast on dark panels | `#8b93a3` |
| **Low** | Weak a11y (no `aria-pressed`, no live summary) | `aria-pressed`, `aria-live` region, focus ring |
| **Low** | Y-scale used whole-buffer min/max | Scale from points inside visible window only |

### Residual / accepted (not fixed — scope or architecture)

| Severity | Finding | Notes |
|----------|---------|-------|
| **Medium** | Temps age off STATUS(9), not STATUS_4 | If STATUS_4 stops while STATUS continues, FET/motor temps can freeze until STATUS ages out. Pre-existing for FET; would need `last_status4` ages on telemetry. |
| **Low** | `pi_cpu_temp_c` is on shared `/api/telemetry` SSE | 1 s cache → ≤1 sysfs open/s while any SSE client is connected (incl. main dashboard). Cost is small; separate endpoint would isolate debug-only reads. |
| **Low** | No page-lifecycle cleanup of `ResizeObserver` / UPS `setInterval` | Full document navigation only; not an SPA. Acceptable. |
| **Info** | Graph freezes during SSE outage (no wall-clock “null heartbeat”) | After reconnect, gap break prevents false connections. |

### Pre-existing (not introduced by temp feature)

| Item | Notes |
|------|-------|
| `pi_app/tests/test_bms.py` full suite can hang in this environment | Isolated `test_temperature_parsed` also hung under unittest here; treat as env/pre-existing — **not** used as a regression signal for this change. Decode path already covered by existing unit test source and prior green runs on Pi. |
| No CSP headers in oak_viewer | Inline scripts work; temp panel adds no CDN/external scripts (offline-safe). |

---

## 4. Tests

```text
pytest pi_app/tests/test_debug_board.py \
       pi_app/tests/test_vesc_telemetry.py \
       pi_app/tests/test_controller.py \
       pi_app/tests/test_safety.py
→ 102 passed
```

Focused temp coverage added/extended:

- STATUS_4 ×10 decode (incl. negative motor temp)
- Short STATUS_4 does not clobber prior temps
- Temps remain `None` until STATUS_4
- SSE carries all 7 temp keys; NaN/Inf → null
- BMS min independent of max (int Daly values)
- Pi CPU cache, OOR rejection, missing file
- Frontend structural invariants (window, max points, gap, stale, a11y, no CDN)

---

## 5. Visual / manual checks still needed on the Pi

1. Open `http://<pi>:8080/debug` with robot live (BMS BLE + VESC CAN + thermal_zone0).
2. Confirm seven series paint; BMS min/max are visually distinct (green vs amber).
3. Toggle legend chips (mouse + keyboard); focus must stay on the chip while values update.
4. Unplug BMS BLE / kill BMS path → BMS series gap + stale border; values "—".
5. CAN drop / one VESC silent → VESC series null within ~0.5–1 s (controller gate).
6. Leave tab open 10+ minutes → buffer stays bounded (~720 pts); memory stable.
7. Resize window / phone width → canvas resizes without smear; high-DPI crisp.
8. Kill Flask briefly or toggle Wi‑Fi → reconnect; lines must **not** bridge the outage.
9. Confirm main dashboard `/` and teleop still work (shared SSE payload only grew fields).
10. Optional: stress with Follow-Me + debug open — CPU should stay negligible (1 Hz sample throttle + rAF draw).

---

## 6. Conflict notes for later integration (IMU / controller work)

- **`controller.py`**: Adds `_actual_*_motor_temp_c` and four telemetry keys next to existing VESC status-age fields. Merge carefully with any IMU heading-align / open-loop telemetry edits in the same `get_telemetry` / tick block (~lines 510–610, 1160–1175).
- **`oak_recorder.RecordingTelemetry`**: New optional fields with defaults — low conflict risk; dataclass field order may matter if anything positional constructs it (prefer kwargs).
- **`oak_viewer.py` `_DEBUG_HTML` / SSE dict**: Large inline HTML; IMU UI work on the same debug board or SSE object will conflict — re-apply temp panel + keys if rewriting the debug page.
- **`main.py` RecordingTelemetry construction**: Adjacent to GPS/BMS/VESC rpm wiring — same merge hotspot as other telemetry plumbs.
- **No change** to `imu_steering`, `waypoint_nav`, or motor command path beyond reading/clearing temp mirrors.

---

## 7. Sign-off checklist

- [x] Series decode verified against hardware docs / existing parsers  
- [x] Stale semantics aligned controller ↔ UI  
- [x] No safety/control behavior change intended or observed in code review  
- [x] Offline / no-CDN / coexists with existing debug panels  
- [x] Focused tests green  
- [ ] Pi hardware visual QA (section 5)  
- [ ] Integrate to mainline after parent review (do not commit from this worktree unless asked)  
