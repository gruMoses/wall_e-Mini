# IMU Instrumentation Snapshot

Date: YYYY-MM-DD
Branch/commit: `<hash>`
Operator: `<name>`
Run profile: `<nominal|stress|soak>`
Duration: `<seconds>`

## Baseline Context

- Baseline source report: `<path>`
- Comparison window: `<before/after window>`
- Notes: `<short context>`

## IMU-0 Instrumentation Metrics

| Metric | Value | Notes |
| --- | --- | --- |
| IMU packets received | `<n>` |  |
| IMU packets consumed | `<n>` |  |
| IMU packets coalesced/dropped | `<n>` |  |
| IMU queue drains | `<n>` |  |
| IMU sample age p50/p95/max (ms) | `<...>` |  |
| Effective IMU cadence avg/p95 (Hz) | `<...>` |  |
| First error observed | `<none|message>` |  |
| Throttled periodic summary count | `<n>` |  |

## Guardrail Snapshot

| Guardrail | Result | Delta vs baseline |
| --- | --- | --- |
| `loop_dt_ms` p95 | `<pass/fail>` | `<+/-...>` |
| `loop_dt_ms` p99 | `<pass/fail>` | `<+/-...>` |
| `% >=30ms` | `<pass/fail>` | `<+/-...>` |
| Max `loop_dt_ms` | `<pass/fail>` | `<+/-...>` |
| App CPU mean | `<pass/fail>` | `<+/-...>` |
| Thermal throttling | `<pass/fail>` | `<yes/no>` |

## Conclusion

- Instrumentation overhead assessment: `<negligible|non-negligible>`
- IMU-0 status: `<ready to advance|hold>`
- Follow-ups: `<short list>`
