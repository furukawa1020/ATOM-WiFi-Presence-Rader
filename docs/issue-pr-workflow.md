# Issue and PR execution plan

This repository follows issue-first development.

## Current status

| Issue ID | Title | Scope | Status |
|---|---|---|---|
| ISS-001 | Bootstrap PlatformIO project | Add PlatformIO config, board definition, and build-time target skeletons | ✅ Done |
| ISS-002 | Base firmware skeleton | Add unified `main.cpp` with role-based startup and runtime scaffolding | ✅ Done |
| ISS-003 | CSI sampling layer | Integrate ESP-IDF CSI callbacks and frame capture | In progress ([Issue #1](https://github.com/furukawa1020/ATOM-WiFi-Presence-Rader/issues/1)) |
| ISS-004 | ESP-NOW transport layer | Implement TX/RX peer registration and packet framing | ⏳ Planned |
| ISS-005 | Occupancy state pipeline | Implement EMPTY/MOVEMENT/PRESENT_STILL/UNCERTAIN/DEGRADED transitions | ⏳ Planned |
| ISS-006 | Calibration flow | Add empty and presence calibration persistence with NVS/Preferences | ⏳ Planned |
| ISS-007 | Health + diagnostics | Add health state reporting and fault handling | ⏳ Planned |
| ISS-008 | Field telemetry UI | Implement serial/USB visibility and debug output format | ⏳ Planned |

## PR rule

- One PR should map to one issue wherever possible.
- If an issue changes behavior across roles (TX and 3 RX), keep it in one PR only when common.
- Before merging a PR, include:
  - Issue ID in title and description
  - Scope note (what is included, what is intentionally out of scope)
  - Link to verification command list (build, lint, or manual checks used)
