# Issue and PR execution plan

This repository follows issue-first development.

## Current status

| Issue ID | Title | Scope | Status |
|---|---|---|---|
| ISS-001 | Bootstrap PlatformIO project | Add PlatformIO config, board definition, and build-time target skeletons | ✅ Done |
| ISS-002 | Base firmware skeleton | Add unified `main.cpp` with role-based startup and runtime scaffolding | ✅ Done |
| ISS-003 | CSI sampling layer | Integrate ESP-IDF CSI callbacks and frame capture | In progress ([Issue #1](https://github.com/furukawa1020/ATOM-WiFi-Presence-Rader/issues/1)) |
| ISS-004 | ESP-NOW transport layer | Implement TX/RX peer registration and packet framing | In progress ([Issue #3](https://github.com/furukawa1020/ATOM-WiFi-Presence-Rader/issues/3)) |
| ISS-005 | CSI parsing and preprocessing | Parse HT-LTF I/Q and apply robust normalization | In progress ([Issue #5](https://github.com/furukawa1020/ATOM-WiFi-Presence-Rader/issues/5)) |
| ISS-006 | Subcarrier selection and features | Select per-link carriers and extract time-window features | Planned |
| ISS-007 | Occupancy state pipeline | Implement EMPTY/MOVEMENT/PRESENT_STILL/UNCERTAIN/DEGRADED transitions | Planned |
| ISS-008 | Calibration flow | Add session-separated training and validation | Planned |
| ISS-009 | Health + diagnostics | Add health state reporting and fault handling | Planned |
| ISS-010 | Multi-link fusion | Fuse Receiver summaries with quality weighting | Planned |
| ISS-011 | Persistent storage | Persist pairing, calibration, model, and CRC metadata | Planned |
| ISS-012 | Field telemetry UI | Implement LCD and USB diagnostic views | Planned |

## PR rule

- One PR should map to one issue wherever possible.
- Issue and PR titles and descriptions are written in Japanese.
- Delivery is split across multiple small PRs; stacked PRs are allowed when dependencies are explicit.
- If an issue changes behavior across roles (TX and 3 RX), keep it in one PR only when common.
- Before merging a PR, include:
  - Issue ID in title and description
  - Scope note (what is included, what is intentionally out of scope)
  - Link to verification command list (build, lint, or manual checks used)
