# NY&E Layout Control System — Implementation Plan

**Version:** 1.2
**Date:** 2026-05-05
**Status:** Active — revised after Session 2.0 scoping (2026-05-02)

---

## Phase 1 — Infrastructure

### Session 1.1 — RPi5 Base Setup
- `setup_ap.sh`: hostapd + dnsmasq config (SSID `NYE_Layout`, `192.168.10.1`, DHCP pool)
- `setup_mosquitto.sh`: broker config — auth, persistence, ACL, bind to layout WiFi only
- systemd unit stubs for all services
- **Completion:** RPi5 creates WiFi AP; Mosquitto accepts authenticated connections

### Session 1.2 — Fast Clock Service
- `clock_service.py`: start/pause/set/reset/speed/set_tick_interval, sync_request response, state persisted to `clock_state.json`
- systemd `rr-clock` unit
- **Completion:** clock runs headlessly, publishes ticks on schedule, survives restart from saved state

### Session 1.2a — Timetable Loader
- Timetable JSON format defined: see `docs/TIMETABLE_FORMAT.md` (v0.1 draft, open questions remain)
- Seed `timetable.json` from Timetable No. 4 (December 31, 1904) — source: `NYELayoutDocs/alt/timetable4.pdf`
- Server loads active timetable on startup (read-only during operations)
- Read-only timetable API endpoint used by dispatcher UI and yardmaster page
- **Completion:** server loads timetable; API returns train schedules by location; next-train queries work

### Session 1.3 — Dispatcher UI: Clock + Status
- FastAPI skeleton + WebSocket MQTT bridge
- Clock display with pause/start/set controls
- Station status panel (7 tiles, online/offline) with next scheduled train per station
- systemd `rr-dispatcher` unit
- **Completion:** working page at `localhost:5000` showing live clock, station status, and next trains

### Session 1.4 — Station_OS: Clock + Network
- MQTT client with LWT and 60s heartbeat; sync_request on reconnect
- Clock screen: large railroad time + station name + next train status line
- CYD provisioning: design decided this session
- **Completion:** CYD units display railroad time and next train; dispatcher UI shows them online

### Session 1.5 — JMRI on RPi5
- Fresh JMRI install on RPi5; PR3 LocoNet USB config (DCS51 physical connection not required)
- JMRI MQTT bridge pointed at layout broker (`trains/turnout/...`)
- systemd `jmri` unit; verify WiThrottle server starts
- **Completion:** JMRI running at `192.168.10.1:8080`; WiThrottle accessible; Switch_Control unchanged

---

## Phase 2 — Operations

### Session 2.0 — WP Yardmaster Page
Scoping complete (2026-05-02). **Hardware needed before this session:** RPi3 + RPi 7" Official Touchscreen (DSI).

- FastAPI `/yard` route — optimized for RPi3 7" touchscreen in Chromium kiosk mode
- NLS arrival notifications from Dispatcher (via `trains/yard/notification`)
- NLS departure lineup (from timetable + dispatcher adjustments)
- C&O schedule display (read-only reference — interchange and main-clear planning)
- Consist report form: train #, engine #, caboose #, loaded cars, empty cars (editable before submit)
- Ready flag combined with consist submission — formal signal to Dispatcher
- MQTT: publishes `trains/yard/consist`; subscribes to `trains/yard/notification`
- **Completion:** Yardmaster can view lineup, receive Dispatcher notifications, submit consist + ready

### Session 2.1 — OS Submission
- Station_OS: OS screen (train number + section number keypad, direction toggle, submit)
- Dispatcher UI: scrolling OS log (station, train, section, direction, RR time)
- **Completion:** station agent submits OS with section number; Dispatcher sees it logged

### Session 2.2 — Train Orders
- Dispatcher UI: freeform TO text entry, multi-station selector, Issue button; TO signal arm auto-raises on issue
- Station_OS: Orders screen (TO text display, N/S signal arm status, ACK button)
- Full ACK flow back to Dispatcher UI
- **Completion:** Dispatcher issues TOs; stations receive, display, and ACK

### Session 2.3 — Clearance Forms
- Dispatcher UI: clearance issuance (train, direction, text, destination station)
- Station_OS: Clearance screen (activates on receipt, ACK button)
- **Completion:** clearances issue and ACK at any station

### Session 2.4 — TO Signal Firmware
- `TO_Signal/src/main.cpp`: 2 servos per ESP32, MQTT sub on `cmd` topics, pub on `state` topics; reuses Switch_Control reconnect pattern
- **Completion:** Dispatcher raises/lowers signal arms from web UI; arms respond and report state

---

## Management Tools (owner-facing, schedule TBD)

These tools are needed before the first operating session. Initial sessions may use hand-edited JSON files where noted. **A dedicated planning session is required before any management tool implementation begins** — see Next Planning Session below.

### TO Type Definitions _(prerequisite for dispatcher UI design)_
- Define all TO types used on the NY&E (meet, wait, running extra, work extra, speed restriction, etc.)
- For each type: required fields, formatted text template, addressed-station rules
- Output: TO type registry used by the dispatcher UI structured form and by the management tools

### Timetable Management Tool
- Create, edit, and version timetables (version number + release date)
- Input: `Stringline.ods` segment profile data (track distance from XTrkCAD, class speeds, stop delays) — **XTrkCAD dependency**
- Generate `timetable.json` including `segments` section (inter-station travel times per class)
- Generate printed timetable in Timetable No. 4 format
- Generate String Table (train scheduling diagram showing meets and crossing points)
- Generate per-station condensed schedule cards (previous station / current station / next station)
- Generate "X" column extra-train travel times for printed timetable
- _Initial sessions: seed JSON hand-edited from timetable.pdf; segments populated once XTrkCAD data is available_

### CC&W Manager (Car Cards & Waybills)
- Car database: car ID, type, road name, description
- Industry database: name, station, commodities accepted/shipped, track capacity
- Waybill database: routing assignments per car (owner sets between sessions)
- Printed outputs: car cards (permanent, print once per car), waybills (print each session cycle)

### Trainmaster Function
- Pre-session tool (owner role): reviews active waybills, matches to scheduled trains, identifies extras needed
- Input: CC&W waybill data, active timetable, yard track data
- Output: `session.json` — train manifests (car-by-car per train), pre-authorized extras and work extras, annulments, active waybill references
- Dispatcher (pre-session) reviews and approves extras before session start
- `session.json` is loaded onto RPi5 by owner at pre-session setup; server reads it at session start

### session.json Format _(design required)_
Generated by Trainmaster function. Contains:
- Session header: date, timetable version, session number
- Train manifests: per-train car list (road name, car type, car ID, destination)
- Pre-authorized extras: engine, type (running/work), direction, stations, planned departure
- Annulments: trains not running this session + reason
- Active waybill references

### yard.json Format _(design required)_
Yardmaster-only data. Separate from `timetable.json`. Contains:
- Yard track numbers and designated functions (caboose, interchange, departure, arrival, local, etc.)
- Track numbers not yet assigned — pending physical layout construction

### Post-Session Report
- Generated by owner page after session ends
- Content: OS log, TOs issued, consists, extras run, annulments, session duration (real + RR time)
- Saved as file for owner's records; future use: Trainmaster comparison of plan vs. actual

---

## CAD — Parallel Track

| Item | Qty | When |
|------|-----|------|
| CYD fascia enclosure | 7 | Deferred — after implementation and testing complete |
| TO Signal ESP32 enclosure | 5 | Alongside Session 2.4 |
| Yardmaster terminal mount | 1 | RPi3 + 7" screen enclosure; before Session 2.0 |
| RPi5 / PR3 server mount | 1 | Any time after Session 1.1 |

---

## Phases 3–6

| Phase | Scope | Trigger |
|-------|-------|---------|
| 3 — Cameras | ESP32-CAM firmware + Dispatcher Display 2 grid | Phase 2 fully in use |
| 4 — RFID | Station approach readers + auto-OS suggestion | After cameras |
| 5 — Lighting | Day/night cycle via fast clock | Scenery far enough along |
| 6 — Dispatcher Assist | Newbie mode, conflict detection | After multiple operating sessions |

---

## Hardware to Order

| Item | Purpose | Needed by |
|------|---------|-----------|
| RPi 7" Official Touchscreen (DSI) | Yardmaster terminal display | Session 2.0 |

---

---

## Next Planning Session — Management Tools + System Diagram

**Goal:** Complete planning for management tools before any implementation begins.

### Agenda

1. **TO type definitions** — define all NY&E TO types and field schemas (prerequisite for dispatcher UI)
2. **Timetable Management Tool design** — inputs, outputs, UI approach (desktop tool or web?)
3. **CC&W Manager design** — car/industry/waybill schema, printed output formats
4. **Trainmaster function design** — session planning workflow, session.json schema
5. **yard.json schema** — track numbers, functions, capacity
6. **Post-session report design** — content and format
7. **Visual system diagram** — research current practice for software architecture diagrams (Mermaid, C4 model, PlantUML, draw.io); select format and produce a diagram of the full system (components, connections, data flows). Candidate formats:
   - **Mermaid** — text-based, renders in GitHub, good for component/sequence diagrams
   - **C4 model** — structured hierarchy (Context → Container → Component); good for communicating architecture at different levels
   - **PlantUML** — text-based, richer diagram types, requires render server

### Deferred from this session (also agenda items)
- **RR_Server design doc** — write the full design document now that decisions are confirmed
- **Dispatcher web app detailed design** — UI state, interaction flows, API spec (requires TO types first)
- **Station_OS firmware design** — state machine, screen transitions, provisioning workflow

---

## Revision History

| Version | Date | Change |
|---------|------|--------|
| 1.0 | 2026-05-02 | Initial plan established |
| 1.1 | 2026-05-02 | Session 1.2a (timetable loader) added; Session 2.0 scoping complete; yardmaster terminal hardware noted; Management Tools added; CC&W defined |
| 1.2 | 2026-05-05 | Management Tools expanded: TO type definitions (prerequisite), Trainmaster function, session.json, yard.json, post-session report. Next Planning Session agenda added. Visual system diagram identified as a planning task. |
