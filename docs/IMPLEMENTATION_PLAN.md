# NY&E Layout Control System — Implementation Plan

**Version:** 1.1
**Date:** 2026-05-02
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

These tools are needed before the first operating session. Initial sessions may use hand-edited JSON files where noted.

### Timetable Management Tool
- Create, edit, and version timetables (version number + release date)
- Generate printed timetable in Timetable No. 4 format
- Generate String Table (train scheduling diagram showing meets and crossing points)
- Generate per-station condensed schedule cards (previous station / current station / next station)
- Export `timetable.json` for RR_Server
- Source methodology: `NYELayoutDocs/alt/Stringline.ods` — track segment profile, class speeds, stop delays
- _Initial sessions: seed JSON hand-edited from timetable.pdf_

### CC&W Manager (Car Cards & Waybills)
- Car database: car ID, type, road name, description
- Industry database: name, station, commodities accepted/shipped, track capacity
- Waybill database: routing assignments per car (owner sets between sessions)
- Printed outputs: car cards (permanent, print once per car), waybills (print each session cycle)

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

## Revision History

| Version | Date | Change |
|---------|------|--------|
| 1.0 | 2026-05-02 | Initial plan established |
| 1.1 | 2026-05-02 | Session 1.2a (timetable loader) added; Session 2.0 scoping complete — expanded to implementation; yardmaster terminal hardware noted; Management Tools section added; CC&W system defined; C&O timetable in scope; OS report adds section number; section 2.1 OS updated |
