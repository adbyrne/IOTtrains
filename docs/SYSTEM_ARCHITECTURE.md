# NY&E Northern Lights Subdivision — Layout Control System Architecture

**Version:** 0.6 (draft for review)
**Date:** 2026-04-30
**Era:** circa 1905 — timetable and train order operations

---

## 1. Design Principles

- **Self-contained layout network** — RPi5 hosts the WiFi AP. No home LAN dependency during operation.
- **MQTT as the message bus** — all inter-component communication; broker lives on RPi5.
- **DCC for traction only** — Digitrax DCS51 provides track power and locomotive control exclusively. All stationary device control (turnouts, signals, lighting, staging) uses MQTT-based ESP32 nodes. JMRI bridges the two systems where needed. Block occupancy detection via DCC hardware is not planned.
- **Prototypically authentic** — OS reports, train orders, clearance forms, TO signals, as per 1905 practice.
- **Phased delivery** — infrastructure first, then operations, then automation and enhancements.
- **Inventory-first hardware** — prefer ESP32 boards and RPis already in stock.

---

## 2. Layout Context

A brief physical and operational description to ground architecture decisions. Full detail in `Trains/docs/NYE_OPERATIONS.md`.

**Railroad:** NY&E Railroad, Northern Lights Subdivision — freelanced branch on a C&O prototype basis, circa 1905.

**Physical layout — two levels:**

| Level | Content |
|-------|---------|
| Upper | C&O mainline — short modeled section between East staging and West staging |
| Lower | NY&E Northern Lights Subdivision — WP (Williamsport) to HC (Hemlock Crest) |

**Track topology:** Point-to-point, single main track. All seven stations have passing sidings. Southward trains are superior by direction.

**Williamsport (WP) — Junction and yard:**
- Junction between C&O upper level and NY&E lower level; shared station building
- Physical yard with local industries on the NY&E side
- C&O operations use East/West staging; trains enter and exit without through running

**IoT scope:** The control system manages the NY&E branch (7 stations) and the WP yard. C&O staging tracks may receive independent treatment in a later phase.

---

## 3. System Components

### 3.1 Central Server — RPi5

Single RPi5 hosts all server-side software and acts as the WiFi AP.

| Service | Software | Port |
|---------|----------|------|
| WiFi Access Point | hostapd + dnsmasq | — |
| MQTT Broker | Mosquitto | 1883 |
| Fast Clock | Python service | — (MQTT master/control only) |
| Dispatcher Web App | Python / FastAPI | 5000 |
| JMRI | JMRI (Java) | 8080 (web), 12090 (WiThrottle) |

**Displays:** RPi5 has two HDMI outputs.
- Display 1 (primary): Dispatcher dashboard — fast clock, OS log, train order issuance, TO signal controls.
- Display 2 (future Phase 3): Station camera grid.

**Fast Clock architecture:** The RPi5 service is the master — it maintains railroad time state and responds to control commands (set, start, pause, stop, sync). It publishes `trains/clock/time` ticks and rebroadcasts `trains/clock/control` commands. Each station CYD uses its ESP32's free-running timer to interpolate locally between ticks: `display_time = rr_sync + ratio × (now − real_sync)`. This gives smooth second-by-second display and resilience to brief WiFi outages. CYDs subscribe to both `trains/clock/time` (sync) and `trains/clock/control` (pause/start/set, applied immediately).

### 3.2 Station Units — ESP32 CYD (ESP32-2432S028R) ×7

One per station, fascia-mounted.

**Station unit provisioning (NVS):**
Each unit is provisioned once via USB serial before installation. Parameters: `station_id`, `wifi_ssid`, `wifi_password`, `mqtt_host`, `mqtt_port`, `mqtt_user`, `mqtt_pass`. Provisioning workflow and tooling TBD — dedicated session needed.

**The dispatcher is a remote user** (separate room) operating the RPi5 web app. The WP station CYD is a standard fascia unit at Williamsport, not a dispatcher interface.

| ID | Name | Type | TO Signal | Clearance Form |
|----|------|------|-----------|----------------|
| WP | Williamsport | South terminus | No | Yes |
| XP | Xina Pass | Register station | Yes | Yes |
| BB | Becs Bend | Standard | Yes | * |
| JC | Jacks Creek | Standard | Yes | * |
| MC | Michelles Cove | Standard | Yes | * |
| SK | Stans Knob | Standard | Yes | * |
| HC | Hemlock Crest | Register / North terminus | No | Yes |

_* Clearance issued when a train originates at this station (e.g. southbound return from an intermediate point)._

**Station unit screens (LVGL):**
1. **Clock** — large fast clock display, station name (default/always visible)
2. **OS** — train number + direction keypad, submit button (all stations)
3. **Orders** — incoming TO text display, N/S TO signal arm status, ACK button
4. **Clearance** — clearance form text display, ACK button (all stations; visible when a clearance is active)
5. **Status** — WiFi/MQTT connection info, firmware version

### 3.3 TO Signal Controllers — ESP32 ×5

One per TO-signal station (XP, BB, JC, MC, SK). Each station's signal has **two arms** — one for northbound trains, one for southbound — controlled independently. The dispatcher raises the N or S arm based on which direction a train order applies to.

Each controller drives two servos (one per arm) from a single ESP32. MQTT topics are direction-specific: `trains/signal/{id}/to/N/cmd` and `trains/signal/{id}/to/S/cmd`.

The existing TrainOrderServo bracket (`CADlayout/Servo/`) is already designed for two arms and two servos. No new CAD design needed for the signal mechanism.

Reuses Switch_Control connection/reconnect pattern. Simple dedicated firmware (`TO_Signal` project).

### 3.4 Turnout Controllers — ESP32 (Switch_Control, existing)

**Optional — most turnouts are hand-thrown** via the SwitchToggle fascia lever and Blue Point / Gold-N-Rod mechanical linkage (see `CADlayout/SwitchToggle/`). Switch_Control firmware is deployed only for specific motorized turnouts where remote or automated control is needed. Which turnouts are motorized is TBD pending physical layout construction.

JMRI-compatible `CLOSED`/`THROWN` topics on `trains/turnout/{id}/state`.

### 3.5 Station Cameras — AI-Thinker ESP32-CAM ×7

**Inventory:** 10 bare modules + 8 programmer boards in stock — sufficient for all 7 stations.
One per station. Streams MJPEG over HTTP. Publishes its stream URL to the broker on startup. Viewed on Dispatcher Display 2 in Phase 3.

---

## 4. Network Topology

```
RPi5 — 192.168.10.1
  WiFi AP: SSID "NYE_Layout" (WPA2, isolated from home LAN)
  DHCP: 192.168.10.10 – 192.168.10.99
  │
  ├── Mosquitto broker :1883
  ├── JMRI :8080 (web) / :12090 (WiThrottle)
  ├── Dispatcher web app :5000
  │
  └── WiFi clients (DHCP):
      ├── Station units (CYD) ×7         → MQTT pub/sub
      ├── TO Signal controllers ×5        → MQTT sub/pub
      ├── Turnout controllers ×N          → MQTT pub/sub
      └── Station cameras (ESP32-CAM) ×7 → HTTP MJPEG + MQTT pub

Engineer phones → WiFi → WiThrottle → JMRI :12090 → DCC
```

The RPi5 may optionally have an Ethernet connection to the home LAN for SSH/maintenance, independent of the layout WiFi. Layout operation requires only the layout WiFi.

---

## 5. Fast Clock

- **Ratio:** 3:1 (one real minute = three railroad minutes); configurable.
- **Master:** Python service on RPi5 (single source of truth; no drift between displays).
- **Tick:** publishes `trains/clock/time` at a configurable interval; default 60 real seconds (= 3 railroad minutes). Tick interval is set via `trains/clock/control`. CYD units interpolate locally between ticks — tick frequency affects resync accuracy only, not display smoothness.
- **Time display:** 12-hour AM/PM format in all human-facing interfaces (TOs, clearance forms, dispatcher web app, station clocks) — consistent with 1905 railroad practice.
- **Day of week:** the clock carries a day-of-week value (1=Monday … 7=Sunday). It advances automatically at railroad midnight. Month and year are not tracked — the owner selects the day at session start, which may represent any date (including holiday or seasonal sessions). Day changes require owner permission.
- **Session continuity:** state (day, time, ratio) is persisted to disk. The next operating session resumes from exactly where the last one ended. A session may span multiple real-world evenings; the clock picks up from the saved state. The dispatcher controls start / pause / stop; day or time override requires owner permission.
- **Session start:** all station units must be online before the clock is started. Dispatcher sets railroad day and time at session start (e.g., Monday 8:00 AM) from the saved state and starts the clock; all CYDs sync from the first tick.
- **Mid-session restart:** a station that reboots mid-session publishes to `trains/clock/sync_request`; the clock service responds with an immediate tick so the restarting unit syncs without waiting for the next scheduled tick.
- **Controls:** dispatcher web app sends start / pause / set-time / reset / speed / set_tick_interval commands via `trains/clock/control`.
- **Timetable:** day of week determines which scheduled trains are active. Trains with day restrictions (e.g. Monday–Friday only) are a Phase 6 dispatcher-assist concern, but the day value is present from Phase 1.

---

## 6. Dispatcher Web Application

Full-screen browser on Display 1 (RPi5). Python FastAPI backend; browser communicates via WebSocket for live MQTT updates.

### Layout

```
┌─────────────────────────────────────────────────────────────────┐
│  10:42 AM  ●  [PAUSE]  [SET TIME]          NYE Layout Control   │
├──────┬──────┬──────┬──────┬──────┬──────┬──────────────────────┤
│  WP  │   XP   │   BB   │   JC   │   MC   │   SK   │  HC  │
│ ◻ CL │ N◉ S◉ │ N◉ S◉ │ N◉ S◉ │ N◉ S◉ │ N◉ S◉ │ ◻ CL │
│      │ ◻ CL  │        │        │        │        │      │
├──────┴──────┴──────┴──────┴──────┴──────┴──────────────────────┤
│  OS LOG                           │  ISSUE TRAIN ORDER          │
│  10:38  Extra 42N OS at XP        │  To: [XP▼] [BB▼] [ ]       │
│  10:41  No. 3N OS at BB           │  ┌──────────────────────┐   │
│  10:44  No. 3N OS at JC           │  │                      │   │
│                                   │  │  (freeform order)    │   │
│                                   │  │                      │   │
│                                   │  └──────────────────────┘   │
│                                   │  [ ISSUE TO ] [ CLEARANCE ] │
└───────────────────────────────────┴────────────────────────────-┘
```

- Station tiles show: N and S TO signal arm status (raised ◉ / lowered ◻), clearance pending indicator, online/offline.
- Clicking a station tile opens a detail panel (last OS, active orders, signal direct control).
- Train orders: freeform text, one or multiple station recipients.
- Clearance forms: issued to any station; WP/XP/HC are standard clearance points, any station for originating trains.

---

## 7. Train Orders and Clearances

### Train Orders
- Freeform text, authentic 1905 style.
- Dispatcher types the order, selects one or more destination stations, issues.
- Each destination station receives the order text; station agent ACKs when copied.
- TO signal arm raises automatically when order is issued (dispatcher can override manually).
- Signal lowers when order is ACKed and dispatcher releases it (or manually).

### Clearance Forms
- **Standard clearance points:** WP (south terminus), XP (register station), HC (north terminus/register).
- **Any station:** a clearance is also issued when a train originates at an intermediate station (e.g. a southbound return train departing from SK or BB).
- Required before a train departs a terminus, proceeds beyond a register point, or departs any originating station.
- Displayed on the station unit clearance screen (Clearance screen active on all units); agent ACKs when issued to the crew.

---

## 8. DCC and JMRI

- **DCC system:** Digitrax DCS51 (primary command station/booster).
- **JMRI:** Migrates to RPi5. Connects to DCS51 via LocoNet USB interface — **PR3**.
- **SPROG RPis:** Remain as alternate/portable DCC system; not part of layout network.
- **Engine Driver (phone throttles):** Connect to JMRI WiThrottle server on RPi5 (`192.168.10.1:12090`). Independent of MQTT.
- **JMRI MQTT bridge:** JMRI publishes/subscribes to `trains/turnout/...` topics on the layout broker. Switch_Control firmware unchanged.
- **JMRI web UI:** Available at `192.168.10.1:8080`; can be opened on Dispatcher Display 1 in a second browser tab if needed.

---

## 9. CAD Requirements for Physical Installation

IoT components are bare PCBs and need enclosures or mounts for layout installation. CAD tasks should be planned alongside each firmware phase, not deferred.

| Item | Qty | Notes |
|------|-----|-------|
| CYD fascia enclosure | 7 | Bezel + box for ESP32-2432S028R; panel-mounts on fascia at each station |
| TO Signal ESP32 enclosure | 5 | Housing for bare ESP32 near TrainOrderServo mechanism (XP, BB, JC, MC, SK) |
| ESP32-CAM mount | 7 | Camera bracket aimed at station scene; angle varies per location (Phase 3) |
| RPi5 / PR3 server mount | 1 | Rack shelf or enclosure for central server hardware |

Existing CADlayout parts (cable clips, generic boxes) should be evaluated for reuse before designing new enclosures.

---

## 10. RFID Train Detection (Future — Phase 4)

RFID tags on locomotives and cabooses only (not all rolling stock). Readers at station approach tracks report the engine or caboose tag to the broker, providing the dispatcher app with a pre-populated OS suggestion. The station agent confirms.

This is an OS assistance tool, not a full block occupancy system. No DCC-based detection is planned.

Topic: `trains/block/{block_id}/state` — reserved now so the namespace is established.

---

## 11. Build Phases

| Phase | Scope |
|-------|-------|
| **1 — Infrastructure** | RPi5: AP, broker, fast clock. Station_OS: network migration, clock display. JMRI migration. |
| **2 — Operations** | OS submission (station → dispatcher). TO issuance + display + ACK. Clearance forms. TO_Signal firmware + hardware. |
| **3 — Camera Views** | ESP32-CAM firmware. Dispatcher Display 2 camera grid. |
| **4 — RFID Detection** | RFID readers at stations. Locomotive/caboose tags. Auto-populated OS suggestion. |
| **5 — Lighting** | Station lighting via fast clock day/night cycle. |
| **6 — Dispatcher Assist** | "Newbie mode": structured TO forms, timetable conflict hints. |

---

## 12. Open Questions / TBDs

### Open

| # | Item | Question | Working Assumption |
|---|------|----------|--------------------|
| 7 | WP Yardmaster unit | WP yardmaster needs a separate physical unit (CYD or RPi) for yard tasks and dispatcher communication. What info is displayed? What tasks does it handle? How does it integrate with the dispatcher web app? | Separate unit from WP fascia CYD — scope and design need a dedicated session |

### Resolved

| # | Item | Decision |
|---|------|----------|
| 1 | RPi5 Ethernet | Yes — Ethernet to home LAN for maintenance; independent of layout WiFi |
| 2 | SSID / password | SSID = `NYE_Layout`; password is config-time only, not in documentation |
| 3 | Session day | Day of week (1=Mon … 7=Sun); auto-advances at railroad midnight; resumes from saved state each real session; dispatcher sets at session start; day/time override requires owner permission; timetable uses day for train scheduling |
| 4 | TO signal auto-lower | Dispatcher releases manually — prototypically authentic |
| 5 | C&O staging IoT | Deferred — assess when layout reaches that stage |
| 6 | Dispatcher secondary display | Web app only |
