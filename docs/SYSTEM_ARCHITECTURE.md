# NY&E Northern Lights Subdivision — Layout Control System Architecture

**Version:** 0.2 (draft for review)
**Date:** 2026-04-28
**Era:** circa 1905 — timetable and train order operations

---

## 1. Design Principles

- **Self-contained layout network** — RPi5 hosts the WiFi AP. No home LAN dependency during operation.
- **MQTT as the message bus** — all inter-component communication; broker lives on RPi5.
- **Prototypically authentic** — OS reports, train orders, clearance forms, TO signals, as per 1905 practice.
- **Phased delivery** — infrastructure first, then operations, then automation and enhancements.
- **Inventory-first hardware** — prefer ESP32 boards and RPis already in stock.

---

## 2. System Components

### 2.1 Central Server — RPi5

Single RPi5 hosts all server-side software and acts as the WiFi AP.

| Service | Software | Port |
|---------|----------|------|
| WiFi Access Point | hostapd + dnsmasq | — |
| MQTT Broker | Mosquitto | 1883 |
| Fast Clock | Python service | — (MQTT only) |
| Dispatcher Web App | Python / FastAPI | 5000 |
| JMRI | JMRI (Java) | 8080 (web), 12090 (WiThrottle) |

**Displays:** RPi5 has two HDMI outputs.
- Display 1 (primary): Dispatcher dashboard — fast clock, OS log, train order issuance, TO signal controls.
- Display 2 (future Phase 3): Station camera grid.

### 2.2 Station Units — ESP32 CYD (ESP32-2432S028R) ×7

One per station, fascia-mounted. Configured via serial provisioning (NVS storage).

| ID | Name | Type | TO Signal | Clearance Form |
|----|------|------|-----------|----------------|
| WP | Williamsport | Terminus / Dispatcher unit | No | Yes |
| XP | Xina Pass | Register station | Yes | Yes |
| BB | Becs Bend | Standard | Yes | No |
| JC | Jacks Creek | Standard | Yes | No |
| MC | Michelles Cove | Standard | Yes | No |
| SK | Stans Knob | Standard | Yes | No |
| HC | Hemlock Crest | Register / Terminus | No | Yes |

**Station unit screens (LVGL):**
1. **Clock** — large fast clock display, station name (default/always visible)
2. **OS** — train number + direction keypad, submit button (all stations including termini)
3. **Orders** — incoming TO text display, TO signal status indicator, ACK button
4. **Clearance** — clearance form text display, ACK button (WP, XP, and HC)
5. **Status** — WiFi/MQTT connection info, firmware version

WP operates in dispatcher-assist mode: the Clock screen also shows a live OS log strip (recent OS
reports from all stations). The OS screen is available for trains departing and arriving at Williamsport.
The Clearance screen displays departure clearances issued from WP.

### 2.3 TO Signal Controllers — ESP32 ×5

One per TO-signal station (XP, BB, JC, MC, SK). Controls the TrainOrderServo seesaw arm (existing CAD design). Receives raise/lower command from dispatcher via MQTT; reports state back.

Reuses Switch_Control connection/reconnect pattern. Simple dedicated firmware (`TO_Signal` project).

### 2.4 Turnout Controllers — ESP32 (Switch_Control, existing)

One per motorized turnout. JMRI-compatible `CLOSED`/`THROWN` topics on `trains/turnout/{id}/state`. No changes needed.

### 2.5 Station Cameras — AI-Thinker ESP32-CAM ×7

**Inventory:** 10 bare modules + 8 programmer boards in stock — sufficient for all 7 stations.
One per station. Streams MJPEG over HTTP. Publishes its stream URL to the broker on startup. Viewed on Dispatcher Display 2 in Phase 3.

---

## 3. Network Topology

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

## 4. Fast Clock

- **Ratio:** 3:1 (one real minute = three railroad minutes); configurable.
- **Master:** Python service on RPi5 (single source of truth; no drift between displays).
- **Tick:** publishes `trains/clock/time` every 10 real seconds (= 30 railroad seconds).
- **Controls:** dispatcher web app sends start / pause / set-time / reset commands via `trains/clock/control`.
- **Session start:** dispatcher sets railroad time at session start (e.g., 8:00 AM); clock advances from there.
- **Persistence:** last-known state saved to disk; survives service restart.

---

## 5. Dispatcher Web Application

Full-screen browser on Display 1 (RPi5). Python FastAPI backend; browser communicates via WebSocket for live MQTT updates.

### Layout

```
┌─────────────────────────────────────────────────────────────────┐
│  10:42 AM  ●  [PAUSE]  [SET TIME]          NYE Layout Control   │
├──────┬──────┬──────┬──────┬──────┬──────┬──────────────────────┤
│  WP  │  XP  │  BB  │  JC  │  MC  │  SK  │  HC  │
│ ◻ CL │ ◉ TO │ ◉ TO │ ◉ TO │ ◉ TO │ ◉ TO │ ◻ CL │
│      │ ◻ CL │      │      │      │      │      │
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

- Station tiles show: TO signal status (raised ◉ / lowered ◻), clearance pending indicator, online/offline.
- Clicking a station tile opens a detail panel (last OS, active orders, signal direct control).
- Train orders: freeform text, one or multiple station recipients.
- Clearance forms: issued to WP, XP, and HC (termini and register stations); separate from TOs.

---

## 6. Train Orders and Clearances

### Train Orders
- Freeform text, authentic 1905 style.
- Dispatcher types the order, selects one or more destination stations, issues.
- Each destination station receives the order text; station agent ACKs when copied.
- TO signal arm raises automatically when order is issued (dispatcher can override manually).
- Signal lowers when order is ACKed and dispatcher releases it (or manually).

### Clearance Forms
- Issued at **WP** (south terminus — departing trains), **XP** (register station — trains proceeding north),
  and **HC** (register / north terminus — trains returning south).
- Required before a train departs a terminus or proceeds beyond a register point.
- Displayed on the station unit clearance screen; agent ACKs when issued to the crew.

---

## 7. DCC and JMRI

- **DCC system:** Digitrax DCS51 (primary command station/booster).
- **JMRI:** Migrates to RPi5. Connects to DCS51 via LocoNet USB interface — **PR3**.
- **SPROG RPis:** Remain as alternate/portable DCC system; not part of layout network.
- **Engine Driver (phone throttles):** Connect to JMRI WiThrottle server on RPi5 (`192.168.10.1:12090`). Independent of MQTT.
- **JMRI MQTT bridge:** JMRI publishes/subscribes to `trains/turnout/...` topics on the layout broker. Switch_Control firmware unchanged.
- **JMRI web UI:** Available at `192.168.10.1:8080`; can be opened on Dispatcher Display 1 in a second browser tab if needed.

---

## 8. Block Detection (Future — Phase 4)

RFID readers at station approach tracks. Train carries RFID tag. Reader reports train ID to broker (`trains/block/{id}/state`). Dispatcher app pre-populates OS submission; station agent confirms.

Topology unchanged — RFID reader ESP32 nodes join layout WiFi as additional MQTT publishers.

---

## 9. Build Phases

| Phase | Scope |
|-------|-------|
| **1 — Infrastructure** | RPi5: AP, broker, fast clock. Station_OS: network migration, clock display. JMRI migration. |
| **2 — Operations** | OS submission (station → dispatcher). TO issuance + display + ACK. Clearance forms. TO_Signal firmware + hardware. |
| **3 — Camera Views** | ESP32-CAM firmware. Dispatcher Display 2 camera grid. |
| **4 — Block Detection** | RFID at stations. Auto-populated OS. |
| **5 — Lighting** | Station lighting via fast clock day/night cycle. |
| **6 — Dispatcher Assist** | "Newbie mode": structured TO forms, timetable conflict hints. |

---

## 10. Open Questions / TBDs

| # | Item | Question | Working Assumption |
|---|------|----------|--------------------|
| 1 | RPi5 Ethernet | Connect RPi5 to home LAN via Ethernet for maintenance? | Yes, independent of layout WiFi |
| 2 | SSID / password | Final layout WiFi credentials | NYE_Layout / TBD |
| 3 | Session day | Does the railroad time include a day counter (Day 1, Day 2 of a session)? | Include day counter, default off |
| 4 | TO signal auto-lower | When exactly does the signal lower — on ACK, or dispatcher releases manually? | Dispatcher releases manually (authentic) |
