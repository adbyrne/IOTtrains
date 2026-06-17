# NY&E WP Yardmaster Terminal — Design Document

**Version:** 1.3  
**Date:** 2026-06-17  
**Status:** Session 2.0a + 2.0b implemented and deployed (software). RPi3 physical kiosk provisioning (§9) pending. Engine/caboose roster (§14) designed, not yet implemented.

---

## 1. Role Definition

The Yardmaster operates exclusively within WP (Williamsport) yard limits. Authority starts and ends at the yard boundary. Once a train clears the yard in either direction the Yardmaster's responsibility for that train ends.

**Three operational jobs:**

| Job | Description |
|-----|-------------|
| **Construct** | Build consists for departing NYE trains — minimum engine + caboose (1905 definition); add freight cars per Trainmaster manifest |
| **Deconstruct** | Receive arriving NYE trains; physically pull the consist apart and assign equipment to yard tracks |
| **Interchange** | Receive/deliver cars at the interchange track (INTX) on the C&O schedule |

**What the system tracks:**

- Consist summary per train: engine #, caboose #, loaded car count, empty car count, track # — on departure and arrival
- Track assignments: which train/consist is on which yard track
- Consist status lifecycle (see Section 5)

**What the system does NOT track:**

- Individual car road numbers or car IDs (CC&W Manager responsibility)
- Physical switch positions (hand-thrown Blue Points for now)
- Car movements during deconstruction

**Schedule knowledge:** The Yardmaster has full NYE and C&O schedules on paper. The digital terminal supplements with alerts for the next expected event — it does not replace paper.

**Dispatcher boundary:** The Dispatcher controls the trackage between WP yard limits and XP register station. Multiple trains may occupy that section simultaneously under rule-based operations. The Dispatcher notifies the Yardmaster of inbound trains; the Yardmaster decides all track routing within the yard independently.

---

## 2. Hardware

| Item | Specification |
|------|--------------|
| Computer | RPi3 (RPi3-1 or RPi3-3 — both standard/ready) |
| Display | ELECROW 7" IPS 1024×600 HDMI touchscreen (InvenTree pk=106) |
| Video | Full-size HDMI (RPi3) → HDMI (display) |
| Touch | USB HID — plug into any RPi3 USB port |
| Input | Touchscreen only — no keyboard or mouse during operation |
| Network | WiFi — NYE_Layout (192.168.10.x); provisioned via home LAN Ethernet first |
| Hostname | `rpi3-yard` |
| Browser | Chromium kiosk mode → `http://192.168.10.1:5000/yard` |

**Yard switches:** Hand-thrown Blue Points for now. The system architecture accommodates future servo conversion for digital route setting — `yard.json` includes a `switches` stub for this purpose.

---

## 3. UI Design

### 3.1 Page Layout — 1024×600 Landscape

```
┌─────────────────────────────────────────────────────────────────────────────┐ 50px
│  ● 10:42 AM  Monday              WP Yardmaster           [EXTRA REQUEST]    │
├──────────────────────┬──────────────────────┬──────────────────────────────┤
│  DEPARTING TRAINS    │   TRACK BOARD        │  ARRIVING TRAINS              │ 500px
│  380px               │   330px              │  314px                        │
│                      │                      │                               │
│  ▶ NEXT UP           │  T1 ▐ No.3  READY   │  ► No.7 S  ETA 10:15 AM     │
│  No.3 N  3:45 PM    │  T2 ▐ No.7  ASMBG   │  ► No.4 S  scheduled 2:29PM │
│  [BUILD CONSIST]     │  T3 ▐ Ext42 CAR RDY │                               │
│                      │  T4 ▐ ——  empty     │                               │
│  No.21 N  1:31 AM   │  CAB▐ Cab 42        │                               │
│  [BUILD CONSIST]     │  INX▐ C&O 93W       │                               │
│                      │                      │                               │
│  Extra 42 N          │                      │                               │
│  ○ CAR BLOCK READY   │                      │                               │
│  [ASSIGN ENG+CAB]    │                      │                               │
├──────────────────────┴──────────────────────┴──────────────────────────────┤ 50px
│  C&O:  WB  No.21 07:00  No.93 09:45  No.91 11:30  │  EB  No.12 08:15  No.6 10:30  │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 3.2 Header (50px)

- **Left:** Connection status dot (green/red) + RR clock, synced via WebSocket
- **Center:** Day of week + "WP Yardmaster"
- **Right:** `[EXTRA REQUEST]` button — YM-initiated extra train request to dispatcher

### 3.3 Departing Trains Panel (380px, scrollable)

One card per WP-departing train, sorted by scheduled departure time.

**"Next Up" section** — the single most prominent element. Shows only the next train to prepare:

```
┌────────────────────────────────────────────┐
│  ▶ NEXT UP                                  │
│  No. 3 Northbound  ·  Depart 3:45 PM       │
│  [BUILD CONSIST]                            │
└────────────────────────────────────────────┘
```

**Remaining departures** — scrollable list of upcoming trains in departure order. Each card ~90px:

```
┌────────────────────────────────────────────┐
│  No. 21 Northbound  ·  Depart 1:31 AM      │
│  ○ NOT STARTED         [BUILD CONSIST]     │
└────────────────────────────────────────────┘
```

**Status badges:**

| Badge | Color | Meaning |
|-------|-------|---------|
| `○ NOT STARTED` | Gray | No consist work begun |
| `◐ ASSEMBLING` | Yellow | Consist started (draft saved) |
| `○ CAR BLOCK READY` | Orange | Extras only — cars on track, no engine/caboose yet |
| `● READY` | Green | Full consist submitted, waiting for clearance |
| `✓ CLEARED` | Blue | Departure clearance issued; train handed to crew |
| `✗ ANNULLED` | Red/strikethrough | Dispatcher has annulled this train |

**Extra trains** appear at the bottom of the departing list after all scheduled trains, labelled "Extra Eng 42 Northbound."

### 3.4 Track Board Panel (330px)

One row per track from `yard.json`, 60px per row. Scrollable if more tracks than fit.

```
┌──────────────────────────────────────┐
│  T1  │  No.3   Northbound   READY    │
│  T2  │  No.7   Northbound   ASMBG    │
│  T3  │  Ext42  Northbound   CAR RDY  │
│  T4  │  ————   empty                 │
│ CAB  │  Cab 42                       │
│ INX  │  C&O No.93 Westbound          │
└──────────────────────────────────────┘
```

Row colour coding:
- Empty: dim gray
- Assembling: yellow
- Car block ready: orange
- Ready: green
- Cleared: blue (briefly, before train departs)
- Incoming (arrival notification received): pulsing amber

Tapping a row opens a small popover for manual track reassignment (secondary flow — primary assignment is via the consist modal).

### 3.5 Arriving Trains Panel (314px)

List of expected arrivals from dispatcher notifications, newest at top:

```
  ► No. 7 Southbound — ETA 10:15 AM
  ► No. 4 Southbound — scheduled 2:29 PM
  ✗ No. 24 — annulled
```

Entries persist until manually dismissed or the session ends. Tapping an entry shows the full notification detail.

### 3.6 C&O Schedule Footer (50px)

Two rows showing today's C&O trains with WP times:

```
  WB: No.21 arr 06:50 dep 07:00  ·  No.93 arr 09:35 dep 09:45  ·  No.91 arr 11:20 dep 11:30
  EB: No.12 arr 08:05 dep 08:15  ·  No.6 arr 10:20 dep 10:30  ·  No.32 arr 11:50 dep 12:00
```

C&O extras (owner-authorized via session.json) appear with `(X)` prefix. If COE timetable data is not yet populated, shows: "C&O schedule: see paper timetable."

### 3.7 Consist Build Modal — Scheduled Trains

Triggered by `[BUILD CONSIST]` on a departing train card. Full-screen overlay.

```
┌──────────────────────────────────────────────────────────────┐
│  No. 3 Northbound  ·  Depart 3:45 PM            [✕ CANCEL]  │
│  ──────────────────────────────────────────────────────────  │
│                                                              │
│  [Engine:   ___ ]    [Caboose:  ___ ]                       │
│  [Loads:    ___ ]    [Empties:  ___ ]                       │
│                                                              │
│  Track: [T1] [T2] [T3] [T4] [CAB] [INX]                   │
│                                                              │
│  ┌──────────────────────────────┐  Active field: Engine     │
│  │   7     8     9              │  Current:  ___            │
│  │   4     5     6              │                           │
│  │   1     2     3              │                           │
│  │  CLR    0     ⌫              │                           │
│  └──────────────────────────────┘                           │
│                                                              │
│  [SAVE DRAFT]                    [SUBMIT — MARK READY]      │
└──────────────────────────────────────────────────────────────┘
```

**UX rules:**
- Tap a field box to select it (highlighted border); numpad digits enter into that field
- Engine # is required; Caboose # is required (minimum 1905 consist)
- `[CLR]` clears the active field; `[⌫]` deletes last digit
- Track selector: tap to toggle; highlighted when selected
- `[SAVE DRAFT]` posts `state: assembling` — saves progress without marking ready; status badge updates to `◐ ASSEMBLING`
- `[SUBMIT — MARK READY]` posts `state: ready` — enabled only when Engine, Caboose, and Track are set; status badge updates to `● READY`
- Track highlighted in amber warning if not set on submit attempt

### 3.8 Consist Build Modal — Extra Trains (Two-Stage)

**Stage 1 — Car Block Assembly**

Triggered by `[BUILD CONSIST]` on an extra train card, or after dispatcher issues a `new_train` notification.

```
┌──────────────────────────────────────────────────────────────┐
│  Extra Train — Car Block Assembly            [✕ CANCEL]      │
│  Eng 42  Northbound  (authorized by TO #12)                  │
│  ──────────────────────────────────────────────────────────  │
│  Assemble freight cars only — engine and caboose assigned    │
│  after dispatcher sets departure time.                       │
│                                                              │
│  [Loads:   ___ ]     [Empties:  ___ ]                       │
│                                                              │
│  Track: [T1] [T2] [T3] [T4]   (no CAB or INX for extras)   │
│                                                              │
│  ┌──────────────────────────────┐                           │
│  │   7     8     9              │  Active: Loads            │
│  │   4     5     6              │  Current: ___             │
│  │   1     2     3              │                           │
│  │  CLR    0     ⌫              │                           │
│  └──────────────────────────────┘                           │
│                                                              │
│  [SAVE DRAFT]          [NOTIFY DISPATCHER — BLOCK READY]    │
└──────────────────────────────────────────────────────────────┘
```

`[NOTIFY DISPATCHER — BLOCK READY]` posts `state: car_block_ready`. Dispatcher sees a `consist_update` event in their UI. This is the YM's signal to the dispatcher that the car block is assembled and they are ready to receive a departure time.

**Stage 2 — Engine + Caboose Assignment**

Unlocked when the dispatcher posts a `departure_time_set` notification. The extra train card updates and shows `[ASSIGN ENG+CAB]`.

```
┌──────────────────────────────────────────────────────────────┐
│  Extra Train — Assign Engine & Caboose       [✕ CANCEL]      │
│  Eng 42  Northbound  ·  Depart 11:30 AM (set by Dispatcher)  │
│  Car block: T3  ·  8 loads, 3 empties                        │
│  ──────────────────────────────────────────────────────────  │
│                                                              │
│  [Engine:   ___ ]    [Caboose:  ___ ]                       │
│                                                              │
│  ┌──────────────────────────────┐                           │
│  │   7     8     9              │  Active: Engine            │
│  │   4     5     6              │  Current: ___             │
│  │   1     2     3              │                           │
│  │  CLR    0     ⌫              │                           │
│  └──────────────────────────────┘                           │
│                                                              │
│  [CANCEL]           [SUBMIT — NOTIFY DISPATCHER & CREW]     │
└──────────────────────────────────────────────────────────────┘
```

`[SUBMIT — NOTIFY DISPATCHER & CREW]` posts `state: ready` with engine + caboose added. Dispatcher and crew (via WP CYD) are notified. Dispatcher then issues the departure clearance.

### 3.9 Extra Request Modal (YM-initiated)

Triggered by `[EXTRA REQUEST]` in the header.

```
┌──────────────────────────────────────────────┐
│  Request Extra Train                          │
│  ────────────────────────────────────────── │
│  Direction:  [NORTH]  [SOUTH]               │
│                                              │
│  Available cars (approx):                   │
│  [Loads:  ___ ]    [Empties:  ___ ]         │
│                                              │
│  [CANCEL]         [NOTIFY DISPATCHER]       │
└──────────────────────────────────────────────┘
```

This sends a `POST /api/yard/extra_request` — server broadcasts a `extra_request` WS event to the dispatcher UI. The dispatcher reviews and decides whether to authorize via a running-extra TO. The YM's terminal shows a pending badge until the dispatcher responds.

---

## 4. Data Design

### 4.1 yard.json

**Location:** `RR_Server/data/yard.json`

Track IDs are populated from the XTrkCAD layout file — not invented placeholders. The content below uses temporary IDs; replace with actual XTrkCAD track identifiers before session 2.0.

```json
{
  "version": "1.0",
  "tracks": [
    {"id": "T1",   "label": "Track 1",       "function": "departure",   "capacity_cars": null},
    {"id": "T2",   "label": "Track 2",       "function": "arrival",     "capacity_cars": null},
    {"id": "T3",   "label": "Track 3",       "function": "local",       "capacity_cars": null},
    {"id": "T4",   "label": "Track 4",       "function": "local",       "capacity_cars": null},
    {"id": "CAB",  "label": "Caboose Track", "function": "caboose",     "capacity_cars": null},
    {"id": "INX",  "label": "Interchange",   "function": "interchange", "capacity_cars": null}
  ],
  "switches": []
}
```

**Track functions:**

| Function | Description |
|----------|-------------|
| `departure` | Trains stage here while consist is being built |
| `arrival` | Inbound trains park here on arrival |
| `local` | General yard storage; local industry car staging |
| `caboose` | Caboose storage |
| `interchange` | C&O interchange — cars swapped between railroads |

**`switches` stub:** Reserved for future servo-controlled switch conversion. When yard switches are motorized, each entry will define the switch ID, the tracks it routes between, and the MQTT topic for control. Empty for now.

**Updating yard.json:** Owner edits the file directly and restarts `rr-dispatcher`. No management UI needed for this yet.

### 4.2 session.json Integration (future — Management Tools session)

When `session.json` is designed (Management Tools planning session), the yardmaster terminal will read:

- **Pre-authorized extras:** Extra trains identified by the Trainmaster function appear in the departing list with status `○ NOT STARTED` before the dispatcher has issued the TO
- **Train manifests:** Per-train car type requirements (e.g., "Train 3 needs 2 FM empty") displayed in the consist modal to guide YM car selection (individual car IDs remain on paper)
- **C&O extras:** Owner-authorized C&O extras appear in the footer C&O schedule

For session 2.0, session.json integration is not implemented. The departing list is built from the timetable only.

---

## 5. Consist Lifecycle

### Scheduled trains

```
NOT STARTED
  │  [YM taps Build Consist, enters fields, taps Save Draft]
  ▼
ASSEMBLING  — state: "assembling"
  │  [YM completes form, taps Submit — Mark Ready]
  ▼
READY       — state: "ready"
  │  [Dispatcher issues WP departure clearance — automatic]
  ▼
CLEARED     — state: "cleared"
```

### Extra trains

```
NOT STARTED  (extra authorized by dispatcher TO)
  │  [YM assembles car block, taps Notify Dispatcher — Block Ready]
  ▼
CAR BLOCK READY  — state: "car_block_ready"  ← extras only
  │  [Dispatcher sets departure time; sends departure_time_set notification]
  ▼
(Stage 2 unlocked on terminal)
  │  [YM assigns engine + caboose, taps Submit — Notify Dispatcher & Crew]
  ▼
READY        — state: "ready"
  │  [Dispatcher issues WP departure clearance — automatic]
  ▼
CLEARED      — state: "cleared"
```

**State field values in MQTT:** `assembling` | `car_block_ready` | `ready` | `cleared`

**Arrival recording:** When a southbound train arrives at WP yard, the system records it as arrived (triggered by dispatcher notification or WP OS at the station CYD). No deconstruction detail is tracked; the physical consist teardown is the YM's work.

---

## 6. Extra Train Workflow — Full Detail

Three paths to an extra train:

### Path A: Pre-session (Trainmaster → session.json)
1. Trainmaster identifies extra needed; records in session.json
2. Server loads session.json at session start
3. Extra appears in departing list as `NOT STARTED`
4. Dispatcher issues running-extra TO when appropriate during session
5. YM proceeds to Stage 1 car block assembly

### Path B: YM-initiated (live session)
1. YM sees car demand, taps `[EXTRA REQUEST]` on terminal
2. Fills direction + approximate loads/empties
3. Server broadcasts `extra_request` WS event to dispatcher UI
4. Dispatcher reviews, issues running-extra TO (or declines)
5. System sends `new_train` notification to YM
6. YM proceeds to Stage 1

### Path C: Dispatcher-initiated (live session)
1. Dispatcher recognizes need, issues running-extra TO
2. System sends `new_train` notification to YM
3. YM proceeds to Stage 1

### After Stage 1 (all paths):
```
YM: Car block assembled → taps [NOTIFY DISPATCHER — BLOCK READY]
  → consists["42"].state = "car_block_ready"
  → MQTT: trains/yard/consist/42  {state: "car_block_ready", cars_loaded: 8, cars_empty: 3, track_id: "T3"}
  → WS broadcast: consist_update → Dispatcher sees it in UI

Dispatcher: Coordinates timing → POST /api/yard/notification
  → type: "departure_time_set", engine: "42", departure_rr_time: "11:30"
  → MQTT: trains/yard/notification
  → WS broadcast: yard_notification → YM terminal unlocks Stage 2

YM: Assigns engine + caboose → taps [SUBMIT — NOTIFY DISPATCHER & CREW]
  → consists["42"].state = "ready"
  → MQTT: trains/yard/consist/42  {state: "ready", engine: "101", caboose: "204", ...}
  → WS broadcast: consist_update → Dispatcher and WP CYD notified

Dispatcher: Issues WP departure clearance (Session 2.3)
  → Server: consists["42"].state = "cleared"
  → MQTT: trains/yard/consist/42  {state: "cleared", cleared_rr_time: "11:32"}
```

---

## 7. Server-Side Design

### 7.1 New API Endpoints

```
GET  /yard                       Yardmaster page HTML
POST /api/yard/consist           YM submits consist (assembling / car_block_ready / ready)
POST /api/yard/notification      Dispatcher sends notification to YM
POST /api/yard/extra_request     YM requests an extra train (notifies dispatcher)
```

**POST /api/yard/consist — body:**
```json
{
  "train": "3",
  "state": "assembling",
  "engine": "101",
  "caboose": "204",
  "cars_loaded": 12,
  "cars_empty": 5,
  "track_id": "T1",
  "extra": false
}
```
For `car_block_ready`: `engine` and `caboose` are omitted/null. `train` field uses engine number for extras.

Server derives `cars_total`, injects `rr_time` + `day` from clock state, publishes to MQTT, broadcasts `consist_update` WS event.

**POST /api/yard/notification — body:**
```json
{ "type": "arrival",          "train": "7", "section": 0, "direction": "S", "expected_rr_time": "10:15" }
{ "type": "departure_change", "train": "3", "new_departure_rr_time": "16:15" }
{ "type": "new_train",        "engine": "42", "direction": "N", "departure_rr_time": "11:30" }
{ "type": "annulment",        "train": "24" }
{ "type": "departure_time_set", "engine": "42", "departure_rr_time": "11:30" }
```

**POST /api/yard/extra_request — body:**
```json
{ "direction": "N", "approx_loads": 8, "approx_empties": 3 }
```
Server broadcasts `extra_request` WS event to dispatcher UI. No MQTT topic (dispatcher-only notification; YM terminal awaits `new_train` notification in response).

### 7.2 AppState Additions (session.py)

```python
consists: dict = field(default_factory=dict)
# key: train_number (engine number for extras)
# value: full consist dict (mirrors MQTT payload)

yard_notifications: list = field(default_factory=list)
# recent arrival/change notifications, newest first, capped at 20

yard_tracks: list = field(default_factory=list)
# loaded from yard.json at startup; included in initial_state WS event
```

On MQTT reconnect: re-subscribe to `trains/yard/consist/+` (retained) to rebuild `consists`.

### 7.3 MQTT Topic Additions

| Topic | Direction | QoS | Retained | Notes |
|-------|-----------|-----|----------|-------|
| `trains/yard/consist/{train}` | YM terminal → All | 1 | Yes | Extended: `car_block_ready` state added |
| `trains/yard/notification` | Server → YM | 1 | No | Extended: `departure_time_set` type added |

### 7.4 New WebSocket Events

| Event | Payload | Notes |
|-------|---------|-------|
| `consist_update` | Full consist dict | Both pages receive; YM shows it, dispatcher can monitor |
| `yard_notification` | Notification dict (type + fields) | Both pages receive |
| `extra_request` | `{direction, approx_loads, approx_empties, rr_time}` | Dispatcher page receives; shows alert to dispatch |

**`initial_state` event additions:**
```json
{
  "consists": { "3": {...}, "42": {...} },
  "yard_tracks": [ {"id": "T1", "label": "Track 1", "function": "departure"}, ... ],
  "yard_notifications": [ {...}, {...} ],
  "coe_trains": [ {"number": "21", "direction": "W", "wp_arrive": "06:50", "wp_depart": "07:00"}, ... ]
}
```

### 7.5 Triggered by Dispatcher Clearance (Session 2.3)

When dispatcher issues a WP departure clearance, the server automatically:
1. Sets `consists[train].state = "cleared"`, adds `cleared_rr_time`
2. Publishes updated `trains/yard/consist/{train}` (retained, state=cleared)
3. Broadcasts `consist_update` WS event

No YM action required for this transition.

---

## 8. Dispatcher Page Changes

One new element added to the existing dispatcher UI.

### 8.1 "Notify YM" Button

Added near the OS log or station table. Opens a modal:

**Fields:**
- Notification type: `[Arrival]` `[Departure Change]` `[Annulment]` (no C&O types — C&O is YM-only domain)
- Train / Engine # (text entry)
- Direction N/S (for arrival)
- Expected RR time / New departure time (type-dependent)

On confirm: `POST /api/yard/notification`.

**Contextual quick-action:** When a southbound train OSes at XP in the OS log, a small `[→ YM]` button appears next to that log entry. Tapping it pre-fills the Notify YM modal with type=arrival, train#, and direction. The dispatcher can adjust the ETA and confirm.

### 8.2 Extra Request Alert

When the YM sends an extra request (Path B), the dispatcher UI shows a banner:
```
⚡ Yardmaster requests extra: ~8 loads, ~3 empties — Northbound  [AUTHORIZE]  [DISMISS]
```

`[AUTHORIZE]` opens the Issue Train Order modal pre-filled as a running_extra TO.

### 8.3 Consist Status Visibility — DECIDED 2026-06-16: include in Session 2.0b

The dispatcher can see consist status — useful for knowing when a train is ready to receive a clearance. This is a read-only view; the dispatcher has no consist edit capability. Implementation: a collapsible "Yard Status" section in the dispatcher page showing the track board and consist states, fed by the same `consist_update` / `initial_state` WebSocket data already specified in §7.4 — no new backend endpoints required, only a dispatcher-side rendering addition in Session 2.0b.

---

## 9. RPi3 Provisioning

### 9.1 OS Setup

1. Flash **Raspberry Pi OS Desktop (64-bit, Bookworm)** to SD card using Raspberry Pi Imager
2. In Imager advanced settings: enable SSH, hostname = `rpi3-yard`, create user, set home LAN WiFi
3. Boot with HDMI connected (display power from USB or external supply per ELECROW spec)
4. Verify SSH access from laptop: `ssh pi@rpi3-yard.local`

### 9.2 Package Installation

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y chromium-browser unclutter
```

### 9.3 Kiosk Autostart

Enable auto-login to desktop: `sudo raspi-config → System Options → Boot / Auto Login → Desktop Autologin`

Create/edit `~/.config/lxsession/LXDE-pi/autostart`:
```
@xset s off
@xset -dpms
@xset s noblank
@unclutter -idle 0.5 -root
@chromium-browser --kiosk --noerrdialogs --disable-infobars --no-first-run --disable-restore-session-state http://192.168.10.1:5000/yard
```

### 9.4 Add NYE_Layout WiFi

```bash
sudo nmcli device wifi connect "NYE_Layout" password "PASSWORD" ifname wlan0
```

Keep home LAN WiFi config as fallback for maintenance. The RPi3 will connect to whichever is in range; `NYE_Layout` should be set as higher priority:
```bash
sudo nmcli connection modify "NYE_Layout" connection.autoconnect-priority 10
```

### 9.5 DHCP Reservation (on RPi5)

Add `rpi3-yard` MAC address to `/etc/dnsmasq.conf` on RPi5:
```
dhcp-host=AA:BB:CC:DD:EE:FF,rpi3-yard,192.168.10.20
```

Restart dnsmasq: `sudo systemctl restart dnsmasq`

### 9.6 Touch Input Verification

After boot, with no keyboard attached:
```bash
# From SSH:
xinput list   # Should show ELECROW touchscreen as a pointer device
```

If touch is not auto-detected, check `dmesg | grep -i touch` for USB HID registration.

### 9.7 Verify Kiosk

1. Reboot RPi3 with no keyboard or mouse attached
2. Desktop should auto-login
3. Chromium should launch full-screen at `/yard`
4. Touch navigation should work

---

## 10. Tests

New file: `tests/test_yard.py`

| Test | Description |
|------|-------------|
| `test_yard_page_loads` | GET /yard → 200 |
| `test_consist_assembling` | POST consist state=assembling → MQTT pub + WS event |
| `test_consist_car_block_ready` | POST consist state=car_block_ready (extra, no engine) → accepted |
| `test_consist_ready_scheduled` | POST consist state=ready with engine+caboose → accepted |
| `test_consist_ready_missing_engine` | POST state=ready without engine → 422 |
| `test_consist_ready_extra_two_stage` | POST car_block_ready → then ready with engine → state sequence correct |
| `test_notification_arrival` | POST notification type=arrival → MQTT pub + WS event |
| `test_notification_departure_change` | POST type=departure_change |
| `test_notification_annulment` | POST type=annulment |
| `test_notification_departure_time_set` | POST type=departure_time_set → stage 2 unlock event |
| `test_extra_request` | POST /api/yard/extra_request → WS extra_request event to dispatcher |
| `test_yard_initial_state` | WS connect → initial_state includes consists, yard_tracks, coe_trains |
| `test_yard_tracks_from_file` | yard.json loads correctly; track IDs present in initial_state |

---

## 11. C&O Schedule Notes

**Data source:** C&O East Central Subdivision schedule from previous layout (`NYELayoutDocs/alt/timetable.ods` Sheet2). Must be populated into the `COE` subdivision in `timetable.json` before session 2.0.

**Dispatcher involvement:** None. C&O traffic is entirely within the Yardmaster's domain. The dispatcher sees the scheduled C&O trains as a read-only reference in the timetable but has no operational control.

**C&O extras:** Owner-authorized pre-session via session.json (Management Tools session). The YM terminal displays them; no dispatcher involvement. C&O consists are not tracked by the system — C&O operations are informational only.

---

## 12. Implementation Order

Split into two sub-sessions to allow server-side work to proceed before the RPi3 is physically set up.

### Session 2.0a — Backend + Data ✅ COMPLETE (2026-06-16)

1. Create `yard.json` with XTrkCAD track IDs (content prerequisite) ✅
2. Populate COE timetable data in `timetable.json` (content prerequisite) ✅
3. `AppState` additions (`consists`, `yard_notifications`, `yard_tracks`) ✅
4. MQTT subscription for `trains/yard/consist/+` ✅
5. New API endpoints: `/yard`, `/api/yard/consist`, `/api/yard/notification`, `/api/yard/extra_request` ✅
6. WebSocket event additions (`consist_update`, `yard_notification`, `extra_request`) ✅
7. `initial_state` extended with yard data and C&O trains (`consists`, `yard_tracks`, `yard_notifications`, `coe_trains`, `yard_departures`) ✅
8. `tests/test_yard.py` — 23 tests, all passing (142 total across the project) ✅
9. Verified end-to-end against the live rpi5-2 deployment (HTTP + WebSocket) ✅

### Session 2.0b — UI + RPi3 _(software complete; RPi3 physical setup pending)_

1. `yard.html` + `yard.js` — full yardmaster page (3 panels + footer) ✅
2. Consist build modals — scheduled and extra (both stages) ✅
3. Extra request modal ✅
4. Dispatcher page: "Notify YM" button + modal + extra request alert, grouped under a "Yardmaster" section heading; plus a `[→ YM]` quick-action on southbound OS log entries ✅
5. Dispatcher yard status read-only view — included per §8.3 decision ✅
6. RPi3 physical setup: OS flash, package install, kiosk autostart — **pending**, needs physical RPi3 + ELECROW display in hand
7. Add NYE_Layout WiFi, DHCP reservation on RPi5 — **pending**
8. End-to-end test: RPi3 kiosk → `/yard` → consist build → MQTT → dispatcher sees update — **pending** (software path verified via desktop browser/HTTP instead)

---

## 13. Items to Consider / Open Questions

These items were identified during planning and require further thought before or after session 2.0. They are not blockers for this session unless noted.

### Pre-session 2.0 (must decide or confirm)

1. **WP yard track IDs from XTrkCAD** — ✅ RESOLVED (2026-06-14 export). `data/yard.json` populated with real track IDs, lengths, and capacities from the XTrkCAD export (RUN, CAB, T2–T8, YL).

2. **C&O timetable data** — ✅ RESOLVED. COE subdivision in `timetable.json` fully populated (10 trains).

3. **Dispatcher yard status view** — ✅ RESOLVED 2026-06-16: include in Session 2.0b. See §8.3.

### Design requiring further thought

4. **Advance notice time constraints** — How much lead time does the YM need before a scheduled train's departure to start building the consist? Does the system issue a "start building" alert based on departure time minus a buffer? (deferred to separate session)

5. **Crew notification when consist is ready** — When the YM marks a consist `ready`, how does the crew know? Options: verbal from YM, visual check of the board, or digital notification to the WP station CYD. Needs design. (deferred)

6. **Arrival deconstruction confirmation** — After a train arrives and the YM deconstructs it, does the system need the YM to explicitly record "Train 7 deconstructed"? Or is the consist simply marked `arrived` and the system moves on? Needed for accurate post-session reporting.

7. **Car block planning tool** — The YM may pre-block cars for an upcoming train (physically grouping them on a track) before starting the formal consist. Does the system track this intermediate state, or is pre-blocking purely physical? The current `assembling` state assumes the YM is ready to enter numbers — there's no "planning" state before that.

8. **C&O consist information** — When a C&O train arrives at the interchange, does the YM record the interchange (loads/empties received from C&O, loads/empties delivered to C&O)? Or is all C&O interchange handled on paper? This would only matter for the post-session report.

9. **Multiple trains on WP–XP section** — ✅ RESOLVED 2026-06-16. Dispatcher-side software control added: a block signal at WP (gates northbound entry into the section) and one at XP (gates southbound entry), independent of XP's existing TO signal arms. Simple raised/lowered toggle, no ACK-gating — `POST /api/signal/block`, `trains/signal/{WP,XP}/block/{cmd,state}` MQTT topics, rendered as a diamond button next to the station name in the dispatcher table. Physical servo hardware at WP is still future work; the software control is ready for it.

10. **YM-initiated extra authorization flow** — When YM requests an extra (Path B), the dispatcher must authorize via a running-extra TO before the YM can begin Stage 1. The current design has the `extra_request` WS event alerting the dispatcher, but the specific dispatcher UI interaction for authorization isn't fully designed. The dispatcher's Issue Train Order flow handles it, but the connection between the alert and the form needs refinement.

11. **Annulled trains in departing list** — When the dispatcher sends an annulment notification, the train card in the departing list shows `✗ ANNULLED` with strikethrough. How long does it stay visible before the YM can dismiss it? Does it need a `[DISMISS]` button, or does it auto-hide after a time?

### Management Tools session (design required before implementation)

12. **session.json manifest integration** — When session.json is designed, the YM terminal will display per-train car type requirements (e.g., "Train 3 needs 2 FM empty"). This feeds into the consist modal to guide car selection. The consist submission stays at count level; the type targeting is display-only.

13. **Pre-authorized extras from session.json** — Extras identified by the Trainmaster before the session appear in the departing list without a dispatcher TO having been issued yet. The display status and the point at which the TO is issued needs to be clarified in the Management Tools design.

14. **C&O extras as owner function** — Owner adds C&O extra trains to session.json before the session. YM terminal displays them in the C&O footer. No dispatcher or live system involvement. The owner management UI for this is a Management Tools design item.

15. **Empty car type requirements** — The question of which specific empty car types (FM, XM, HM, etc.) are needed for delivery to industries on a given train is the Trainmaster's output, surfaced to the YM via the session.json manifest. How this is presented in the consist modal (alongside the numeric entry) needs to be designed as part of session.json integration.

---

## 14. Engine/Caboose Roster (Design — 2026-06-17)

**Motivation:** Engine and Caboose fields in the consist modals (§3.7, §3.8) are currently free-keypad numeric entry. The fleet is small and fixed between operating sessions — a tap-to-select roster avoids YM mis-keying a road number and matches the existing Track selector pattern (§3.7: `[T1] [T2] [T3]...`).

### 14.1 Data: `RR_Server/data/roster.json` (new file)

A new file, separate from `yard.json` (track infrastructure) and `timetable.json` (schedule data) — the roster is equipment data with its own lifecycle (a future CC&W Manager / Management Tools session will extend it with bad-order status, per the deferred "Bad order Yardmaster feature").

```json
{
  "version": "1.0",
  "engines": [
    {"road_number": "10", "road_name": "NY&E", "type": "0-6-0",   "max_cars": 6,  "dcc_addr": 10, "sound": false, "road_eligible": false, "notes": "Yard/mine switcher only"},
    {"road_number": "11", "road_name": "NY&E", "type": "0-6-0",   "max_cars": 6,  "dcc_addr": 11, "sound": false, "road_eligible": false, "notes": "Yard/mine switcher only"},
    {"road_number": "12", "road_name": "NY&E", "type": "0-6-0",   "max_cars": 6,  "dcc_addr": 12, "sound": false, "road_eligible": false, "notes": "Yard/mine switcher only"},
    {"road_number": "14", "road_name": "NY&E", "type": "0-6-0",   "max_cars": 6,  "dcc_addr": 14, "sound": false, "road_eligible": false, "notes": "Yard/mine switcher only"},
    {"road_number": "21", "road_name": "NY&E", "type": "4-4-0",   "max_cars": 6,  "dcc_addr": 21, "sound": true,  "road_eligible": true},
    {"road_number": "22", "road_name": "NY&E", "type": "4-4-0",   "max_cars": 6,  "dcc_addr": 22, "sound": true,  "road_eligible": true},
    {"road_number": "30", "road_name": "NY&E", "type": "2-6-0",   "max_cars": 6,  "dcc_addr": 30, "sound": false, "road_eligible": true},
    {"road_number": "46", "road_name": "NY&E", "type": "4-6-0",   "max_cars": 6,  "dcc_addr": 46, "sound": false, "road_eligible": true}
  ],
  "cabooses": [
    {"road_number": "10", "road_name": "NY&E"},
    {"road_number": "11", "road_name": "NY&E"},
    {"road_number": "12", "road_name": "NY&E"},
    {"road_number": "13", "road_name": "NY&E"},
    {"road_number": "14", "road_name": "NY&E"},
    {"road_number": "15", "road_name": "NY&E"}
  ]
}
```

- **Engines:** the 8 NY&E road engines from `NYE_OPERATIONS.md` §5, minus the two foreign-road units (C&O #1592, Pennsylvania #1492) — those operate exclusively on C&O tracks at Williamsport and are never assigned to an NY&E consist the YM builds.
- **`road_eligible`** — new field, defaults `true`. Engines 10/11/12/14 (the four 0-6-0 switchers) are set `false`: restricted to yard switching and the mine branches (QM1/QM2, Kiel, O'Haras, Timber Ltd) and not eligible for road (scheduled or extra) consists. No other engine is restricted today, but the field is per-engine rather than hardcoded to these four specifically so it generalizes — toggling any future engine's road eligibility is a `roster.json` edit, no schema change needed. A dedicated management UI for toggling this is a future Management Tools candidate, not part of this design.
- **Cabooses:** road numbers 10–15 (6 cabooses) — first roster entry for cabooses; `NYE_OPERATIONS.md` §6 only ever covered the 94 freight cars, not cabooses, so this is new data, not a migration.
- **Owner-edited:** same convention as `yard.json` — owner edits the file directly and restarts `rr-dispatcher`; no management UI needed yet.

### 14.2 Server changes

- `AppState` gains `roster: dict` (engines + cabooses lists), loaded from `roster.json` at startup — mirrors how `yard_tracks` loads from `yard.json`.
- `initial_state` WS event extended with `roster: {engines: [...], cabooses: [...]}`.
- No new API endpoints — this is read-only reference data for populating the selector UI, not something the YM submits changes to.

### 14.3 UI change: tap-to-select button grid

Replaces the readonly numpad-target text inputs (`cs-engine`, `cs-caboose` in the scheduled-train modal; `ce-engine`, `ce-caboose` in the extra-train Stage 2 modal — see `yard.html`/`yard.js`).

```
│  Engine: [21] [22] [30] [46]                                 │
│  Caboose: [10] [11] [12] [13] [14] [15]                      │
```

- Buttons rendered from `roster.engines`/`roster.cabooses` (populated client-side from the `initial_state`/reconnect payload, same pattern as the Track selector buttons).
- **Engine grid is filtered to `road_eligible: true` entries** — both the scheduled-train modal and the extra-train Stage 2 modal build consists for trains that run the mainline, so engines 10/11/12/14 never appear in either. Neither modal represents a yard-local switching job, so there's no path in the YM terminal where a restricted engine would need to be selectable — they're used for yard/mine switching entirely outside the digital consist-tracking system.
- Tap to select (highlighted when active), tap again or tap another to change — same toggle behavior as the existing Track button row.
- The shared numpad (§3.7 mockup) keeps its role for **Loads** and **Empties** only; Engine/Caboose no longer route through `data-field` numpad targeting.
- Required-field validation is unchanged: Engine and Caboose still required for `[SUBMIT — MARK READY]` (scheduled trains) and `[SUBMIT — NOTIFY DISPATCHER & CREW]` (extras Stage 2); track field still flashes amber on a submit attempt with a field unset, same as today.

### 14.4 Tests to add (`tests/test_yard.py`)

| Test | Description |
|------|--------------|
| `test_roster_loads_from_file` | `roster.json` loads correctly at startup |
| `test_yard_initial_state_includes_roster` | WS `initial_state` includes `roster.engines` and `roster.cabooses` |

### 14.5 Out of scope for this design

- Bad-order flagging/filtering of roster equipment — separate "Bad order Yardmaster feature" session (already queued, needs its own design).
- Editing the roster from a UI — owner edits `roster.json` directly, same as `yard.json`.
- DCC address use in the consist flow — `dcc_addr` is carried in the data for future use (e.g. a JMRI throttle integration) but the consist modal only needs `road_number` today.
