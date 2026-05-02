# NY&E Layout Control System — MQTT Topic Specification

**Version:** 0.3 (draft for review)
**Date:** 2026-04-30
**Broker:** Mosquitto on RPi5 at `192.168.10.1:1883`

---

## 1. Conventions

- All topics are under the `trains/` prefix.
- Station IDs are uppercase two-letter codes: `WP`, `XP`, `BB`, `JC`, `MC`, `SK`, `HC`.
- Device IDs for turnouts and other nodes are lowercase slugs (e.g., `north_yard_01`).
- Payloads are JSON unless noted. JMRI-compatible topics use plain string payloads.
- **QoS 0** — fire and forget (clock ticks, camera URL, heartbeat)
- **QoS 1** — at-least-once (OS reports, signal commands, status)
- **QoS 2** — exactly-once (train orders, clearance forms — critical operations)
- **Retained** topics persist their last value; new subscribers receive it immediately on connect.

---

## 2. Topic Reference

### 2.1 Fast Clock

#### `trains/clock/time`
**Direction:** Fast Clock service → All  
**QoS:** 0 | **Retained:** Yes  
Published at a configurable interval (default: 60 real seconds) while running. CYD units interpolate locally between ticks; tick frequency affects resync accuracy only.

```json
{
  "time": "10:42 AM",
  "hour": 10,
  "minute": 42,
  "day": 1,
  "speed": 3,
  "running": true
}
```

| Field | Type | Description |
|-------|------|-------------|
| `time` | string | Railroad time, 12-hour AM/PM (e.g. "10:42 AM", "12:00 PM") — for display |
| `hour` | int | 0–23 — for internal calculations |
| `minute` | int | 0–59 |
| `day` | int | Day of week: 1=Monday … 7=Sunday; advances at railroad midnight |
| `speed` | int | Clock multiplier (3 = 3:1) |
| `running` | bool | False when paused |

#### `trains/clock/control`
**Direction:** Dispatcher UI → Fast Clock service  
**QoS:** 1 | **Retained:** No

```json
{ "action": "start" }
{ "action": "pause" }
{ "action": "set",   "hour": 8, "minute": 0, "day": 1 }
{ "action": "reset" }
{ "action": "speed", "speed": 3 }
{ "action": "set_tick_interval", "seconds": 60 }
```

`set` fields: `hour` 0–23, `minute` 0–59, `day` 1–7 (1=Monday). All three optional — omitted fields keep their current value. Typically used at session start to resume from saved state. Day or time override requires owner permission (enforced in the dispatcher UI, not the protocol).

#### `trains/clock/sync_request`
**Direction:** Station unit → Fast Clock service  
**QoS:** 0 | **Retained:** No  
**Payload:** `{"station_id": "BB"}`

Published by a station unit that restarts mid-session and needs an immediate clock sync. The fast clock service responds by publishing an immediate `trains/clock/time` tick. Normal session start does not require this — all units must be online before the clock is started.

---

### 2.2 Station Status (Heartbeat / LWT)

#### `trains/station/{station_id}/status`
**Direction:** Station unit → All  
**QoS:** 1 | **Retained:** Yes

Published on connect and every 60 seconds. The MQTT **Last Will and Testament** (LWT) is set by each station unit to publish `{"online": false}` to this topic if it disconnects unexpectedly.

```json
{
  "online": true,
  "station_id": "BB",
  "firmware": "1.0.0",
  "rssi": -65,
  "clock_sync": true
}
```

---

### 2.3 OS Reports (On Sheet)

#### `trains/os/{station_id}`
**Direction:** Station unit → Dispatcher UI  
**QoS:** 1 | **Retained:** No

Published when the station agent submits an OS report via the CYD keypad.

```json
{
  "station_id": "BB",
  "train": "3",
  "section": 0,
  "direction": "N",
  "extra": false,
  "rr_time": "10:41",
  "day": 1
}
```

| Field | Type | Description |
|-------|------|-------------|
| `train` | string | Train number (e.g., "3") or engine number for extra trains |
| `section` | int | Section number; `0` = not a sectioned train; `1`, `2`, … for each section |
| `direction` | string | `"N"` or `"S"` |
| `extra` | bool | True if extra train (not in timetable) |
| `rr_time` | string | Railroad time of OS, from local clock state |
| `day` | int | Session day |

---

### 2.4 Train Orders

#### `trains/to/{station_id}`
**Direction:** Dispatcher UI → Station unit  
**QoS:** 2 | **Retained:** No

One message per addressed station. The `addressed_to` field lists all stations receiving this order (for context display).

```json
{
  "seq": 7,
  "text": "No. 3 Eng 101 take siding at Becs Bend and wait for Extra 42 North.",
  "addressed_to": ["BB", "XP"],
  "issued_rr_time": "10:30",
  "day": 1
}
```

| Field | Type | Description |
|-------|------|-------------|
| `seq` | int | Order sequence number (session-unique, used to correlate ACK) |
| `text` | string | Full freeform order text |
| `addressed_to` | array | All station IDs receiving this order |
| `issued_rr_time` | string | Railroad time of issuance |

#### `trains/to/{station_id}/ack`
**Direction:** Station unit → Dispatcher UI  
**QoS:** 1 | **Retained:** No

```json
{
  "seq": 7,
  "station_id": "BB",
  "rr_time": "10:33",
  "copies": 2
}
```

`copies` reflects the number of carbon copies the agent signs (prototype practice; defaults to 2).

---

### 2.5 Clearance Forms

Issued to **any station** where a train originates or a register point is reached. Standard clearance points are WP (south terminus), XP (register), and HC (north terminus/register). Any intermediate station may also receive a clearance when a train originates there (e.g. southbound return from SK). All CYD units have a Clearance screen; it activates when a clearance is received.

#### `trains/clearance/{station_id}`
**Direction:** Dispatcher UI → Station unit  
**QoS:** 2 | **Retained:** No

```json
{
  "seq": 3,
  "train": "3",
  "direction": "N",
  "text": "No. 3 Eng 101 has right of track Xina Pass to Hemlock Crest. No train orders.",
  "issued_rr_time": "08:05",
  "day": 1
}
```

#### `trains/clearance/{station_id}/ack`
**Direction:** Station unit → Dispatcher UI  
**QoS:** 1 | **Retained:** No

```json
{
  "seq": 3,
  "station_id": "XP",
  "rr_time": "08:07"
}
```

---

### 2.6 TO Signal Control

Each TO-signal station has **two independent arms** — N (northbound) and S (southbound) — each driven by its own servo. The dispatcher commands each arm independently. The controller reports each arm's state back separately.

**TO-signal-capable stations:** XP, BB, JC, MC, SK  
HC and WP do not have TO signal arms.

#### `trains/signal/{station_id}/to/{dir}/cmd`
**Direction:** Dispatcher UI → TO Signal controller  
**QoS:** 1 | **Retained:** Yes  
**`{dir}`:** `N` (northbound arm) or `S` (southbound arm)  
_(Retained so each arm recovers its commanded state on controller reconnect.)_

```json
{ "state": "raised" }
{ "state": "lowered" }
```

#### `trains/signal/{station_id}/to/{dir}/state`
**Direction:** TO Signal controller → Dispatcher UI  
**QoS:** 1 | **Retained:** Yes  
**`{dir}`:** `N` or `S`

```json
{
  "state": "raised",
  "station_id": "BB",
  "dir": "N",
  "rr_time": "10:30"
}
```

**Examples:**
- `trains/signal/BB/to/N/cmd` — raise/lower BB northbound arm
- `trains/signal/BB/to/S/cmd` — raise/lower BB southbound arm
- `trains/signal/BB/to/N/state` — BB northbound arm current state
- `trains/signal/BB/to/S/state` — BB southbound arm current state

---

### 2.7 Turnouts (JMRI-compatible, existing)

#### `trains/turnout/{device_id}/state`
**Direction:** Bidirectional (JMRI ↔ Switch_Control firmware)  
**QoS:** 1 | **Retained:** Yes  
**Payload:** Plain string — `CLOSED` or `THROWN`

No change from existing Switch_Control design.

---

### 2.8 Station Cameras (Phase 3)

#### `trains/camera/{station_id}/url`
**Direction:** ESP32-CAM → All  
**QoS:** 0 | **Retained:** Yes  
**Payload:** Plain string — HTTP URL of the MJPEG stream.

Example: `http://192.168.10.42/stream`

Published on boot. Dispatcher UI uses this URL to embed the live feed.

---

### 2.9 Block Detection (Phase 4)

#### `trains/block/{block_id}/state`
**Direction:** RFID reader node → All  
**QoS:** 1 | **Retained:** Yes

```json
{
  "block_id": "BB_approach",
  "state": "occupied",
  "train_tag": "003",
  "rr_time": "10:40"
}
```

Block IDs TBD. Designed now so the topic namespace is reserved.

---

### 2.10 Yard Messages

Communication between the Dispatcher web app and the Yardmaster terminal (RPi3 + 7" touchscreen at Williamsport yard).

#### `trains/yard/consist`
**Direction:** Yardmaster terminal → Dispatcher UI  
**QoS:** 1 | **Retained:** No

Published when the Yardmaster submits a consist report. May be submitted multiple times as the consist is assembled and adjusted (e.g., a car is cut due to defect, or an empty is added). The `ready` flag set to `true` is the formal signal that the train is assembled and the crew is on board. A final submission with `ready: true` is required before the Dispatcher issues a departure clearance.

```json
{
  "train": "3",
  "engine": "101",
  "caboose": "204",
  "cars_loaded": 12,
  "cars_empty": 5,
  "ready": true,
  "rr_time": "10:30",
  "day": 1
}
```

| Field | Type | Description |
|-------|------|-------------|
| `train` | string | Train number, or engine number for extras |
| `engine` | string | Engine number |
| `caboose` | string | Caboose number |
| `cars_loaded` | int | Count of loaded freight cars in consist |
| `cars_empty` | int | Count of empty freight cars in consist |
| `ready` | bool | `true` = train formally marked ready for departure |
| `rr_time` | string | Railroad time of submission |
| `day` | int | Session day |

#### `trains/yard/notification`
**Direction:** Dispatcher UI → Yardmaster terminal  
**QoS:** 1 | **Retained:** No

Sent by the Dispatcher to notify the Yardmaster of operational changes. The `type` field determines which additional fields are present.

```json
{ "type": "arrival",          "train": "24", "section": 0, "direction": "S", "expected_rr_time": "11:15", "day": 1 }
{ "type": "departure_change", "train": "3",  "new_departure_rr_time": "04:15", "day": 1 }
{ "type": "new_train",        "engine": "101", "direction": "N", "departure_rr_time": "06:00", "day": 1 }
{ "type": "cancellation",     "train": "141", "day": 1 }
```

| `type` | Meaning | Key fields |
|--------|---------|-----------|
| `arrival` | Train inbound from XP; prepare yard track, align entrance switch | `train`, `section`, `direction`, `expected_rr_time` |
| `departure_change` | Scheduled departure time adjusted | `train`, `new_departure_rr_time` |
| `new_train` | Extra train authorized by Train Order | `engine`, `direction`, `departure_rr_time` |
| `cancellation` | Scheduled train cancelled for this session | `train` |

---

## 3. Topic Summary Table

| Topic | Pub | Sub | QoS | Retained |
|-------|-----|-----|-----|----------|
| `trains/clock/time` | Clock svc | Station units, UI | 0 | Yes |
| `trains/clock/control` | UI | Clock svc, Station units | 1 | No |
| `trains/clock/sync_request` | Station unit | Clock svc | 0 | No |
| `trains/station/{id}/status` | Station unit | UI | 1 | Yes |
| `trains/os/{id}` | Station unit | UI | 1 | No |
| `trains/to/{id}` | UI | Station unit | 2 | No |
| `trains/to/{id}/ack` | Station unit | UI | 1 | No |
| `trains/clearance/{id}` | UI | Station unit | 2 | No |
| `trains/clearance/{id}/ack` | Station unit | UI | 1 | No |
| `trains/signal/{id}/to/{dir}/cmd` | UI | TO Signal ctrl | 1 | Yes |
| `trains/signal/{id}/to/{dir}/state` | TO Signal ctrl | UI | 1 | Yes |
| `trains/turnout/{id}/state` | JMRI / Turnout ctrl | Both | 1 | Yes |
| `trains/camera/{id}/url` | ESP32-CAM | UI | 0 | Yes |
| `trains/block/{id}/state` | RFID node | UI | 1 | Yes |
| `trains/yard/consist` | Yardmaster | UI | 1 | No |
| `trains/yard/notification` | UI | Yardmaster | 1 | No |

---

## 4. MQTT Client Configuration

All ESP32 clients must configure:
- **LWT topic:** `trains/station/{station_id}/status`
- **LWT payload:** `{"online": false, "station_id": "{id}"}`
- **LWT QoS:** 1
- **LWT retain:** true
- **Keep-alive:** 30 seconds

Broker authentication: username/password (configured in Mosquitto — credentials TBD).
