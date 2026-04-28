# NY&E Layout Control System — MQTT Topic Specification

**Version:** 0.1 (draft for review)
**Date:** 2026-04-28
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
Published every 10 real seconds while running.

```json
{
  "time": "10:42",
  "hour": 10,
  "minute": 42,
  "day": 1,
  "speed": 3,
  "running": true
}
```

| Field | Type | Description |
|-------|------|-------------|
| `time` | string | Railroad time, HH:MM (24-hour) |
| `hour` | int | 0–23 |
| `minute` | int | 0–59 |
| `day` | int | Session day counter (1, 2, …) |
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
```

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
  "direction": "N",
  "extra": false,
  "rr_time": "10:41",
  "day": 1
}
```

| Field | Type | Description |
|-------|------|-------------|
| `train` | string | Train number (e.g., "3") or engine number for extra trains |
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

Issued only at register stations: **XP** and **HC**. A clearance authorizes a train to proceed beyond the register point.

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

The dispatcher directly commands each TO signal arm. The TO_Signal controller reports its state back.

#### `trains/signal/{station_id}/to/cmd`
**Direction:** Dispatcher UI → TO Signal controller  
**QoS:** 1 | **Retained:** Yes  
_(Retained so the signal recovers its commanded state on reconnect.)_

```json
{ "state": "raised" }
{ "state": "lowered" }
```

#### `trains/signal/{station_id}/to/state`
**Direction:** TO Signal controller → Dispatcher UI  
**QoS:** 1 | **Retained:** Yes

```json
{
  "state": "raised",
  "station_id": "BB",
  "rr_time": "10:30"
}
```

**TO-signal-capable stations:** XP, BB, JC, MC, SK  
HC and WP do not have TO signal arms.

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

## 3. Topic Summary Table

| Topic | Pub | Sub | QoS | Retained |
|-------|-----|-----|-----|----------|
| `trains/clock/time` | Clock svc | Station units, UI | 0 | Yes |
| `trains/clock/control` | UI | Clock svc | 1 | No |
| `trains/station/{id}/status` | Station unit | UI | 1 | Yes |
| `trains/os/{id}` | Station unit | UI | 1 | No |
| `trains/to/{id}` | UI | Station unit | 2 | No |
| `trains/to/{id}/ack` | Station unit | UI | 1 | No |
| `trains/clearance/{id}` | UI | Station unit | 2 | No |
| `trains/clearance/{id}/ack` | Station unit | UI | 1 | No |
| `trains/signal/{id}/to/cmd` | UI | TO Signal ctrl | 1 | Yes |
| `trains/signal/{id}/to/state` | TO Signal ctrl | UI | 1 | Yes |
| `trains/turnout/{id}/state` | JMRI / Turnout ctrl | Both | 1 | Yes |
| `trains/camera/{id}/url` | ESP32-CAM | UI | 0 | Yes |
| `trains/block/{id}/state` | RFID node | UI | 1 | Yes |

---

## 4. MQTT Client Configuration

All ESP32 clients must configure:
- **LWT topic:** `trains/station/{station_id}/status`
- **LWT payload:** `{"online": false, "station_id": "{id}"}`
- **LWT QoS:** 1
- **LWT retain:** true
- **Keep-alive:** 30 seconds

Broker authentication: username/password (configured in Mosquitto — credentials TBD).
