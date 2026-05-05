# RR_Server — RPi5 Layout Control Server

Python-based server software for the NY&E Northern Lights Subdivision layout control system. Runs on the dedicated RPi5 alongside Mosquitto and JMRI.

**Design decisions recorded here are from planning sessions — no implementation has started.**

---

## Architecture

### Process split

Two independent systemd services communicating via MQTT only:

| Service | Software | Description |
|---------|----------|-------------|
| `mosquitto` | Mosquitto | MQTT broker |
| `rr-clock` | `fast_clock/clock_service.py` | Fast clock daemon — publishes ticks, handles control commands |
| `rr-dispatcher` | `dispatcher/app.py` (FastAPI) | Dispatcher web app + yardmaster terminal + owner page |
| `jmri` | JMRI (Java) | DCC / WiThrottle server |

`rr-clock` and `rr-dispatcher` share no state directly. The dispatcher app reads clock state from MQTT ticks (`trains/clock/time`), not from the clock service process.

### MQTT as the state store

The server's in-memory state is a **cache of MQTT**, rebuilt on reconnect:

- **Retained messages** restore automatically on reconnect: station statuses, signal arm states, camera URLs, turnout states, last clock tick, per-train consist states, active extra schedules.
- **Persistent MQTT session** (`clean_session=False`, fixed `client_id`) — Mosquitto redelivers any QoS 1/2 messages not fully acknowledged before a crash (pending TO ACKs, clearance ACKs).
- **OS log** is in-memory only — starts fresh after restart. Paper dispatcher sheet is the authority.
- **`clock_state.json`** is the one exception: the clock service generates state rather than receiving it from MQTT.

_Note: persistent sessions accumulate stale subscriptions if topic structure changes. Wipe via a clean-session reconnect when redeploying during development._

### Web interface split

- **WebSocket (server → browser):** typed events only — the browser never sees raw MQTT. Server translates MQTT messages into clean browser events (`os_report`, `station_status`, `signal_state`, `clock_tick`, `consist_update`, etc.).
- **REST POST (browser → server):** all commands. Server translates to MQTT publishes. Endpoints: `/api/clock/control`, `/api/to/issue`, `/api/clearance/issue`, `/api/signal/{id}/{dir}`, `/api/yard/notification`, `/api/extra/new`.
- **Initial state on connect:** server sends an `initial_state` WebSocket event when a browser connects — full snapshot of current in-memory state. No separate REST state endpoint needed.

### Session state — no SQLite

All operational state is either in-memory (rebuilt from MQTT) or in `clock_state.json`. No database required.

---

## Directory Structure

```
RR_Server/
  common/
    timetable.py          ← timetable query module (active_trains, next_train, locations, segments)
  fast_clock/
    clock_service.py      ← fast clock daemon; publishes trains/clock/time
    clock_state.json      ← persisted clock state (day, time, speed, running)
  dispatcher/
    app.py                ← FastAPI app (dispatcher /, yardmaster /yard, owner /owner)
    mqtt_client.py        ← MQTT bridge → WebSocket typed events
    session.py            ← in-memory session state (OS log, active TOs, clearances, consists, extras)
    static/               ← CSS, JS
    templates/            ← Jinja2 HTML (dispatcher, yardmaster, owner pages)
  mosquitto/
    mosquitto.conf        ← broker config (auth, persistence, ACL, bind to layout WiFi)
  scripts/
    setup_ap.sh           ← configure hostapd + dnsmasq on RPi5
    setup_mosquitto.sh    ← install and configure broker
    setup_jmri.sh         ← JMRI install + LocoNet config notes
    install_services.sh   ← systemd unit files for all services
  data/
    timetable.json        ← active timetable (read-only during operations)
    session.json          ← pre-session data: manifests, extras, annulments (loaded by owner pre-session)
    yard.json             ← yard track numbers and functions (yardmaster reference)
  requirements.txt
  README.md
```

---

## Web Pages

| Path | User | Device |
|------|------|--------|
| `/` | Dispatcher | RPi5 Display 1 (full-screen browser) |
| `/yard` | Yardmaster | RPi3 + 7" DSI touchscreen (Chromium kiosk) |
| `/owner` | Owner | Any device on layout WiFi (phone, laptop) |

### Dispatcher page (`/`)
- Fast clock display with pause/start/set controls (deliberate two-step for time/day changes)
- Station tiles (×7): online/offline, TO signal arm states (N/S), clearance pending indicator
- OS log (quick-reference; in-memory)
- Active TOs panel (outstanding orders awaiting all ACKs)
- Active clearances panel (pending ACK)
- Active extras list (running extras and work extras in progress)
- Consist display per train (engine, caboose, cars loaded/empty/total, state, yard track)
- TO issuance: structured form by TO type (types defined by management function)
- Clearance issuance
- Yard arrival notification (pre-fills expected RR time from timetable segments + current clock)
- Extra train schedule generation (running extra: calculates reference schedule for dispatcher's paper)

### Yardmaster page (`/yard`)
- Departure lineup (from timetable + session.json annulments/extras)
- Consist submission form per train (state: assembling → ready)
- Arrival notification display (from dispatcher)
- C&O schedule (read-only reference)
- Optional: manifest view (car list per train from session.json)

### Owner page (`/owner`)
- **Pre-session:** load/verify session.json; confirm all services healthy; confirm station units online
- **During session:** read-only — service status (rr-clock, rr-dispatcher, mosquitto, jmri), station connectivity, MQTT broker stats, session elapsed time
- **Post-session:** generate session report (OS log, TOs, consists, extras run, annulments, RR time span)

---

## Mosquitto Authentication

Per device class (not per device — reduces provisioning overhead; can migrate to per-device later):

| Username | Used by |
|----------|---------|
| `rr_clock` | fast_clock service |
| `rr_dispatcher` | dispatcher FastAPI app |
| `cyd_unit` | all 7 station CYD units |
| `to_signal` | all 5 TO signal controllers |
| `jmri` | JMRI MQTT bridge |

---

## Network Configuration (RPi5)

- **WiFi (wlan0):** hostapd AP — SSID `NYE_Layout`, static IP `192.168.10.1`
- **DHCP (dnsmasq):** `192.168.10.10` – `192.168.10.99`
- **Ethernet (eth0):** home LAN (DHCP, for SSH/maintenance) — optional, independent of layout WiFi
- **Mosquitto:** binds to `192.168.10.1:1883` (layout WiFi only)
- **Dispatcher web app:** `http://localhost:5000` (Display 1); `http://192.168.10.1:5000` (layout WiFi)

---

## Services (systemd)

All services start at boot. `rr-clock` and `rr-dispatcher` depend on `mosquitto` being ready.

---

## Phase 1 Deliverables

- [ ] hostapd + dnsmasq configuration (`setup_ap.sh`)
- [ ] Mosquitto config with auth, persistence, ACL (`setup_mosquitto.sh`, `mosquitto.conf`)
- [ ] Fast clock service: start/pause/set/reset/speed/set_tick_interval, sync_request response, state persisted to `clock_state.json`
- [ ] Dispatcher UI: clock display with controls, station status tiles
- [ ] Timetable loader (`common/timetable.py`): reads `timetable.json`, serves active-trains and next-train queries
- [ ] systemd unit files for all services
- [ ] WebSocket event stream + initial_state on connect

---

## timetable.json — Segments (planned, XTrkCAD-dependent)

A `segments` section will be added to the NLS subdivision in `timetable.json` to support extra train schedule calculation and yard arrival time estimation. Each segment provides inter-station travel times per train class, derived from XTrkCAD distance data and nominal class speeds. This section is a placeholder until XTrkCAD data is available.

```json
"segments": [
  { "from": "WP", "to": "XP", "class1_min": 231, "class2_min": 258, "class3_min": 262 },
  ...
]
```

---

## Dependencies

```
fastapi
uvicorn[standard]
paho-mqtt
jinja2
python-dotenv
websockets
```
