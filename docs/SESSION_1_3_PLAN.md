# Session 1.3 Plan — Dispatcher UI: Clock + Status

**Prerequisite:** Sessions 1.1 + 1.2 + 1.2a ✅ complete

---

## Goal

Working page at `http://192.168.10.1:5000` showing:
- Live railroad clock with pause/start/set controls
- Station status panel — 7 tiles (one per CYD station), online/offline, next scheduled train per tile

Dispatcher can monitor all stations and the fast clock from a single browser page.

---

## Scope

### What's in scope
- FastAPI app in `RR_Server/dispatcher/`
- WebSocket endpoint — bridge between MQTT events and browser
- Clock control panel (shows current RR time; pause/resume, speed, set-time controls)
- Station tiles (7 tiles: WP, XP, BB, JC, MC, SK, HC)
  - Next scheduled train per station per direction (calls `timetable.next_train()`)
  - Both NB and SB rows always shown; "—" when no train (e.g. terminus, wrong-direction switchback)
  - Server pushes updated next-train data on every clock tick
- systemd `rr-dispatcher` unit started and verified

### What's out of scope (later sessions)
- Station online/offline indicator — deferred until after Session 1.4 (Station_OS heartbeat not yet defined)
- OS submission log (Session 2.1)
- Train Order issuance (Session 2.2)
- Clearance forms (Session 2.3)
- TO signal arm control from UI (needs Sessions 2.2/2.4 integration)
- Yardmaster page (Session 2.0)

---

## MQTT Topics to Subscribe (dispatcher)

| Topic | Purpose |
|-------|---------|
| `trains/clock/state` | Retained — current clock state (running, rr_time, speed) |
| `trains/clock/tick` | Live tick events — update displayed time + push next-train recalc |

## REST Endpoints (dispatcher controls broker via MQTT publish)

| Method | Path | Action |
|--------|------|--------|
| POST | `/api/clock/control` | `{"action": "start"\|"pause"\|"reset"\|"set", "time": "HH:MM"}` |
| POST | `/api/clock/speed` | `{"speed": N}` |
| GET  | `/` | Serve dispatcher HTML page |
| GET  | `/ws` | WebSocket — streams typed events to browser |

## WebSocket Event Types (server → browser)

```json
{ "type": "clock_state",   "data": { "running": true, "rr_time": "08:30", "speed": 3 } }
{ "type": "clock_tick",    "data": { "rr_time": "08:31", "stations": [ ... ] } }
{ "type": "initial_state", "data": { "clock": {...}, "stations": [ ... ] } }
```

`initial_state` is sent immediately on WebSocket connect — browser gets full snapshot before any incremental events.

`clock_tick` carries the updated next-train data for all 7 stations (server-push on every tick).

### Station data shape (inside `stations` array)

```json
{
  "id": "XP",
  "name": "Xina Pass",
  "next_N": { "number": "2", "time": "10:15" },
  "next_S": null
}
```

`next_N` / `next_S` are `null` when no upcoming train. `day` is always `1` (Monday/weekday) — no calendar tracking in fast clock.

Station online/offline status is not included in 1.3; added after Session 1.4 (Station_OS heartbeat).

---

## File Structure

```
RR_Server/
  dispatcher/
    app.py            ← FastAPI app, REST endpoints, WebSocket handler
    mqtt_client.py    ← Paho MQTT client, subscribe + event dispatch
    session.py        ← In-memory state (clock state, station statuses)
    static/
      style.css
    templates/
      dispatcher.html ← Jinja2 template (clock panel + station tiles)
```

`app.py` imports `common.timetable` for next-train lookups per station tile.

---

## Station Tile Design

Each tile shows:
- Station name
- Next NB train: number + time (or "—" if none)
- Next SB train: number + time (or "—" if none)

Both direction rows always shown. Next-train data is server-push on every clock tick (`clock_tick` event carries `stations` array). `day=1` (weekday) always — no calendar in fast clock.

Online/offline indicator deferred to after Session 1.4.

---

## systemd Unit

`rr-dispatcher` stub is already installed (Session 1.1). Update it to actually start the FastAPI app:

```ini
[Unit]
Description=NY&E Dispatcher UI
After=mosquitto.service rr-clock.service

[Service]
WorkingDirectory=/opt/rr_server
ExecStart=/opt/rr_server/venv/bin/uvicorn dispatcher.app:app --host 0.0.0.0 --port 5000
Restart=always
User=abyrne

[Install]
WantedBy=multi-user.target
```

---

## Completion Criteria

- [ ] `http://192.168.10.1:5000` returns the dispatcher page
- [ ] Clock displays current RR time, updates each tick
- [ ] Pause/resume and set-time controls work
- [ ] All 7 station tiles show next NB and next SB train (or "—"); updates on each tick
- [ ] `rr-dispatcher` systemd unit starts at boot

---

## Notes

- **requirements.txt** updated to add `fastapi`, `uvicorn[standard]`, `jinja2` — `setup_venv.sh` / `deploy.sh` will pick these up automatically
- Mosquitto listener is now `192.168.10.1:1883` — dispatcher must connect on that address (already in `config.json`)
- Browser access must be from a device on `NYE_Layout` WiFi (192.168.10.x) or via eth0 (192.168.86.x for dev testing — broker is on layout WiFi, but the web server port 5000 can be accessible via eth0 if `--host 0.0.0.0`)
- Consider binding uvicorn to `0.0.0.0` so the dev machine can reach the UI over eth0 during development, even though the broker is layout-WiFi-only
- `location_by_id()` is available in `common/timetable.py` — use it to get station metadata (name, cyd, show_times) when building tile data
