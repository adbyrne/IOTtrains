"""NY&E Dispatcher Web App.

FastAPI application serving the dispatcher UI.

Routes:
  GET  /                       — dispatcher page
  GET  /yard                   — yardmaster page
  POST /api/clock/control      — clock commands (start/pause/reset/set/speed)
  POST /api/to/issue           — issue a structured train order
  POST /api/signal/arm         — raise or lower a TO signal arm
  POST /api/yard/consist       — yardmaster submits/updates a consist
  POST /api/yard/notification  — dispatcher sends a yard notification
  POST /api/yard/extra_request — yardmaster requests an extra train
  WS   /ws                     — server-push events (clock_update, station_status, initial_state,
                                 os_report, to_signal_update, to_issued, to_ack, consist_update,
                                 yard_notification, extra_request)
"""

import asyncio
import json
import logging
import secrets
import sys
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Any

from fastapi import Depends, FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse
from fastapi.security import HTTPBasic, HTTPBasicCredentials
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates

BASE_DIR = Path(__file__).parent.parent
sys.path.insert(0, str(BASE_DIR))

from common import timetable
from .session import AppState, STATION_IDS, STATION_NAMES, TO_SIGNAL_STATIONS, BLOCK_SIGNAL_STATIONS
from .mqtt_client import MQTTClient

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s %(name)s %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger(__name__)

CONFIG_FILE = BASE_DIR / "config.json"
TO_TYPES_FILE = BASE_DIR / "data" / "to_types.json"
ACTIVE_FORMS_FILE  = BASE_DIR / "data" / "active_forms.json"
LAYOUT_RULES_FILE  = BASE_DIR / "data" / "layout_rules.json"
YARD_FILE = BASE_DIR / "data" / "yard.json"
ROSTER_FILE = BASE_DIR / "data" / "roster.json"
TEMPLATE_DIR = Path(__file__).parent / "templates"
STATIC_DIR = Path(__file__).parent / "static"
SUBDIVISION = "NLS"
DISPATCHER_VERSION = "2.5"
YARDMASTER_VERSION = "2.0b"
BLOCK_SIGNAL_PULSE_SECONDS = 60  # real seconds a triggered block signal stays "raised" before auto-revert

_YARD_NOTIFICATION_TYPES = {
    "arrival": "train",
    "departure_change": "train",
    "new_train": "engine",
    "annulment": "train",
    "departure_time_set": "engine",
}

_ALL_FORM_LETTERS = set("ABCEFGHJKLMPRSTUVWXYZ")
_basic_security = HTTPBasic()

state = AppState()
mqtt_client: MQTTClient | None = None
to_types: dict = {}       # loaded from to_types.json at startup
active_forms: list = []   # loaded from active_forms.json at startup
layout_rules: dict = {}   # loaded from layout_rules.json at startup
_owner_username: str = ""
_owner_password: str = ""


async def _set_equipment_status(
    kind: str, road_number: str, status: str,
    rr_time: str | None = None, day: int | None = None,
) -> None:
    """Update one equipment entry in state, publish retained MQTT, and broadcast."""
    kind_key = kind + "s"  # "engine" → "engines", "caboose" → "cabooses"
    if road_number not in state.equipment_status.get(kind_key, {}):
        return
    entry = {"status": status, "rr_time": rr_time, "day": day}
    state.equipment_status[kind_key][road_number] = entry
    if mqtt_client is not None:
        mqtt_client.publish_roster_status(kind, road_number, entry)
    await state.broadcast({"type": "equipment_status_update", "kind": kind,
                           "road_number": road_number, "status": entry})


def _require_owner(credentials: HTTPBasicCredentials = Depends(_basic_security)) -> None:
    ok_user = secrets.compare_digest(credentials.username.encode(), _owner_username.encode())
    ok_pass = secrets.compare_digest(credentials.password.encode(), _owner_password.encode())
    if not (ok_user and ok_pass):
        raise HTTPException(
            status_code=401,
            headers={"WWW-Authenticate": 'Basic realm="NY&E Management"'},
        )


def build_next_trains(clock: dict) -> dict[str, dict]:
    """Compute next N and S trains for all stations from current clock state."""
    if not clock:
        return {sid: {"N": None, "S": None} for sid in STATION_IDS}
    hour = clock.get("hour", 0)
    minute = clock.get("minute", 0)
    day = clock.get("day", 1)
    rr_time = f"{hour:02d}:{minute:02d}"
    return {
        sid: {
            "N": timetable.next_train(SUBDIVISION, sid, "N", rr_time, day),
            "S": timetable.next_train(SUBDIVISION, sid, "S", rr_time, day),
        }
        for sid in STATION_IDS
    }


def build_yard_departures(day: int) -> list[dict]:
    """WP-departing (northbound) trains for `day`, merged with live consist state,
    plus any not-yet-cleared extra trains appended after the scheduled ones (§3.3)."""
    scheduled = []
    for train in timetable.active_trains(SUBDIVISION, day):
        if train["direction"] != "N":
            continue
        schedule = train.get("schedule", [])
        if not schedule or schedule[0]["location"] != "WP":
            continue
        number = train["number"]
        scheduled.append({
            "train": number,
            "direction": "N",
            "service": train["service"],
            "class": train["class"],
            "depart": schedule[0].get("depart"),
            "extra": False,
            "consist": state.consists.get(number),
        })
    scheduled.sort(key=lambda t: t["depart"] or "99:99")

    extras = [
        {
            "train": num,
            "direction": c.get("direction", "N"),
            "service": "Extra",
            "class": None,
            "depart": None,
            "extra": True,
            "consist": c,
        }
        for num, c in state.consists.items()
        if c.get("extra") and c.get("state") != "cleared"
    ]
    return scheduled + extras


def build_initial_state() -> dict:
    return {
        "type": "initial_state",
        "clock": state.clock,
        "stations": state.stations,
        "to_signals": state.to_signals,
        "os_log": state.os_log,
        "to_log": state.to_log,
        "to_types": to_types,
        "active_forms": active_forms,
        "layout_rules": layout_rules,
        "next_trains": build_next_trains(state.clock),
        "station_ids": STATION_IDS,
        "station_names": STATION_NAMES,
        "to_signal_stations": list(TO_SIGNAL_STATIONS),
        "block_signal_stations": list(BLOCK_SIGNAL_STATIONS),
        "block_signals": state.block_signals,
        "station_data": timetable.station_display_data(SUBDIVISION, STATION_IDS),
        "extra_times": timetable.inter_station_times(SUBDIVISION, STATION_IDS),
        "consists": state.consists,
        "yard_tracks": state.yard_tracks,
        "roster": state.roster,
        "equipment_status": state.equipment_status,
        "yard_notifications": state.yard_notifications,
        "coe_trains": timetable.coe_schedule(state.clock.get("day") or 1),
        "yard_departures": build_yard_departures(state.clock.get("day") or 1),
    }


def _derive_trains(to_type: str, fields: dict) -> list[str]:
    """Compute the trains[] array from TO type and fields."""
    if to_type == "meet":
        return [str(fields["train_a"]), str(fields["train_b"])]
    if to_type in ("wait", "annulment", "sections"):
        return [str(fields["train"])]
    if to_type in ("running_extra", "work_extra"):
        return [str(fields["engine"])]
    return []


def _validate_to_fields(to_type: str, fields: dict) -> str | None:
    """Return an error message if required fields are missing, else None."""
    type_def = to_types.get("to_types", {}).get(to_type)
    if type_def is None:
        return f"unknown to_type: {to_type!r}"
    for f in type_def.get("fields", []):
        if f.get("required") and f["id"] not in fields:
            return f"missing required field: {f['id']!r}"
    return None


@asynccontextmanager
async def lifespan(app: FastAPI):
    global mqtt_client, to_types, active_forms, layout_rules, _owner_username, _owner_password
    try:
        config = json.loads(CONFIG_FILE.read_text())
    except (FileNotFoundError, json.JSONDecodeError) as e:
        log.error("Config error: %s", e)
        yield
        return
    owner_cfg = config.get("owner", {})
    _owner_username = owner_cfg.get("username", "owner")
    _owner_password = owner_cfg.get("password", "")

    timetable.load(BASE_DIR / "data" / "timetable.json")
    try:
        to_types = json.loads(TO_TYPES_FILE.read_text())
    except (FileNotFoundError, json.JSONDecodeError) as e:
        log.error("to_types.json error: %s", e)
    try:
        active_forms = json.loads(ACTIVE_FORMS_FILE.read_text()).get("active_forms", [])
    except (FileNotFoundError, json.JSONDecodeError):
        active_forms = list("ABCEFGHJKL")
    try:
        layout_rules = json.loads(LAYOUT_RULES_FILE.read_text())
        layout_rules.setdefault("signal_reset_mode", "hard")
        layout_rules.setdefault("auto_notify_ym_arrival", False)
    except (FileNotFoundError, json.JSONDecodeError):
        layout_rules = {"superior_direction": "S", "signal_reset_mode": "hard",
                         "auto_notify_ym_arrival": False}
    try:
        yard_data = json.loads(YARD_FILE.read_text())
        state.yard_tracks = yard_data.get("tracks", [])
    except (FileNotFoundError, json.JSONDecodeError) as e:
        log.error("yard.json error: %s", e)
    try:
        roster_data = json.loads(ROSTER_FILE.read_text())
        state.roster = {
            "engines": roster_data.get("engines", []),
            "cabooses": roster_data.get("cabooses", []),
        }
        state.equipment_status = {
            "engines":  {e["road_number"]: {"status": "available", "rr_time": None, "day": None}
                         for e in state.roster["engines"]},
            "cabooses": {c["road_number"]: {"status": "available", "rr_time": None, "day": None}
                         for c in state.roster["cabooses"]},
        }
    except (FileNotFoundError, json.JSONDecodeError) as e:
        log.error("roster.json error: %s", e)
    state.block_signals = {sid: "lowered" for sid in BLOCK_SIGNAL_STATIONS}
    mqtt_client = MQTTClient(config, state, build_next_trains, lambda: layout_rules)
    mqtt_client.start(asyncio.get_running_loop())
    log.info("Dispatcher started (v%s)", DISPATCHER_VERSION)
    yield
    if mqtt_client:
        mqtt_client.stop()
    log.info("Dispatcher stopped")


app = FastAPI(lifespan=lifespan, title="NY&E Dispatcher")
app.mount("/static", StaticFiles(directory=str(STATIC_DIR)), name="static")
templates = Jinja2Templates(directory=str(TEMPLATE_DIR))


@app.get("/")
async def index(request: Request):
    return templates.TemplateResponse(request, "index.html",
                                      {"version": DISPATCHER_VERSION})


@app.post("/api/clock/control")
async def clock_control(request: Request):
    if mqtt_client is None:
        return JSONResponse({"error": "MQTT not connected"}, status_code=503)
    body = await request.json()
    action = body.get("action")
    if not action:
        return JSONResponse({"error": "missing action"}, status_code=400)

    kwargs: dict[str, Any] = {}
    if action == "reset":
        state.reset_seq()
        state.current_day = 0
    elif action == "set":
        for field in ("hour", "minute", "day"):
            if field in body:
                kwargs[field] = body[field]
    elif action == "speed":
        if "speed" not in body:
            return JSONResponse({"error": "missing speed"}, status_code=400)
        kwargs["speed"] = body["speed"]

    mqtt_client.publish_clock_control(action, **kwargs)
    return JSONResponse({"ok": True})


@app.post("/api/to/issue")
async def to_issue(request: Request):
    if mqtt_client is None:
        return JSONResponse({"error": "MQTT not connected"}, status_code=503)
    body = await request.json()

    to_type = body.get("to_type", "")
    fields  = body.get("fields", {})
    addressed_to = body.get("addressed_to", [])

    err = _validate_to_fields(to_type, fields)
    if err:
        return JSONResponse({"error": err}, status_code=400)
    if not addressed_to:
        return JSONResponse({"error": "addressed_to must not be empty"}, status_code=400)
    invalid_stations = [s for s in addressed_to if s not in STATION_IDS]
    if invalid_stations:
        return JSONResponse({"error": f"unknown station(s): {invalid_stations}"}, status_code=400)

    clock = state.clock
    seq = state.next_seq()
    form = body.get("form") or to_types.get("default_form", "19")
    trains = _derive_trains(to_type, fields)

    payload = {
        "seq":            seq,
        "form":           form,
        "to_type":        to_type,
        "trains":         trains,
        "addressed_to":   addressed_to,
        "issued_rr_time": f"{clock.get('hour', 0):02d}:{clock.get('minute', 0):02d}",
        "day":            clock.get("day", 1),
        "fields":         fields,
    }

    for station_id in addressed_to:
        mqtt_client.publish_to_order(station_id, payload)

    to_entry = {k: v for k, v in payload.items()}
    state.record_to(to_entry)

    event = {"type": "to_issued", "to": to_entry}
    await state.broadcast(event)

    if to_type == "running_extra":
        await _set_extra_departure_time(fields.get("engine"), fields.get("departure_rr_time"))

    log.info("TO #%d issued: %s → %s", seq, to_type, addressed_to)
    return JSONResponse({"ok": True, "seq": seq})


async def _set_extra_departure_time(engine: str | None, departure_rr_time: str | None) -> None:
    """Issuing a running_extra TO is when the dispatcher commits to an approximate
    departure time for that extra — update the matching consist record (found by
    engine number) so the Yardmaster's Departing Trains entry shows it instead of
    the original request time."""
    if not engine or not departure_rr_time or mqtt_client is None:
        return
    for train_id, c in state.consists.items():
        if c.get("extra") and str(c.get("engine")) == str(engine):
            updated = dict(c)
            updated["departure_rr_time"] = departure_rr_time
            entry = state.record_consist(train_id, updated)
            mqtt_client.publish_yard_consist(train_id, entry)
            await state.broadcast({"type": "consist_update", "consist": entry})
            break


@app.post("/api/signal/arm")
async def signal_arm(request: Request):
    if mqtt_client is None:
        return JSONResponse({"error": "MQTT not connected"}, status_code=503)
    body = await request.json()

    station_id = body.get("station_id", "")
    direction  = body.get("direction", "")
    arm_state  = body.get("state", "")

    if station_id not in TO_SIGNAL_STATIONS:
        return JSONResponse({"error": f"{station_id!r} has no TO signal arm"}, status_code=400)
    if direction not in ("N", "S"):
        return JSONResponse({"error": "direction must be N or S"}, status_code=400)
    if arm_state not in ("raised", "lowered"):
        return JSONResponse({"error": "state must be raised or lowered"}, status_code=400)

    mqtt_client.publish_signal_arm(station_id, direction, arm_state)

    # Optimistic update — reflect the command in state immediately so the UI
    # updates now; the firmware's state-topic echo will override this if it
    # reports a different value when it reconnects.
    state.to_signals.setdefault(station_id, {})[direction] = arm_state
    await state.broadcast({
        "type": "to_signal_update",
        "station_id": station_id,
        "direction": direction,
        "state": arm_state,
    })

    return JSONResponse({"ok": True})


@app.post("/api/signal/block")
async def signal_block(request: Request):
    """Trigger a momentary block-signal clear pulse.

    The signal is normally "lowered" (stop). A trigger raises it for
    BLOCK_SIGNAL_PULSE_SECONDS real seconds, then it auto-reverts. There is
    no manual "lowered" command — only a trigger. A trigger while a pulse is
    already active for that station is ignored (409).
    """
    if mqtt_client is None:
        return JSONResponse({"error": "MQTT not connected"}, status_code=503)
    body = await request.json()

    station_id = body.get("station_id", "")

    if station_id not in BLOCK_SIGNAL_STATIONS:
        return JSONResponse({"error": f"{station_id!r} has no block signal"}, status_code=400)
    if state.block_signals.get(station_id) == "raised":
        return JSONResponse({"error": "pulse already active"}, status_code=409)

    async def _set(arm_state: str) -> None:
        mqtt_client.publish_block_signal(station_id, arm_state)
        state.block_signals[station_id] = arm_state
        await state.broadcast({
            "type": "block_signal_update",
            "station_id": station_id,
            "state": arm_state,
        })

    async def _pulse() -> None:
        try:
            await asyncio.sleep(BLOCK_SIGNAL_PULSE_SECONDS)
            await _set("lowered")
        finally:
            state.block_signal_tasks.pop(station_id, None)

    await _set("raised")
    state.block_signal_tasks[station_id] = asyncio.create_task(_pulse())

    return JSONResponse({"ok": True})


@app.get("/yard")
async def yard_page(request: Request):
    return templates.TemplateResponse(request, "yard.html",
                                      {"version": YARDMASTER_VERSION})


@app.post("/api/yard/consist")
async def yard_consist(request: Request):
    if mqtt_client is None:
        return JSONResponse({"error": "MQTT not connected"}, status_code=503)
    body = await request.json()

    train = body.get("train")
    consist_state = body.get("state", "")
    extra = bool(body.get("extra", False))

    if not train:
        return JSONResponse({"error": "missing train"}, status_code=422)
    if consist_state not in ("assembling", "car_block_ready", "ready"):
        return JSONResponse(
            {"error": "state must be assembling, car_block_ready, or ready"}, status_code=422
        )
    if consist_state == "car_block_ready" and not extra:
        return JSONResponse(
            {"error": "car_block_ready is only valid for extra trains"}, status_code=422
        )
    if consist_state == "ready" and (not body.get("engine") or not body.get("caboose")):
        return JSONResponse(
            {"error": "engine and caboose are required to mark a consist ready"}, status_code=422
        )

    payload = {k: v for k, v in body.items() if k != "train"}
    payload["state"] = consist_state
    payload["extra"] = extra

    mqtt_client.publish_yard_consist(train, payload)
    entry = state.record_consist(train, payload)
    await state.broadcast({"type": "consist_update", "consist": entry})

    # Lock engine and caboose as soon as they're assigned to any open consist.
    clock = state.clock
    rr_time_str = f"{clock.get('hour', 0):02d}:{clock.get('minute', 0):02d}"
    day_val = clock.get("day")
    for field_key, kind in (("engine", "engine"), ("caboose", "caboose")):
        rn = str(body.get(field_key) or "")
        kind_key = kind + "s"
        if (rn and rn in state.equipment_status.get(kind_key, {})
                and state.equipment_status[kind_key][rn]["status"] == "available"):
            await _set_equipment_status(kind, rn, "out", rr_time_str, day_val)

    return JSONResponse({"ok": True, "consist": entry})


@app.post("/api/yard/notification")
async def yard_notification(request: Request):
    if mqtt_client is None:
        return JSONResponse({"error": "MQTT not connected"}, status_code=503)
    body = await request.json()

    notif_type = body.get("type", "")
    key_field = _YARD_NOTIFICATION_TYPES.get(notif_type)
    if key_field is None:
        return JSONResponse({"error": f"unknown notification type: {notif_type!r}"}, status_code=422)
    if not body.get(key_field):
        return JSONResponse({"error": f"missing {key_field!r}"}, status_code=422)

    clock = state.clock
    entry = dict(body)
    entry["rr_time"] = f"{clock.get('hour', 0):02d}:{clock.get('minute', 0):02d}"
    entry["day"] = clock.get("day", 1)

    mqtt_client.publish_yard_notification(entry)
    state.record_notification(entry)
    await state.broadcast({"type": "yard_notification", "notification": entry})

    return JSONResponse({"ok": True})


@app.post("/api/yard/extra_request")
async def yard_extra_request(request: Request):
    if mqtt_client is None:
        return JSONResponse({"error": "MQTT not connected"}, status_code=503)
    body = await request.json()
    direction = body.get("direction", "")
    if direction not in ("N", "S"):
        return JSONResponse({"error": "direction must be N or S"}, status_code=422)

    clock = state.clock
    rr_time = f"{clock.get('hour', 0):02d}:{clock.get('minute', 0):02d}"

    # Placeholder consist so the YM can start building immediately — no engine
    # number exists yet. Identified as "XTRA{n}" until the YM enters one; see
    # occupantTrainId() in yard.js. The dispatcher only issues the running-extra
    # TO once the YM separately notifies them the full consist (incl. engine +
    # caboose) is ready — see /api/to/issue's running_extra handling below.
    placeholder_id = f"XTRA{state.next_extra_seq()}"
    consist_payload = {
        "state": "assembling",
        "extra": True,
        "direction": direction,
        "cars_loaded": body.get("approx_loads"),
        "cars_empty": body.get("approx_empties"),
        "requested_rr_time": rr_time,
    }
    mqtt_client.publish_yard_consist(placeholder_id, consist_payload)
    consist_entry = state.record_consist(placeholder_id, consist_payload)
    await state.broadcast({"type": "consist_update", "consist": consist_entry})

    event = {
        "type": "extra_request",
        "direction": direction,
        "approx_loads": body.get("approx_loads"),
        "approx_empties": body.get("approx_empties"),
        "rr_time": rr_time,
        "train": placeholder_id,
    }
    await state.broadcast(event)
    return JSONResponse({"ok": True, "train": placeholder_id})


@app.get("/manage")
async def manage_page(request: Request, _=Depends(_require_owner)):
    return templates.TemplateResponse(request, "manage.html", {})


@app.get("/api/active_forms")
async def get_active_forms(_=Depends(_require_owner)):
    return JSONResponse({"active_forms": active_forms})


@app.post("/api/active_forms")
async def post_active_forms(request: Request, _=Depends(_require_owner)):
    global active_forms
    body = await request.json()
    forms = body.get("active_forms", [])
    invalid = [f for f in forms if f not in _ALL_FORM_LETTERS]
    if invalid:
        return JSONResponse({"error": f"unknown form letters: {invalid}"}, status_code=400)
    active_forms = forms
    ACTIVE_FORMS_FILE.write_text(json.dumps({"active_forms": active_forms}, indent=2))
    return JSONResponse({"ok": True, "active_forms": active_forms})


@app.get("/api/layout_rules")
async def get_layout_rules(_=Depends(_require_owner)):
    return JSONResponse(layout_rules)


@app.post("/api/layout_rules")
async def post_layout_rules(request: Request, _=Depends(_require_owner)):
    global layout_rules
    body = await request.json()
    sup_dir = body.get("superior_direction", "")
    reset_mode = body.get("signal_reset_mode", "hard")
    auto_notify = bool(body.get("auto_notify_ym_arrival", False))
    if sup_dir not in ("N", "S"):
        return JSONResponse({"error": "superior_direction must be N or S"}, status_code=400)
    if reset_mode not in ("hard", "soft"):
        return JSONResponse({"error": "signal_reset_mode must be hard or soft"}, status_code=400)
    layout_rules = {"superior_direction": sup_dir, "signal_reset_mode": reset_mode,
                     "auto_notify_ym_arrival": auto_notify}
    LAYOUT_RULES_FILE.write_text(json.dumps(layout_rules, indent=2))
    return JSONResponse({"ok": True, "layout_rules": layout_rules})


@app.get("/api/timetable")
async def get_timetable(_=Depends(_require_owner)):
    data = timetable.get()
    if data is None:
        return JSONResponse({"error": "timetable not loaded"}, status_code=503)
    return JSONResponse(data)


@app.post("/api/timetable/meta")
async def post_timetable_meta(request: Request, _=Depends(_require_owner)):
    body = await request.json()
    data = timetable.get()
    if data is None:
        return JSONResponse({"error": "timetable not loaded"}, status_code=503)
    import copy
    data = copy.deepcopy(data)
    meta = data.setdefault("timetable", {})
    for field in ("number", "title", "effective"):
        if field in body:
            meta[field] = body[field]
    if "notes" in body:
        if not isinstance(body["notes"], list):
            return JSONResponse({"error": "notes must be a list"}, status_code=400)
        meta["notes"] = body["notes"]
    try:
        timetable.save(data)
    except Exception as e:
        return JSONResponse({"error": str(e)}, status_code=500)
    return JSONResponse({"ok": True, "timetable": data["timetable"]})


@app.post("/api/timetable/train")
async def post_timetable_train(request: Request, _=Depends(_require_owner)):
    body = await request.json()
    subdivision_id = body.get("subdivision", SUBDIVISION)
    number = body.get("number", "")
    direction = body.get("direction", "")
    if not number or direction not in ("N", "S"):
        return JSONResponse({"error": "number and direction (N/S) required"}, status_code=400)
    data = timetable.get()
    if data is None:
        return JSONResponse({"error": "timetable not loaded"}, status_code=503)
    import copy
    data = copy.deepcopy(data)
    for sub in data["subdivisions"]:
        if sub["id"] == subdivision_id:
            trains = sub["trains"]
            # Upsert: replace existing train with same number+direction, else append
            for i, t in enumerate(trains):
                if t["number"] == number and t["direction"] == direction:
                    trains[i] = body
                    break
            else:
                trains.append(body)
            break
    else:
        return JSONResponse({"error": f"subdivision {subdivision_id!r} not found"}, status_code=400)
    try:
        timetable.save(data)
    except Exception as e:
        return JSONResponse({"error": str(e)}, status_code=500)
    return JSONResponse({"ok": True})


@app.delete("/api/timetable/train")
async def delete_timetable_train(request: Request, _=Depends(_require_owner)):
    body = await request.json()
    subdivision_id = body.get("subdivision", SUBDIVISION)
    number = body.get("number", "")
    direction = body.get("direction", "")
    data = timetable.get()
    if data is None:
        return JSONResponse({"error": "timetable not loaded"}, status_code=503)
    import copy
    data = copy.deepcopy(data)
    for sub in data["subdivisions"]:
        if sub["id"] == subdivision_id:
            before = len(sub["trains"])
            sub["trains"] = [t for t in sub["trains"]
                             if not (t["number"] == number and t["direction"] == direction)]
            if len(sub["trains"]) == before:
                return JSONResponse({"error": "train not found"}, status_code=404)
            break
    try:
        timetable.save(data)
    except Exception as e:
        return JSONResponse({"error": str(e)}, status_code=500)
    return JSONResponse({"ok": True})


@app.post("/api/yard/caboose_paperwork")
async def caboose_paperwork(request: Request):
    body = await request.json()
    road_number = str(body.get("road_number", ""))
    if not road_number:
        return JSONResponse({"error": "missing road_number"}, status_code=400)
    entry = state.equipment_status.get("cabooses", {}).get(road_number)
    if entry is None:
        return JSONResponse({"error": "unknown road_number"}, status_code=404)
    if entry["status"] != "awaiting_paperwork":
        return JSONResponse({"error": "caboose is not awaiting_paperwork"}, status_code=404)
    clock = state.clock
    await _set_equipment_status(
        "caboose", road_number, "available",
        f"{clock.get('hour', 0):02d}:{clock.get('minute', 0):02d}", clock.get("day"),
    )
    return JSONResponse({"ok": True})


@app.post("/api/hostler/roundhouse")
async def hostler_roundhouse(request: Request):
    body = await request.json()
    road_number = str(body.get("road_number", ""))
    if not road_number:
        return JSONResponse({"error": "missing road_number"}, status_code=400)
    entry = state.equipment_status.get("engines", {}).get(road_number)
    if entry is None:
        return JSONResponse({"error": "unknown road_number"}, status_code=404)
    if entry["status"] != "being_serviced":
        return JSONResponse({"error": "engine is not being_serviced"}, status_code=404)
    clock = state.clock
    await _set_equipment_status(
        "engine", road_number, "available",
        f"{clock.get('hour', 0):02d}:{clock.get('minute', 0):02d}", clock.get("day"),
    )
    return JSONResponse({"ok": True})


@app.get("/api/equipment_status")
async def get_equipment_status(_=Depends(_require_owner)):
    return JSONResponse({"equipment_status": state.equipment_status})


@app.post("/api/equipment_status")
async def post_equipment_status(request: Request, _=Depends(_require_owner)):
    body = await request.json()
    kind = body.get("kind", "")
    road_number = str(body.get("road_number", ""))
    out_of_service = bool(body.get("out_of_service", False))
    if kind not in ("engine", "caboose"):
        return JSONResponse({"error": "kind must be engine or caboose"}, status_code=400)
    kind_key = kind + "s"
    if road_number not in state.equipment_status.get(kind_key, {}):
        return JSONResponse({"error": "unknown road_number"}, status_code=404)
    new_status = "out_of_service" if out_of_service else "available"
    await _set_equipment_status(kind, road_number, new_status)
    return JSONResponse({"ok": True})


@app.post("/api/timetable/reload")
async def post_timetable_reload(_=Depends(_require_owner)):
    try:
        timetable.reload()
    except Exception as e:
        return JSONResponse({"error": str(e)}, status_code=500)
    return JSONResponse({"ok": True})


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await state.connect(websocket)
    try:
        await websocket.send_json(build_initial_state())
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
        pass
    finally:
        state.disconnect(websocket)
