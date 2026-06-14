"""NY&E Dispatcher Web App.

FastAPI application serving the dispatcher UI.

Routes:
  GET  /                    — dispatcher page
  POST /api/clock/control   — clock commands (start/pause/reset/set/speed)
  POST /api/to/issue        — issue a structured train order
  POST /api/signal/arm      — raise or lower a TO signal arm
  WS   /ws                  — server-push events (clock_update, station_status, initial_state,
                              os_report, to_signal_update, to_issued, to_ack)
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
from .session import AppState, STATION_IDS, STATION_NAMES, TO_SIGNAL_STATIONS
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
TEMPLATE_DIR = Path(__file__).parent / "templates"
STATIC_DIR = Path(__file__).parent / "static"
SUBDIVISION = "NLS"
DISPATCHER_VERSION = "2.3"

_ALL_FORM_LETTERS = set("ABCEFGHJKLMPRSTUVWXYZ")
_basic_security = HTTPBasic()

state = AppState()
mqtt_client: MQTTClient | None = None
to_types: dict = {}       # loaded from to_types.json at startup
active_forms: list = []   # loaded from active_forms.json at startup
layout_rules: dict = {}   # loaded from layout_rules.json at startup
_owner_username: str = ""
_owner_password: str = ""


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
        "station_data": timetable.station_display_data(SUBDIVISION, STATION_IDS),
        "extra_times": timetable.inter_station_times(SUBDIVISION, STATION_IDS),
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
    except (FileNotFoundError, json.JSONDecodeError):
        layout_rules = {"superior_direction": "S", "signal_reset_mode": "hard"}
    mqtt_client = MQTTClient(config, state, build_next_trains)
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

    log.info("TO #%d issued: %s → %s", seq, to_type, addressed_to)
    return JSONResponse({"ok": True, "seq": seq})


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
    if sup_dir not in ("N", "S"):
        return JSONResponse({"error": "superior_direction must be N or S"}, status_code=400)
    if reset_mode not in ("hard", "soft"):
        return JSONResponse({"error": "signal_reset_mode must be hard or soft"}, status_code=400)
    layout_rules = {"superior_direction": sup_dir, "signal_reset_mode": reset_mode}
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
