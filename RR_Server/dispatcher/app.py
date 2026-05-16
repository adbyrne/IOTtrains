"""NY&E Dispatcher Web App.

FastAPI application serving the dispatcher UI.

Routes:
  GET  /           — dispatcher page
  POST /api/clock/control   — clock commands (start/pause/reset/set/speed)
  WS   /ws         — server-push events (clock_update, station_status, initial_state)
"""

import asyncio
import json
import logging
import sys
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Any

from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse
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
TEMPLATE_DIR = Path(__file__).parent / "templates"
STATIC_DIR = Path(__file__).parent / "static"
SUBDIVISION = "NLS"

state = AppState()
mqtt_client: MQTTClient | None = None


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
        "next_trains": build_next_trains(state.clock),
        "station_ids": STATION_IDS,
        "station_names": STATION_NAMES,
        "to_signal_stations": list(TO_SIGNAL_STATIONS),
    }


@asynccontextmanager
async def lifespan(app: FastAPI):
    global mqtt_client
    try:
        config = json.loads(CONFIG_FILE.read_text())
    except (FileNotFoundError, json.JSONDecodeError) as e:
        log.error("Config error: %s", e)
        yield
        return

    timetable.load(BASE_DIR / "data" / "timetable.json")
    mqtt_client = MQTTClient(config, state, build_next_trains)
    mqtt_client.start(asyncio.get_running_loop())
    log.info("Dispatcher started")
    yield
    if mqtt_client:
        mqtt_client.stop()
    log.info("Dispatcher stopped")


app = FastAPI(lifespan=lifespan, title="NY&E Dispatcher")
app.mount("/static", StaticFiles(directory=str(STATIC_DIR)), name="static")
templates = Jinja2Templates(directory=str(TEMPLATE_DIR))


@app.get("/")
async def index(request: Request):
    return templates.TemplateResponse(request, "index.html")


@app.post("/api/clock/control")
async def clock_control(request: Request):
    if mqtt_client is None:
        return JSONResponse({"error": "MQTT not connected"}, status_code=503)
    body = await request.json()
    action = body.get("action")
    if not action:
        return JSONResponse({"error": "missing action"}, status_code=400)

    kwargs: dict[str, Any] = {}
    if action == "set":
        for field in ("hour", "minute", "day"):
            if field in body:
                kwargs[field] = body[field]
    elif action == "speed":
        if "speed" not in body:
            return JSONResponse({"error": "missing speed"}, status_code=400)
        kwargs["speed"] = body["speed"]

    mqtt_client.publish_clock_control(action, **kwargs)
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
