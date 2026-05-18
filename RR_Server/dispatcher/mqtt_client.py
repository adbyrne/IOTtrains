"""MQTT bridge: subscribes to broker topics, pushes events to WebSocket clients.

Runs paho in a background thread; uses asyncio.run_coroutine_threadsafe to
dispatch events to the asyncio event loop.
"""

import asyncio
import json
import logging
import threading
import time

import paho.mqtt.client as mqtt

from .session import AppState

log = logging.getLogger(__name__)

TOPIC_CLOCK_TIME = "trains/clock/time"
TOPIC_STATION_STATUS = "trains/station/+/status"
TOPIC_TO_STATE = "trains/signal/+/to/+/state"
TOPIC_CLOCK_CONTROL = "trains/clock/control"
TOPIC_OS = "trains/os/+"

CLIENT_ID = "rr_dispatcher"


class MQTTClient:
    def __init__(self, config: dict, state: AppState, build_next_trains):
        broker = config["broker"]
        self._host: str = broker["host"]
        self._port: int = broker.get("port", 1883)
        self._password: str = config["credentials"][CLIENT_ID]
        self._state = state
        self._build_next_trains = build_next_trains
        self._loop: asyncio.AbstractEventLoop | None = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

        self._client = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2, client_id=CLIENT_ID
        )
        self._client.username_pw_set(CLIENT_ID, self._password)
        self._client.on_connect = self._on_connect
        self._client.on_message = self._on_message
        self._client.on_disconnect = self._on_disconnect

    def start(self, loop: asyncio.AbstractEventLoop) -> None:
        self._loop = loop
        self._thread = threading.Thread(target=self._run, daemon=True, name="mqtt")
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        try:
            self._client.disconnect()
        except Exception:
            pass

    def publish_clock_control(self, action: str, **kwargs) -> None:
        payload: dict = {"action": action}
        payload.update(kwargs)
        self._client.publish(TOPIC_CLOCK_CONTROL, json.dumps(payload), qos=1)

    # ── Background thread ────────────────────────────────────────────────────

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                self._client.connect(self._host, self._port, keepalive=60)
                self._client.loop_forever()
            except OSError as e:
                if self._stop.is_set():
                    return
                log.warning("MQTT connect error: %s — retry in 5s", e)
                time.sleep(5)

    # ── MQTT callbacks (paho thread) ─────────────────────────────────────────

    def _on_connect(self, client, userdata, flags, reason_code, properties) -> None:
        if reason_code.is_failure:
            log.error("MQTT connect failed: %s", reason_code)
            return
        log.info("MQTT connected to %s:%d", self._host, self._port)
        client.subscribe([
            (TOPIC_CLOCK_TIME, 0),
            (TOPIC_STATION_STATUS, 1),
            (TOPIC_TO_STATE, 1),
            (TOPIC_OS, 1),
        ])

    def _on_disconnect(self, client, userdata, flags, reason_code, properties) -> None:
        if not self._stop.is_set():
            log.warning("MQTT disconnected (%s)", reason_code)

    def _on_message(self, client, userdata, msg) -> None:
        try:
            payload = json.loads(msg.payload)
        except (json.JSONDecodeError, UnicodeDecodeError):
            log.warning("Bad payload on %s", msg.topic)
            return

        if msg.topic == TOPIC_CLOCK_TIME:
            self._state.clock = payload
            event = {
                "type": "clock_update",
                "clock": payload,
                "next_trains": self._build_next_trains(payload),
            }
        elif msg.topic.startswith("trains/station/") and msg.topic.endswith("/status"):
            parts = msg.topic.split("/")
            if len(parts) == 4:
                station_id = parts[2]
                self._state.stations[station_id] = payload
                event = {
                    "type": "station_status",
                    "station_id": station_id,
                    "status": payload,
                }
            else:
                return
        elif msg.topic.startswith("trains/signal/") and msg.topic.endswith("/state"):
            # trains/signal/{station_id}/to/{dir}/state
            parts = msg.topic.split("/")
            if len(parts) == 6:
                station_id = parts[2]
                direction = parts[4]
                sig_state = payload.get("state")
                self._state.to_signals.setdefault(station_id, {})[direction] = sig_state
                event = {
                    "type": "to_signal_update",
                    "station_id": station_id,
                    "direction": direction,
                    "state": sig_state,
                }
            else:
                return
        elif msg.topic.startswith("trains/os/"):
            from .session import OS_LOG_MAX
            entry = {
                "station_id": payload.get("station_id", "?"),
                "train":      payload.get("train", "?"),
                "section":    payload.get("section", 0),
                "direction":  payload.get("direction", "?"),
                "extra":      payload.get("extra", False),
                "work_extra": payload.get("work_extra", False),
                "rr_time":    payload.get("rr_time", "?"),
                "day":        payload.get("day", 1),
            }
            self._state.os_log.insert(0, entry)
            if len(self._state.os_log) > OS_LOG_MAX:
                self._state.os_log = self._state.os_log[:OS_LOG_MAX]
            event = {"type": "os_report", "entry": entry}
        else:
            return

        if self._loop and not self._stop.is_set():
            asyncio.run_coroutine_threadsafe(
                self._state.broadcast(event), self._loop
            )
