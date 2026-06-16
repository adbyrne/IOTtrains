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
TOPIC_BLOCK_STATE = "trains/signal/+/block/state"
TOPIC_CLOCK_CONTROL = "trains/clock/control"
TOPIC_OS = "trains/os/+"
TOPIC_TO_ACK = "trains/to/+/ack"
TOPIC_TO_TRAIN_RCVD = "trains/to/+/train_rcvd"
TOPIC_YARD_CONSIST = "trains/yard/consist/+"
TOPIC_YARD_NOTIFICATION = "trains/yard/notification"

CLIENT_ID = "rr_dispatcher"


class MQTTClient:
    def __init__(self, config: dict, state: AppState, build_next_trains, get_layout_rules=lambda: {}):
        broker = config["broker"]
        self._host: str = broker["host"]
        self._port: int = broker.get("port", 1883)
        self._password: str = config["credentials"][CLIENT_ID]
        self._state = state
        self._build_next_trains = build_next_trains
        self._get_layout_rules = get_layout_rules
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

    def publish_to_order(self, station_id: str, payload: dict) -> None:
        topic = f"trains/to/{station_id}"
        self._client.publish(topic, json.dumps(payload), qos=2)

    def publish_signal_arm(self, station_id: str, direction: str, arm_state: str) -> None:
        topic = f"trains/signal/{station_id}/to/{direction}/cmd"
        self._client.publish(topic, json.dumps({"state": arm_state}), qos=1, retain=True)

    def publish_block_signal(self, station_id: str, arm_state: str) -> None:
        topic = f"trains/signal/{station_id}/block/cmd"
        self._client.publish(topic, json.dumps({"state": arm_state}), qos=1, retain=True)

    def publish_yard_consist(self, train: str, payload: dict) -> None:
        topic = f"trains/yard/consist/{train}"
        self._client.publish(topic, json.dumps(payload), qos=1, retain=True)

    def publish_yard_notification(self, payload: dict) -> None:
        self._client.publish(TOPIC_YARD_NOTIFICATION, json.dumps(payload), qos=1)

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
            (TOPIC_BLOCK_STATE, 1),
            (TOPIC_OS, 1),
            (TOPIC_TO_ACK, 1),
            (TOPIC_TO_TRAIN_RCVD, 1),
            (TOPIC_YARD_CONSIST, 1),
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
            if self._state.check_day(payload.get("day", 0)):
                log.info("RR day changed to %d — TO sequence reset (Rule 203)", payload.get("day"))
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
        elif msg.topic.startswith("trains/signal/") and msg.topic.endswith("/block/state"):
            # trains/signal/{station_id}/block/state
            parts = msg.topic.split("/")
            if len(parts) != 5:
                return
            station_id = parts[2]
            sig_state = payload.get("state")
            self._state.block_signals[station_id] = sig_state
            event = {
                "type": "block_signal_update",
                "station_id": station_id,
                "state": sig_state,
            }
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

            # Auto-notify YM of arrival when enabled (owner-configurable in
            # /manage, default off — manual Notify YM is the default flow;
            # this just saves the dispatcher a click when turned on).
            if (entry["direction"] == "S" and entry["station_id"] == "XP"
                    and self._get_layout_rules().get("auto_notify_ym_arrival")):
                notif = {
                    "type": "arrival",
                    "train": entry["train"],
                    "direction": "S",
                    "expected_rr_time": entry["rr_time"],
                    "rr_time": entry["rr_time"],
                    "day": entry["day"],
                }
                self._state.record_notification(notif)
                if self._loop and not self._stop.is_set():
                    asyncio.run_coroutine_threadsafe(
                        self._state.broadcast({"type": "yard_notification", "notification": notif}),
                        self._loop,
                    )
        elif msg.topic.startswith("trains/to/") and msg.topic.endswith("/ack"):
            # trains/to/{station_id}/ack
            parts = msg.topic.split("/")
            if len(parts) != 4:
                return
            station_id = parts[2]
            seq = payload.get("seq")
            if seq is None:
                return
            ack_data = {
                "station_id": station_id,
                "rr_time":    payload.get("rr_time", "?"),
                "copies":     payload.get("copies", 2),
            }
            to_entry = self._state.record_ack(seq, station_id, ack_data)
            if to_entry is None:
                return
            event = {
                "type":       "to_ack",
                "seq":        seq,
                "station_id": station_id,
                "ack":        ack_data,
                "all_acked":  all(v is not None for v in to_entry["acks"].values()),
            }
        elif msg.topic.startswith("trains/to/") and msg.topic.endswith("/train_rcvd"):
            # trains/to/{station_id}/train_rcvd
            parts = msg.topic.split("/")
            if len(parts) != 4:
                return
            seq = payload.get("seq")
            train = payload.get("train")
            rr_time = payload.get("rr_time", "?")
            if seq is None or not train:
                return
            to_entry = self._state.record_train_rcvd(seq, train, rr_time)
            if to_entry is None:
                return
            event = {
                "type":    "to_train_rcvd",
                "seq":     seq,
                "train":   train,
                "rr_time": rr_time,
            }
        elif msg.topic.startswith("trains/yard/consist/"):
            parts = msg.topic.split("/")
            if len(parts) != 4:
                return
            train = parts[3]
            entry = self._state.record_consist(train, payload)
            event = {"type": "consist_update", "consist": entry}
        else:
            return

        if self._loop and not self._stop.is_set():
            asyncio.run_coroutine_threadsafe(
                self._state.broadcast(event), self._loop
            )
