#!/usr/bin/env python3
"""NY&E Fast Clock Service.

Publishes railroad time over MQTT at a configurable real-time interval.
Subscribers interpolate locally between ticks using the published speed value.

Topics:
  Publish:    trains/clock/time         (QoS 0, retained)
  Subscribe:  trains/clock/control      (QoS 1)
  Subscribe:  trains/clock/sync_request (QoS 0)

Config: ../config.json  (relative to RR_Server root)
State:  clock_state.json (same directory as this file — regenerated at runtime)
"""

import json
import logging
import signal
import sys
import threading
import time
from pathlib import Path

import paho.mqtt.client as mqtt

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger(__name__)

BASE_DIR = Path(__file__).parent.parent   # RR_Server/
CONFIG_FILE = BASE_DIR / "config.json"
STATE_FILE = Path(__file__).parent / "clock_state.json"

TOPIC_TIME = "trains/clock/time"
TOPIC_CONTROL = "trains/clock/control"
TOPIC_SYNC = "trains/clock/sync_request"

CLIENT_ID = "rr_clock"


def _fmt(h: int, m: int) -> str:
    """Return 12-hour AM/PM string matching MQTT spec (e.g. '10:42 AM', '12:00 PM')."""
    if h == 0:
        return f"12:{m:02d} AM"
    if h < 12:
        return f"{h}:{m:02d} AM"
    if h == 12:
        return f"12:{m:02d} PM"
    return f"{h - 12}:{m:02d} PM"


class FastClock:
    def __init__(self, config: dict):
        broker = config["broker"]
        self._host: str = broker["host"]
        self._port: int = broker.get("port", 1883)
        self._password: str = config["credentials"][CLIENT_ID]

        clock_cfg = config.get("clock", {})
        self._speed: int = clock_cfg.get("default_speed", 3)
        self._tick_interval: int = clock_cfg.get("default_tick_interval", 60)

        # Railroad time state
        self._hour: int = 8
        self._minute: int = 0
        self._day: int = 1
        self._running: bool = False

        # Continuous-time tracking: current RR minutes = _rr_base + elapsed_real_s * speed / 60
        self._rr_base: float = 8 * 60.0   # RR minutes at last start/set
        self._real_base: float | None = None  # monotonic time at last start/set

        self._lock = threading.Lock()
        self._timer: threading.Timer | None = None
        self.client: mqtt.Client | None = None

        self._load_state()

    # ── Persistence ─────────────────────────────────────────────────────────

    def _load_state(self) -> None:
        try:
            state = json.loads(STATE_FILE.read_text())
            self._hour = state.get("hour", self._hour)
            self._minute = state.get("minute", self._minute)
            self._day = state.get("day", self._day)
            self._speed = state.get("speed", self._speed)
            self._tick_interval = state.get("tick_interval", self._tick_interval)
            self._rr_base = self._hour * 60.0 + self._minute
            self._running = False  # always paused after restart
            log.info("Loaded state: %s day=%d", _fmt(self._hour, self._minute), self._day)
        except (FileNotFoundError, json.JSONDecodeError):
            log.info("No state file — using defaults")

    def _save_state(self) -> None:
        STATE_FILE.write_text(json.dumps({
            "hour": self._hour,
            "minute": self._minute,
            "day": self._day,
            "speed": self._speed,
            "running": self._running,
            "tick_interval": self._tick_interval,
        }, indent=2))

    # ── Time helpers ─────────────────────────────────────────────────────────

    def _current_rr_mins(self) -> float:
        """RR minutes since midnight, accounting for elapsed real time."""
        if not self._running or self._real_base is None:
            return self._rr_base
        elapsed = time.monotonic() - self._real_base
        return self._rr_base + (elapsed / 60.0) * self._speed

    def _decompose(self, rr_mins: float) -> tuple[int, int, int]:
        """Break RR minutes into (hour, minute, day) with midnight rollover."""
        days_passed = int(rr_mins) // (24 * 60)
        today_mins = rr_mins % (24 * 60)
        day = ((self._day - 1 + days_passed) % 7) + 1
        return int(today_mins) // 60, int(today_mins) % 60, day

    def _commit(self) -> None:
        """Capture current time into state fields and reset the tracking baseline."""
        h, m, day = self._decompose(self._current_rr_mins())
        self._hour, self._minute, self._day = h, m, day
        self._rr_base = h * 60.0 + m
        if self._running:
            self._real_base = time.monotonic()

    def _build_payload(self) -> str:
        h, m, day = self._decompose(self._current_rr_mins())
        return json.dumps({
            "time": _fmt(h, m),
            "hour": h,
            "minute": m,
            "day": day,
            "speed": self._speed,
            "running": self._running,
        })

    def _publish(self) -> None:
        """Publish current time. Must be called with _lock held."""
        self.client.publish(TOPIC_TIME, self._build_payload(), qos=0, retain=True)

    # ── Timer ────────────────────────────────────────────────────────────────

    def _cancel_timer(self) -> None:
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None

    def _schedule_tick(self) -> None:
        self._cancel_timer()
        if self._running:
            self._timer = threading.Timer(self._tick_interval, self._on_tick)
            self._timer.daemon = True
            self._timer.start()

    def _on_tick(self) -> None:
        with self._lock:
            if not self._running:
                return
            self._commit()
            self._save_state()
            self._publish()
            log.info("Tick: %s day=%d", _fmt(self._hour, self._minute), self._day)
            self._schedule_tick()

    # ── Control actions ──────────────────────────────────────────────────────

    def handle_control(self, payload: dict) -> None:
        action = payload.get("action")
        with self._lock:
            if action == "start":
                if not self._running:
                    self._running = True
                    self._rr_base = self._hour * 60.0 + self._minute
                    self._real_base = time.monotonic()
                    self._save_state()
                    self._publish()
                    self._schedule_tick()
                    log.info("Started: %s day=%d", _fmt(self._hour, self._minute), self._day)

            elif action == "pause":
                if self._running:
                    self._commit()
                    self._running = False
                    self._cancel_timer()
                    self._save_state()
                    self._publish()
                    log.info("Paused: %s day=%d", _fmt(self._hour, self._minute), self._day)

            elif action == "set":
                was_running = self._running
                if was_running:
                    self._cancel_timer()
                    self._running = False
                self._hour = max(0, min(23, payload.get("hour", self._hour)))
                self._minute = max(0, min(59, payload.get("minute", self._minute)))
                self._day = max(1, min(7, payload.get("day", self._day)))
                self._rr_base = self._hour * 60.0 + self._minute
                if was_running:
                    self._running = True
                    self._real_base = time.monotonic()
                    self._schedule_tick()
                self._save_state()
                self._publish()
                log.info("Set: %s day=%d", _fmt(self._hour, self._minute), self._day)

            elif action == "reset":
                self._cancel_timer()
                self._running = False
                self._hour = self._minute = 0
                self._day = 1
                self._rr_base = 0.0
                self._real_base = None
                self._save_state()
                self._publish()
                log.info("Reset")

            elif action == "speed":
                new_speed = max(1, int(payload.get("speed", self._speed)))
                if self._running:
                    self._commit()
                    self._real_base = time.monotonic()
                self._speed = new_speed
                self._save_state()
                self._publish()
                log.info("Speed: %d", self._speed)

            elif action == "set_tick_interval":
                new_interval = max(1, int(payload.get("seconds", self._tick_interval)))
                self._tick_interval = new_interval
                self._save_state()
                if self._running:
                    self._schedule_tick()
                log.info("Tick interval: %ds", self._tick_interval)

            else:
                log.warning("Unknown action: %s", action)

    def handle_sync_request(self, payload: dict) -> None:
        station = payload.get("station_id", "?")
        with self._lock:
            self._publish()
        log.info("Sync: %s", station)

    # ── MQTT callbacks ───────────────────────────────────────────────────────

    def _on_connect(self, client, userdata, flags, reason_code, properties):
        if reason_code.is_failure:
            log.error("MQTT connect failed: %s", reason_code)
            return
        log.info("MQTT connected to %s:%d", self._host, self._port)
        client.subscribe([(TOPIC_CONTROL, 1), (TOPIC_SYNC, 0)])
        with self._lock:
            self._publish()

    def _on_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload)
        except json.JSONDecodeError:
            log.warning("Bad JSON on %s", msg.topic)
            return
        if msg.topic == TOPIC_CONTROL:
            self.handle_control(payload)
        elif msg.topic == TOPIC_SYNC:
            self.handle_sync_request(payload)

    # ── Entry point ──────────────────────────────────────────────────────────

    def run(self) -> None:
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=CLIENT_ID)
        self.client.username_pw_set(CLIENT_ID, self._password)
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message

        log.info("Connecting to %s:%d", self._host, self._port)
        self.client.connect(self._host, self._port, keepalive=60)
        self.client.loop_forever()


def main() -> None:
    try:
        config = json.loads(CONFIG_FILE.read_text())
    except FileNotFoundError:
        log.error("Config not found: %s", CONFIG_FILE)
        sys.exit(1)
    except json.JSONDecodeError as e:
        log.error("Bad config JSON: %s", e)
        sys.exit(1)

    clock = FastClock(config)

    def _shutdown(sig, frame):
        log.info("Shutting down (signal %d)", sig)
        clock._cancel_timer()
        if clock.client:
            clock.client.disconnect()
        sys.exit(0)

    signal.signal(signal.SIGTERM, _shutdown)
    signal.signal(signal.SIGINT, _shutdown)

    clock.run()


if __name__ == "__main__":
    main()
