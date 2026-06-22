"""Unit tests for the WP Yardmaster Terminal backend (Session 2.0a).

Same approach as test_dispatcher.py: MQTTClient is patched for HTTP-level
tests; a real MQTTClient instance with loop=None is used to inject simulated
MQTT messages for state-mutation tests. No broker required.
"""

import json
import sys
from pathlib import Path
from unittest.mock import MagicMock, patch

import pytest
from starlette.testclient import TestClient

sys.path.insert(0, str(Path(__file__).parent.parent))

from common import timetable
from dispatcher.app import app
from dispatcher.session import AppState

DATA = Path(__file__).parent.parent / "data" / "timetable.json"
CONFIG_EXAMPLE = Path(__file__).parent.parent / "config.json.example"


@pytest.fixture(scope="module", autouse=True)
def loaded_timetable():
    timetable.load(DATA)


@pytest.fixture()
def client():
    with patch("dispatcher.app.MQTTClient") as mock_cls, \
         patch("dispatcher.app.CONFIG_FILE", CONFIG_EXAMPLE):
        mock_cls.return_value = MagicMock()
        with TestClient(app) as c:
            yield c


def _make_real_mqtt_client(state):
    from dispatcher.mqtt_client import MQTTClient
    config = {
        "broker": {"host": "127.0.0.1", "port": 1883},
        "credentials": {"rr_dispatcher": "test"},
    }
    return MQTTClient(config, state, lambda c: {})


def _inject_consist(mqtt_obj, train: str, payload: dict) -> None:
    msg = MagicMock()
    msg.topic = f"trains/yard/consist/{train}"
    msg.payload = json.dumps(payload).encode()
    mqtt_obj._on_message(None, None, msg)


# ── /yard page ──────────────────────────────────────────────────────────────

class TestYardPage:
    def test_yard_page_loads(self, client):
        r = client.get("/yard")
        assert r.status_code == 200


# ── /api/yard/consist ─────────────────────────────────────────────────────────

class TestYardConsist:
    def test_consist_assembling(self, client):
        r = client.post("/api/yard/consist", json={
            "train": "3", "state": "assembling", "cars_loaded": 5, "cars_empty": 2,
            "track_id": "T1", "extra": False,
        })
        assert r.status_code == 200
        assert r.json()["ok"] is True
        from dispatcher.app import state as app_state
        assert app_state.consists["3"]["state"] == "assembling"

    def test_consist_car_block_ready(self, client):
        r = client.post("/api/yard/consist", json={
            "train": "42", "state": "car_block_ready", "cars_loaded": 8,
            "cars_empty": 3, "track_id": "T3", "extra": True,
        })
        assert r.status_code == 200
        from dispatcher.app import state as app_state
        assert app_state.consists["42"]["state"] == "car_block_ready"

    def test_consist_ready_scheduled(self, client):
        r = client.post("/api/yard/consist", json={
            "train": "21", "state": "ready", "engine": "101", "caboose": "204",
            "cars_loaded": 12, "cars_empty": 5, "track_id": "T2", "extra": False,
        })
        assert r.status_code == 200
        from dispatcher.app import state as app_state
        entry = app_state.consists["21"]
        assert entry["state"] == "ready"
        assert entry["cars_total"] == 17

    def test_consist_ready_missing_engine(self, client):
        r = client.post("/api/yard/consist", json={
            "train": "7", "state": "ready", "caboose": "204", "extra": False,
        })
        assert r.status_code == 422

    def test_consist_car_block_ready_requires_extra(self, client):
        r = client.post("/api/yard/consist", json={
            "train": "8", "state": "car_block_ready", "extra": False,
        })
        assert r.status_code == 422

    def test_consist_ready_extra_two_stage(self, client):
        # Stage 1: car block assembled
        r1 = client.post("/api/yard/consist", json={
            "train": "55", "state": "car_block_ready", "cars_loaded": 6,
            "cars_empty": 1, "track_id": "T4", "extra": True,
        })
        assert r1.status_code == 200
        from dispatcher.app import state as app_state
        assert app_state.consists["55"]["state"] == "car_block_ready"
        assert "engine" not in app_state.consists["55"] or not app_state.consists["55"].get("engine")

        # Stage 2: engine + caboose assigned, marked ready
        r2 = client.post("/api/yard/consist", json={
            "train": "55", "state": "ready", "engine": "303", "caboose": "404",
            "cars_loaded": 6, "cars_empty": 1, "track_id": "T4", "extra": True,
        })
        assert r2.status_code == 200
        assert app_state.consists["55"]["state"] == "ready"
        assert app_state.consists["55"]["engine"] == "303"

    def test_consist_invalid_state_is_422(self, client):
        r = client.post("/api/yard/consist", json={"train": "9", "state": "bogus"})
        assert r.status_code == 422

    def test_consist_missing_train_is_422(self, client):
        r = client.post("/api/yard/consist", json={"state": "assembling"})
        assert r.status_code == 422


# ── /api/yard/notification ────────────────────────────────────────────────────

class TestYardNotification:
    def test_notification_arrival(self, client):
        r = client.post("/api/yard/notification", json={
            "type": "arrival", "train": "7", "section": 0,
            "direction": "S", "expected_rr_time": "10:15",
        })
        assert r.status_code == 200
        from dispatcher.app import state as app_state
        assert app_state.yard_notifications[0]["type"] == "arrival"

    def test_notification_departure_change(self, client):
        r = client.post("/api/yard/notification", json={
            "type": "departure_change", "train": "3", "new_departure_rr_time": "16:15",
        })
        assert r.status_code == 200

    def test_notification_annulment(self, client):
        r = client.post("/api/yard/notification", json={"type": "annulment", "train": "24"})
        assert r.status_code == 200

    def test_notification_departure_time_set(self, client):
        r = client.post("/api/yard/notification", json={
            "type": "departure_time_set", "engine": "42", "departure_rr_time": "11:30",
        })
        assert r.status_code == 200
        from dispatcher.app import state as app_state
        assert app_state.yard_notifications[0]["type"] == "departure_time_set"

    def test_notification_unknown_type_is_422(self, client):
        r = client.post("/api/yard/notification", json={"type": "bogus", "train": "1"})
        assert r.status_code == 422

    def test_notification_missing_key_field_is_422(self, client):
        r = client.post("/api/yard/notification", json={"type": "arrival"})
        assert r.status_code == 422


# ── /api/yard/extra_request ───────────────────────────────────────────────────

class TestYardExtraRequest:
    def test_extra_request(self, client):
        r = client.post("/api/yard/extra_request", json={
            "direction": "N", "approx_loads": 8, "approx_empties": 3,
        })
        assert r.status_code == 200
        body = r.json()
        assert body["ok"] is True
        assert body["train"].startswith("XTRA")

    def test_extra_request_invalid_direction_is_422(self, client):
        r = client.post("/api/yard/extra_request", json={"direction": "E"})
        assert r.status_code == 422

    def test_extra_request_creates_placeholder_consist(self, client):
        r = client.post("/api/yard/extra_request", json={
            "direction": "S", "approx_loads": 4, "approx_empties": 1,
        })
        train_id = r.json()["train"]
        from dispatcher.app import state as app_state
        entry = app_state.consists[train_id]
        assert entry["extra"] is True
        assert entry["state"] == "assembling"
        assert entry["direction"] == "S"
        assert "requested_rr_time" in entry
        assert "engine" not in entry or not entry.get("engine")

    def test_running_extra_to_updates_matching_consist_departure_time(self, client):
        # Build a placeholder, then assign it an engine via the normal consist flow.
        r = client.post("/api/yard/extra_request", json={
            "direction": "N", "approx_loads": 5, "approx_empties": 2,
        })
        train_id = r.json()["train"]
        client.post("/api/yard/consist", json={
            "train": train_id, "state": "ready", "extra": True, "direction": "N",
            "engine": "909", "caboose": "111", "cars_loaded": 5, "cars_empty": 2,
            "track_id": "T4",
        })
        client.post("/api/to/issue", json={
            "to_type": "running_extra",
            "fields": {
                "engine": "909", "direction": "N", "from_station": "WP",
                "to_station": "HC", "departure_rr_time": "13:15",
            },
            "addressed_to": ["WP"],
        })
        from dispatcher.app import state as app_state
        assert app_state.consists[train_id]["departure_rr_time"] == "13:15"


# ── initial_state ─────────────────────────────────────────────────────────────

class TestYardInitialState:
    def test_yard_initial_state(self, client):
        with client.websocket_connect("/ws") as ws:
            data = ws.receive_json()
        assert "consists" in data
        assert "yard_tracks" in data
        assert "yard_notifications" in data
        assert "coe_trains" in data
        assert "yard_departures" in data
        assert "os_log" in data  # arrivals panel derives from this directly

    def test_yard_tracks_from_file(self, client):
        with client.websocket_connect("/ws") as ws:
            data = ws.receive_json()
        track_ids = {t["id"] for t in data["yard_tracks"]}
        assert "CAB" in track_ids
        assert "YL" in track_ids

    def test_roster_loads_from_file(self, client):
        with client.websocket_connect("/ws") as ws:
            data = ws.receive_json()
        engine_numbers = {e["road_number"] for e in data["roster"]["engines"]}
        caboose_numbers = {c["road_number"] for c in data["roster"]["cabooses"]}
        assert "21" in engine_numbers
        assert "10" in caboose_numbers
        switcher = next(e for e in data["roster"]["engines"] if e["road_number"] == "10")
        assert switcher["road_eligible"] is False

    def test_yard_initial_state_includes_roster(self, client):
        with client.websocket_connect("/ws") as ws:
            data = ws.receive_json()
        assert "roster" in data
        assert "engines" in data["roster"]
        assert "cabooses" in data["roster"]

    def test_coe_trains_have_wp_times(self, client):
        with client.websocket_connect("/ws") as ws:
            data = ws.receive_json()
        assert len(data["coe_trains"]) > 0
        for t in data["coe_trains"]:
            assert "number" in t
            assert "direction" in t

    def test_yard_departures_are_northbound(self, client):
        with client.websocket_connect("/ws") as ws:
            data = ws.receive_json()
        for dep in data["yard_departures"]:
            assert dep["direction"] in ("N", "S")  # extras may report consist direction
            if not dep["extra"]:
                assert dep["direction"] == "N"


# ── MQTT consist message handling ─────────────────────────────────────────────

class TestConsistMqttHandling:
    def test_consist_message_updates_state(self):
        state = AppState()
        mc = _make_real_mqtt_client(state)
        _inject_consist(mc, "3", {
            "state": "ready", "engine": "101", "caboose": "204",
            "cars_loaded": 12, "cars_empty": 5, "extra": False,
        })
        assert state.consists["3"]["state"] == "ready"
        assert state.consists["3"]["cars_total"] == 17

    def test_consist_message_retained_rebuild(self):
        """Simulates reconnect rebuild: retained messages replay through _on_message."""
        state = AppState()
        mc = _make_real_mqtt_client(state)
        _inject_consist(mc, "42", {"state": "car_block_ready", "extra": True, "cars_loaded": 8, "cars_empty": 3})
        _inject_consist(mc, "3", {"state": "ready", "engine": "101", "caboose": "204", "extra": False})
        assert set(state.consists.keys()) == {"42", "3"}


# ── Auto-notify YM of arrival (management setting, default off) ──────────────

OS_XP_SOUTH_PAYLOAD = {
    "station_id": "XP", "train": "9", "section": 0, "direction": "S",
    "extra": False, "work_extra": False, "rr_time": "11:05", "day": 1,
}


def _make_mqtt_client_with_rules(state, layout_rules: dict):
    from dispatcher.mqtt_client import MQTTClient
    config = {
        "broker": {"host": "127.0.0.1", "port": 1883},
        "credentials": {"rr_dispatcher": "test"},
    }
    return MQTTClient(config, state, lambda c: {}, get_layout_rules=lambda: layout_rules)


def _inject_os(mqtt_obj, payload: dict) -> None:
    msg = MagicMock()
    msg.topic = f"trains/os/{payload['station_id']}"
    msg.payload = json.dumps(payload).encode()
    mqtt_obj._on_message(None, None, msg)


# NOTE: these tests cover the logic with a simulated MQTT message only — the
# real trigger (a CYD station publishing an OS report at XP) has never been
# exercised end-to-end since no CYD hardware is deployed yet. Re-verify this
# live once CYD stations are physically installed and OSing for real trains.
class TestAutoNotifyYmArrival:
    def test_disabled_by_default(self):
        state = AppState()
        mc = _make_mqtt_client_with_rules(state, {})
        _inject_os(mc, OS_XP_SOUTH_PAYLOAD)
        assert state.yard_notifications == []

    def test_creates_arrival_notification_when_enabled(self):
        state = AppState()
        mc = _make_mqtt_client_with_rules(state, {"auto_notify_ym_arrival": True})
        _inject_os(mc, OS_XP_SOUTH_PAYLOAD)
        assert len(state.yard_notifications) == 1
        n = state.yard_notifications[0]
        assert n["type"] == "arrival"
        assert n["train"] == "9"
        assert n["direction"] == "S"
        assert n["expected_rr_time"] == "11:05"

    def test_ignores_wrong_station_even_when_enabled(self):
        state = AppState()
        mc = _make_mqtt_client_with_rules(state, {"auto_notify_ym_arrival": True})
        _inject_os(mc, dict(OS_XP_SOUTH_PAYLOAD, station_id="BB"))
        assert state.yard_notifications == []

    def test_ignores_wrong_direction_even_when_enabled(self):
        state = AppState()
        mc = _make_mqtt_client_with_rules(state, {"auto_notify_ym_arrival": True})
        _inject_os(mc, dict(OS_XP_SOUTH_PAYLOAD, direction="N"))
        assert state.yard_notifications == []
