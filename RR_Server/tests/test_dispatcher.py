"""Unit tests for dispatcher/ (Session 1.3).

Covers pure logic and HTTP/WebSocket routes — no MQTT broker required.
The MQTTClient is patched so tests run standalone.

Not covered here (noted for future smoke test):
  - End-to-end MQTT → WebSocket event flow against a live broker
  - TO signal state updates propagating to the browser
  - Clock control commands actually reaching the clock service
  See: TODO smoke_test_dispatcher.py (requires RPi5 + broker up)
"""

import sys
from pathlib import Path
from unittest.mock import MagicMock, patch

import pytest
from starlette.testclient import TestClient

sys.path.insert(0, str(Path(__file__).parent.parent))

from common import timetable
from dispatcher.app import app, build_next_trains
from dispatcher.session import STATION_IDS, TO_SIGNAL_STATIONS

DATA = Path(__file__).parent.parent / "data" / "timetable.json"


@pytest.fixture(scope="module", autouse=True)
def loaded_timetable():
    timetable.load(DATA)


CONFIG_EXAMPLE = Path(__file__).parent.parent / "config.json.example"


@pytest.fixture()
def client():
    """TestClient with MQTT patched out and config.json.example substituted.

    config.json is not committed (contains credentials), so CI patches
    CONFIG_FILE to use the example file instead.
    """
    with patch("dispatcher.app.MQTTClient") as mock_cls, \
         patch("dispatcher.app.CONFIG_FILE", CONFIG_EXAMPLE):
        mock_cls.return_value = MagicMock()
        with TestClient(app) as c:
            yield c


# ── build_next_trains ────────────────────────────────────────────────────────

class TestBuildNextTrains:
    def test_returns_all_stations(self):
        result = build_next_trains({"hour": 8, "minute": 0, "day": 1})
        assert set(result.keys()) == set(STATION_IDS)

    def test_each_station_has_both_directions(self):
        result = build_next_trains({"hour": 8, "minute": 0, "day": 1})
        for sid in STATION_IDS:
            assert "N" in result[sid]
            assert "S" in result[sid]

    def test_train_entry_shape(self):
        result = build_next_trains({"hour": 8, "minute": 0, "day": 1})
        for sid in STATION_IDS:
            for direction in ("N", "S"):
                train = result[sid][direction]
                if train is not None:
                    assert "number" in train
                    assert "arrive" in train
                    assert "depart" in train
                    assert "direction" in train
                    return
        pytest.skip("No trains found in timetable")

    def test_empty_clock_returns_none_for_all(self):
        result = build_next_trains({})
        for sid in STATION_IDS:
            assert result[sid]["N"] is None
            assert result[sid]["S"] is None

    def test_late_clock_still_returns_result(self):
        # Midnight wrap — should not raise
        result = build_next_trains({"hour": 23, "minute": 59, "day": 1})
        assert set(result.keys()) == set(STATION_IDS)


# ── Static routes ────────────────────────────────────────────────────────────

class TestStaticRoutes:
    def test_index_200(self, client):
        r = client.get("/")
        assert r.status_code == 200
        assert "NY&amp;E" in r.text

    def test_css_served(self, client):
        assert client.get("/static/style.css").status_code == 200

    def test_js_served(self, client):
        assert client.get("/static/dispatcher.js").status_code == 200


# ── Clock control endpoint ───────────────────────────────────────────────────

class TestClockControl:
    def test_start(self, client):
        r = client.post("/api/clock/control", json={"action": "start"})
        assert r.status_code == 200
        assert r.json() == {"ok": True}

    def test_pause(self, client):
        assert client.post("/api/clock/control", json={"action": "pause"}).status_code == 200

    def test_reset(self, client):
        assert client.post("/api/clock/control", json={"action": "reset"}).status_code == 200

    def test_set_with_all_fields(self, client):
        r = client.post("/api/clock/control",
                        json={"action": "set", "hour": 10, "minute": 30, "day": 2})
        assert r.status_code == 200

    def test_set_with_partial_fields(self, client):
        # Partial set is valid — omitted fields keep current value
        r = client.post("/api/clock/control", json={"action": "set", "hour": 14})
        assert r.status_code == 200

    def test_speed(self, client):
        r = client.post("/api/clock/control", json={"action": "speed", "speed": 6})
        assert r.status_code == 200

    def test_missing_action_is_400(self, client):
        assert client.post("/api/clock/control", json={"hour": 8}).status_code == 400

    def test_speed_without_value_is_400(self, client):
        assert client.post("/api/clock/control", json={"action": "speed"}).status_code == 400


# ── WebSocket initial_state ───────────────────────────────────────────────────

class TestWebSocketInitialState:
    def _get_initial(self, client):
        with client.websocket_connect("/ws") as ws:
            return ws.receive_json()

    def test_event_type(self, client):
        assert self._get_initial(client)["type"] == "initial_state"

    def test_station_ids_complete_and_ordered(self, client):
        data = self._get_initial(client)
        assert data["station_ids"] == STATION_IDS

    def test_station_names_present(self, client):
        data = self._get_initial(client)
        assert set(data["station_names"].keys()) == set(STATION_IDS)

    def test_next_trains_has_all_stations(self, client):
        data = self._get_initial(client)
        assert set(data["next_trains"].keys()) == set(STATION_IDS)

    def test_next_trains_has_both_directions(self, client):
        data = self._get_initial(client)
        for sid in STATION_IDS:
            assert "N" in data["next_trains"][sid]
            assert "S" in data["next_trains"][sid]

    def test_to_signal_stations(self, client):
        data = self._get_initial(client)
        assert set(data["to_signal_stations"]) == TO_SIGNAL_STATIONS

    def test_to_signals_key_present(self, client):
        data = self._get_initial(client)
        assert "to_signals" in data

    def test_stations_key_present(self, client):
        data = self._get_initial(client)
        assert "stations" in data
