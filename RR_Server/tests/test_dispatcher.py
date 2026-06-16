"""Unit tests for dispatcher/ (Sessions 1.3 + 2.2).

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

    def test_os_log_key_present(self, client):
        data = self._get_initial(client)
        assert "os_log" in data
        assert isinstance(data["os_log"], list)


# ── OS report handling ────────────────────────────────────────────────────────

OS_REPORT_PAYLOAD = {
    "station_id": "BB",
    "train": "3",
    "section": 0,
    "direction": "N",
    "extra": False,
    "work_extra": False,
    "rr_time": "10:41",
    "day": 1,
}

EXTRA_PAYLOAD = {
    "station_id": "XP",
    "train": "101",
    "section": 0,
    "direction": "S",
    "extra": True,
    "work_extra": False,
    "rr_time": "14:05",
    "day": 2,
}

WORK_EXTRA_PAYLOAD = {
    "station_id": "SK",
    "train": "55",
    "section": 0,
    "direction": "N",
    "extra": True,
    "work_extra": True,
    "rr_time": "09:00",
    "day": 3,
}


def _make_real_mqtt_client(state):
    """Create a real MQTTClient instance with a fake config, loop=None (no broadcast)."""
    from dispatcher.mqtt_client import MQTTClient
    config = {
        "broker": {"host": "127.0.0.1", "port": 1883},
        "credentials": {"rr_dispatcher": "test"},
    }
    # build_next_trains stub — not needed for OS tests
    return MQTTClient(config, state, lambda c: {})


def _inject_os(mqtt_obj, payload: dict) -> None:
    """Simulate receiving an OS MQTT message (no broker connection needed)."""
    import json
    msg = MagicMock()
    msg.topic = f"trains/os/{payload['station_id']}"
    msg.payload = json.dumps(payload).encode()
    mqtt_obj._on_message(None, None, msg)


class TestOsReport:
    def test_os_state_appended(self):
        """OS MQTT message is prepended to AppState.os_log."""
        from dispatcher.session import AppState
        state = AppState()
        mc = _make_real_mqtt_client(state)
        _inject_os(mc, OS_REPORT_PAYLOAD)
        assert len(state.os_log) == 1
        entry = state.os_log[0]
        assert entry["station_id"] == "BB"
        assert entry["train"] == "3"
        assert entry["direction"] == "N"
        assert entry["extra"] is False
        assert entry["work_extra"] is False
        assert entry["rr_time"] == "10:41"
        assert entry["day"] == 1

    def test_os_log_newest_first(self):
        """Multiple OS messages are prepended so newest is at index 0."""
        from dispatcher.session import AppState
        state = AppState()
        mc = _make_real_mqtt_client(state)
        _inject_os(mc, OS_REPORT_PAYLOAD)   # BB first
        _inject_os(mc, EXTRA_PAYLOAD)       # XP second
        assert state.os_log[0]["station_id"] == "XP"
        assert state.os_log[1]["station_id"] == "BB"

    def test_os_log_capped_at_50(self):
        """os_log is capped at OS_LOG_MAX (50) entries."""
        from dispatcher.session import AppState, OS_LOG_MAX
        state = AppState()
        mc = _make_real_mqtt_client(state)
        payload = dict(OS_REPORT_PAYLOAD)
        for i in range(OS_LOG_MAX + 5):
            payload = dict(OS_REPORT_PAYLOAD, train=str(i))
            _inject_os(mc, payload)
        assert len(state.os_log) == OS_LOG_MAX

    def test_extra_flags_preserved(self):
        """extra and work_extra flags round-trip through the handler."""
        from dispatcher.session import AppState
        state = AppState()
        mc = _make_real_mqtt_client(state)
        _inject_os(mc, EXTRA_PAYLOAD)
        entry = state.os_log[0]
        assert entry["extra"] is True
        assert entry["work_extra"] is False

    def test_work_extra_flags_preserved(self):
        """work_extra flag is stored correctly."""
        from dispatcher.session import AppState
        state = AppState()
        mc = _make_real_mqtt_client(state)
        _inject_os(mc, WORK_EXTRA_PAYLOAD)
        entry = state.os_log[0]
        assert entry["extra"] is True
        assert entry["work_extra"] is True

    def test_initial_state_includes_os_log(self, client):
        """initial_state WebSocket event includes os_log with accumulated entries."""
        import json
        with patch("dispatcher.app.MQTTClient") as mock_cls, \
             patch("dispatcher.app.CONFIG_FILE", CONFIG_EXAMPLE):
            mock_cls.return_value = MagicMock()
            with TestClient(app) as c:
                from dispatcher.app import state as app_state
                # Directly inject an entry into state (bypasses MQTT plumbing)
                app_state.os_log.insert(0, dict(OS_REPORT_PAYLOAD))
                with c.websocket_connect("/ws") as ws:
                    data = ws.receive_json()
                    assert data["type"] == "initial_state"
                    assert any(e["train"] == "3" for e in data["os_log"])


# ── TO issue endpoint ─────────────────────────────────────────────────────────

MEET_FIELDS = {
    "train_a": "3",
    "engine_a": "101",
    "direction_a": "S",
    "train_b_is_extra": True,
    "train_b": "42",
    "direction_b": "N",
    "station": "BB",
}

WAIT_FIELDS = {
    "train": "5",
    "engine": "202",
    "station": "XP",
    "until_time": "11:30",
}

RUNNING_EXTRA_FIELDS = {
    "engine": "99",
    "direction": "N",
    "from_station": "WP",
    "to_station": "HC",
    "departure_rr_time": "06:00",
}

WORK_EXTRA_FIELDS = {
    "engine": "55",
    "from_station": "BB",
    "to_station": "JC",
    "start_rr_time": "10:00",
    "end_rr_time": "12:30",
}

ANNULMENT_FIELDS = {"train": "141"}

SECTIONS_FIELDS = {"train": "3", "engine": "101", "section_count": 2}


class TestToIssue:
    def test_valid_meet_returns_200_and_seq(self, client):
        r = client.post("/api/to/issue", json={
            "to_type": "meet",
            "fields": MEET_FIELDS,
            "addressed_to": ["BB", "XP"],
        })
        assert r.status_code == 200
        body = r.json()
        assert body["ok"] is True
        assert isinstance(body["seq"], int)

    def test_seq_increments_across_orders(self, client):
        r1 = client.post("/api/to/issue", json={
            "to_type": "wait", "fields": WAIT_FIELDS, "addressed_to": ["XP"],
        })
        r2 = client.post("/api/to/issue", json={
            "to_type": "wait", "fields": WAIT_FIELDS, "addressed_to": ["BB"],
        })
        assert r2.json()["seq"] == r1.json()["seq"] + 1

    def test_missing_to_type_is_400(self, client):
        r = client.post("/api/to/issue", json={
            "fields": MEET_FIELDS, "addressed_to": ["BB"],
        })
        assert r.status_code == 400

    def test_invalid_to_type_is_400(self, client):
        r = client.post("/api/to/issue", json={
            "to_type": "bogus", "fields": {}, "addressed_to": ["BB"],
        })
        assert r.status_code == 400

    def test_empty_addressed_to_is_400(self, client):
        r = client.post("/api/to/issue", json={
            "to_type": "wait", "fields": WAIT_FIELDS, "addressed_to": [],
        })
        assert r.status_code == 400

    def test_invalid_station_id_is_400(self, client):
        r = client.post("/api/to/issue", json={
            "to_type": "wait", "fields": WAIT_FIELDS, "addressed_to": ["ZZ"],
        })
        assert r.status_code == 400

    def test_missing_required_field_is_400(self, client):
        r = client.post("/api/to/issue", json={
            "to_type": "meet",
            "fields": {"train_a": "3"},   # engine_a, train_b, etc. missing
            "addressed_to": ["BB"],
        })
        assert r.status_code == 400

    def test_trains_array_meet(self, client):
        r = client.post("/api/to/issue", json={
            "to_type": "meet", "fields": MEET_FIELDS, "addressed_to": ["BB"],
        })
        assert r.status_code == 200
        from dispatcher.app import state as app_state
        entry = next(e for e in app_state.to_log if e["to_type"] == "meet"
                     and "3" in e["trains"] and "42" in e["trains"])
        assert set(entry["trains"]) == {"3", "42"}

    def test_trains_array_wait(self, client):
        r = client.post("/api/to/issue", json={
            "to_type": "wait", "fields": WAIT_FIELDS, "addressed_to": ["XP"],
        })
        assert r.status_code == 200
        from dispatcher.app import state as app_state
        entry = next(e for e in app_state.to_log
                     if e["to_type"] == "wait" and "5" in e["trains"])
        assert entry["trains"] == ["5"]

    def test_trains_array_running_extra(self, client):
        r = client.post("/api/to/issue", json={
            "to_type": "running_extra", "fields": RUNNING_EXTRA_FIELDS,
            "addressed_to": ["WP"],
        })
        assert r.status_code == 200
        from dispatcher.app import state as app_state
        entry = next(e for e in app_state.to_log
                     if e["to_type"] == "running_extra")
        assert entry["trains"] == ["99"]

    def test_to_stored_in_state(self, client):
        before = len([e for e in __import__("dispatcher.app", fromlist=["state"]).state.to_log])
        client.post("/api/to/issue", json={
            "to_type": "annulment", "fields": ANNULMENT_FIELDS, "addressed_to": ["HC"],
        })
        from dispatcher.app import state as app_state
        assert len(app_state.to_log) > before

    def test_to_entry_has_ack_dict(self, client):
        client.post("/api/to/issue", json={
            "to_type": "sections", "fields": SECTIONS_FIELDS,
            "addressed_to": ["WP", "XP"],
        })
        from dispatcher.app import state as app_state
        entry = next(e for e in app_state.to_log if e["to_type"] == "sections")
        assert "WP" in entry["acks"]
        assert "XP" in entry["acks"]
        assert entry["acks"]["WP"] is None
        assert entry["acks"]["XP"] is None

    def test_initial_state_includes_to_log_and_types(self, client):
        with client.websocket_connect("/ws") as ws:
            data = ws.receive_json()
        assert "to_log" in data
        assert "to_types" in data
        assert isinstance(data["to_log"], list)
        assert isinstance(data["to_types"], dict)


# ── Signal arm endpoint ───────────────────────────────────────────────────────

class TestSignalArm:
    def test_raise_valid_station_returns_200(self, client):
        r = client.post("/api/signal/arm", json={
            "station_id": "BB", "direction": "N", "state": "raised",
        })
        assert r.status_code == 200
        assert r.json()["ok"] is True

    def test_lower_valid_station_returns_200(self, client):
        r = client.post("/api/signal/arm", json={
            "station_id": "XP", "direction": "S", "state": "lowered",
        })
        assert r.status_code == 200

    def test_non_signal_station_is_400(self, client):
        r = client.post("/api/signal/arm", json={
            "station_id": "WP", "direction": "N", "state": "raised",
        })
        assert r.status_code == 400

    def test_invalid_direction_is_400(self, client):
        r = client.post("/api/signal/arm", json={
            "station_id": "BB", "direction": "E", "state": "raised",
        })
        assert r.status_code == 400

    def test_invalid_state_is_400(self, client):
        r = client.post("/api/signal/arm", json={
            "station_id": "BB", "direction": "N", "state": "halfway",
        })
        assert r.status_code == 400


# ── Block signal endpoint (WP-XP section) ─────────────────────────────────────

class TestBlockSignal:
    def test_raise_wp_returns_200(self, client):
        r = client.post("/api/signal/block", json={"station_id": "WP", "state": "raised"})
        assert r.status_code == 200
        assert r.json()["ok"] is True

    def test_lower_xp_returns_200(self, client):
        r = client.post("/api/signal/block", json={"station_id": "XP", "state": "lowered"})
        assert r.status_code == 200

    def test_non_block_station_is_400(self, client):
        r = client.post("/api/signal/block", json={"station_id": "BB", "state": "raised"})
        assert r.status_code == 400

    def test_invalid_state_is_400(self, client):
        r = client.post("/api/signal/block", json={"station_id": "WP", "state": "halfway"})
        assert r.status_code == 400

    def test_initial_state_includes_block_signal_stations(self, client):
        with client.websocket_connect("/ws") as ws:
            data = ws.receive_json()
        assert set(data["block_signal_stations"]) == {"WP", "XP"}
        assert "block_signals" in data


def _inject_block_signal(mqtt_obj, station_id: str, state: str) -> None:
    import json
    msg = MagicMock()
    msg.topic = f"trains/signal/{station_id}/block/state"
    msg.payload = json.dumps({"state": state}).encode()
    mqtt_obj._on_message(None, None, msg)


class TestBlockSignalMqttHandling:
    def test_block_state_updates_state(self):
        from dispatcher.session import AppState
        state = AppState()
        mc = _make_real_mqtt_client(state)
        _inject_block_signal(mc, "WP", "lowered")
        assert state.block_signals["WP"] == "lowered"

    def test_block_state_independent_per_station(self):
        from dispatcher.session import AppState
        state = AppState()
        mc = _make_real_mqtt_client(state)
        _inject_block_signal(mc, "WP", "lowered")
        _inject_block_signal(mc, "XP", "raised")
        assert state.block_signals == {"WP": "lowered", "XP": "raised"}


# ── ACK handling ──────────────────────────────────────────────────────────────

def _inject_to_ack(mqtt_obj, seq: int, station_id: str, rr_time: str = "10:33") -> None:
    import json
    msg = MagicMock()
    msg.topic = f"trains/to/{station_id}/ack"
    msg.payload = json.dumps({"seq": seq, "station_id": station_id,
                               "rr_time": rr_time, "copies": 2}).encode()
    mqtt_obj._on_message(None, None, msg)


class TestToAck:
    def test_ack_updates_state(self):
        from dispatcher.session import AppState
        state = AppState()
        state.record_to({
            "seq": 1, "to_type": "wait", "trains": ["5"],
            "addressed_to": ["XP"], "fields": {},
        })
        mc = _make_real_mqtt_client(state)
        _inject_to_ack(mc, seq=1, station_id="XP")
        assert state.to_log[0]["acks"]["XP"] is not None
        assert state.to_log[0]["acks"]["XP"]["rr_time"] == "10:33"

    def test_ack_unknown_seq_is_ignored(self):
        from dispatcher.session import AppState
        state = AppState()
        mc = _make_real_mqtt_client(state)
        # Should not raise even with no matching TO
        _inject_to_ack(mc, seq=999, station_id="BB")

    def test_all_acked_flag_set_when_complete(self):
        from dispatcher.session import AppState
        state = AppState()
        state.record_to({
            "seq": 2, "to_type": "wait", "trains": ["5"],
            "addressed_to": ["BB", "XP"], "fields": {},
        })
        mc = _make_real_mqtt_client(state)
        _inject_to_ack(mc, seq=2, station_id="BB")
        _inject_to_ack(mc, seq=2, station_id="XP")
        assert all(v is not None for v in state.to_log[0]["acks"].values())

    def test_partial_ack_does_not_set_all_acked(self):
        from dispatcher.session import AppState
        state = AppState()
        state.record_to({
            "seq": 3, "to_type": "meet", "trains": ["3", "42"],
            "addressed_to": ["BB", "XP", "JC"], "fields": {},
        })
        mc = _make_real_mqtt_client(state)
        _inject_to_ack(mc, seq=3, station_id="BB")
        assert state.to_log[0]["acks"]["BB"] is not None
        assert state.to_log[0]["acks"]["XP"] is None
        assert state.to_log[0]["acks"]["JC"] is None
