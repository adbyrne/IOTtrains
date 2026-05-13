"""Unit tests for fast_clock/clock_service.py.

Covers pure logic (no MQTT broker required):
  - _fmt() — 12-hour formatting edge cases
  - _decompose() — time math including midnight rollover and day-of-week wrap
  - FastClock control actions — state transitions for all six actions
  - State persistence — survives restart, always paused after reload
  - Payload structure — retained flag, required fields
  - Input clamping — out-of-range hour/minute/day/speed/interval values
"""

import json
import threading
import pytest
from unittest.mock import MagicMock, call

import fast_clock.clock_service as svc
from fast_clock.clock_service import FastClock, _fmt

MINIMAL_CONFIG = {
    "broker": {"host": "127.0.0.1", "port": 1883},
    "credentials": {"rr_clock": "test"},
    "clock": {"default_speed": 3, "default_tick_interval": 60},
}


@pytest.fixture
def clock(tmp_path, monkeypatch):
    monkeypatch.setattr(svc, "STATE_FILE", tmp_path / "clock_state.json")
    c = FastClock(MINIMAL_CONFIG)
    c.client = MagicMock()
    yield c
    c._cancel_timer()


# ── _fmt ────────────────────────────────────────────────────────────────────

@pytest.mark.parametrize("h,m,expected", [
    (0,  0,  "12:00 AM"),   # midnight
    (0,  30, "12:30 AM"),   # early AM
    (1,  0,  "1:00 AM"),    # 1 AM
    (11, 59, "11:59 AM"),   # just before noon
    (12, 0,  "12:00 PM"),   # noon
    (12, 30, "12:30 PM"),   # afternoon
    (13, 0,  "1:00 PM"),    # 1 PM
    (23, 59, "11:59 PM"),   # last minute of day
])
def test_fmt(h, m, expected):
    assert _fmt(h, m) == expected


# ── _decompose ───────────────────────────────────────────────────────────────

def test_decompose_normal(clock):
    clock._day = 1
    assert clock._decompose(8 * 60 + 30) == (8, 30, 1)

def test_decompose_end_of_day(clock):
    clock._day = 1
    assert clock._decompose(23 * 60 + 59) == (23, 59, 1)

def test_decompose_midnight_rollover(clock):
    clock._day = 3
    h, m, day = clock._decompose(24 * 60)  # exactly midnight next day
    assert (h, m, day) == (0, 0, 4)

def test_decompose_day7_wraps_to_day1(clock):
    clock._day = 7
    h, m, day = clock._decompose(24 * 60 + 60)  # 1:00 AM next day
    assert (h, m, day) == (1, 0, 1)

def test_decompose_two_day_rollover(clock):
    clock._day = 5
    h, m, day = clock._decompose(2 * 24 * 60 + 90)  # 48h 30m in
    assert (h, m, day) == (1, 30, 7)


# ── Control: start / pause ───────────────────────────────────────────────────

def test_start_sets_running(clock):
    clock.handle_control({"action": "start"})
    assert clock._running is True

def test_start_creates_timer(clock):
    clock.handle_control({"action": "start"})
    assert clock._timer is not None

def test_start_is_idempotent(clock):
    clock.handle_control({"action": "start"})
    t1 = clock._timer
    clock.handle_control({"action": "start"})  # second call — already running
    assert clock._timer is t1  # timer not replaced

def test_pause_clears_running(clock):
    clock.handle_control({"action": "start"})
    clock.handle_control({"action": "pause"})
    assert clock._running is False

def test_pause_cancels_timer(clock):
    clock.handle_control({"action": "start"})
    clock.handle_control({"action": "pause"})
    assert clock._timer is None

def test_pause_when_already_paused_does_nothing(clock):
    clock.handle_control({"action": "pause"})  # no-op: was never started
    assert clock._running is False


# ── Control: set ─────────────────────────────────────────────────────────────

def test_set_time(clock):
    clock.handle_control({"action": "set", "hour": 14, "minute": 30, "day": 3})
    assert (clock._hour, clock._minute, clock._day) == (14, 30, 3)

def test_set_partial_keeps_other_fields(clock):
    clock._hour = 9
    clock._minute = 15
    clock._day = 2
    clock.handle_control({"action": "set", "hour": 11})
    assert clock._hour == 11
    assert clock._minute == 15   # unchanged
    assert clock._day == 2       # unchanged

def test_set_preserves_running_state(clock):
    clock.handle_control({"action": "start"})
    clock.handle_control({"action": "set", "hour": 10, "minute": 0})
    assert clock._running is True

def test_set_clamps_hour_max(clock):
    clock.handle_control({"action": "set", "hour": 99})
    assert clock._hour == 23

def test_set_clamps_minute_max(clock):
    clock.handle_control({"action": "set", "minute": 99})
    assert clock._minute == 59

def test_set_clamps_day_max(clock):
    clock.handle_control({"action": "set", "day": 8})
    assert clock._day == 7

def test_set_clamps_day_min(clock):
    clock.handle_control({"action": "set", "day": 0})
    assert clock._day == 1


# ── Control: reset ───────────────────────────────────────────────────────────

def test_reset_zeros_time(clock):
    clock.handle_control({"action": "start"})
    clock.handle_control({"action": "set", "hour": 14, "minute": 30, "day": 5})
    clock.handle_control({"action": "reset"})
    assert (clock._hour, clock._minute, clock._day) == (0, 0, 1)

def test_reset_stops_clock(clock):
    clock.handle_control({"action": "start"})
    clock.handle_control({"action": "reset"})
    assert clock._running is False
    assert clock._timer is None


# ── Control: speed ───────────────────────────────────────────────────────────

def test_speed_change(clock):
    clock.handle_control({"action": "speed", "speed": 6})
    assert clock._speed == 6

def test_speed_minimum_is_1(clock):
    clock.handle_control({"action": "speed", "speed": 0})
    assert clock._speed == 1

def test_speed_negative_clamped(clock):
    clock.handle_control({"action": "speed", "speed": -5})
    assert clock._speed == 1


# ── Control: set_tick_interval ───────────────────────────────────────────────

def test_set_tick_interval(clock):
    clock.handle_control({"action": "set_tick_interval", "seconds": 30})
    assert clock._tick_interval == 30

def test_tick_interval_minimum_is_1(clock):
    clock.handle_control({"action": "set_tick_interval", "seconds": 0})
    assert clock._tick_interval == 1


# ── Unknown action ───────────────────────────────────────────────────────────

def test_unknown_action_does_not_raise(clock):
    clock.handle_control({"action": "teleport"})  # should not raise


# ── Payload structure ────────────────────────────────────────────────────────

def test_payload_has_required_fields(clock):
    clock._hour = 10
    clock._minute = 42
    clock._day = 2
    clock._rr_base = 10 * 60 + 42
    p = json.loads(clock._build_payload())
    for field in ("time", "hour", "minute", "day", "speed", "running"):
        assert field in p, f"missing field: {field}"

def test_payload_time_string_matches_hour_minute(clock):
    clock._hour = 10
    clock._minute = 42
    clock._rr_base = 10 * 60 + 42
    p = json.loads(clock._build_payload())
    assert p["time"] == "10:42 AM"
    assert p["hour"] == 10
    assert p["minute"] == 42

def test_publish_uses_retain(clock):
    clock.handle_control({"action": "start"})
    for c in clock.client.publish.call_args_list:
        topic = c.args[0]
        if topic == svc.TOPIC_TIME:
            assert c.kwargs.get("retain") is True
            break
    else:
        pytest.fail("No publish call to TOPIC_TIME found")


# ── Sync request ─────────────────────────────────────────────────────────────

def test_sync_request_publishes_time(clock):
    clock.handle_sync_request({"station_id": "BB"})
    assert clock.client.publish.call_count >= 1
    topic = clock.client.publish.call_args.args[0]
    assert topic == svc.TOPIC_TIME


# ── State persistence ────────────────────────────────────────────────────────

def test_state_survives_restart(tmp_path, monkeypatch):
    state_file = tmp_path / "clock_state.json"
    monkeypatch.setattr(svc, "STATE_FILE", state_file)

    c1 = FastClock(MINIMAL_CONFIG)
    c1.client = MagicMock()
    c1.handle_control({"action": "set", "hour": 14, "minute": 30, "day": 5})
    c1.handle_control({"action": "speed", "speed": 6})
    c1.handle_control({"action": "set_tick_interval", "seconds": 30})
    c1._cancel_timer()

    c2 = FastClock(MINIMAL_CONFIG)
    assert c2._hour == 14
    assert c2._minute == 30
    assert c2._day == 5
    assert c2._speed == 6
    assert c2._tick_interval == 30

def test_always_paused_after_restart(tmp_path, monkeypatch):
    state_file = tmp_path / "clock_state.json"
    state_file.write_text(json.dumps({
        "hour": 9, "minute": 0, "day": 1,
        "speed": 3, "running": True, "tick_interval": 60,
    }))
    monkeypatch.setattr(svc, "STATE_FILE", state_file)
    c = FastClock(MINIMAL_CONFIG)
    assert c._running is False  # never auto-starts
    c._cancel_timer()

def test_missing_state_file_uses_defaults(tmp_path, monkeypatch):
    monkeypatch.setattr(svc, "STATE_FILE", tmp_path / "nonexistent.json")
    c = FastClock(MINIMAL_CONFIG)
    assert c._running is False
    assert c._speed == 3
    c._cancel_timer()
