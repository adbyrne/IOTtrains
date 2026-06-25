import pytest
from pathlib import Path
from common import timetable

DATA = Path(__file__).parent.parent / "data" / "timetable.json"


@pytest.fixture(autouse=True)
def loaded():
    timetable.load(DATA)


# ── locations ────────────────────────────────────────────────────────────────

def test_locations_ordered_south_to_north():
    locs = timetable.locations("NLS")
    ids = [l["id"] for l in locs]
    assert ids[0] == "WP_YARD"
    assert ids[-1] == "HC"
    # XP must come before BB
    assert ids.index("XP") < ids.index("BB")


def test_locations_unknown_subdivision():
    with pytest.raises(KeyError):
        timetable.locations("UNKNOWN")


# ── active_trains ─────────────────────────────────────────────────────────────

def test_active_trains_all_daily():
    for day in range(1, 8):
        trains = timetable.active_trains("NLS", day)
        numbers = {t["number"] for t in trains}
        # All trains are daily — every train should appear every day
        assert "1" in numbers
        assert "2" in numbers
        assert "141" in numbers
        assert "142" in numbers


def test_active_trains_count():
    trains = timetable.active_trains("NLS", 1)
    # 12 NB + 12 SB = 24 trains
    assert len(trains) == 24


def test_active_trains_directions():
    trains = timetable.active_trains("NLS", 1)
    nb = [t for t in trains if t["direction"] == "N"]
    sb = [t for t in trains if t["direction"] == "S"]
    assert len(nb) == 12
    assert len(sb) == 12


# ── train_schedule ────────────────────────────────────────────────────────────

def test_train_schedule_found():
    t = timetable.train_schedule("NLS", "3")
    assert t is not None
    assert t["number"] == "3"
    assert t["class"] == 1
    assert t["service"] == "Pass"
    assert t["direction"] == "N"


def test_train_schedule_stops_monotonic():
    """NB train times must increase stop-to-stop."""
    from common.timetable import _time_minutes, _stop_time
    for num in ["1", "3", "21", "23", "25"]:
        t = timetable.train_schedule("NLS", num)
        times = [_time_minutes(_stop_time(s)) for s in t["schedule"] if _stop_time(s)]
        assert times == sorted(times), f"Train {num} times not monotonic: {times}"


def test_train_schedule_sb_stops_monotonic():
    """SB train times must increase stop-to-stop (HC → WP).
    Train 26 is excluded — it departs HC at 22:18 and arrives WP at 00:04, crossing
    midnight, so raw minutes are not monotonic by design."""
    from common.timetable import _time_minutes, _stop_time
    for num in ["2", "4", "22", "24"]:
        t = timetable.train_schedule("NLS", num)
        times = [_time_minutes(_stop_time(s)) for s in t["schedule"] if _stop_time(s)]
        assert times == sorted(times), f"Train {num} times not monotonic: {times}"


def test_train_schedule_train1_xp_note():
    t = timetable.train_schedule("NLS", "1")
    xp_stop = next(s for s in t["schedule"] if s["location"] == "XP")
    assert xp_stop["arrive"] == "07:19"
    assert xp_stop["depart"] == "07:29"


def test_train_schedule_not_found():
    assert timetable.train_schedule("NLS", "999") is None


def test_train_schedule_train2_full():
    t = timetable.train_schedule("NLS", "2")
    stops = {s["location"]: s for s in t["schedule"]}
    assert stops["HC"]["depart"] == "09:00"
    assert stops["XP"]["arrive"] == "10:09"
    assert stops["XP"]["depart"] == "10:19"
    assert stops["WP"]["arrive"] == "10:28"


# ── next_train ────────────────────────────────────────────────────────────────

def test_next_train_basic():
    result = timetable.next_train("NLS", "WP", "N", "06:00", 1)
    assert result is not None
    assert result["number"] == "1"   # departs 07:10, first NB pass after 06:00
    assert result["arrive"] is None
    assert result["depart"] == "07:10"


def test_next_train_after_1_gets_3():
    result = timetable.next_train("NLS", "WP", "N", "08:00", 1)
    assert result is not None
    assert result["number"] == "23"  # departs 11:00


def test_next_train_sb_at_hc():
    result = timetable.next_train("NLS", "HC", "S", "05:00", 1)
    assert result is not None
    assert result["number"] == "52"  # departs HC 06:15


def test_next_train_sb_at_hc_after_52():
    result = timetable.next_train("NLS", "HC", "S", "06:30", 1)
    assert result is not None
    assert result["number"] == "2"   # departs HC 09:00


def test_next_train_wraps_midnight():
    # After 23:00, train 26 already departed HC at 22:18, so it wraps.
    # Next SB departure from HC is train 22 at 03:31 (wrapped = next calendar day).
    result = timetable.next_train("NLS", "HC", "S", "23:00", 1)
    assert result is not None
    assert result["number"] == "22"
    assert result["arrive"] is None
    assert result["depart"] == "03:31"


def test_next_train_terminus_arrive_is_valid():
    # SB trains arrive at WP_YARD (terminus). next_train reports their arrival time,
    # which is useful for yardmaster "next arrival" queries.
    result = timetable.next_train("NLS", "WP_YARD", "S", "00:00", 1)
    assert result is not None
    assert result["direction"] == "S"


def test_next_train_holding_remains_visible():
    # Train 2 arrives XP at 09:59, departs 10:09. At 10:04 (after arrive, before depart)
    # the train is still at the station — it must still be returned as next.
    result = timetable.next_train("NLS", "XP", "S", "10:04", 1)
    assert result is not None
    assert result["number"] == "2"

def test_next_train_at_intermediate_stop():
    # At BB heading north, at 11:30 → train 23 is holding (arrived 11:24, departs 11:34).
    # Depart-first comparison keeps it visible until it actually leaves.
    result = timetable.next_train("NLS", "BB", "N", "11:30", 1)
    assert result is not None
    assert result["number"] == "23"
    assert result["arrive"] == "11:24"
    assert result["depart"] == "11:34"

def test_next_train_after_holding_departs():
    # At BB heading north, after train 23 departs (11:35) → next is train 131 (departs 12:13)
    result = timetable.next_train("NLS", "BB", "N", "11:35", 1)
    assert result is not None
    assert result["number"] == "131"
    assert result["arrive"] is None
    assert result["depart"] == "12:13"


def test_next_train_xp_southbound():
    # At XP heading south around 09:45 → train 2 (arrive 10:09, depart 10:19).
    # Both times returned; dispatcher shows Ar for meet planning, CYD time screen shows Ar.
    result = timetable.next_train("NLS", "XP", "S", "09:45", 1)
    assert result is not None
    assert result["number"] == "2"
    assert result["arrive"] == "10:09"
    assert result["depart"] == "10:19"


# ── location_by_id ───────────────────────────────────────────────────────────

def test_location_by_id_found():
    loc = timetable.location_by_id("NLS", "XP")
    assert loc is not None
    assert loc["id"] == "XP"
    assert loc["name"] == "Xina Pass"
    assert loc["cyd"] is True
    assert loc["switchback"] is False


def test_location_by_id_not_found():
    assert timetable.location_by_id("NLS", "NONEXISTENT") is None


# ── COE subdivision stub ──────────────────────────────────────────────────────

def test_coe_locations_present():
    locs = timetable.locations("COE")
    ids = [l["id"] for l in locs]
    assert "WP_COE" in ids


def test_coe_has_trains():
    trains = timetable.active_trains("COE", 1)
    assert len(trains) == 10
    numbers = [t["number"] for t in trains]
    assert "91" in numbers
