"""
Timetable loader and query module for RR_Server.
Loads timetable.json at startup; supports hot-reload and atomic save via manage API.
"""

import json
import os
import tempfile
from pathlib import Path
from typing import Optional


_timetable: Optional[dict] = None
_data_path: Optional[Path] = None


def load(path: str | Path) -> None:
    global _timetable, _data_path
    _data_path = Path(path)
    with open(_data_path) as f:
        _timetable = json.load(f)


def reload() -> None:
    """Re-read and re-parse the timetable file in place (hot-reload, no restart needed)."""
    if _data_path is None:
        raise RuntimeError("Timetable not loaded — call timetable.load() first")
    load(_data_path)


def save(data: dict) -> None:
    """Atomically write `data` to the timetable file, then hot-reload.

    Writes to a temp file in the same directory and renames to avoid partial writes.
    """
    if _data_path is None:
        raise RuntimeError("Timetable not loaded — call timetable.load() first")
    dir_ = _data_path.parent
    fd, tmp = tempfile.mkstemp(dir=dir_, suffix=".tmp")
    try:
        with os.fdopen(fd, "w") as f:
            json.dump(data, f, indent=2)
        os.replace(tmp, _data_path)
    except Exception:
        try:
            os.unlink(tmp)
        except OSError:
            pass
        raise
    reload()


def get() -> Optional[dict]:
    """Return the raw timetable dict (read-only reference)."""
    return _timetable


def _require_loaded() -> None:
    if _timetable is None:
        raise RuntimeError("Timetable not loaded — call timetable.load() first")


def _subdivision(subdivision_id: str) -> dict:
    _require_loaded()
    for sub in _timetable["subdivisions"]:
        if sub["id"] == subdivision_id:
            return sub
    raise KeyError(f"Subdivision not found: {subdivision_id}")


def _time_minutes(t: str) -> int:
    """Convert 'HH:MM' to minutes since midnight."""
    h, m = t.split(":")
    return int(h) * 60 + int(m)


def _stop_time(stop: dict) -> Optional[str]:
    """Return comparison time for a stop: depart if present, else arrive.

    Depart-first ensures a holding train stays visible until it actually leaves.
    Falls back to arrive for terminus arrival-only stops.
    """
    return stop.get("depart") or stop.get("arrive")


def locations(subdivision_id: str) -> list[dict]:
    """Return ordered location list for a subdivision."""
    return _subdivision(subdivision_id)["locations"]


def location_by_id(subdivision_id: str, location_id: str) -> Optional[dict]:
    """Return the location dict for location_id, or None if not found."""
    for loc in locations(subdivision_id):
        if loc["id"] == location_id:
            return loc
    return None


def active_trains(subdivision_id: str, day: int) -> list[dict]:
    """
    Return trains running on `day` (1=Mon … 7=Sun) for a subdivision.
    """
    return [t for t in _subdivision(subdivision_id)["trains"] if day in t["days"]]


def train_schedule(subdivision_id: str, train_number: str) -> Optional[dict]:
    """Return the full train dict (including schedule) for a given train number."""
    for train in _subdivision(subdivision_id)["trains"]:
        if train["number"] == train_number:
            return train
    return None


def inter_station_times(subdivision_id: str, station_ids: list[str]) -> dict:
    """
    Compute estimated running times between adjacent stations for extra train guidance.

    Uses the first Frght train in each direction as the reference; falls back to any
    train in that direction if Frght doesn't cover a segment (e.g. WP↔XP uses Pass).

    Returns: {station_id: {"N": minutes|None, "S": minutes|None}}
      N = estimated minutes from this station to the next station northward
      S = estimated minutes from this station to the next station southward
    """
    sub = _subdivision(subdivision_id)
    trains = sub["trains"]

    def sched_of(train: dict) -> dict:
        return {s["location"]: s for s in train.get("schedule", [])}

    def _depart(stop: dict) -> Optional[int]:
        t = stop.get("depart") or stop.get("arrive")
        return _time_minutes(t) if t else None

    def _arrive(stop: dict) -> Optional[int]:
        t = stop.get("arrive") or stop.get("depart")
        return _time_minutes(t) if t else None

    def run_time(from_id: str, to_id: str, direction: str) -> Optional[int]:
        # Prefer Frght; fall back to any direction-matching train
        ordered = sorted(
            [t for t in trains if t["direction"] == direction],
            key=lambda t: (0 if t["service"] == "Frght" else 1),
        )
        for train in ordered:
            sm = sched_of(train)
            f = sm.get(from_id)
            t = sm.get(to_id)
            if not f or not t:
                continue
            dep = _depart(f)
            arr = _arrive(t)
            if dep is None or arr is None:
                continue
            minutes = arr - dep
            if minutes < 0:
                minutes += 24 * 60
            if minutes > 0:
                return minutes
        return None

    result: dict = {sid: {"N": None, "S": None} for sid in station_ids}
    for i in range(len(station_ids) - 1):
        south_id = station_ids[i]
        north_id = station_ids[i + 1]
        result[south_id]["N"] = run_time(south_id, north_id, "N")
        result[north_id]["S"] = run_time(north_id, south_id, "S")
    return result


def station_display_data(subdivision_id: str, station_ids: list[str]) -> dict:
    """
    Return display metadata for specified stations.

    Returns: {station_id: {"types": [...], "flagging_required": bool, "siding_length_cars": int|None}}
    """
    loc_map = {l["id"]: l for l in _subdivision(subdivision_id)["locations"]}
    result = {}
    for sid in station_ids:
        loc = loc_map.get(sid, {})
        result[sid] = {
            "types": [t for t in loc.get("types", []) if t != "I"],
            "flagging_required": loc.get("flagging_required", False),
            "siding_length_cars": loc.get("siding_length_cars"),
        }
    return result


def next_train(
    subdivision_id: str,
    location_id: str,
    direction: str,
    rr_time: str,
    day: int,
) -> Optional[dict]:
    """
    Return the next train after `rr_time` at `location_id` in `direction`.

    `rr_time` is 'HH:MM' in 24-hour format.
    `direction` is 'N' or 'S'.

    Returns a dict with keys: number, class, service, direction, time, note.
    `time` is the arrive or depart time at the stop (whichever is present).
    Returns None if no qualifying train is found.
    """
    now = _time_minutes(rr_time)
    best: Optional[dict] = None
    best_minutes = 10_000  # sentinel > 24*60

    for train in active_trains(subdivision_id, day):
        if train["direction"] != direction:
            continue
        for stop in train["schedule"]:
            if stop["location"] != location_id:
                continue
            # Depart-first: train stays visible until it actually leaves
            t_cmp = stop.get("depart") or stop.get("arrive")
            if t_cmp is None:
                continue
            t_min = _time_minutes(t_cmp)
            # Wrap past midnight: if we're late in the day and the train is early AM
            if t_min < now:
                t_min += 24 * 60
            if t_min < best_minutes:
                best_minutes = t_min
                best = {
                    "number": train["number"],
                    "class": train["class"],
                    "service": train["service"],
                    "direction": train["direction"],
                    "arrive": stop.get("arrive"),
                    "depart": stop.get("depart"),
                    "note": stop.get("note"),
                }
            break  # each train appears at most once per location

    return best
