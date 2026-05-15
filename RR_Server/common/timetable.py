"""
Timetable loader and query module for RR_Server.
Loads timetable.json at startup; all functions are read-only during operations.
"""

import json
from pathlib import Path
from typing import Optional


_timetable: Optional[dict] = None
_data_path: Optional[Path] = None


def load(path: str | Path) -> None:
    global _timetable, _data_path
    _data_path = Path(path)
    with open(_data_path) as f:
        _timetable = json.load(f)


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
    """Return the single most relevant time from a schedule stop."""
    return stop.get("arrive") or stop.get("depart")


def locations(subdivision_id: str) -> list[dict]:
    """Return ordered location list for a subdivision."""
    return _subdivision(subdivision_id)["locations"]


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
            t = _stop_time(stop)
            if t is None:
                continue
            t_min = _time_minutes(t)
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
                    "time": t,
                    "note": stop.get("note"),
                }
            break  # each train appears at most once per location

    return best
