#!/usr/bin/env python3
"""
Generate Station_OS/data/schedule.json from data/timetable.json.

Run from the RR_Server directory:
    python3 scripts/gen_cyd_schedule.py
"""
import json
import sys
from pathlib import Path

BASE = Path(__file__).resolve().parent.parent
TIMETABLE = BASE / "data" / "timetable.json"
SCHEDULE  = BASE.parent / "Station_OS" / "data" / "schedule.json"


def hhmm_to_min(t: str | None) -> int | None:
    if not t:
        return None
    h, m = t.split(":")
    return int(h) * 60 + int(m)


def main():
    with open(TIMETABLE) as f:
        tt = json.load(f)

    sub = tt["subdivisions"][0]
    locations = sub["locations"]
    trains = sub.get("trains", [])

    # CYD stations in layout order
    cyd_ids   = [l["id"]   for l in locations if l.get("cyd")]
    cyd_names = {l["id"]: l["name"] for l in locations if l.get("cyd")}
    cyd_set   = set(cyd_ids)

    # Build per-station N/S lists
    stations: dict[str, dict] = {
        sid: {"name": cyd_names[sid], "N": [], "S": []}
        for sid in cyd_ids
    }

    for tr in trains:
        d   = tr["direction"]
        num = tr["number"]
        for stop in tr["schedule"]:
            sid = stop["location"]
            if sid not in cyd_set:
                continue
            entry: dict = {"num": num}
            arr = hhmm_to_min(stop.get("arrive"))
            dep = hhmm_to_min(stop.get("depart"))
            if arr is not None:
                entry["arrive"] = arr
            if dep is not None:
                entry["depart"] = dep
            stations[sid][d].append(entry)

    # Sort each list by departure (falling back to arrival for origin-only stops)
    for sid in cyd_ids:
        for d in ("N", "S"):
            stations[sid][d].sort(key=lambda e: e.get("depart", e.get("arrive", 0)))

    schedule = {"station_order": cyd_ids}
    schedule.update(stations)

    with open(SCHEDULE, "w") as f:
        json.dump(schedule, f, separators=(",", ":"))
    print(f"Written: {SCHEDULE}")

    # Print counts for verification
    for sid in cyd_ids:
        nb = len(stations[sid]["N"])
        sb = len(stations[sid]["S"])
        flag = " !" if nb != sb else ""
        print(f"  {sid}: NB={nb} SB={sb}{flag}")


if __name__ == "__main__":
    main()
