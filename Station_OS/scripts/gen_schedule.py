#!/usr/bin/env python3
"""Generate schedule.json for Station_OS LittleFS from the server timetable.

Usage: python3 gen_schedule.py [--timetable PATH] [--out PATH]

Outputs a compact schedule file containing all CYD station stops.
The file includes a station_order array so each CYD unit can determine
its previous and next neighbors by looking up its own station_id.
"""

import argparse
import json
import sys
from pathlib import Path

HERE = Path(__file__).parent
DEFAULT_TIMETABLE = HERE / "../../RR_Server/data/timetable.json"
DEFAULT_OUT       = HERE / "../data/schedule.json"


def time_to_min(t: str) -> int:
    h, m = map(int, t.split(":"))
    return h * 60 + m


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timetable", type=Path, default=DEFAULT_TIMETABLE)
    parser.add_argument("--out",       type=Path, default=DEFAULT_OUT)
    args = parser.parse_args()

    with open(args.timetable) as f:
        data = json.load(f)

    nls = next(s for s in data["subdivisions"] if s["id"] == "NLS")

    # Build ordered list of CYD stations (preserves timetable order: S→N)
    cyd_stations = [
        loc for loc in nls["locations"] if loc.get("cyd")
    ]
    station_order = [loc["id"] for loc in cyd_stations]

    schedule = {}
    for loc in cyd_stations:
        schedule[loc["id"]] = {
            "name": loc["name"],
            "N": [],
            "S": [],
        }

    for train in nls["trains"]:
        num = train["number"]
        direction = train["direction"]
        for stop in train["schedule"]:
            loc_id = stop["location"]
            if loc_id not in schedule:
                continue
            # Prefer depart time for en-route reference; fall back to arrive
            t_str = stop.get("depart") or stop.get("arrive")
            if not t_str:
                continue
            schedule[loc_id][direction].append({"num": num, "time": time_to_min(t_str)})

    # Sort by scheduled time within each direction
    for sta in schedule.values():
        sta["N"].sort(key=lambda x: x["time"])
        sta["S"].sort(key=lambda x: x["time"])

    output = {"station_order": station_order, **schedule}

    args.out.parent.mkdir(parents=True, exist_ok=True)
    with open(args.out, "w") as f:
        json.dump(output, f, separators=(",", ":"))

    total_trains = sum(
        len(s["N"]) + len(s["S"]) for s in schedule.values()
    )
    size = args.out.stat().st_size
    print(f"Generated {args.out}")
    print(f"  {len(station_order)} CYD stations, {total_trains} stop entries, {size} bytes")


if __name__ == "__main__":
    main()
