# Timetable Management — Design Document

## Problem

The timetable (`data/timetable.json`) is the authoritative source for:
- Next-train lookups on the dispatcher panel and Station OS CYD units
- Train number/class/direction data used in superiority calculations
- Employee information (siding lengths, service facilities, crew notes) for ops sessions

Currently it is hand-edited JSON with no in-app tooling. A new timetable issue (e.g. schedule change, new train) requires editing the file in a text editor and restarting the server. There is also no way to publish a formatted timetable for operators.

## Goals

1. **Edit train schedules** without touching raw JSON or restarting the server
2. **Hot-reload** timetable data instantly after a save (no dispatcher restart)
3. **Publish** a formatted employee timetable (HTML table, suitable for printing)
4. **Future:** Import siding-length and milepost data from XTrkCAD file (`~/XTrkCAD/nyelayout/layout.xtc`)

## Data Model Layers

The timetable JSON has four logical layers with different editing frequencies:

| Layer | Fields | Edit frequency | Scope |
|-------|--------|----------------|-------|
| **Metadata** | `number`, `title`, `effective`, `notes[]` | Rare (new TT issue) | Phase 1 |
| **Trains** | `number`, `class`, `service`, `direction`, `days`, `schedule[]` | Occasionally (schedule changes) | Phase 1 |
| **Train stops** | `location`, `arrive`, `depart`, `note` | With train edits | Phase 1 |
| **Locations** | `name`, `milepost`, `siding_length_cars`, `service_facilities`, `crew_notes`, etc. | Rarely; some TBD pending remodel | Phase 2 |
| **XTrkCAD import** | `milepost`, `siding_length_cars` | One-time import + updates | Phase 3 |

## Proposed Architecture

### Hot-Reload

Add a `timetable.reload()` function to `common/timetable.py` that re-reads and re-parses the file in place. The `POST /api/timetable` endpoint calls `reload()` after writing. No server restart needed.

Since `timetable.py` uses module-level globals, a reload just replaces the `_timetable` dict — thread-safe with FastAPI's single-threaded async event loop (the MQTT callback uses `run_coroutine_threadsafe`, which doesn't call `next_train()` concurrently).

### API Endpoints (owner-protected)

```
GET  /api/timetable              → full timetable JSON (read)
POST /api/timetable/meta         → update metadata (number, title, effective, notes)
POST /api/timetable/train        → create or update a train (upsert by number+direction)
DELETE /api/timetable/train/{id} → delete a train
POST /api/timetable/reload       → force reload from disk (if edited externally)
GET  /api/timetable/export/html  → rendered employee timetable HTML
```

Writes go to `data/timetable.json` atomically (write temp → rename). On each write, `timetable.reload()` is called so the running server picks up changes immediately.

### /manage UI — Timetable Section

Three panels below the existing Layout Rules and Active Forms panels:

**Panel 1: Timetable Info**
- Fields: TT number, title, effective date
- Textarea: notes (one per line)
- Save button → `POST /api/timetable/meta`

**Panel 2: Train Schedules**
- Table: one row per train (number | class | service | direction | days | actions)
- Click a row → expands to an inline stop editor (location dropdown + arrive/depart/note fields per stop)
- "Add Train" button → blank expand form
- "Delete" button per train (confirm dialog)
- Save per train → `POST /api/timetable/train`

**Panel 3: Export**
- "Generate Employee Timetable" button → opens `/api/timetable/export/html` in a new tab
- Print-ready HTML following Stringline.ods conventions:
  - Two-column layout: N-direction trains left, S-direction trains right
  - Superior direction (S) trains on the right half, time values increasing upward
  - Station rows, with employee columns (siding length, flagging, facilities)
  - Notes section at bottom

## Employee Timetable Format (Print)

Based on Stringline.ods conventions and prototype timetable format:

```
NEW YORK AND EASTERN RAILROAD
NORTHERN LIGHTS SUBDIVISION — EMPLOYEE TIMETABLE NO. 4
Effective: December 31, 1904

SECOND CLASS   FIRST CLASS  |         | FIRST CLASS   SECOND CLASS   THIRD CLASS
No. 25  No. 23  No. 21      | STATION | No. 1   No. 3              No. 141 ...
                             |         |
21:36   12:21   02:52  HC   |  135.0  |  08:19  16:54
                             |  103.2  |
21:17   12:02   02:33   SK   |         |  08:03  16:38
...
SOUTHWARD TRAINS ARE SUPERIOR BY DIRECTION
[notes listed here]
```

Superior trains (S) appear on the right half. Times read bottom-up for southward trains (prototype convention for employee TTs).

## What XTrkCAD Will Provide (Phase 3)

From `~/XTrkCAD/nyelayout/layout.xtc` (MP = 1 ft real track at layout scale):
- **Siding lengths** in feet → convert to car counts using average car length (~7 cars/siding at HO)
- **Physical layout positions** → confirm/correct mileposts in timetable.json

The import would be semi-automated: parse XTrkCAD file, compute proposed values, present a diff for review before applying to timetable.json.

## StringTable Publication (Train Graph)

The `StringTable19-1.ods` spreadsheet (in `NYELayoutDocs/alt/`) is a visual train graph — a matrix of time (rows, 5-min fast-clock intervals) vs. location (columns), with train numbers placed at their position. It is the primary tool for verifying that meets are correctly scheduled and detecting conflicts.

**This must also be published for the dispatcher** alongside the employee timetable.

The StringTable can be generated from the timetable data:
- Columns: mainline locations in geographic order (WP → HC)
- Rows: time intervals (every 5 RR-minutes, covering 24 hours = 288 rows)
- Cell content: train number where a train is present (between depart of previous station and arrive at next)
- Visual convention: N trains move left-to-right (top of page = WP), S trains right-to-left

**Add to `/api/timetable/export/`:**
```
GET /api/timetable/export/html         → employee timetable (tabular)
GET /api/timetable/export/stringtable  → train graph (visual matrix)
```

Both can be opened in a browser tab and printed. The stringtable is particularly useful at a dispatcher session for predicting meets.

**Source files for reference:**
- `NYELayoutDocs/alt/StringTable19-1.ods` — current version (old station names)
- `NYELayoutDocs/alt/Stringline.ods` — speed/distance calculation basis

---

## Speed and Schedule Configuration

The `Stringline.ods` spreadsheet contains the speed model used to compute schedule times:

| Train Class | Speed (in/s model) | Approx mph |
|-------------|-------------------|------------|
| Passenger   | 9                 | ~36 mph    |
| Fast Freight | 7                | ~28 mph    |
| Freight     | 5                 | ~20 mph    |
| Local       | 4                 | ~16 mph    |
| Loaded Coal | 3                 | ~12 mph    |
| Yard        | 1                 | ~4 mph     |

Track and engine multipliers applied to base speed:

| Condition | Multiplier |
|-----------|-----------|
| On Grade  | 0.5       |
| Switchback track | 1 (max speed capped at 1 in/s) |
| Tender First | 0.75  |
| Pushing   | 0.2       |

These multipliers determine how long each track segment takes to traverse, and therefore where station times land.

**New management function: Speed Configuration** (`/manage → Speed Settings`):
- Owner-editable table of train class speeds (in/s or mph)
- Engine placement multipliers (tender-first, pushing)
- Track type overrides (grade %, switchback cap)
- Stored in `data/speed_config.json`
- Used by a schedule calculator to recompute suggested station times when a speed changes
- Schedule calculator output is a preview only — dispatcher reviews and saves the proposed schedule

This allows the owner to tune speeds to match observed model performance and recalculate affected trains without hand-editing all the times.

**`data/speed_config.json` structure (proposed):**
```json
{
  "train_classes": {
    "Pass":    { "speed_ins": 9,  "description": "Passenger" },
    "Frght":   { "speed_ins": 5,  "description": "Freight" },
    "Coal":    { "speed_ins": 3,  "description": "Loaded Coal" },
    "Coke":    { "speed_ins": 3,  "description": "Loaded Coke" }
  },
  "engine_placement": {
    "forward":      1.0,
    "tender_first": 0.75,
    "pushing":      0.2
  },
  "track_types": {
    "normal":     { "multiplier": 1.0, "max_speed_ins": null },
    "grade":      { "multiplier": 0.5, "max_speed_ins": null },
    "switchback": { "multiplier": 1.0, "max_speed_ins": 1.0 },
    "yard":       { "multiplier": 1.0, "max_speed_ins": 1.0 }
  }
}
```

---

## Known Timetable Changes Applied (2026-05-19)

- **QM1 → XP spur**: Quilly Mine #1 reclassified as industry spur off Xina Pass. Trains 101/121/141 now terminate at XP (setout); trains 102/122/142 originate at XP (pickup). QM1 location kept for reference with `within_limits_of: "XP"`.
- **QM2 → MC spur**: Quilly Mine #2 reclassified as industry spur off Michelles Cove. MC milepost updated to 99.2 (QM2 prototype distance, pending XTrkCAD survey). Trains 111/131 now terminate at MC; trains 112/132 originate from MC. QM2 location kept with `within_limits_of: "MC"`.

---

## Deferred

- Location editor (name, milepost, employee fields) — deferred pending layout remodel completion
- Day-of-week filtering UI — currently all trains run daily (days=[1..7]), deferred
- Conflict detection (automatic meet verification) — partially served by StringTable
- XTrkCAD import parser (Phase 3) — provides siding lengths and final mileposts for MC, QM1 spur, QM2 spur

## Implementation Order

1. `timetable.py`: add `reload()` and atomic `save(data)` functions
2. `app.py`: add owner-protected timetable API endpoints
3. `manage.html`: add Timetable Info panel + Train Schedules panel (read-only view first)
4. Train CRUD UI (edit/add/delete trains and stops)
5. HTML timetable export endpoint + template
6. StringTable (train graph) export endpoint + generator
7. `data/speed_config.json` + Speed Settings management panel
8. Schedule calculator (recompute suggested times from speed config)
