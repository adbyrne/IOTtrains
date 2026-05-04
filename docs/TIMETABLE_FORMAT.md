# NY&E Timetable JSON Format

**Version:** 1.0
**Date:** 2026-05-04
**Status:** Schema complete — content gaps tracked below

---

## Purpose

`timetable.json` is an **employee (working) timetable** — not a public passenger timetable. It contains the full operational data that crews, the Dispatcher, and the Yardmaster need to do their jobs. A simplified public timetable can be derived from it for posting at stations, but the JSON itself is the authoritative working document.

`timetable.json` serves two consumers:

1. **RR_Server** — loads on startup; answers queries: which trains run today, expected time at a given station, next train per station per direction. Must be machine-parseable with minimal logic.
2. **Timetable Management Tool** (future) — reads/writes this format to generate: the printed employee timetable (Timetable No. 4 layout), per-station condensed schedule cards, string table, extras travel-time columns, and duty reference materials.

The employee timetable contains data not shown in public timetables:
- Siding lengths (cars) — crews need to know if a train clears before a meet
- Flagging requirements — where flagmen must be posted
- Grade and curvature notes — affects speed and braking
- Industry switching notes — what to do at each industry location
- Switchback operating rules — engine/caboose swap procedure and OS timing
- Service facility locations — water, coal, sand, ice

Times are stored as 24-hour strings internally; display as 12-hour AM/PM is the rendering layer's job.

---

## Source Reference

**Timetable No. 4, December 31, 1904** — `NYELayoutDocs/alt/timetable4.pdf`

Current timetable is the starting point. Known changes pending remodel:
- Station name changes: James Crest → Jacks Creek (JC), Michelle Cove → Michelles Cove (MC), High Crust → Hemlock Crest (HC)
- Michelle Cove: type S → TO; CYD installed; gains scheduled arrival/departure times in the new timetable
- Quilly Mines: all three moving from standalone mainline sidings to within adjacent station limits. Within station limits they operate as internal yard moves under Yardmaster direction — no main-line OS, no clearance required. Timetable entries will use Kiel Co style notation (main junction MP + switchback lead distance). Exact host stations and MP data TBD pending remodel confirmation.
- New industries or stations possible

The old timetable data will be transcribed into JSON and updated when remodel decisions are confirmed.

---

## Top-Level Structure

```json
{
  "timetable": { ... },
  "subdivisions": [ ... ]
}
```

### `timetable` object

```json
{
  "timetable": {
    "number": "4",
    "title": "New York and Eastern Division Timetable No. 4",
    "effective": "1904-12-31",
    "notes": [
      "RUN AWAY COAL CARS HAVE PRIORITY",
      "FLAGMEN MUST BE POSTED AT ALL SWITCHBACKS NORTH OF XINA PASS",
      "ALL TRAINS MUST REGISTER AT XINA PASS",
      "SOUTHWARD TRAINS ARE SUPERIOR BY DIRECTION"
    ]
  }
}
```

`notes` appears verbatim in the printed timetable Special Instructions section.

---

## Subdivisions

`subdivisions` is an **ordered array** — print order matters (NLS first, then C&O for yardmaster display).

```json
{
  "subdivisions": [
    {
      "id": "NLS",
      "name": "Northern Lights Subdivision",
      "railroad": "NY&E",
      "direction_superior": "S",
      "locations": [ ... ],
      "trains": [ ... ]
    },
    {
      "id": "COE",
      "name": "C&O East Central Subdivision",
      "railroad": "C&O",
      "direction_superior": null,
      "locations": [ ... ],
      "trains": [ ... ]
    }
  ]
}
```

`direction_superior` is `"N"` or `"S"` — used for OS reporting context and display. `null` for C&O (not applicable to NY&E operations).

---

## Locations

Ordered **south to north** (or west to east for C&O). This order defines printed timetable row order.

```json
{
  "locations": [
    {
      "id": "WP_YARD",
      "name": "Williamsport Yard East",
      "milepost": 0,
      "milepost_exit": null,
      "switchback": false,
      "within_limits_of": null,
      "types": ["YL", "R"],
      "cyd": false,
      "show_times": true,
      "show_both_times": false,
      "siding_length_cars": null,
      "flagging_required": false,
      "service_facilities": [],
      "crew_notes": []
    },
    {
      "id": "WP",
      "name": "Williamsport Station",
      "milepost": 0,
      "milepost_exit": null,
      "switchback": false,
      "within_limits_of": null,
      "types": ["TO", "R"],
      "cyd": true,
      "show_times": true,
      "show_both_times": false,
      "siding_length_cars": null,
      "flagging_required": false,
      "service_facilities": ["water", "coal", "sand", "ice"],
      "crew_notes": []
    },
    {
      "id": "KIEL",
      "name": "Kiel Co",
      "milepost": 4.1,
      "milepost_exit": null,
      "milepost_note": "switchback lead ~1.5 mi — milepost_exit TBD pending remodel",
      "switchback": true,
      "types": ["I"],
      "cyd": false,
      "show_times": false,
      "show_both_times": false
    },
    {
      "id": "OHARAS_LF",
      "name": "O'Haras Lumber and Feed",
      "milepost": 5.6,
      "milepost_exit": null,
      "switchback": false,
      "types": ["I"],
      "cyd": false,
      "show_times": false,
      "show_both_times": false
    },
    {
      "id": "OHARAS_CS",
      "name": "O'Haras Coal Service",
      "milepost": 15.6,
      "milepost_exit": null,
      "switchback": false,
      "types": ["I"],
      "cyd": false,
      "show_times": false,
      "show_both_times": false
    },
    {
      "id": "XP",
      "name": "Xina Pass",
      "milepost": 37,
      "milepost_exit": 43.5,
      "switchback": true,
      "types": ["TO", "R"],
      "cyd": true,
      "show_times": true,
      "show_both_times": true
    },
    {
      "id": "QM1",
      "name": "Quilly Mine #1",
      "milepost": 51.4,
      "milepost_exit": null,
      "switchback": false,
      "types": ["S"],
      "cyd": false,
      "show_times": false,
      "show_both_times": false
    },
    {
      "id": "BB",
      "name": "Becs Bend",
      "milepost": 58.4,
      "milepost_exit": null,
      "switchback": false,
      "types": ["TO"],
      "cyd": true,
      "show_times": true,
      "show_both_times": false
    },
    {
      "id": "JC",
      "name": "Jacks Creek",
      "milepost": 76.4,
      "milepost_exit": null,
      "switchback": false,
      "types": ["TO"],
      "cyd": true,
      "show_times": true,
      "show_both_times": false
    },
    {
      "id": "MC",
      "name": "Michelles Cove",
      "milepost": 78.2,
      "milepost_exit": null,
      "milepost_note": "switchback lead ~1.5 mi southward — milepost_exit TBD pending remodel",
      "switchback": true,
      "within_limits_of": null,
      "types": ["TO"],
      "cyd": true,
      "show_times": true,
      "show_both_times": false,
      "siding_length_cars": null,
      "flagging_required": true,
      "service_facilities": [],
      "crew_notes": []
    },
    {
      "id": "QM2",
      "name": "Quilly Mine #2",
      "milepost": 99.2,
      "milepost_exit": null,
      "switchback": false,
      "types": ["S"],
      "cyd": false,
      "show_times": false,
      "show_both_times": false
    },
    {
      "id": "SK",
      "name": "Stans Knob",
      "milepost": 103.2,
      "milepost_exit": null,
      "switchback": false,
      "types": ["TO"],
      "cyd": true,
      "show_times": true,
      "show_both_times": false
    },
    {
      "id": "QM3",
      "name": "Quilly Mine #3",
      "milepost": 112.2,
      "milepost_exit": 118.2,
      "switchback": true,
      "types": ["S"],
      "cyd": false,
      "show_times": false,
      "show_both_times": false
    },
    {
      "id": "TIMBER",
      "name": "Timber Ltd",
      "milepost": 128.1,
      "milepost_exit": null,
      "switchback": false,
      "types": ["I"],
      "cyd": false,
      "show_times": false,
      "show_both_times": false
    },
    {
      "id": "HC",
      "name": "Hemlock Crest",
      "milepost": 135,
      "milepost_exit": null,
      "switchback": false,
      "types": ["TO", "R"],
      "cyd": true,
      "show_times": true,
      "show_both_times": false
    }
  ]
}
```

### Location field notes

Fields are grouped: **identification**, **physical**, **classification**, **IoT**, **timetable display**, **employee operational**.

#### Identification and Physical

| Field | Type | Notes |
|-------|------|-------|
| `id` | string | Stable short ID — used by trains, MQTT, and RR_Server. Never changes after assignment, even if the name changes. |
| `name` | string | Display name — may change with remodel. |
| `milepost` | float | Entry milepost — where the southbound train enters, or the northbound train first reaches the location. |
| `milepost_exit` | float or null | Switchback stations only: the milepost at the far end of the switchback where the train continues after reversing and reconnecting. Printed as "37 / 43.5". Null for non-switchback locations. |
| `milepost_note` | string | Optional — temporary annotation for unclear secondary milepost values. Remove once confirmed. |
| `switchback` | bool | True = train must reverse at this location; engine and caboose swap ends. Governs OS reporting rule (see Switchback OS Rule section). |
| `within_limits_of` | string or null | If this location operates within the yard limits of another station (e.g. Quilly Mine within Stans Knob limits), set to that station's `id`. Null otherwise. Locations within station limits: no main-line OS, no clearance, Yardmaster-directed operations. |

#### Classification

| Field | Type | Notes |
|-------|------|-------|
| `types` | array of string | One or more: `YL` Yard Limit, `TO` Train Order, `R` Register, `S` Industry Siding, `I` Industry/Freight Stop. |

#### IoT

| Field | Type | Notes |
|-------|------|-------|
| `cyd` | bool | True = CYD unit installed; station participates in digital OS, train orders, and clearance system. |

#### Timetable Display

| Field | Type | Notes |
|-------|------|-------|
| `show_times` | bool | True = time columns appear in the printed timetable. False = station row appears but cells are blank (industry sidings, staging tracks). |
| `show_both_times` | bool | True = printed timetable shows separate Arrive and Depart rows. Set for pass-through register stations (trains must log arrival for the register, then receive clearance before departing). Independent of `switchback`. |

#### Employee Operational Data

These fields appear in the employee timetable only, not in simplified public-facing materials.

| Field | Type | Notes |
|-------|------|-------|
| `siding_length_cars` | int or null | Number of cars the passing siding holds. Crews use this to determine if a train clears before a meet. Null = not applicable (terminus, yard, industry without siding). |
| `flagging_required` | bool | True = flagmen must be posted whenever a train is stopped here (per operating rules: all switchbacks north of Xina Pass). |
| `service_facilities` | array of string | Engine service available: `"water"`, `"coal"`, `"sand"`, `"ice"` (icing for reefer cars). Empty array = no service. |
| `crew_notes` | array of string | Free-text operational notes for crews at this location. Appears in the employee timetable special instructions column and on per-station condensed cards. Examples: switching instructions, caboose swap requirement, industry access procedure. |

### Switchback OS Rule

Switchback stations (`switchback: true`) have a modified OS rule that must be understood by all crews and implemented in the CYD Station_OS screen:

- **Standard:** Train is not OS until the engine and caboose are reconnected after the reversal. The station agent submits the OS report at that point.
- **Work exception:** If the train has switching work to perform at the switchback location, the train OSs on **arrival** (before the reversal). The train must take the siding on arrival. OS is not re-submitted after reconnection.

RR_Server does not enforce this rule — it is a crew and station-agent operating rule. The CYD OS screen should display a reminder at switchback stations: "Work here? OS now + take siding. Otherwise OS after reconnect."

### Location type codes

| Code | Meaning |
|------|---------|
| `YL` | Yard Limit |
| `TO` | Train Order station — signal arm present |
| `R` | Register station — all trains must report; clearance required |
| `S` | Industry siding — switching; no passenger service |
| `I` | Industry / freight stop — local freight only |

---

## Trains

```json
{
  "trains": [
    {
      "number": "3",
      "class": 1,
      "service": "Pass",
      "direction": "N",
      "days": [1, 2, 3, 4, 5, 6, 7],
      "schedule": [
        { "location": "WP",     "depart": "15:45" },
        { "location": "XP",     "arrive": "19:54", "depart": "20:04" },
        { "location": "BB",     "depart": "20:25" },
        { "location": "JC",     "arrive": "20:42", "depart": "20:52" },
        { "location": "SK",     "arrive": "21:02", "depart": "21:12" },
        { "location": "HC",     "arrive": "21:21" }
      ]
    },
    {
      "number": "101",
      "class": 3,
      "service": "Coal",
      "direction": "N",
      "days": [1, 2, 3, 4, 5, 6, 7],
      "schedule": [
        { "location": "WP_YARD", "depart": "03:15" },
        { "location": "XP",      "arrive": "07:31", "depart": "07:41" },
        { "location": "QM1",     "depart": "08:49" },
        { "location": "BB",      "depart": "09:27" },
        { "location": "SK",      "arrive": "09:27", "depart": "09:27" },
        { "location": "HC",      "arrive": "10:21" }
      ]
    }
  ]
}
```

### Train field notes

| Field | Type | Notes |
|-------|------|-------|
| `number` | string | Train number as string (allows "Extra" or future lettered trains). |
| `class` | int | 1 = First, 2 = Second, 3 = Third. Determines print column grouping and priority. |
| `service` | string | `Pass`, `Frght`, `Coal`, `Coke`. Appears in printed timetable column header. |
| `direction` | string | `"N"` or `"S"`. |
| `days` | array of int | 1=Monday … 7=Sunday. `[1,2,3,4,5,6,7]` = Daily. RR_Server filters by current session day. |
| `schedule` | array | Ordered list of stops. Only locations where the train has a time appear. |

### Schedule stop fields

| Field | Type | Notes |
|-------|------|-------|
| `location` | string | Must match a `locations[].id` in this subdivision. |
| `arrive` | string | 24-hour time, "HH:MM". Omit if not shown in timetable. |
| `depart` | string | 24-hour time, "HH:MM". Omit if not shown in timetable. |
| `note` | string | Optional footnote text (e.g. "#112 – wait for No. 112"). Appears in printed timetable as footnote. |

If a stop has only one time, use `depart` for intermediate stops and `arrive` for terminus. RR_Server uses whichever is present for "expected time" calculations.

---

## Printed Timetable Column Order

For generating the printed timetable, trains must be sorted into columns. The sort key is:

- **Northward:** Third Class descending number (141, 131 … 101), then Second Class descending (25, 23, 21), then First Class descending (11, 3, 1) — left to right.
- **Southward:** First Class ascending (2, 4, 52), then Second Class ascending (22, 24, 26), then Third Class ascending (102, 112 … 142) — left to right.

### Extras Columns ("X")

The "X" columns at the outer edges of the printed timetable are **not scheduled trains**. They show the inter-station travel times derived from class speeds and track profile — giving the Dispatcher and crews a reference for how long an extra train takes between any two stations. These values are computed from the Stringline data (`NYELayoutDocs/alt/Stringline.ods`), not from actual train schedules.

The extras columns do **not** appear in `trains[]`. The Timetable Management Tool generates them separately from the Stringline segment data when printing.

---

## C&O Subdivision (Stub — to be expanded)

C&O trains appear in the timetable for Yardmaster awareness only (interchange timing, keeping the main clear). They do not participate in NY&E OS, Train Orders, or Clearances.

```json
{
  "id": "COE",
  "name": "C&O East Central Subdivision",
  "railroad": "C&O",
  "direction_superior": null,
  "locations": [
    { "id": "WP_EAST_STG",  "name": "East Staging (Cumberland / Erland MD)",  "milepost": null, "types": [], "cyd": false, "show_times": true, "show_both_times": false },
    { "id": "WP_COE",       "name": "Williamsport (C&O)",                       "milepost": null, "types": [], "cyd": false, "show_times": true, "show_both_times": true  },
    { "id": "WP_WEST_STG",  "name": "West Staging (Covington WV)",              "milepost": null, "types": [], "cyd": false, "show_times": true, "show_both_times": false }
  ],
  "trains": []
}
```

C&O train data to be added when the C&O schedule is defined. Minimum fields needed: train number/name, direction, days, Williamsport arrival and departure time.

---

## RR_Server Usage

The server loads `timetable.json` at startup (read-only during operations). Queries it supports:

- **Active trains today:** filter `trains` where `days` contains current session day.
- **Next train at station:** for a given `location_id` and direction, find the active train with the smallest future `arrive` or `depart` time relative to current RR time.
- **Train schedule:** return full stop list for a given train number.
- **Station list:** return all locations for a subdivision (used by Dispatcher UI station tiles).

---

## Pending Content (schema complete — data gaps only)

The JSON schema is finalized. The following fields have placeholder or stub values pending source data. These do not block RR_Server implementation.

| Gap | Placeholder | Source |
|-----|-------------|--------|
| Quilly Mine host stations (`within_limits_of`) and `milepost_exit` values for Kiel Co and MC | `milepost_note` strings in place | XTrkCAD MCP query once tool is available; MP = 1 ft real track |
| Siding lengths (`siding_length_cars`) at all stations | `null` | XTrkCAD MCP query |
| Service facilities and crew_notes per location | `[]` / `[]` | Hand-entered from ops knowledge |
| C&O subdivision trains | Stub `"trains": []` | Pull from Stringline.ods spreadsheet |
| New industries or stations from remodel | Current timetable data only | Update incrementally as remodel decisions are confirmed |

### Schema decisions (resolved)

- **Footnotes:** Free text `note` field on schedule stops. Meets are shown in the string table, not as footnote cross-references.
- **Days:** All current trains are daily (`[1,2,3,4,5,6,7]`). The `days` field is ready for restricted-day trains when added.
- **Employee data in print:** JSON stores all fields fully. How employee data appears in the printed timetable (symbols+legend or extra tables) is a Timetable Management Tool rendering decision, deferred to that tool's design session.
- **JSON vs printed format:** `timetable.json` is the base data layer. The printed timetable layout follows the existing Stringline.ods spreadsheet format.

---

## Revision History

| Version | Date | Change |
|---------|------|--------|
| 0.1 | 2026-05-04 | Initial draft — schema design, NLS locations from Timetable No. 4 |
| 0.2 | 2026-05-04 | Dual milepost → `milepost` + `milepost_exit` + `switchback` fields; switchback OS rule documented; `show_both_times` clarified as register-station driven, not switchback driven; extras "X" columns documented as Stringline-derived travel times |
| 0.3 | 2026-05-04 | Kiel Co and Michelle Cove confirmed as switchbacks (lead ~1.5 mi, milepost_exit deferred pending remodel) |
| 0.4 | 2026-05-04 | Employee timetable vs public timetable distinction established as core design principle; employee operational fields added (`siding_length_cars`, `flagging_required`, `service_facilities`, `crew_notes`); `within_limits_of` field added for station-limits locations; Quilly Mines confirmed as moving to station limits (Kiel Co MP notation style, yard-rules operation); MC confirmed `show_times: true`; field notes reorganized by group |
| 1.0 | 2026-05-04 | Schema complete. Remaining open questions resolved: footnotes = free text; all current trains daily; employee print format deferred to Timetable Mgmt Tool; C&O and XTrkCAD gaps tracked as content (not schema blockers). Open questions section replaced with Pending Content table. |
