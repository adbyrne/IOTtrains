# Session 2.2 Plan — Train Orders

**Prerequisite:** Sessions 1.1 + 1.2 + 1.3 + 1.4 + 2.1 + 2.4 ✅ complete  
**TO type definitions:** `data/to_types.json` v1.1 ✅ complete (2026-05-19)

---

## Goal

Dispatcher issues a structured Train Order from the web UI. The order text is generated from
the type definition and field values. Addressed station CYDs receive the order, display it after
the crew OS's, and ACK. Dispatcher sees per-station ACK status. TO signal arms raise on issue.

---

## Scope

### In scope
- **Server:** load `data/to_types.json` at startup; new module `to_renderer.py` for template
  rendering and field validation
- **Dispatcher UI:** TO form — type selector, dynamic fields per type, live text preview,
  addressed-station multi-select, Issue button
- **Dispatcher UI:** active TOs panel — per-TO row with per-station ACK status
- **Dispatcher UI:** TO signal arms auto-raise (both N and S) at all addressed stations on issue;
  dispatcher lowers manually
- **Station_OS:** subscribe `trains/to/{station_id}`, store pending TOs in memory
- **Station_OS:** TO text screen — display order text and form, ACK button; shown after OS
  submission if a matching TO is pending for the submitted train number
- **Station_OS:** ACK publish → `trains/to/{station_id}/ack`
- **Server:** ACK handling — update AppState, broadcast `to_ack` WebSocket event

### Out of scope
- Multiple pending TOs for the same train → show first match; "next order" button deferred
- Clearance forms → Session 2.3
- Phase 6 smarter direction inference for signal arm auto-raise
- `annulment` side effect on Yardmaster page (yard notification) → Session 2.0

---

## New Server Module — `to_renderer.py`

Loaded at startup by `app.py`. Provides two public functions.

### `load_to_types(path) → dict`

Reads `data/to_types.json`. Returns the parsed registry. Called once at startup; stored on
`AppState`. Station name lookup table built from `timetable.py` `locations()`.

### `validate_fields(type_def, fields) → list[str]`

Returns a list of error strings (empty = valid). Checks:
- All `required: true` fields are present and non-empty
- Fields with `hidden_when` conditions are skipped when hidden
- `form` field stamped from `default_form` (not user-supplied)

### `render_to(to_type, fields, station_names) → str`

Renders the text template. Substitutes field values and computed variables:

| Computed var | Derivation |
|---|---|
| `{station_name}` | `station_names[fields["station"]]` |
| `{from_station_name}` | `station_names[fields["from_station"]]` |
| `{to_station_name}` | `station_names[fields["to_station"]]` |
| `{direction_word}` | `"North"` if `direction == "N"` else `"South"` |
| `{direction_b_word}` | `"North"` if `direction_b == "N"` else `"South"` |
| `{train_b_ref}` | if `train_b_is_extra`: `"Extra {train_b} {direction_b_word}"`; else: `"No. {train_b} Eng {engine_b}"` |
| `{partial_suffix}` | if both `from_station` + `to_station` present: `" {from} to {to}"`; else `""` |

### `trains_for_to(to_type, fields) → list[str]`

Returns the `trains[]` array for the MQTT payload — the train/engine numbers the CYD uses
to match this TO to an OS submission:

| Type | Returns |
|------|---------|
| `meet` | `[train_a, train_b]` |
| `wait` | `[train]` |
| `running_extra` | `[engine]` |
| `work_extra` | `[engine]` |
| `annulment` | `[train]` |
| `sections` | `[train]` |

---

## Dispatcher UI Changes

### TO form panel (replaces placeholder in current `index.html`)

```
┌────────────────────────────────────────────┐
│ ISSUE TRAIN ORDER                          │
│ Type: [ Meet Order              ▼ ]        │
│ ──────────────────────────────────────     │
│ Train A (takes siding): [___]  Eng: [___]  │
│ Train B is extra: [ ]                      │
│ Train B:          [___]  Eng: [___]        │
│                         Dir: [ N ▼ ]       │
│ Meet Station: [ BB ▼ ]                     │
│ ──────────────────────────────────────     │
│ Stations:                                  │
│ [ ] WP  [✓] XP  [✓] BB  [ ] JC            │
│ [ ] MC  [ ] SK  [ ] HC                     │
│ ──────────────────────────────────────     │
│ No. 3 Eng 101 take siding at Becs Bend     │
│ and wait for Extra 42 North.               │
│ ──────────────────────────────────────     │
│              [ ISSUE ]                     │
└────────────────────────────────────────────┘
```

- Type dropdown populated from `GET /api/to_types` on page load.
- Fields rendered dynamically in JS when type changes; `hidden_when` conditions applied.
- Text preview updates on any field change (client-side rendering matching server logic).
- Station checkboxes: all 7 stations, none pre-selected.
- Issue button disabled until all required fields are filled and at least one station selected.

### Active TOs panel (new section below OS log)

```
ACTIVE TRAIN ORDERS
Seq  Type   Text (truncated)                  WP  XP  BB  JC  MC  SK  HC
 7   meet   No. 3 Eng 101 take siding at…    —   ✓   ✓   —   —   —   —
 8   wait   No. 141 Eng 14 take siding at…  —   —   ✓   ✓   —   —   —
```

- ACK tick (✓) per station when received; addressed stations shown; non-addressed show `—`.
- Issued RR time and day shown in expandable row detail.

### API endpoints

#### `GET /api/to_types`
Returns `to_types.json` content (the full registry for the UI to render forms).

#### `POST /api/to/issue`
```json
{
  "to_type": "meet",
  "fields": {
    "train_a": "3", "engine_a": "101",
    "train_b_is_extra": true, "train_b": "42",
    "direction_b": "N", "station": "BB"
  },
  "addressed_to": ["XP", "BB"]
}
```
Server response (200):
```json
{
  "seq": 7,
  "form": "19",
  "to_type": "meet",
  "trains": ["3", "42"],
  "text": "No. 3 Eng 101 take siding at Becs Bend and wait for Extra 42 North.",
  "addressed_to": ["XP", "BB"],
  "issued_rr_time": "10:30",
  "day": 1
}
```
On issue the server:
1. Validates fields via `to_renderer.validate_fields()`.
2. Renders text via `to_renderer.render_to()`.
3. Stamps `form` from `to_types["default_form"]`.
4. Assigns next `to_seq` from AppState.
5. Publishes to `trains/to/{station_id}` (QoS 2) for each addressed station.
6. Publishes `{"state": "raised"}` to `trains/signal/{station_id}/to/N/cmd` and `.../to/S/cmd`
   (QoS 1, retained) for each addressed station that has a TO signal (XP, BB, JC, MC, SK).
7. Appends to `AppState.active_tos`; broadcasts `to_issued` WebSocket event.

### WebSocket events

```json
{ "type": "to_issued", "to": { ...full payload... } }
{ "type": "to_ack",    "seq": 7, "station_id": "BB", "rr_time": "10:33", "copies": 2 }
```

`initial_state` includes `"active_tos": [...]`.

---

## AppState Changes (`session.py`)

```python
@dataclass
class ToRecord:
    seq: int
    form: str
    to_type: str
    trains: list
    text: str
    addressed_to: list
    issued_rr_time: str
    day: int
    acks: dict   # station_id → {"rr_time": ..., "copies": ...}

@dataclass
class AppState:
    ...
    to_seq: int = 0
    active_tos: list = field(default_factory=list)   # ToRecord list, newest first
```

---

## Station_OS Changes (`main.cpp`)

### Subscribe + store TOs

On `trains/to/{station_id}` receipt (QoS 2):
- Parse payload into `PendingTO` struct.
- Append to `pendingTOs` vector (up to 10 entries; oldest dropped if full).

```cpp
struct PendingTO {
    int  seq;
    char form[4];
    char to_type[16];
    char trains[4][8];   // up to 4 train numbers from trains[] array
    int  trains_count;
    char text[512];
    char issued_rr_time[8];
    int  day;
};
static std::vector<PendingTO> pendingTOs;
```

### Screen state machine update

After OS submit, before transitioning to NEXT_STATION:
1. Search `pendingTOs` for first entry where any `trains[i]` matches `os_train`.
2. If found → transition to `TO_TEXT` screen; store matched TO seq as `activeTOSeq`.
3. If not found → skip directly to `NEXT_STATION`.

Updated flow:
```
CLOCK ──[touch]──► OS_ENTRY ──[submit]──► [TO match?] ──Yes──► TO_TEXT ──[ACK]──► NEXT_STATION ──► CLOCK
                                 │                               │
                          [15s timeout]                    [15s timeout]
                                 │                               │
                               CLOCK                           CLOCK
```

### TO_TEXT screen layout (320×240)

```
┌──────────────────────────────────┐
│  TRAIN ORDER  ·  Form 19         │   header bar, muted yellow bg
│  ──────────────────────────────  │
│                                  │
│  No. 3 Eng 101 take siding at    │   montserrat_14, white, word-wrap
│  Becs Bend and wait for          │
│  Extra 42 North.                 │
│                                  │
│                                  │
│  Issued: 10:30 AM                │   montserrat_10, muted
│                                  │
│           [ ✓ ACK ]              │   large green button, 80×40px, centered
└──────────────────────────────────┘
```

- 15s inactivity timeout → return to CLOCK (unACK'd order stays in `pendingTOs`).
- ACK button: publish `trains/to/{station_id}/ack`, remove matched entry from `pendingTOs`,
  transition to NEXT_STATION.

### ACK publish

```cpp
char topic[64], payload[128];
snprintf(topic, sizeof(topic), "trains/to/%s/ack", cfg.sta_id);
snprintf(payload, sizeof(payload),
    "{\"seq\":%d,\"station_id\":\"%s\",\"rr_time\":\"%02d:%02d\",\"copies\":2}",
    activeTOSeq, cfg.sta_id, rr_h, rr_m);
mqttClient.publish(topic, 1, false, payload);
```

`copies: 2` is hardcoded per prototype practice (agent signs two carbon copies).

---

## File Changes

| File | Change |
|------|--------|
| `RR_Server/dispatcher/to_renderer.py` | New: load_to_types, validate_fields, render_to, trains_for_to |
| `RR_Server/dispatcher/app.py` | GET /api/to_types; POST /api/to/issue; handle trains/to/+/ack |
| `RR_Server/dispatcher/mqtt_client.py` | Subscribe trains/to/+/ack; publish TO to stations; publish signal arm raise |
| `RR_Server/dispatcher/session.py` | Add ToRecord, active_tos, to_seq to AppState |
| `RR_Server/dispatcher/static/dispatcher.js` | TO form UI with live preview; active TOs panel with ACK status |
| `RR_Server/dispatcher/static/style.css` | TO form and active TOs panel styles |
| `RR_Server/dispatcher/templates/index.html` | TO form and active TOs panel HTML |
| `Station_OS/src/main.cpp` | Subscribe trains/to/{id}; PendingTO storage; TO_TEXT screen state; ACK publish |
| `RR_Server/tests/test_to_renderer.py` | New: rendering tests for all 6 types; validation tests |
| `RR_Server/tests/test_dispatcher.py` | Add: TO issuance endpoint; ACK handling; initial_state includes active_tos |

---

## Completion Criteria

- [ ] `GET /api/to_types` returns registry; dispatcher form fields update on type change
- [ ] All 6 TO types render valid text from known-good field sets
- [ ] Invalid/missing required fields return 422 with field-level error messages
- [ ] `form: "19"` stamped on every issued TO from `default_form` in to_types.json
- [ ] Issuing a TO publishes QoS 2 message to each addressed station
- [ ] Both N and S signal arms raised at addressed TO-signal stations (XP/BB/JC/MC/SK) on issue
- [ ] CYD receives and stores TO; TO text screen appears after OS submit for matching train
- [ ] TO text screen shows order text, form number, and issued time
- [ ] ACK button publishes correct payload; entry removed from pendingTOs; transitions to NEXT_STATION
- [ ] Dispatcher UI active TOs panel shows ACK tick per station on receipt
- [ ] `initial_state` includes active_tos (reconnecting browser sees current TO state)
- [ ] 15s timeout on TO text screen returns to CLOCK (unACK'd TO stays stored)
- [ ] All existing tests still pass; new tests cover rendering, issuance, and ACK flow
