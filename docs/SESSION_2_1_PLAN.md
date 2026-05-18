# Session 2.1 Plan — OS Submission

**Prerequisite:** Sessions 1.1 + 1.2 + 1.3 + 1.4 ✅ complete

---

## Goal

Station agent can submit an OS report from the CYD touchscreen. Dispatcher sees all OS
submissions in a scrolling log. After submitting, the agent sees the next-station timetable
entry before the display returns to the clock screen.

---

## Scope

### In scope
- **Station_OS:** full screen state machine (CLOCK → OS → NEXT_STATION → CLOCK)
- **Station_OS:** OS entry screen — 4×4 keypad, direction select, extra flag, submit
- **Station_OS:** Next-station screen — direction, station name, train depart time
- **Station_OS:** full schedule.json load (all stations) for next-station lookup
- **Dispatcher:** subscribe to `trains/os/+`, OS log in AppState, OS log panel in UI
- **MQTT spec:** add `trains` array field to TO payload (used in Session 2.2)

### Out of scope
- TO text screen and ACK flow → Session 2.2
- Clearance screen → Session 2.3
- Section numbers (always `section: 0` — sections assigned via TO, not this screen)
- Analog clock face → future
- Extras DO appear in the OS entry (the X and WX keys are wired); they just won't
  receive next-station data (no timetable entry exists for an extra)

---

## Screen State Machine

```
             ┌──────────────────────────────────────┐
             │                                      │
             ▼                                      │
          CLOCK ──[touch anywhere]──► OS_ENTRY      │
                                          │         │
                                 [15s no input]     │
                                          │         │
                                          ▼         │
                                        CLOCK       │
                                                    │
          OS_ENTRY ──[✓ valid]──► publish OS ──► NEXT_STATION
                                                    │
                                        [touch or 30s timeout]
                                                    │
                                                    ▼
                                                  CLOCK
```

**OS_ENTRY valid conditions for ✓:** at least 1 digit entered AND direction set (N or S).
Direction has no default — both N and S buttons start unselected; ✓ is disabled until one
is pressed.

**Timeout resets:** every keypress in OS_ENTRY resets the 15s inactivity timer.

---

## OS Entry Screen — Layout (320×240)

```
┌──────────────────────────────────┐   y = 0
│  No. 142          [N]            │   display area (70px)
│                                  │   train number left; direction badge right
│  ──────────────────────────────  │   divider
├────────┬────────┬────────┬───────┤   y = 70
│   1    │   2    │   3    │  [N]  │   row 0 (43px)
├────────┼────────┼────────┼───────┤   y = 113
│   4    │   5    │   6    │  [S]  │   row 1
├────────┼────────┼────────┼───────┤   y = 156
│   7    │   8    │   9    │  [X]  │   row 2
├────────┼────────┼────────┼───────┤   y = 199
│   ⌫    │   0    │   ✓    │  [WX] │   row 3
└────────┴────────┴────────┴───────┘   y = 242
```

**Button widths:** 80px each × 4 = 320px. **Button heights:** 43px × 4 = 172px + 70px display = 242px (fits with small margin).

**Display area (y 0–70):**
- Left: `No. 142` — montserrat_20, white. Shows `No. ─` when empty.
- Right: direction badge — `[N]` green or `[S]` cyan, or `[ ]` muted when unset.
- Extra badge appears below direction when X or WX is active: `X` amber, `WX` amber.

**Button styling:**
| Key | Normal | Active/Selected |
|-----|--------|-----------------|
| 0–9 | dark btn | — |
| N | dark btn | green bg, white text |
| S | dark btn | cyan bg, dark text |
| X | dark btn | amber bg, dark text (toggles) |
| WX | dark btn | amber bg, dark text (toggles) |
| ⌫ | dark btn | — |
| ✓ | grey/disabled | green when valid (digit + direction set) |

**Key behaviors:**
- **Digit:** append to train number string (max 4 digits)
- **N / S:** set direction; deselects the other; N and S are mutually exclusive
- **X:** toggle extra flag (clears WX if set); extra train uses engine# not train#
- **WX:** toggle work-extra flag (clears X if set)
- **⌫:** remove last character; if string already empty, does nothing (timeout returns to clock)
- **✓:** if valid, snapshot OS data, publish MQTT, transition to NEXT_STATION

**LVGL implementation:** `lv_buttonmatrix` widget — the keypad string array from the previous
Station_OS program is reused directly:

```cpp
static const char* btnm_map[] = {
    "1", "2", "3", "N", "\n",
    "4", "5", "6", "S", "\n",
    "7", "8", "9", "X", "\n",
    LV_SYMBOL_BACKSPACE, "0", LV_SYMBOL_OK, "WX", ""
};
```

---

## Next-Station Screen — Layout (320×240)

```
┌──────────────────────────────────┐
│                                  │   y = 0
│         NORTHBOUND               │   direction — montserrat_20, green (N) or cyan (S)
│                                  │   y = 50
│       Hemlock Crest              │   next station name — montserrat_20, white
│                                  │   y = 110
│   No. 3      Dp  10:42 AM       │   train + depart — montserrat_16, muted
│                                  │
│                                  │
│                                  │
│   touch to return to clock       │   montserrat_8, muted, y = 220
└──────────────────────────────────┘
```

**Alternate content:**

| Situation | Line 3 shows |
|-----------|-------------|
| Train found, depart known | `No. 3    Dp  10:42 AM` |
| Train found, depart absent (terminus arrive-only) | `No. 3    Ar  10:42 AM` |
| Train not in next-station schedule (extra or wrong direction) | `No. 42   not scheduled` |
| Current station is a terminus (no next station) | line 2: `End of line`, line 3 blank |

**Touch:** any touch on the screen returns to CLOCK immediately (resets 30s timer too).

---

## Next-Station Lookup Logic (firmware)

The full `schedule.json` is loaded at startup into an in-memory map. On OS submit:

1. Find current station's index in `station_order[]`.
2. Advance index: `+1` for N, `−1` for S.
3. If out of bounds → terminus, show "End of line".
4. Get next station ID and name from `allSchedules[next_id].name`.
5. Search `allSchedules[next_id][direction]` for `num == os_train_num`.
6. If found: display depart time (fall back to arrive if depart absent).
7. If not found: display "not scheduled".

**Data structure change — load all stations:**

```cpp
// Replace station-specific schedN/schedS with:
struct StationData {
    char name[48];
    std::vector<TrainEntry> N;
    std::vector<TrainEntry> S;
};
static std::map<std::string, StationData> allSchedules;
static std::vector<std::string> stationOrder;

// Clock screen still uses pointers into allSchedules for current station:
static const std::vector<TrainEntry>* schedN = nullptr;
static const std::vector<TrainEntry>* schedS = nullptr;
// Set in loadSchedule(): schedN = &allSchedules[cfg.sta_id].N;
```

Loading all 7 stations (~86 entries × ~16 bytes each ≈ 1.4 KB) is well within RAM budget
(currently at 19.1% of ~327 KB = ~62 KB used; 1.4 KB is negligible).

---

## OS Publish (firmware)

On ✓ press with valid entry:

```cpp
// Snapshot clock state at moment of submission
int rr_min = getInterpolatedMin();
int rr_h = rr_min / 60, rr_m = rr_min % 60;

char topic[48], payload[192];
snprintf(topic, sizeof(topic), "trains/os/%s", cfg.sta_id);
snprintf(payload, sizeof(payload),
    "{\"station_id\":\"%s\",\"train\":\"%s\",\"section\":0,"
    "\"direction\":\"%s\",\"extra\":%s,\"work_extra\":%s,"
    "\"rr_time\":\"%02d:%02d\",\"day\":%d}",
    cfg.sta_id, os_train, os_dir,
    os_extra ? "true" : "false",
    os_work_extra ? "true" : "false",
    rr_h, rr_m, clk.day);
mqttClient.publish(topic, 1, false, payload);
```

Note: `work_extra` is not in the current MQTT spec `extra` field — add it (see spec update below).

---

## MQTT Spec Updates

### OS payload: add `work_extra`

Add `"work_extra": false` field to `trains/os/{station_id}` payload. Regular extra uses
`"extra": true, "work_extra": false`; work extra uses `"extra": true, "work_extra": true`.

### TO payload: add `trains` array

Add `"trains": ["3"]` field (array, to support meet orders addressing two trains):

```json
{
  "seq": 7,
  "to_type": "meet",
  "trains": ["3", "42"],
  "text": "No. 3 Eng 101 take siding at Becs Bend and wait for Extra 42 North.",
  "addressed_to": ["BB", "XP"],
  "issued_rr_time": "10:30",
  "day": 1
}
```

Used in Session 2.2 for CYD-to-TO matching after OS submission.

---

## Dispatcher Server Changes

### `mqtt_client.py`
- Subscribe to `trains/os/+` (QoS 1)
- On message: parse payload, append to `AppState.os_log`, broadcast `os_report` event

### `session.py`
```python
@dataclass
class AppState:
    ...
    os_log: list = field(default_factory=list)   # newest first, capped at 50 entries
```

### WebSocket event
```json
{
  "type": "os_report",
  "entry": {
    "station_id": "BB",
    "train": "3",
    "section": 0,
    "direction": "N",
    "extra": false,
    "work_extra": false,
    "rr_time": "10:41",
    "day": 1
  }
}
```

`initial_state` includes `"os_log": [...]` (last 50 entries, newest first).

### `dispatcher.js` / `style.css` / `index.html`
- Add OS log panel below the station table
- Columns: RR Time | Station | Train | Dir | Type
- Type column: blank for scheduled, `X` for extra, `WX` for work extra
- Newest entry at top; max 50 shown
- Green flash animation on new entry

---

## File Changes

| File | Change |
|------|--------|
| `Station_OS/src/main.cpp` | State machine, OS screen, next-station screen, full schedule load |
| `RR_Server/dispatcher/mqtt_client.py` | Subscribe `trains/os/+`, append to os_log |
| `RR_Server/dispatcher/session.py` | Add `os_log` field |
| `RR_Server/dispatcher/app.py` | Include `os_log` in `initial_state` |
| `RR_Server/dispatcher/static/dispatcher.js` | OS log panel rendering |
| `RR_Server/dispatcher/static/style.css` | OS log panel styles |
| `RR_Server/dispatcher/templates/index.html` | OS log panel HTML |
| `RR_Server/tests/test_dispatcher.py` | OS event handling tests |
| `docs/MQTT_SPEC.md` | Add `work_extra` to OS payload; add `trains` to TO payload |
| `docs/IMPLEMENTATION_PLAN.md` | Mark 2.1 as planned |

---

## Completion Criteria

- [ ] Touch on clock screen enters OS entry screen
- [ ] Digit keys build train number; N/S select direction; X/WX toggle extra type
- [ ] ✓ disabled until at least one digit and a direction are set
- [ ] ⌫ removes last digit; 15s inactivity returns to clock from any key state
- [ ] Submitting OS publishes correct MQTT payload to `trains/os/{id}`
- [ ] Next-station screen shows correct station name and depart time for the submitted train
- [ ] Next-station screen returns to clock on touch or 30s timeout
- [ ] Extras show "not scheduled" on next-station screen
- [ ] Terminus stations show "End of line" on next-station screen
- [ ] Dispatcher OS log panel shows all submissions, newest first
- [ ] New OS entry flashes in the dispatcher UI on receipt
- [ ] `initial_state` includes os_log (reconnecting browser sees full session history)
- [ ] All existing tests still pass; new tests cover OS event dispatch
