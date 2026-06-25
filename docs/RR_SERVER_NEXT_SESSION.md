# RR_Server — Next Session Plan

_Updated 2026-06-25_

---

## Status at end of this session

| Item | State |
|------|-------|
| Engine/caboose roster, tap-to-select (`YARDMASTER_DESIGN.md` §14) | ✅ Implemented, tested, committed (`5ea3248`) |
| Equipment status tracking + Hostler role (§15) | ✅ Implemented, tested, committed (this session) |
| Yard CYD firmware (Hostler job) | 📝 Scoped in §15.5 — blocked on hardware |
| Ice House yard job | 🔭 Identified only — needs its own research/design session |
| Helper duty digital workflow | 📝 Workflow defined this session — needs design before implementation (see below) |
| Crew job descriptions | ⏸ Not started — needs scoping conversation |
| Test suite | 164 passing (`pytest` in `RR_Server/`), all green |

---

## Priority 1 — Helper duty digital workflow (design session required first)

Helper duty (an engine assisting a heavy train up the BB–JC grade) now has a defined operational workflow but no implementation. A design session is needed before any code is written.

### Confirmed workflow

1. **Dispatcher signals YM** that a helper is needed — dispatcher-initiated notification (inverse of the existing YM→Dispatcher extra request flow; similar to §6.3 Path C)
2. **YM assigns an engine** — engine locks to `out` at assignment time (same lock trigger as §15, but needs a dedicated assignment path since the helper travels light — no caboose, no cars)
3. **TO #1 — Dispatcher issues running extra** (Form G) to move the engine light from WP to helper location (e.g., BB); white extra markers indicate it is an extra
4. **TO #2 — Helper duty order** — authorizes the engine to assist a specific train between two points; **form TBD** (check OPSIG rules book — may be Form C "right over another train," a specific helper rule, or a new TO type)
5. **TO #3 — Dispatcher issues running extra** (Form G) to return engine to WP
6. Crew OSes at WP southbound → engine → `being_serviced` → hostler → `available` (§15 already handles this tail end)

### Open design questions

- **Dispatcher UI**: what button/panel triggers the "helper needed" signal to YM? Probably a new notification type on the dispatcher page, mirroring how the dispatcher receives extra requests
- **YM UI**: how does the YM assign the helper engine? The current consist modal assumes a full train. Options: a "light engine" consist flag, a dedicated helper assignment panel, or a simplified modal with engine-only (no caboose/cars fields)
- **Helper duty TO form**: research in OPSIG rules book (`/home/abyrne/Documents/Trains/OPSIG/19copythreewest/`) — determines what fields the TO dialog needs and whether a new `to_type` is required
- **Lock timing**: engine must lock to `out` at YM assignment, before TO #1 is issued — the assignment step (not the first TO) is the trigger

### What is already in place

- `equipment_status` field/filter is already general-purpose — any path that sets an engine to `out` makes it disappear from the road selector, no rework needed
- `running_extra` TO type (Form G) handles TO #1 and TO #3 with no changes
- WP southbound return trigger and hostler roundhouse flow handle the full return cycle

---

## Priority 2 — Yard CYD firmware (Hostler job only)

**Blocked on hardware** — needs a physical CYD in hand for bench testing (same pattern as Station_OS's early "hardware tested" commits). New PlatformIO project `IOTtrains/YardCYD/`, stack mirrors Station_OS minus the PCA9685 servo dependency. Job-select home screen with Hostler as the only real job; Ice House gets a placeholder until its own design lands. Full spec: §15.5.

Server-side prerequisites are now complete (§15 implemented this session).

---

## Priority 3 — Crew job descriptions

Still entirely undiscussed beyond the job names themselves. Needs a scoping conversation before any writing starts:
- Destination: new doc? Expand `NYE_OPERATIONS.md`? Fold into the `controlsystem/` web tour?
- Jobs to cover: road crew, mine crew, WP local, helper service (BB–JC, optional/session-attendance-dependent), ice house switching, hostler — "there may be more jobs added later"
- Helper duty workflow (Priority 1 above) should be resolved first so the helper crew description is accurate

---

## Priority 4 — Ice House job design (separate session, explicitly deferred)

User: "will need research and discussion in its own chat." Open questions going in: what should the Yard CYD actually do for this job (status checklist vs. active timers), is tracking per-car/per-train/facility-level, what "not in order" actually means operationally. Only locked-in constraint so far: whatever this job does, it reports status back over MQTT the same way the Hostler job does.

---

## Notes carried from this session

- `roster.json` engines 10/11/12/14 (the four 0-6-0 switchers) stay `road_eligible: false` and excluded from road-train selectors — unaffected by §15, still yard/mine-only and outside the digital consist system.
- `equipment_status` is keyed as nested dict `{"engines": {rn: {...}}, "cabooses": {rn: {...}}}` — road numbers 10/11/12/14 appear in both lists so a flat dict is not viable.
- IOTInventory: 11 CYDs in stock (`ESP32-2432S028R`, pk=2) as of 2026-06-22, against 7 needed for stations — check current stock again before any new purchase for Yard CYD / spares.
- Car/industry database (car types required per industry) deferred to the management module — YM does not need this operationally, it is an owner planning tool.
