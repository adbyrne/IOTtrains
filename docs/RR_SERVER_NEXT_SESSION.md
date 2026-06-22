# RR_Server — Next Session Plan

_Updated 2026-06-22_

---

## Status at end of this session

| Item | State |
|------|-------|
| Engine/caboose roster, tap-to-select (`YARDMASTER_DESIGN.md` §14) | ✅ Implemented, tested, committed (`5ea3248`) |
| Equipment status tracking + Hostler role (§15) | 📝 Designed, **not implemented** — full spec in `YARDMASTER_DESIGN.md` §15 |
| Yard CYD firmware (renamed from Hostler CYD) | 📝 Scoped in §15.5 — shared device, job-select screen; only the Hostler job specified |
| Ice House yard job | 🔭 Identified only — cleaning/pre-icing/post-icing, time-delayed, not strictly ordered. Needs its own research/design session before it touches §15 or the Yard CYD firmware at all |
| Crew job descriptions (road crew, mine crew, WP local, helper BB–JC, ice house, hostler) | ⏸ Not started — got sidetracked into the roster/§15 design thread this session |
| Test suite | 157 passing (`pytest` in `RR_Server/`), all green |

---

## Priority 1 — Implement §15 (Equipment Status Tracking)

Can be built and tested headless — no new hardware needed for this part. Full spec: `YARDMASTER_DESIGN.md` §15.1–15.4, §15.6–15.7.

- `AppState.equipment_status` dict, loaded `available` for every roster entry at startup
- MQTT: `MQTTClient.publish_roster_status()`, retained, topics `trains/roster/{engine,caboose}/{road_number}/status`
- Lock trigger: any open consist assignment (`assembling`/`car_block_ready`/`ready`) sets `out` — wire into the existing `/api/yard/consist` path
- Return trigger: OS report handler (`mqtt_client.py`) gains a `station_id == "WP"` + `direction == "S"` case — looks up the consist for that train, sets engine → `being_serviced`, caboose → `awaiting_paperwork`. **Do not touch the existing XP-southbound auto-notify case** — that stays exactly as-is, it's a different concern (early heads-up vs. actual return)
- New endpoints: `POST /api/yard/caboose_paperwork`, `POST /api/hostler/roundhouse` (§15.3)
- YM UI: roster selector grids filter on live `status` now, not just `road_eligible` (§15.4) — hidden, not grayed-out
- YM UI: "Paperwork Delivered" button on Arriving Trains tiles once that consist's equipment has actually returned (§15.4)
- Owner `/manage`: new Equipment Status table + out-of-service toggle, owner-only (§15.6)
- 7 new tests specified in §15.7 — write these alongside the implementation, not after

## Priority 2 — Yard CYD firmware (Hostler job only)

**Blocked on hardware** — needs a physical CYD in hand for bench testing (same pattern as Station_OS's early "hardware tested" commits). New PlatformIO project `IOTtrains/YardCYD/`, stack mirrors Station_OS minus the PCA9685 servo dependency. Job-select home screen with Hostler as the only real job; Ice House gets a placeholder until its own design lands. Full spec: §15.5.

Don't start this until Priority 1 is merged — the firmware needs the `trains/roster/engine/+/status` topic and the roundhouse-confirm flow to actually exist server-side first.

## Priority 3 — Crew job descriptions

Still entirely undiscussed beyond the job names themselves. Needs a scoping conversation before any writing starts:
- Destination: new doc? Expand `NYE_OPERATIONS.md`? Fold into the `controlsystem/` web tour?
- Jobs to cover: road crew, mine crew, WP local, helper service (BB–JC, optional/session-attendance-dependent), ice house switching, hostler — "there may be more jobs added later"
- User welcomed graphics suggestions — not yet discussed what form those would take

## Priority 4 — Ice House job design (separate session, explicitly deferred)

User: "will need research and discussion in its own chat." Open questions going in: what should the Yard CYD actually do for this job (status checklist vs. active timers), is tracking per-car/per-train/facility-level, what "not in order" actually means operationally. Only locked-in constraint so far: whatever this job does, it reports status back over MQTT the same way the Hostler job does.

---

## Notes carried from this session

- `roster.json` engines 10/11/12/14 (the four 0-6-0 switchers) stay `road_eligible: false` and excluded from road-train selectors — unaffected by §15, still yard/mine-only and outside the digital consist system.
- Helper-duty locking an engine (mentioned as a reason for "hidden, not grayed-out" in §15.4) is **not** wired up yet — deferred until helper duty itself has a digital workflow. The status field/filter is general enough to support it later without rework.
- IOTInventory: 11 CYDs in stock (`ESP32-2432S028R`, pk=2) as of 2026-06-22, against 7 needed for stations — check current stock again before any new purchase for Yard CYD / spares.
