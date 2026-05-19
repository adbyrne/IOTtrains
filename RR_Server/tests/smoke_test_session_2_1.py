#!/usr/bin/env python3
"""Interactive smoke tests for Session 2.1 — OS Submission.

Requires:
  - CYD unit running current Station_OS firmware, connected to broker
  - MQTT broker accessible from this machine
  - Dispatcher web app open in a browser (for UI section)
  - rr-dispatcher service running (for clock setup)

Usage:
  python3 tests/smoke_test_session_2_1.py [--broker 192.168.10.1] [--station BB]
                                           [--dispatcher http://192.168.86.36:5000]

The MQTT section is semi-automated: the script subscribes to trains/os/+
and captures messages so you don't need to watch a MQTT client separately.
The script sets the RR clock to a known time (08:00 Mon) before the next-station
tests so that expected times are deterministic.
All other checks are prompt-and-confirm.
"""

import argparse
import json
import queue
import sys
import threading
import time
import urllib.request
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

try:
    import paho.mqtt.client as mqtt_lib
    HAS_PAHO = True
except ImportError:
    HAS_PAHO = False

# ── Colour helpers ────────────────────────────────────────────────────────────

GREEN  = "\033[32m"
RED    = "\033[31m"
YELLOW = "\033[33m"
CYAN   = "\033[36m"
BOLD   = "\033[1m"
RESET  = "\033[0m"

PASS_TAG = f"{GREEN}PASS{RESET}"
FAIL_TAG = f"{RED}FAIL{RESET}"
SKIP_TAG = f"{YELLOW}SKIP{RESET}"
INFO_TAG = f"{CYAN}INFO{RESET}"

# ── Result tracking ───────────────────────────────────────────────────────────

results: list[tuple[str, str]] = []   # (name, 'pass'|'fail'|'skip')


def _ask_yn(label: str) -> str | None:
    """Prompt for y/n/s, return 'pass', 'fail', or None (skip)."""
    while True:
        raw = input(f"  {label} [y/n/s]: ").strip().lower()
        if raw in ("y", "yes"):   return "pass"
        if raw in ("n", "no"):    return "fail"
        if raw in ("s", "skip"):  return None
        print("  Enter y (yes), n (no), or s (skip).")


def step(name: str, action: str, expected: str = "") -> str | None:
    """Print action/expected, ask y/n/s, record result. Returns result string."""
    print(f"\n{BOLD}  ▶ {name}{RESET}")
    print(f"    Action:   {action}")
    if expected:
        print(f"    Expected: {expected}")
    result = _ask_yn("Pass?")
    if result == "pass":
        print(f"    {PASS_TAG}")
    elif result == "fail":
        print(f"    {FAIL_TAG}")
    else:
        print(f"    {SKIP_TAG}")
    results.append((name, result or "skip"))
    return result


def info(msg: str) -> None:
    print(f"  {INFO_TAG} {msg}")


def section(title: str) -> None:
    print(f"\n{BOLD}{CYAN}{'─'*60}{RESET}")
    print(f"{BOLD}{CYAN}  {title}{RESET}")
    print(f"{BOLD}{CYAN}{'─'*60}{RESET}")


# ── MQTT listener ─────────────────────────────────────────────────────────────

_os_queue: queue.Queue = queue.Queue()
_mqtt_client = None
_mqtt_connected = False


def _start_mqtt_listener(broker_host: str, broker_port: int,
                          username: str, password: str) -> bool:
    """Subscribe to trains/os/+ in a background thread. Returns True if connected."""
    global _mqtt_client, _mqtt_connected

    if not HAS_PAHO:
        info("paho-mqtt not importable — MQTT auto-capture disabled")
        return False

    connected_evt = threading.Event()

    def on_connect(client, userdata, flags, rc, props=None):
        global _mqtt_connected
        if hasattr(rc, 'is_failure'):   # paho v2
            ok = not rc.is_failure
        else:
            ok = (rc == 0)
        _mqtt_connected = ok
        if ok:
            client.subscribe("trains/os/+", qos=1)
        connected_evt.set()

    def on_message(client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode())
        except Exception:
            payload = {"raw": msg.payload.decode()}
        _os_queue.put({"topic": msg.topic, "payload": payload, "ts": time.time()})

    try:
        client = mqtt_lib.Client(
            mqtt_lib.CallbackAPIVersion.VERSION2,
            client_id="smoke_test_2_1",
        )
        client.on_connect = on_connect
        client.on_message = on_message
        if username:
            client.username_pw_set(username, password)
        client.connect_async(broker_host, broker_port, keepalive=30)
        client.loop_start()
        connected_evt.wait(timeout=5)
        _mqtt_client = client
        return _mqtt_connected
    except Exception as e:
        info(f"MQTT connect error: {e}")
        return False


def _wait_for_os(timeout: int = 15) -> dict | None:
    """Block until an OS message arrives or timeout. Returns message dict or None."""
    # Drain any stale messages first
    while not _os_queue.empty():
        try:
            _os_queue.get_nowait()
        except queue.Empty:
            break
    try:
        return _os_queue.get(timeout=timeout)
    except queue.Empty:
        return None


def _check_os_payload(msg: dict, station: str, expected: dict) -> list[str]:
    """Compare received payload against expected fields. Returns list of failures."""
    p = msg["payload"]
    failures = []
    for k, v in expected.items():
        if p.get(k) != v:
            failures.append(f"{k}: got {p.get(k)!r}, want {v!r}")
    if p.get("station_id") != station:
        failures.append(f"station_id: got {p.get('station_id')!r}, want {station!r}")
    return failures


# ── Clock setup ──────────────────────────────────────────────────────────────

# Known test clock: 08:00 AM Monday (480 min).  All expected times computed from this.
TEST_CLOCK_HOUR = 8
TEST_CLOCK_MIN  = 0
TEST_CLOCK_DAY  = 1  # Monday


def _dispatcher_post(dispatcher_url: str, path: str, body: dict) -> bool:
    """POST JSON to the dispatcher REST API. Returns True on HTTP 2xx."""
    try:
        data = json.dumps(body).encode()
        req  = urllib.request.Request(
            f"{dispatcher_url}{path}",
            data=data,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with urllib.request.urlopen(req, timeout=3):
            pass
        return True
    except Exception as e:
        info(f"Dispatcher API error: {e}")
        return False


def _set_test_clock(dispatcher_url: str) -> bool:
    """Set clock to TEST_CLOCK_HOUR:TEST_CLOCK_MIN and start it. Returns True on success."""
    ok  = _dispatcher_post(dispatcher_url, "/api/clock/control",
                           {"action": "set", "hour": TEST_CLOCK_HOUR,
                            "minute": TEST_CLOCK_MIN, "day": TEST_CLOCK_DAY})
    ok &= _dispatcher_post(dispatcher_url, "/api/clock/control", {"action": "start"})
    return ok


# ── Schedule helpers ──────────────────────────────────────────────────────────

def _load_schedule() -> dict:
    """Load schedule.json from Station_OS/data/."""
    sched_path = Path(__file__).parent.parent.parent / "Station_OS" / "data" / "schedule.json"
    with open(sched_path) as f:
        return json.load(f)


def _fmt_min(total_min: int) -> str:
    h, m = total_min // 60, total_min % 60
    ap   = "AM" if h < 12 else "PM"
    h12  = h % 12 or 12
    return f"{h12}:{m:02d} {ap}"


def _expected_next_station(station: str, train_num: str, direction: str,
                            clock_min: int) -> dict:
    """Return expected next-station screen values for a given OS submission."""
    sched     = _load_schedule()
    order     = sched["station_order"]
    go_north  = direction == "N"

    try:
        cur_idx  = order.index(station)
    except ValueError:
        return {"error": f"station {station!r} not in schedule"}

    next_idx = cur_idx + (1 if go_north else -1)
    if next_idx < 0 or next_idx >= len(order):
        return {"end_of_line": True}

    next_id   = order[next_idx]
    next_data = sched.get(next_id, {})
    ns_name   = next_data.get("name", next_id)
    same_dir  = next_data.get("N" if go_north else "S", [])
    opp_dir   = next_data.get("S" if go_north else "N", [])

    # Line 1: arrival of OS'd train
    te = next((t for t in same_dir if t["num"] == train_num), None)
    if te:
        t = te.get("arrive", -1)
        label = "Ar"
        if t < 0:
            t = te.get("depart", -1)
            label = "Dp"
        line1 = f"No. {train_num}    {label}  {_fmt_min(t)}" if t >= 0 else f"No. {train_num}    --:--"
    else:
        line1 = f"No. {train_num}    not scheduled"

    # Line 2: next opposing train after clock_min (wraps to first)
    line2 = None
    if opp_dir:
        found = next(
            (o for o in opp_dir
             if (o["depart"] if o.get("depart", -1) >= 0 else o.get("arrive", -1)) >= clock_min),
            opp_dir[0],
        )
        t2 = found["depart"] if found.get("depart", -1) >= 0 else found.get("arrive", -1)
        line2 = f"No. {found['num']}    Dp  {_fmt_min(t2)}"

    return {"next_id": next_id, "ns_name": ns_name, "line1": line1, "line2": line2}


# ── Test sections ─────────────────────────────────────────────────────────────

def test_preflight(broker_connected: bool, station: str, dispatcher_url: str) -> None:
    section("0 — Preflight")
    if broker_connected:
        info(f"MQTT broker connected — listening on trains/os/+")
    else:
        info("MQTT broker not connected — MQTT payload checks will be skipped")

    step("CYD powered and on clock screen",
         "Verify CYD is showing the clock screen (station name, time, NB/SB next trains).",
         "Clock screen visible, status dot green.")

    # Set clock to known time so next-station expected values are deterministic
    clock_str = f"{TEST_CLOCK_HOUR:02d}:{TEST_CLOCK_MIN:02d} (day {TEST_CLOCK_DAY})"
    if _set_test_clock(dispatcher_url):
        info(f"RR clock set to {clock_str} and started via dispatcher API.")
    else:
        info(f"Could not set clock via {dispatcher_url} — next-station times may not match expected.")


def test_clock_to_os(station: str) -> None:
    section("1 — Clock screen → OS entry")
    step("Touch anywhere → OS entry",
         "Touch anywhere on the clock screen.",
         "OS entry screen appears: 4×4 keypad visible, 'No. ─' in top-left, '[ ]' in top-right.")


def test_os_keypad(station: str) -> None:
    section("2 — OS entry keypad behaviour")
    step("Digit keys build train number",
         "Press 1, then 4, then 2.",
         "Display shows 'No. 142'.")
    step("⌫ removes last digit",
         "Press ⌫ once.",
         "Display shows 'No. 14'.")
    step("✓ disabled with no direction",
         "Observe ✓ key.",
         "✓ appears greyed/disabled (not pressable).")
    step("N selects northbound",
         "Press N.",
         "'[N]' appears green in top-right; N button highlighted.")
    step("✓ now enabled — green",
         "Observe ✓ key after pressing N.",
         "✓ turns green (was grey); it is now pressable.")
    step("S deselects N",
         "Press S.",
         "'[S]' appears cyan; S button highlighted; N no longer selected.")
    step("X extra flag",
         "Press X.",
         "'X' badge appears amber in top-right below direction.")
    step("WX clears X",
         "Press WX.",
         "'WX' badge replaces 'X'; WX button highlighted, X not highlighted.")
    step("X clears WX",
         "Press X.",
         "'X' badge back; X button highlighted, WX not highlighted.")
    step("X again toggles off",
         "Press X again.",
         "Extra badge disappears; X button back to normal.")
    step("Inactivity timeout (15 s)",
         "Stop touching the screen and wait 15 seconds.",
         "Screen returns to clock automatically.")


def test_os_submit_mqtt(station: str) -> None:
    section("3 — OS submit + MQTT payload verification")
    print(f"\n  Set up: press 3 → press N → press ✓ on the CYD.")
    print(f"  (Train No. 3, northbound, no extra.)")
    print( "  Waiting up to 15 s for MQTT message on trains/os/...\n")

    # Re-enter OS screen first
    step("Re-enter OS from clock screen",
         "Touch clock screen to open OS entry. Enter: 3, direction N.",
         "OS entry shows 'No. 3' and '[N]', ✓ enabled.")

    if _mqtt_connected:
        # Clear stale messages
        while not _os_queue.empty():
            try: _os_queue.get_nowait()
            except queue.Empty: break

        input("  Press ✓ on the CYD now, then press Enter here.")
        msg = _wait_for_os(timeout=15)
        if msg is None:
            print(f"  {FAIL_TAG} MQTT — no message received within 15 s")
            results.append(("OS MQTT — message received", "fail"))
        else:
            print(f"\n  Received on {msg['topic']}:")
            print(f"  {json.dumps(msg['payload'], indent=4)}")
            failures = _check_os_payload(msg, station, {
                "train":      "3",
                "direction":  "N",
                "section":    0,
                "extra":      False,
                "work_extra": False,
            })
            if failures:
                print(f"\n  {FAIL_TAG} MQTT — payload mismatches:")
                for f in failures:
                    print(f"    • {f}")
                results.append(("OS MQTT — payload correct", "fail"))
            else:
                print(f"  {PASS_TAG} MQTT — payload matches expected fields")
                results.append(("OS MQTT — message received", "pass"))
                results.append(("OS MQTT — payload correct", "pass"))
    else:
        info("Skipping MQTT auto-check (broker not connected).")
        step("Submit train 3 northbound",
             "Press ✓ on the CYD.",
             "Next-station screen appears.")


def test_next_station(station: str) -> None:
    clock_min = TEST_CLOCK_HOUR * 60 + TEST_CLOCK_MIN
    exp = _expected_next_station(station, "3", "N", clock_min)

    section("4 — Next-station screen")
    step("Direction label correct",
         "Observe the next-station screen after submitting No. 3 northbound.",
         "Top label shows 'NORTHBOUND' in green.")

    if exp.get("end_of_line"):
        step("End of line",
             f"Station {station} is the northern terminus.",
             "Screen shows 'End of line' below direction label.")
    else:
        step("Station name shown",
             "Observe the station name line.",
             f"Shows '{exp['ns_name']}' ({exp['next_id']}).")
        if exp.get("line2"):
            step("Next opposing train shown",
                 "Observe the time line (cyan for NB, green for SB).",
                 f"Shows '{exp['line2']}'.")

    step("Touch returns to clock",
         "Touch anywhere on the next-station screen.",
         "Returns to clock screen immediately.")

    section("4a — Next-station edge cases")
    step("Extra shows 'not scheduled'",
         "Re-enter OS, type any number, press X for extra, press N, press ✓.",
         "Next-station screen shows 'No. XX    not scheduled' on line 1.")
    step("Terminus shows 'End of line'",
         f"If {station} is WP or HC: submit any northbound (WP) or southbound (HC) OS.",
         "Next-station screen shows 'End of line' below direction label.")
    step("30 s timeout returns to clock",
         "Submit a valid OS and let the next-station screen sit for 30 seconds.",
         "Screen automatically returns to clock after ~30 s.")


def test_dispatcher_ui() -> None:
    section("5 — Dispatcher UI")
    step("OS log panel visible",
         "Open the dispatcher page in a browser.",
         "An OS Log panel is visible below the station table.")
    step("New OS entry appears in log",
         "Submit a valid OS from the CYD.",
         "A new row appears at the top of the OS log (newest first).")
    step("Green flash on new entry",
         "Watch the log panel when the OS arrives.",
         "The new row briefly flashes green before settling to normal background.")
    step("Log row content correct",
         "Read the new log row.",
         "Columns show: RR Time | Station ID | No. X | N or S | (blank/X/WX).")
    step("Multiple entries — newest first",
         "Submit two more OS reports.",
         "Each new report appears at the top; earlier reports push down.")
    step("Reconnect sees history",
         "Reload the dispatcher browser page.",
         "The OS log still shows all entries from this session (initial_state includes os_log).")


def test_summary() -> None:
    section("Summary")
    total  = len(results)
    passed = sum(1 for _, r in results if r == "pass")
    failed = sum(1 for _, r in results if r == "fail")
    skipped = sum(1 for _, r in results if r == "skip")

    print(f"\n  {BOLD}Results: {passed}/{total} passed  "
          f"({failed} failed, {skipped} skipped){RESET}\n")

    if failed:
        print(f"  {BOLD}Failures:{RESET}")
        for name, result in results:
            if result == "fail":
                print(f"    {FAIL_TAG}  {name}")
    if passed == total - skipped and failed == 0:
        print(f"  {GREEN}{BOLD}All checks passed.{RESET}")
    print()


# ── Entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--broker", default="192.168.10.1",
                        help="MQTT broker host (default: 192.168.10.1)")
    parser.add_argument("--port",   type=int, default=1883)
    parser.add_argument("--station", default="BB",
                        help="Station ID of the CYD under test (default: BB)")
    parser.add_argument("--username", default="cyd_unit")
    parser.add_argument("--password", default="",
                        help="MQTT password (blank = no auth; or read from broker config)")
    parser.add_argument("--skip-mqtt", action="store_true",
                        help="Skip MQTT auto-capture (visual checks only)")
    parser.add_argument("--dispatcher", default="http://192.168.86.36:5000",
                        help="Dispatcher base URL for clock setup (default: http://192.168.86.36:5000)")
    args = parser.parse_args()

    print(f"\n{BOLD}Session 2.1 — OS Submission — Interactive Smoke Tests{RESET}")
    print(f"  Broker: {args.broker}:{args.port}   Station: {args.station}")
    print(f"  Dispatcher: {args.dispatcher}")
    print(f"  Test clock: {TEST_CLOCK_HOUR:02d}:{TEST_CLOCK_MIN:02d} day {TEST_CLOCK_DAY}")
    print(f"  Press Ctrl-C at any time to abort.")

    broker_connected = False
    if not args.skip_mqtt:
        info(f"Connecting to broker {args.broker}:{args.port}...")
        broker_connected = _start_mqtt_listener(
            args.broker, args.port, args.username, args.password
        )
        if broker_connected:
            info("Connected — MQTT capture active.")
        else:
            info("Could not connect to broker. Run with --skip-mqtt to suppress this.")

    try:
        test_preflight(broker_connected, args.station, args.dispatcher)
        test_clock_to_os(args.station)
        test_os_keypad(args.station)
        test_os_submit_mqtt(args.station)
        test_next_station(args.station)
        test_dispatcher_ui()
    except KeyboardInterrupt:
        print(f"\n\n  {YELLOW}Aborted by user.{RESET}")
    finally:
        if _mqtt_client:
            _mqtt_client.loop_stop()
            _mqtt_client.disconnect()
        test_summary()


if __name__ == "__main__":
    main()
