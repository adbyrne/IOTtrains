#!/usr/bin/env python3
"""
Station_OS hardware-in-the-loop test.

Verifies a provisioned CYD unit:
  1. Comes online and publishes correct status payload
  2. Sends sync_request on connect
  3. Updates its display when a clock tick is received (manual verification)
  4. Publishes a heartbeat within the expected window (optional)

The test asks you to power-cycle the CYD after it is ready, so the initial
MQTT connect sequence is captured cleanly.

Usage:
    python3 tests/hw_test.py [options]

Options:
    --broker HOST       MQTT broker host  (default: 192.168.86.36)
    --port   PORT       MQTT port         (default: 1883)
    --user   USER       MQTT username     (default: rr_dispatcher)
    --pass   PASS       MQTT password     (default: NYE1905)
    --station ID        Station ID, e.g. BB  (prompted if omitted)
    --no-heartbeat      Skip the 75-second heartbeat wait
    --schedule PATH     Path to schedule.json
                        (default: ../data/schedule.json relative to this script)
"""

import argparse
import json
import sys
import threading
import time
from pathlib import Path

try:
    import paho.mqtt.client as mqtt
except ImportError:
    sys.exit("paho-mqtt not found. Install it: pip install paho-mqtt")

# ── Defaults ──────────────────────────────────────────────────────────────
DEFAULT_BROKER   = "192.168.10.1"
DEFAULT_PORT     = 1883
DEFAULT_USER     = "rr_dispatcher"
DEFAULT_PASS     = "NYE1905"
ONLINE_TIMEOUT   = 45   # seconds to wait for CYD to come online
SYNC_TIMEOUT     = 10   # seconds to wait for sync_request after online
HEARTBEAT_WINDOW = 75   # seconds — HEARTBEAT_SECS(60) + 15s grace

# ── Test clock tick payload ────────────────────────────────────────────────
TEST_CLOCK_HOUR   = 10
TEST_CLOCK_MINUTE = 42
TEST_CLOCK_DAY    = 1
TEST_CLOCK_SPEED  = 3

# ── Colour codes ──────────────────────────────────────────────────────────
G = "\033[32m✓\033[0m"   # pass
F = "\033[31m✗\033[0m"   # fail
W = "\033[33m-\033[0m"   # skipped/warn
B = "\033[36m»\033[0m"   # info


def hr(char="─", width=60):
    print(char * width)


# ── Schedule lookup ───────────────────────────────────────────────────────

def load_schedule(path: Path) -> dict:
    if not path.exists():
        return {}
    with open(path) as f:
        return json.load(f)


def next_train_str(sched_list: list, cur_min: int) -> str:
    if not sched_list:
        return "─────"
    found = next((e for e in sched_list if e["time"] >= cur_min), sched_list[0])
    h, m = divmod(found["time"], 60)
    ampm = "AM" if h < 12 else "PM"
    h12 = h % 12 or 12
    return f"No.{found['num']}  {h12}:{m:02d} {ampm}"


# ── MQTT test client ───────────────────────────────────────────────────────

class StationTest:
    def __init__(self, broker, port, user, pw, station_id):
        self.broker     = broker
        self.port       = port
        self.station_id = station_id
        self.status_topic   = f"trains/station/{station_id}/status"
        self.sync_topic     = "trains/clock/sync_request"
        self.clock_topic    = "trains/clock/time"

        self._online_event   = threading.Event()
        self._sync_event     = threading.Event()
        self._heartbeat_event = threading.Event()
        self._online_payload  = None
        self._saw_offline     = False
        self._first_online    = True   # True until we see first "online: true"

        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,
                                  client_id="hw_test_client")
        self.client.username_pw_set(user, pw)
        self.client.on_connect    = self._on_connect
        self.client.on_message    = self._on_message
        self.client.on_disconnect = self._on_disconnect

    def connect(self):
        self.client.connect(self.broker, self.port, keepalive=30)
        self.client.loop_start()

    def disconnect(self):
        self.client.loop_stop()
        self.client.disconnect()

    def _on_connect(self, client, userdata, flags, rc, props=None):
        if rc != 0:
            print(f"  {F} MQTT connect failed: rc={rc}")
            return
        client.subscribe(self.status_topic,  qos=1)
        client.subscribe(self.sync_topic,    qos=0)

    def _on_message(self, client, userdata, msg):
        topic   = msg.topic
        payload = msg.payload.decode(errors="replace")

        if topic == self.status_topic:
            try:
                data = json.loads(payload)
            except json.JSONDecodeError:
                return
            online = data.get("online", False)
            sid    = data.get("station_id", "?")
            if sid != self.station_id:
                return
            if online and self._first_online:
                self._first_online    = False
                self._online_payload  = data
                self._online_event.set()
            elif not online:
                self._saw_offline = True
            elif online and not self._first_online:
                # Retained message or subsequent heartbeat
                self._heartbeat_event.set()

        elif topic == self.sync_topic:
            try:
                data = json.loads(payload)
            except json.JSONDecodeError:
                return
            if data.get("station_id") == self.station_id:
                self._sync_event.set()

    def _on_disconnect(self, client, userdata, flags, rc, props=None):
        pass

    def publish_clock_tick(self):
        payload = json.dumps({
            "time":    f"{TEST_CLOCK_HOUR % 12 or 12}:{TEST_CLOCK_MINUTE:02d} "
                       f"{'AM' if TEST_CLOCK_HOUR < 12 else 'PM'}",
            "hour":    TEST_CLOCK_HOUR,
            "minute":  TEST_CLOCK_MINUTE,
            "day":     TEST_CLOCK_DAY,
            "speed":   TEST_CLOCK_SPEED,
            "running": True,
        })
        self.client.publish(self.clock_topic, payload, qos=0, retain=True)

    def wait_online(self, timeout) -> bool:
        return self._online_event.wait(timeout)

    def wait_sync(self, timeout) -> bool:
        return self._sync_event.wait(timeout)

    def wait_heartbeat(self, timeout) -> bool:
        # Clear the initial retained message effect; wait for a fresh publish
        self._heartbeat_event.clear()
        return self._heartbeat_event.wait(timeout)


# ── Main test ─────────────────────────────────────────────────────────────

def run_test(args):
    station_id = args.station or input("Enter station ID to test (e.g. BB): ").strip().upper()
    if not station_id:
        sys.exit("Station ID required.")

    schedule_path = Path(args.schedule) if args.schedule else \
                    Path(__file__).parent.parent / "data" / "schedule.json"
    schedule      = load_schedule(schedule_path)

    hr("═")
    print(f"  Station_OS Hardware Test — Station {station_id}")
    print(f"  Broker:  {args.broker}:{args.port}")
    print(f"  Schedule: {schedule_path} "
          f"({'found' if schedule else 'NOT FOUND — next-train display check skipped'})")
    hr("═")

    results = {}

    # ── Connect test client ────────────────────────────────────────────────
    tester = StationTest(args.broker, args.port, args.user,
                         getattr(args, "pass"), station_id)
    print(f"\n{B} Connecting to MQTT broker {args.broker}:{args.port}…")
    try:
        tester.connect()
    except Exception as e:
        sys.exit(f"  {F} Cannot reach broker: {e}")
    time.sleep(0.5)   # let subscriptions settle
    print(f"  {G} Connected and subscribed")

    # ── Power-cycle prompt ────────────────────────────────────────────────
    print()
    hr()
    print(f"  ACTION REQUIRED:")
    print(f"    Power-cycle (or press RST on) the CYD unit for station {station_id}.")
    print(f"    The unit must already be provisioned with:")
    print(f"      sta_id  = {station_id}")
    print(f"      mqtt_ip = {args.broker}  (or the layout broker if on NYE_Layout)")
    print()
    print(f"    The test will detect the unit coming online. Do NOT press ENTER —")
    print(f"    just cycle power now.")
    hr()

    # ── 1: Wait for station to come online ────────────────────────────────
    print(f"\n{B} Waiting for station {station_id} to come online (timeout {ONLINE_TIMEOUT}s)…")
    if tester.wait_online(ONLINE_TIMEOUT):
        d = tester._online_payload
        print(f"  {G} Station {station_id} online")
        print(f"      firmware={d.get('firmware','?')}  "
              f"rssi={d.get('rssi','?')} dBm  "
              f"clock_sync={d.get('clock_sync','?')}")
        results["online"] = True
    else:
        print(f"  {F} Station {station_id} did not come online within {ONLINE_TIMEOUT}s")
        print("       Check: WiFi credentials, MQTT IP, sta_id provisioning")
        results["online"] = False
        _print_results(results)
        tester.disconnect()
        return results

    # ── 2: sync_request (should arrive within seconds of online) ──────────
    print(f"\n{B} Waiting for sync_request from {station_id} (timeout {SYNC_TIMEOUT}s)…")
    if tester.wait_sync(SYNC_TIMEOUT):
        print(f"  {G} sync_request received — CYD asked for immediate clock sync")
        results["sync_request"] = True
    else:
        print(f"  {W} sync_request not received within {SYNC_TIMEOUT}s")
        print("       (non-fatal: may have been sent before subscriptions were ready)")
        results["sync_request"] = False

    # ── 3: Publish clock tick + manual display check ───────────────────────
    test_min = TEST_CLOCK_HOUR * 60 + TEST_CLOCK_MINUTE
    time_str = (f"{TEST_CLOCK_HOUR % 12 or 12}:{TEST_CLOCK_MINUTE:02d} "
                f"{'AM' if TEST_CLOCK_HOUR < 12 else 'PM'}")
    print(f"\n{B} Publishing test clock tick: {time_str} (day {TEST_CLOCK_DAY}, speed {TEST_CLOCK_SPEED})…")
    tester.publish_clock_tick()
    time.sleep(0.5)

    print()
    hr()
    print("  DISPLAY VERIFICATION — confirm the CYD shows:")
    print(f"    Station name: as provisioned (e.g. 'Becs Bend' for BB)")
    print(f"    Time:         {time_str}")

    if station_id in schedule:
        sta = schedule[station_id]
        nb  = next_train_str(sta.get("N", []), test_min)
        sb  = next_train_str(sta.get("S", []), test_min)
        print(f"    NB next:      {nb}")
        print(f"    SB next:      {sb}")
    else:
        print(f"    Next trains:  (schedule not loaded — check display manually)")

    print(f"    Status dot:   green (connected)")
    hr()

    resp = input("  Display correct? [y/n/skip]: ").strip().lower()
    if resp == "y":
        print(f"  {G} Display verified by operator")
        results["display"] = True
    elif resp == "n":
        print(f"  {F} Display incorrect — investigate firmware or provisioning")
        results["display"] = False
    else:
        print(f"  {W} Display check skipped")
        results["display"] = None

    # ── 4: Heartbeat (optional) ───────────────────────────────────────────
    if args.no_heartbeat:
        print(f"\n{W} Heartbeat check skipped (--no-heartbeat)")
        results["heartbeat"] = None
    else:
        print(f"\n{B} Waiting for heartbeat within {HEARTBEAT_WINDOW}s "
              f"(press Ctrl+C to skip)…")
        try:
            if tester.wait_heartbeat(HEARTBEAT_WINDOW):
                print(f"  {G} Heartbeat received")
                results["heartbeat"] = True
            else:
                print(f"  {F} No heartbeat within {HEARTBEAT_WINDOW}s")
                results["heartbeat"] = False
        except KeyboardInterrupt:
            print(f"\n  {W} Heartbeat check interrupted")
            results["heartbeat"] = None

    tester.disconnect()
    _print_results(results)
    return results


def _print_results(results):
    hr("═")
    checks = {
        "online":       "Station came online",
        "sync_request": "sync_request sent on connect",
        "display":      "Display verified by operator",
        "heartbeat":    "Heartbeat within 75s",
    }
    passed = skipped = failed = 0
    for key, label in checks.items():
        val = results.get(key)
        if val is True:
            print(f"  {G} {label}")
            passed += 1
        elif val is False:
            print(f"  {F} {label}")
            failed += 1
        else:
            print(f"  {W} {label} (skipped)")
            skipped += 1
    hr()
    total = passed + failed
    print(f"  {passed}/{total} checks passed"
          + (f", {skipped} skipped" if skipped else ""))
    hr("═")
    if failed:
        sys.exit(1)


# ── CLI ───────────────────────────────────────────────────────────────────

def main():
    p = argparse.ArgumentParser(description="Station_OS hardware-in-the-loop test")
    p.add_argument("--broker",       default=DEFAULT_BROKER)
    p.add_argument("--port",         type=int, default=DEFAULT_PORT)
    p.add_argument("--user",         default=DEFAULT_USER)
    p.add_argument("--pass",         default=DEFAULT_PASS, dest="pass_")
    p.add_argument("--station",      default=None)
    p.add_argument("--no-heartbeat", action="store_true")
    p.add_argument("--schedule",     default=None)
    args = p.parse_args()
    # Expose password under the name the class expects
    args.__dict__["pass"] = args.pass_
    run_test(args)


if __name__ == "__main__":
    main()
