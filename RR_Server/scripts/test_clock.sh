#!/usr/bin/env bash
# test_clock.sh — Integration smoke test for the rr-clock service.
# Requires the service to be running and the broker to be reachable.
#
# Usage:
#   bash test_clock.sh                      # on RPi5 (broker = localhost)
#   BROKER_IP=192.168.10.1 bash test_clock.sh  # from another machine

set -euo pipefail

BROKER="${BROKER_IP:-127.0.0.1}"
PORT=1883
USER="rr_dispatcher"

ok=0; fail=0

# Prompt once for the dispatcher password
read -rsp "rr_dispatcher password: " PASS; echo
echo ""
echo "=== Clock integration test: $BROKER:$PORT ==="
echo ""

# ── helpers ──────────────────────────────────────────────────────────────────

pub() {
    mosquitto_pub -h "$BROKER" -p "$PORT" -u "$USER" -P "$PASS" "$@"
}

get_retained() {
    # Returns the current retained trains/clock/time payload (1s timeout)
    mosquitto_sub -h "$BROKER" -p "$PORT" -u "$USER" -P "$PASS" \
        -t "trains/clock/time" -C 1 -W 2 2>/dev/null || true
}

ctrl() {
    pub -t "trains/clock/control" -q 1 -m "$1"
    sleep 0.4   # allow service to process and publish
}

check_field() {
    local label="$1" payload="$2" field="$3" expected="$4"
    actual=$(echo "$payload" | python3 -c \
        "import sys,json; print(json.load(sys.stdin)['$field'])" 2>/dev/null || echo "PARSE_ERROR")
    if [[ "$actual" == "$expected" ]]; then
        echo "  ✓  $label"
        ok=$((ok + 1))
    else
        echo "  ✗  $label  (expected $field=$expected, got $field=$actual)"
        echo "     payload: $payload"
        fail=$((fail + 1))
    fi
}

# ── check service is publishing ───────────────────────────────────────────────

echo "Checking retained clock/time..."
retained=$(get_retained)
if [[ -z "$retained" ]]; then
    echo "  ✗  No retained trains/clock/time — is rr-clock running?"
    echo ""
    echo "Aborted: start the service first with: sudo systemctl start rr-clock"
    exit 1
fi
echo "  ✓  Service is publishing (got retained message)"
ok=$((ok + 1))
echo ""

# ── reset to known state ──────────────────────────────────────────────────────

echo "Resetting to 00:00 day 1 (paused)..."
ctrl '{"action":"reset"}'
p=$(get_retained)
check_field "reset: running=false"   "$p" running False
check_field "reset: hour=0"          "$p" hour    0
check_field "reset: minute=0"        "$p" minute  0
check_field "reset: day=1"           "$p" day     1
echo ""

# ── set time ─────────────────────────────────────────────────────────────────

echo "Setting time to 08:30 day 2..."
ctrl '{"action":"set","hour":8,"minute":30,"day":2}'
p=$(get_retained)
check_field "set: hour=8"   "$p" hour   8
check_field "set: minute=30" "$p" minute 30
check_field "set: day=2"    "$p" day    2
echo ""

# ── speed ─────────────────────────────────────────────────────────────────────

echo "Setting speed to 6..."
ctrl '{"action":"speed","speed":6}'
p=$(get_retained)
check_field "speed: speed=6" "$p" speed 6
echo ""

# ── start / pause ─────────────────────────────────────────────────────────────

echo "Starting clock..."
ctrl '{"action":"start"}'
p=$(get_retained)
check_field "start: running=true" "$p" running True
echo ""

echo "Pausing clock..."
ctrl '{"action":"pause"}'
p=$(get_retained)
check_field "pause: running=false" "$p" running False
echo ""

# ── restore defaults and leave paused ─────────────────────────────────────────

ctrl '{"action":"speed","speed":3}'
ctrl '{"action":"reset"}'

# ── results ───────────────────────────────────────────────────────────────────

echo "=== Results: $ok passed, $fail failed ==="
if [[ $fail -eq 0 ]]; then
    echo "Clock service OK."
else
    echo "Failures above — check journalctl -u rr-clock for service logs."
    exit 1
fi
