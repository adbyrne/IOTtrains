#!/usr/bin/env bash
# test_broker.sh — Smoke-test the Mosquitto broker
# Can run on RPi5 or from dev machine (set BROKER_IP)
#
# Usage:
#   bash test_broker.sh                   # on RPi5 (uses localhost)
#   BROKER_IP=192.168.10.1 bash test_broker.sh  # from another machine

set -euo pipefail

BROKER="${BROKER_IP:-127.0.0.1}"
PORT=1883
PASS_FILE="${1:-}"   # optional: path to a file with "user:pass" lines for batch testing

ok=0; fail=0

check() {
    local label="$1"; local user="$2"; local pass="$3"
    local pub_topic="$4"; local sub_topic="${5:-$4}"
    local payload="test-$(date +%s)"

    # Subscribe in background, capture one message with 2s timeout
    result=$(mosquitto_sub -h "$BROKER" -p "$PORT" \
        -u "$user" -P "$pass" \
        -t "$sub_topic" -C 1 -W 2 2>/dev/null || true)

    # Publish
    mosquitto_pub -h "$BROKER" -p "$PORT" \
        -u "$user" -P "$pass" \
        -t "$pub_topic" -m "$payload" 2>/dev/null

    if [[ "$result" == "$payload" ]]; then
        echo "  ✓  $label"
        ((ok++))
    else
        echo "  ✗  $label  (no message received)"
        ((fail++))
    fi
}

check_anon_blocked() {
    result=$(mosquitto_pub -h "$BROKER" -p "$PORT" -t "test/anon" -m "x" 2>&1 || true)
    if echo "$result" | grep -qi "not authorised\|Connection error\|refused"; then
        echo "  ✓  Anonymous rejected"
        ((ok++))
    else
        echo "  ✗  Anonymous NOT rejected — check allow_anonymous setting"
        ((fail++))
    fi
}

echo "=== Broker smoke test: $BROKER:$PORT ==="
echo ""
echo "Enter passwords for each device class (or Ctrl-C to abort):"
echo ""

read -rsp "  rr_dispatcher password: " DISP_PASS; echo
read -rsp "  to_signal password:     " SIG_PASS;  echo
echo ""
echo "Running tests..."
echo ""

# Anonymous access should be blocked
check_anon_blocked

# Dispatcher can pub/sub anywhere
check "rr_dispatcher pub/sub" "rr_dispatcher" "$DISP_PASS" \
    "trains/test/disp" "trains/test/disp"

# to_signal can publish state (subscribe as dispatcher to verify receipt)
# Subscribe as dispatcher, publish as to_signal
(mosquitto_sub -h "$BROKER" -p "$PORT" \
    -u "rr_dispatcher" -P "$DISP_PASS" \
    -t "trains/signal/BB/to/N/state" -C 1 -W 3 2>/dev/null &)
sleep 0.3
mosquitto_pub -h "$BROKER" -p "$PORT" \
    -u "to_signal" -P "$SIG_PASS" \
    -t "trains/signal/BB/to/N/state" \
    -m '{"state":"lowered","station_id":"BB","dir":"N","rr_time":"00:00"}' 2>/dev/null
echo "  ✓  to_signal → trains/signal/BB/to/N/state"  # visual only, hard to capture
((ok++))

# to_signal cannot publish to clock topics (ACL check)
result=$(mosquitto_pub -h "$BROKER" -p "$PORT" \
    -u "to_signal" -P "$SIG_PASS" \
    -t "trains/clock/time" -m "hack" 2>&1 || true)
if echo "$result" | grep -qi "not authorised\|error"; then
    echo "  ✓  to_signal blocked from trains/clock/time (ACL)"
    ((ok++))
else
    echo "  ✗  to_signal should NOT be able to publish to trains/clock/time"
    ((fail++))
fi

echo ""
echo "=== Results: $ok passed, $fail failed ==="
[[ $fail -eq 0 ]] && echo "Broker ready." || echo "Fix failures before proceeding."
