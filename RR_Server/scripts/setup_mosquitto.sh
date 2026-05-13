#!/usr/bin/env bash
# setup_mosquitto.sh — Install and configure Mosquitto for NY&E layout
# Run on RPi5 with sudo: sudo bash setup_mosquitto.sh
#
# Sets up auth for all 5 device classes. Record passwords somewhere safe —
# they go into each ESP32 via serial CLI (TO_Signal) or firmware config.

set -euo pipefail

PASSWD_FILE="/etc/mosquitto/passwd"
ACL_FILE="/etc/mosquitto/acl"
CONF_DEST="/etc/mosquitto/mosquitto.conf"   # main file, not conf.d (Mosquitto 2.0 local-only if conf.d only)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ $EUID -ne 0 ]]; then echo "Run as root: sudo bash $0"; exit 1; fi

echo "=== NY&E Mosquitto Setup ==="
echo ""

# Install
apt-get install -y mosquitto mosquitto-clients
systemctl stop mosquitto

# ── Password file ─────────────────────────────────────────────────────────
# Helper: add/update one user in passwd file
set_user_pass() {
    local user="$1"
    local label="$2"
    local pass1 pass2
    while true; do
        read -rsp "  [$label] password: " pass1;  echo
        read -rsp "  [$label] confirm:  " pass2;  echo
        [[ "$pass1" == "$pass2" && -n "$pass1" ]] && break
        echo "  Mismatch or empty. Try again."
    done
    if [[ ! -f "$PASSWD_FILE" ]]; then
        mosquitto_passwd -c -b "$PASSWD_FILE" "$user" "$pass1"
    else
        mosquitto_passwd -b "$PASSWD_FILE" "$user" "$pass1"
    fi
    echo "  ✓ $user"
    echo ""
}

echo "Set a password for each MQTT device class."
echo "Record these — you will need them when provisioning each device."
echo ""
set_user_pass "rr_clock"      "Fast clock service"
set_user_pass "rr_dispatcher" "Dispatcher web app"
set_user_pass "cyd_unit"      "Station CYD units (all 7 share this)"
set_user_pass "to_signal"     "TO signal controllers (all 5 share this)"
set_user_pass "jmri"          "JMRI MQTT bridge"

chmod 640 "$PASSWD_FILE"
chown mosquitto:mosquitto "$PASSWD_FILE"

# ── ACL + config ──────────────────────────────────────────────────────────
install -m 640 -o mosquitto -g mosquitto "$SCRIPT_DIR/../mosquitto/acl"  "$ACL_FILE"
install -m 644 "$SCRIPT_DIR/../mosquitto/mosquitto.conf" "$CONF_DEST"

# conf.d files are only loaded if include_dir is present in mosquitto.conf.
# Since we ARE mosquitto.conf, nothing else to do here.

# ── Start ─────────────────────────────────────────────────────────────────
systemctl enable mosquitto
systemctl start mosquitto

sleep 1
if systemctl is-active --quiet mosquitto; then
    echo "=== Mosquitto running ==="
    echo "  Port:  1883 (all interfaces)"
    echo "  Auth:  required"
    echo "  ACL:   $ACL_FILE"
    echo ""
    echo "Run scripts/test_broker.sh to verify connectivity."
else
    echo "ERROR: mosquitto failed to start"
    systemctl status mosquitto
    exit 1
fi
