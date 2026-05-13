#!/usr/bin/env bash
# setup_ap.sh — Configure wlan0 as WiFi Access Point for NY&E layout network
# Run on RPi5 with sudo: sudo bash setup_ap.sh
#
# Creates: SSID NYE_Layout, 192.168.10.1/24, DHCP 192.168.10.10–.99
# Uses NetworkManager (already installed on Raspberry Pi OS trixie)

set -euo pipefail

SSID="NYE_Layout"
BAND="bg"
CHANNEL=6
AP_IP="192.168.10.1/24"
DHCP_START="192.168.10.10"
DHCP_END="192.168.10.99"
DHCP_LEASE="12h"
COUNTRY="US"

if [[ $EUID -ne 0 ]]; then echo "Run as root: sudo bash $0"; exit 1; fi

echo "=== NY&E Layout AP Setup ==="
echo "  SSID:    $SSID"
echo "  IP:      192.168.10.1"
echo "  Channel: $CHANNEL ($BAND band)"
echo ""

# Prompt for WiFi password
while true; do
    read -rsp "AP password (min 8 chars): " AP_PASS; echo
    read -rsp "Confirm password: "           AP_PASS2; echo
    if [[ "$AP_PASS" == "$AP_PASS2" && ${#AP_PASS} -ge 8 ]]; then break; fi
    echo "Passwords don't match or too short. Try again."
done

# Set WiFi regulatory country
echo "Setting WiFi country to $COUNTRY..."
raspi-config nonint do_wifi_country "$COUNTRY"

# Remove any existing NYE_Layout connection
nmcli con delete "$SSID" 2>/dev/null && echo "Removed existing $SSID connection." || true

# Create the AP connection (NM handles DHCP via its internal dnsmasq)
nmcli con add \
    type wifi \
    ifname wlan0 \
    con-name "$SSID" \
    ssid "$SSID" \
    mode ap \
    ipv4.method shared \
    ipv4.addresses "$AP_IP" \
    802-11-wireless.band "$BAND" \
    802-11-wireless.channel "$CHANNEL" \
    802-11-wireless-security.key-mgmt wpa-psk \
    802-11-wireless-security.psk "$AP_PASS" \
    connection.autoconnect yes

# Override DHCP pool range for NM's built-in dnsmasq
mkdir -p /etc/NetworkManager/dnsmasq-shared.d
cat > /etc/NetworkManager/dnsmasq-shared.d/layout.conf <<EOF
dhcp-range=${DHCP_START},${DHCP_END},${DHCP_LEASE}
EOF

# Bring up the AP
nmcli con up "$SSID"

echo ""
echo "=== AP is up ==="
echo "  SSID:      $SSID"
echo "  IP:        192.168.10.1"
echo "  DHCP pool: ${DHCP_START} – ${DHCP_END}  (lease ${DHCP_LEASE})"
echo "  Autostart: yes (on boot)"
echo ""
echo "Verify: scan for '$SSID' from your phone and connect."
echo "Phone should receive an IP in the 192.168.10.x range."
