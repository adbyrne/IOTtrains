#!/usr/bin/env bash
# setup_ap.sh — Configure wlan0 as WiFi Access Point for NY&E layout network
# Run on RPi5 with sudo: sudo bash setup_ap.sh
#
# Creates: SSID NYE_Layout, 192.168.10.1/24, DHCP 192.168.10.10–.99, OPEN (no WPA)
# Uses NetworkManager (already installed on Raspberry Pi OS trixie)
#
# NOTE: WPA-PSK AP mode is intentionally disabled. NM 1.52 (Debian trixie) has a
# regression where WPA-PSK AP connections always request a secrets agent even with
# psk-flags=0, preventing headless activation on reboot. Security is provided at
# the MQTT layer (all devices require username/password to publish or subscribe).
# If WPA is needed, bypass NM and use hostapd+dnsmasq directly.

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
echo "  SSID:     $SSID"
echo "  IP:       192.168.10.1"
echo "  Channel:  $CHANNEL ($BAND band)"
echo "  Security: OPEN (auth at MQTT layer)"
echo ""

# Set WiFi regulatory country
echo "Setting WiFi country to $COUNTRY..."
raspi-config nonint do_wifi_country "$COUNTRY"

# Remove any existing NYE_Layout connection
nmcli con delete "$SSID" 2>/dev/null && echo "Removed existing $SSID connection." || true

# Create the AP connection (NM handles DHCP via its internal dnsmasq)
# No WPA security — see note above
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
echo "  SSID:      $SSID  (open — no password)"
echo "  IP:        192.168.10.1"
echo "  DHCP pool: ${DHCP_START} – ${DHCP_END}  (lease ${DHCP_LEASE})"
echo "  Autostart: yes (on boot)"
echo ""
echo "Verify: scan for '$SSID' from your phone and connect."
echo "Phone should receive an IP in the 192.168.10.x range."
