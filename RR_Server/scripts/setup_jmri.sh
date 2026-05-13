#!/usr/bin/env bash
# setup_jmri.sh — Install JMRI on RPi5 (Session 1.5)
# Run on RPi5 with sudo: sudo bash setup_jmri.sh
#
# Installs JMRI, configures the MQTT bridge pointed at the layout broker,
# and creates the jmri systemd service. The PR3 USB LocoNet adapter does NOT
# need to be physically connected during install.
#
# Prerequisites:
#   - Java 11+ (installed by this script)
#   - Mosquitto running (setup_mosquitto.sh complete)
#   - JMRI MQTT bridge password set (done in setup_mosquitto.sh)

set -euo pipefail

JMRI_VERSION="5.10"
JMRI_DEB="JMRI.${JMRI_VERSION}+R${JMRI_VERSION}.deb"
JMRI_URL="https://github.com/JMRI/JMRI/releases/download/v${JMRI_VERSION}/${JMRI_DEB}"
JMRI_PROFILE_DIR="/home/abyrne/.jmri"
MQTT_BROKER="192.168.10.1"
MQTT_PORT="1883"
MQTT_ROOT="trains"

if [[ $EUID -ne 0 ]]; then echo "Run as root: sudo bash $0"; exit 1; fi

echo "=== JMRI Install (Session 1.5) ==="
echo "  Version: $JMRI_VERSION"
echo "  MQTT broker: $MQTT_BROKER:$MQTT_PORT  root: $MQTT_ROOT"
echo ""

# Java
apt-get install -y default-jre-headless

# Download and install JMRI
echo "Downloading JMRI $JMRI_VERSION..."
TMP=$(mktemp -d)
wget -q --show-progress -O "$TMP/$JMRI_DEB" "$JMRI_URL"
dpkg -i "$TMP/$JMRI_DEB" || apt-get -f install -y
rm -rf "$TMP"

echo ""
read -rsp "JMRI MQTT password (set in setup_mosquitto.sh for 'jmri'): " JMRI_MQTT_PASS; echo

# JMRI MQTT bridge config goes in the user profile
# JMRI stores connection prefs in ~/.jmri/PanelPro/PanelPro.xml
# The MQTT broker settings are set via the JMRI preferences UI on first run,
# OR we can write them here. For now, write a preference stub that gets
# completed on first GUI launch.

mkdir -p "$JMRI_PROFILE_DIR"
cat > "$JMRI_PROFILE_DIR/mqtt_notes.txt" <<EOF
JMRI MQTT bridge settings (configure via JMRI Preferences → MQTT):
  Broker:   $MQTT_BROKER
  Port:     $MQTT_PORT
  Username: jmri
  Password: (set in setup_mosquitto.sh)
  Root topic prefix: $MQTT_ROOT
EOF
chown -R abyrne:abyrne "$JMRI_PROFILE_DIR"

# systemd unit for JMRI (headless PanelPro)
cat > /etc/systemd/system/jmri.service <<EOF
[Unit]
Description=JMRI PanelPro (DCC / WiThrottle server)
After=network.target mosquitto.service
Requires=mosquitto.service

[Service]
Type=simple
User=abyrne
Environment=DISPLAY=:0
ExecStart=/opt/JMRI/PanelPro.sh -profile default
Restart=on-failure
RestartSec=10

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
systemctl enable jmri

echo ""
echo "=== JMRI installed ==="
echo "  Service: jmri (enabled, NOT started — configure MQTT on first run)"
echo "  Next: connect PR3 USB adapter, then:"
echo "    sudo systemctl start jmri"
echo "    # open JMRI web interface at http://192.168.10.1:12080"
echo "    # configure MQTT connection via Preferences → MQTT"
echo "  Notes: $JMRI_PROFILE_DIR/mqtt_notes.txt"
