#!/usr/bin/env bash
# install_services.sh — Install systemd unit files for all RR_Server services
# Run on RPi5 with sudo: sudo bash install_services.sh
#
# Installs units for: mosquitto (already enabled by setup_mosquitto.sh),
# rr-clock, rr-dispatcher.
# rr-clock and rr-dispatcher units are stubs — services not started until
# Sessions 1.2 and 1.3 implement the actual software.

set -euo pipefail

UNIT_DIR="/etc/systemd/system"
SERVER_DIR="/opt/rr_server"

if [[ $EUID -ne 0 ]]; then echo "Run as root: sudo bash $0"; exit 1; fi

# Create deployment directory
mkdir -p "$SERVER_DIR"

# ── rr-clock ─────────────────────────────────────────────────────────────
cat > "$UNIT_DIR/rr-clock.service" <<'EOF'
[Unit]
Description=NY&E Fast Clock Service
After=network.target mosquitto.service
Requires=mosquitto.service

[Service]
Type=simple
User=abyrne
WorkingDirectory=/opt/rr_server
ExecStart=/opt/rr_server/venv/bin/python fast_clock/clock_service.py
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

# ── rr-dispatcher ────────────────────────────────────────────────────────
cat > "$UNIT_DIR/rr-dispatcher.service" <<'EOF'
[Unit]
Description=NY&E Dispatcher Web App
After=network.target mosquitto.service rr-clock.service
Requires=mosquitto.service

[Service]
Type=simple
User=abyrne
WorkingDirectory=/opt/rr_server
ExecStart=/opt/rr_server/venv/bin/uvicorn dispatcher.app:app --host 0.0.0.0 --port 5000
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload

# Enable but don't start — software not yet deployed
systemctl enable rr-clock rr-dispatcher

echo "=== Services installed ==="
echo "  mosquitto    — enabled, running (from setup_mosquitto.sh)"
echo "  rr-clock     — enabled, NOT started (Session 1.2)"
echo "  rr-dispatcher — enabled, NOT started (Session 1.3)"
echo ""
echo "Deployment path: $SERVER_DIR"
echo "Python venv will be created at: $SERVER_DIR/venv"
