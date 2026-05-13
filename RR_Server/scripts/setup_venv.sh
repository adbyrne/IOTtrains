#!/usr/bin/env bash
# setup_venv.sh — Copy deployed files to /opt/rr_server and create Python venv
# Run on RPi5 after deploy.sh: sudo bash /home/abyrne/rr_server/scripts/setup_venv.sh

set -euo pipefail

DEPLOY_DIR="/home/abyrne/rr_server"
SERVER_DIR="/opt/rr_server"

if [[ $EUID -ne 0 ]]; then echo "Run as root: sudo bash $0"; exit 1; fi

mkdir -p "$SERVER_DIR"

echo "Copying files to $SERVER_DIR..."
rsync -a --exclude='__pycache__' --exclude='*.pyc' --exclude='venv' \
    "$DEPLOY_DIR/" "$SERVER_DIR/"

if [[ ! -d "$SERVER_DIR/venv" ]]; then
    echo "Creating venv..."
    python3 -m venv "$SERVER_DIR/venv"
fi

echo "Installing dependencies..."
"$SERVER_DIR/venv/bin/pip" install -q --upgrade pip
"$SERVER_DIR/venv/bin/pip" install -r "$SERVER_DIR/requirements.txt"

chown -R abyrne:abyrne "$SERVER_DIR"

echo ""
echo "=== Venv ready: $SERVER_DIR/venv ==="
echo "To start the clock:  sudo systemctl start rr-clock"
echo "To check status:     systemctl status rr-clock"
echo "To follow logs:      journalctl -u rr-clock -f"
