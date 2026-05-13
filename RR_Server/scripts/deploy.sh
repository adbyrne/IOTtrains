#!/usr/bin/env bash
# deploy.sh — Sync RR_Server files to RPi5 and run setup scripts
# Run from dev machine: bash scripts/deploy.sh
#
# Requires SSH key auth to abyrne@192.168.86.36 (home LAN eth0)

set -euo pipefail

RPI="abyrne@192.168.86.36"
REMOTE_DIR="/home/abyrne/rr_server"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SERVER_DIR="$(dirname "$SCRIPT_DIR")"  # RR_Server/

echo "Deploying RR_Server → $RPI:$REMOTE_DIR"

# Sync the whole RR_Server tree (excludes .pyc, __pycache__, venv)
rsync -avz --exclude='__pycache__' --exclude='*.pyc' --exclude='venv' \
    "$SERVER_DIR/" "$RPI:$REMOTE_DIR/"

echo ""
echo "Files synced. Available setup commands on RPi5:"
echo "  sudo bash $REMOTE_DIR/scripts/setup_ap.sh"
echo "  sudo bash $REMOTE_DIR/scripts/setup_mosquitto.sh"
echo "  sudo bash $REMOTE_DIR/scripts/install_services.sh"
echo "  sudo bash $REMOTE_DIR/scripts/setup_venv.sh     ← installs to /opt, creates venv"
echo "  sudo bash $REMOTE_DIR/scripts/setup_jmri.sh"
echo "  bash $REMOTE_DIR/scripts/test_broker.sh"
