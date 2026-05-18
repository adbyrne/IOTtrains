#!/usr/bin/env bash
# deploy.sh — Sync RR_Server files to RPi5, update /opt/rr_server, restart services
# Run from dev machine: bash scripts/deploy.sh [--no-restart]
#
# Requires SSH key auth to abyrne@192.168.86.36 (home LAN eth0)
# Requires passwordless sudo on RPi5 for the rsync + systemctl steps.
# If sudo requires a password, run the remote commands manually (see output).

set -euo pipefail

RPI="abyrne@192.168.86.36"
REMOTE_DIR="/home/abyrne/rr_server"
OPT_DIR="/opt/rr_server"
RESTART=true

for arg in "$@"; do
    [[ "$arg" == "--no-restart" ]] && RESTART=false
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SERVER_DIR="$(dirname "$SCRIPT_DIR")"  # RR_Server/

echo "=== Deploying RR_Server → $RPI ==="

# Step 1: sync to staging dir (passwordless)
echo "Step 1: rsync to $RPI:$REMOTE_DIR"
rsync -avz --exclude='__pycache__' --exclude='*.pyc' --exclude='venv' \
    --exclude='.venv' --exclude='clock_state.json' \
    "$SERVER_DIR/" "$RPI:$REMOTE_DIR/"

# Step 2 + 3: copy to /opt and restart (requires sudo on RPi5)
REMOTE_CMD="sudo rsync -a --exclude='__pycache__' --exclude='*.pyc' --exclude='venv' \
    --exclude='.venv' --exclude='clock_state.json' \
    $REMOTE_DIR/ $OPT_DIR/"

if $RESTART; then
    REMOTE_CMD="$REMOTE_CMD && sudo systemctl restart rr-dispatcher"
fi

echo "Step 2: copy to $OPT_DIR and restart rr-dispatcher"
if ssh "$RPI" "$REMOTE_CMD"; then
    echo ""
    echo "=== Deployed and restarted ==="
    if $RESTART; then
        echo "rr-dispatcher restarted. Check: ssh $RPI 'journalctl -u rr-dispatcher -n 20'"
    fi
else
    echo ""
    echo "=== Remote step failed (sudo password required?) ==="
    echo "Run manually on RPi5:"
    echo "  sudo rsync -a --exclude='__pycache__' --exclude='*.pyc' --exclude='venv' \\"
    echo "      $REMOTE_DIR/ $OPT_DIR/"
    echo "  sudo systemctl restart rr-dispatcher"
fi
