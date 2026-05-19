#!/usr/bin/env bash
set -e

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
SERVICE_SRC="$ROOT/traffic_platform/systemd/traffic-platform.service"
SERVICE_DST="/etc/systemd/system/traffic-platform.service"

sudo cp "$SERVICE_SRC" "$SERVICE_DST"
sudo systemctl daemon-reload
sudo systemctl enable traffic-platform.service
sudo systemctl restart traffic-platform.service
sudo systemctl status traffic-platform.service --no-pager
