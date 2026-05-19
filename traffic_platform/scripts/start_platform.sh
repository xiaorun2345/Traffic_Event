#!/usr/bin/env bash
set -e

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

exec python3 traffic_platform/backend/app.py --host 0.0.0.0 --port "${TRAFFIC_PLATFORM_PORT:-8080}"
