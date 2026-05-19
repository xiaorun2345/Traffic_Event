#!/usr/bin/env bash
set -e

PORT="${TRAFFIC_PLATFORM_PORT:-8080}"
curl -sS -X POST \
  -H 'Content-Type: application/json' \
  -d '{}' \
  "http://127.0.0.1:${PORT}/api/video/stop"
echo
