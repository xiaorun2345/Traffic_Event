#!/usr/bin/env bash
set -e

pkill -f "traffic_platform/backend/app.py" || true
