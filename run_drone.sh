#!/usr/bin/env bash
set -euo pipefail

CONNECT="${CONNECT:-udp:127.0.0.1:14550}"
LAT_B="${LAT_B:-50.443326}"
LON_B="${LON_B:-30.448078}"
ALT="${ALT:-100}"
ARRIVAL_RADIUS="${ARRIVAL_RADIUS:-3}"
LOG_INTERVAL="${LOG_INTERVAL:-1}"
SPEED_SCALE="${SPEED_SCALE:-4.0}"

if [[ -x "venv/bin/python" ]]; then
  PYTHON_BIN="venv/bin/python"
else
  PYTHON_BIN="${PYTHON_BIN:-python3}"
fi

PYTHONUNBUFFERED=1 "$PYTHON_BIN" -u Tools/autotest/stabilize_rc_override_mission.py \
  --connect "$CONNECT" \
  --lat-b "$LAT_B" --lon-b "$LON_B" --alt "$ALT" \
  --arrival-radius "$ARRIVAL_RADIUS" --log-interval "$LOG_INTERVAL" \
  --speed-scale "$SPEED_SCALE" \
  "$@" 2>&1 | tee ./run_drone.log
