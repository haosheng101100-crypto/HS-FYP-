#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "$0")/env.sh"
for name in cooperation attitude; do
  file="$HS_STAGE1_ROOT/results/$name.pid"
  if [[ -f "$file" ]]; then
    args=$(ps -p "$(cat "$file")" -o args= || true)
    if [[ "$args" == *"$HS_STAGE1_ROOT/tools/cooperation_demo.py"* || "$args" == *"$HS_STAGE1_ROOT/tools/attitude_demo.py"* ]]; then
      echo 'A flight demonstration is already running.';exit 2
    fi
  fi
done
nohup setsid --wait "$HS_STAGE1_ROOT/venv/bin/python" -u "$HS_STAGE1_ROOT/tools/cooperation_demo.py" "$@" > "$HS_STAGE1_ROOT/results/cooperation.log" 2>&1 < /dev/null &
echo $! > "$HS_STAGE1_ROOT/results/cooperation.pid"
echo 'Cooperative scene demonstration started; view results/cooperation.log for progress.'
