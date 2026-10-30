#!/usr/bin/env bash
set -u
source "$(dirname "$0")/env.sh"
cd "$HS_STAGE1_ROOT/runtime"
child=''
stop_fcu() {
  if [[ -n "$child" ]]; then kill "$child" 2>/dev/null || true;wait "$child" 2>/dev/null || true;fi
  exit 0
}
trap stop_fcu TERM INT
for attempt in 1 2 3; do
  echo "Starting ArduCopter, attempt $attempt"
  "$HS_STAGE1_ROOT/vendor/arducopter" --model JSON --home=-35.363262,149.165237,584,0 --defaults "$HS_STAGE1_ROOT/config/copter.parm,$HS_STAGE1_ROOT/config/gazebo-iris.parm,$HS_STAGE1_ROOT/vendor/ardupilot_gazebo/config/gazebo-iris-gimbal.parm,$HS_STAGE1_ROOT/config/stage1.parm" --serial0 tcp:0 &
  child=$!;begin=$SECONDS
  wait "$child";code=$?
  echo "ArduCopter exited: status=$code, uptime=$((SECONDS-begin))s"
  # Retry only an early startup exit. Never restart a running flight session.
  if (( SECONDS-begin>=20 )); then exit "$code";fi
  child='';sleep 2
done
echo 'ArduCopter could not remain running after three startup attempts'
exit 1
