#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "$0")/env.sh"
world="$HS_STAGE1_ROOT/worlds/uav_vehicle_stage1.sdf"
world_name=uav_vehicle_stage1
if [[ "${1:-}" == --official ]]; then
  world="$HS_STAGE1_ROOT/vendor/ardupilot_gazebo/worlds/iris_runway.sdf"
  world_name=iris_runway
else
  python3 "$HS_STAGE1_ROOT/tools/create_scene.py"
fi
mkdir -p "$HS_STAGE1_ROOT/results" "$HS_STAGE1_ROOT/runtime"
for name in gazebo arducopter; do
  if [ -f "$HS_STAGE1_ROOT/results/$name.pid" ] && kill -0 "$(cat "$HS_STAGE1_ROOT/results/$name.pid")" 2>/dev/null; then
    echo "$name already running; use stop_scene.sh before restarting."; exit 2
  fi
done
export DISPLAY="${DISPLAY:-:0}"
export XAUTHORITY="${XAUTHORITY:-/run/user/1000/gdm/Xauthority}"
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/1000}"
nohup setsid --wait gz sim -r -v 3 --gui-config "$HS_GZ_PREFIX/share/gz/gz-sim8/gui/gui.config" "$world" > "$HS_STAGE1_ROOT/results/gazebo.log" 2>&1 < /dev/null &
echo $! > "$HS_STAGE1_ROOT/results/gazebo.pid"
"$HS_STAGE1_ROOT/venv/bin/python" "$HS_STAGE1_ROOT/tools/wait_scene.py" --world "$world_name"
cd "$HS_STAGE1_ROOT/runtime"
nohup setsid --wait bash "$HS_STAGE1_ROOT/scripts/run_fcu.sh" > "$HS_STAGE1_ROOT/results/arducopter.log" 2>&1 < /dev/null &
echo $! > "$HS_STAGE1_ROOT/results/arducopter.pid"
echo "Gazebo scene and ArduCopter started: $world"
echo "The scene opens on the Ubuntu desktop. UAV starts disarmed."
