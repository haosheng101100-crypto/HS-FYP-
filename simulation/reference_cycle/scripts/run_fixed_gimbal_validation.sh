#!/bin/bash
set -u
source "./scripts/env.sh"
export GZ_PARTITION=hs_fixed_gimbal_terminal_004
export DISPLAY=:0
export XAUTHORITY=/run/user/1000/gdm/Xauthority
export XDG_RUNTIME_DIR=/run/user/1000
gz sim -s -r -v 3 ./worlds/uav_vehicle_stage1.sdf > ./results/gazebo.log 2>&1 &
gazebo_pid=$!
echo "$gazebo_pid" > ./results/gazebo.pid
python3 ./tools/record_model_states.py ./results > ./results/model_states.log 2>&1 &
logger_pid=$!
echo "$logger_pid" > ./results/model_states.pid
bash ./scripts/run_fcu.sh > ./results/arducopter.log 2>&1 &
fcu_pid=$!
echo "$fcu_pid" > ./results/arducopter.pid
./venv/bin/python ./tools/wait_scene.py --world uav_vehicle_stage1 > ./results/wait_scene.log 2>&1
scene_code=$?
if test "$scene_code" != 0; then
 kill "$logger_pid" "$fcu_pid" "$gazebo_pid" 2>/dev/null || true
 wait "$logger_pid" 2>/dev/null || true
 exit "$scene_code"
fi
./venv/bin/python -u ./tools/terminal_demo.py --method terminal > ./results/cooperation.log 2>&1
flight_code=$?
echo "$flight_code" > ./results/controller_exit_code.txt
if test "$flight_code" != 0; then sleep 20; fi
kill "$logger_pid" 2>/dev/null || true
wait "$logger_pid" 2>/dev/null || true
if test -f ./results/cooperation_status.json; then python3 ./analysis/verify_terminal.py > ./results/verification.log 2>&1; fi
kill "$fcu_pid" "$gazebo_pid" 2>/dev/null || true
wait "$fcu_pid" 2>/dev/null || true
wait "$gazebo_pid" 2>/dev/null || true
echo 'Owned simulation processes stopped after capture'
exit "$flight_code"
