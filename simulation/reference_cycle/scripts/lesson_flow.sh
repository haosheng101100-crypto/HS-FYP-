#!/usr/bin/env bash
# Actual commands, staged briefly so the user can observe the model before flight.
set -euo pipefail
cd "$(dirname "$0")/.."
printf '\n现场讲解：代码如何变成无人机的实际运动\n项目：%s\n\n' "$PWD"
mkdir -p runtime results
printf 'preparing\n' > runtime/lesson_stage
archive="results/replay_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$archive"
for file in attitude_demo.csv attitude_demo.log attitude_demo_status.json; do
  if [[ -f "results/$file" ]]; then cp "results/$file" "$archive/$file"; fi
done
printf '1. 打开实验环境：场景 + 虚拟飞控\n'
printf '$ cd ~/hs_ardupilot_stage1\n$ bash scripts/stop_scene.sh\n'
bash scripts/stop_scene.sh
printf '$ bash scripts/start_scene.sh\n'
bash scripts/start_scene.sh
printf '\n场景已启动，飞机仍在地面。此时还没有执行起飞任务。\n'
printf '停留 30 秒用于观察 Gazebo，然后运行下面的控制程序。\n\n'
printf 'scene_ready\n' > runtime/lesson_stage
sleep 30
printf '2. 启动任务程序，它调用我们自己的姿态 PID\n'
printf '$ bash scripts/run_attitude.sh --task follow --car-speed 0.25 --drive-seconds 10\n'
bash scripts/run_attitude.sh --task follow --car-speed 0.25 --drive-seconds 10
printf 'controller_running\n' > runtime/lesson_stage
printf '\n3. 查看真实运行日志，并观察 Gazebo 中的动作\n'
printf '$ tail -f results/attitude_demo.log\n\n'
printf 'READY 准备 → ARM 解锁 → TAKEOFF 起飞 → HOVER 悬停\n'
printf '→ YAW_DEMO 转向 → ALIGN 对准 → FOLLOW 跟车 → LAND_DECK 下降 → COMPLETE 完成\n\n'
tail -n +1 -f results/attitude_demo.log
