#!/usr/bin/env bash
# Visible Ubuntu terminal walkthrough; every displayed command is actually run.
set -euo pipefail
cd "$(dirname "$0")/.."
printf '\n车机协同仿真：实际运行流程\n项目目录：%s\n\n' "$PWD"
printf 'VS Code 用来阅读和编辑代码；下面的 Ubuntu 命令启动实际仿真。\n\n'
printf '第 1 步：进入项目目录\n$ cd ~/hs_ardupilot_stage1\n\n'
printf '第 2 步：保存上次结果，并让无人机回到初始地面场景\n'
mkdir -p results/pid_first_run
for file in attitude_demo.csv attitude_demo.log attitude_demo_status.json; do
  if [[ -f "results/$file" && ! -e "results/pid_first_run/$file" ]]; then
    cp "results/$file" "results/pid_first_run/$file"
  fi
done
printf '$ bash scripts/stop_scene.sh\n'
bash scripts/stop_scene.sh
printf '\n第 3 步：启动 Gazebo 场景和 ArduCopter 虚拟飞控\n$ bash scripts/start_scene.sh\n'
bash scripts/start_scene.sh
printf '\n第 4 步：启动自己的姿态 PID 飞行程序\n'
printf '$ bash scripts/run_attitude.sh --task follow --car-speed 0.25 --drive-seconds 10\n'
bash scripts/run_attitude.sh --task follow --car-speed 0.25 --drive-seconds 10
printf '\n第 5 步：查看真实飞行日志，同时观察 Gazebo 窗口\n'
printf '$ tail -f results/attitude_demo.log\n\n'
printf '阶段：READY → ARM → TAKEOFF → HOVER → YAW_DEMO → ALIGN → FOLLOW → LAND_DECK → COMPLETE\n'
printf '结果文件：results/attitude_demo.csv 和 results/attitude_demo_status.json\n'
printf '按 Ctrl+C 只结束日志查看；停止整个场景使用 bash scripts/stop_scene.sh。\n\n'
tail -n +1 -f results/attitude_demo.log
