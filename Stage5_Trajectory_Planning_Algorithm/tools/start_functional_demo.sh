#!/usr/bin/env bash
# Current Ubuntu installation; creates a fresh native simulation and recording.
set -euo pipefail
project=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
axis=${1:-all}
case "$axis" in x|y|xy) axes=("$axis");; all) axes=(x y xy);; *) echo '用法：start_functional_demo.sh [x|y|xy|all]'; exit 2;; esac
workspace=${HS_WORKSPACE:-}
if [[ -z "$workspace" ]]; then
  candidate=$project
  while [[ "$candidate" != / ]]; do
    if [[ -d "$candidate/work/reacquisition_20261102/deployment/reference" ]]; then workspace=$candidate; break; fi
    candidate=$(dirname -- "$candidate")
  done
fi
if [[ -z "$workspace" || ! -d "$workspace/work/reacquisition_20261102/deployment/reference" ]]; then
  echo '未找到已验证的本机仿真环境。请设置 HS_WORKSPACE 为包含 work/reacquisition_20261102/deployment 的工作区。'; exit 2
fi
workspace=$(cd -- "$workspace" && pwd)
case "$project/" in "$workspace/"*) ;; *) echo '演示工程必须位于 HS_WORKSPACE 内。'; exit 2;; esac
image=hs-fyp-demo:20261102-verified
sudo -v
sudo docker image inspect "$image" >/dev/null
if [[ -z "${DISPLAY:-}" || ! -f "${XAUTHORITY:-}" ]]; then echo '请在已登录的 Ubuntu 桌面终端运行；需要当前图形连接。'; exit 2; fi
stamp=$(date +%Y%m%d_%H%M%S)
container="hs-demo-${stamp}-$$"
output="$workspace/outputs/毕设功能演示_${stamp}_$$"
inside_project="/workspace/${project#"$workspace/"}"
inside_output="/workspace/${output#"$workspace/"}"
# Each run gets the CURRENT desktop authorization; old containers are untouched.
sudo docker run -d --name "$container" --network host --ipc host \
  -v "$workspace:/workspace" -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
  -v "$XAUTHORITY:/tmp/host.Xauthority:ro" \
  -e QT_AUTO_SCREEN_SCALE_FACTOR=0 -e QT_ENABLE_HIGHDPI_SCALING=0 -e QT_SCALE_FACTOR=1 -e GDK_SCALE=1 \
  -e "DISPLAY=$DISPLAY" -e XAUTHORITY=/tmp/host.Xauthority -e XDG_RUNTIME_DIR=/tmp/hs-gui-runtime \
  "$image" sleep infinity >/dev/null
sudo docker exec "$container" sh -c 'mkdir -p /tmp/hs-gui-runtime; chmod 700 /tmp/hs-gui-runtime'
echo "正在演示 ${axes[*]}，结果和录屏保存至：$output"
# Do not use an EXIT trap that kills a running aircraft. Native LAND/disarm owns cleanup.
set +e
sudo systemd-inhibit --what=sleep:idle --who='HS UAV demonstration' --why='Native simulation and actual recording' --mode=block \
  docker exec -w "$inside_project" "$container" /opt/hs-venv/bin/python -u tools/run_functional_demo.py \
  --base /workspace/work/reacquisition_20261102/deployment --output "$inside_output" --axes "${axes[@]}" --record
result=$?
set -e
sudo docker exec "$container" chown -R "$(id -u):$(id -g)" "$inside_output"
echo "演示运行结束（退出码 $result）。视频位于 $output/videos；验收结论见 demonstration_acceptance.json。"
exit "$result"
