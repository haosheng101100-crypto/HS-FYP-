#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "$0")/env.sh"
export DISPLAY="${DISPLAY:-:0}"
export XAUTHORITY="${XAUTHORITY:-/run/user/1000/gdm/Xauthority}"
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/1000}"
exec gz sim -g --gui-config "$HS_GZ_PREFIX/share/gz/gz-sim8/gui/gui.config"
