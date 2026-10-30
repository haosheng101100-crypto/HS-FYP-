#!/usr/bin/env bash
HS_STAGE1_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
export HS_STAGE1_ROOT
export GZ_VERSION=harmonic
export GZ_PARTITION=hs_ardupilot_stage1
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
export QT_QPA_PLATFORM=xcb
HS_GZ_PREFIX="$HS_STAGE1_ROOT/gz_prefix/usr"
export HS_GZ_PREFIX
if [[ -d "$HS_GZ_PREFIX" ]]; then
  export PATH="$HS_GZ_PREFIX/bin:$PATH"
  export LD_LIBRARY_PATH="$HS_GZ_PREFIX/lib/x86_64-linux-gnu/OGRE-2.3:$HS_GZ_PREFIX/lib/x86_64-linux-gnu:$HS_GZ_PREFIX/lib:${LD_LIBRARY_PATH:-}"
  export CMAKE_PREFIX_PATH="$HS_GZ_PREFIX:${CMAKE_PREFIX_PATH:-}"
  export PKG_CONFIG_PATH="$HS_GZ_PREFIX/lib/x86_64-linux-gnu/pkgconfig:${PKG_CONFIG_PATH:-}"
  export PYTHONPATH="$HS_GZ_PREFIX/lib/python3/dist-packages:/usr/lib/python3/dist-packages:${PYTHONPATH:-}"
  export GZ_CONFIG_PATH="$HS_STAGE1_ROOT/runtime/gz_config"
  export GZ_SIM_SYSTEM_PLUGIN_PATH="$HS_GZ_PREFIX/lib/x86_64-linux-gnu/gz-sim-8/plugins:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
  export GZ_SIM_GUI_PLUGIN_PATH="$HS_GZ_PREFIX/lib/x86_64-linux-gnu/gz-sim-8/plugins:${GZ_SIM_GUI_PLUGIN_PATH:-}"
  export GZ_GUI_PLUGIN_PATH="$HS_GZ_PREFIX/lib/x86_64-linux-gnu/gz-sim-8/plugins/gui:$HS_GZ_PREFIX/lib/x86_64-linux-gnu/gz-gui-8/plugins:${GZ_GUI_PLUGIN_PATH:-}"
  export QML2_IMPORT_PATH="$HS_GZ_PREFIX/lib/x86_64-linux-gnu/gz-sim-8/plugins/gui:${QML2_IMPORT_PATH:-}"
  export GZ_RENDERING_PLUGIN_PATH="$HS_GZ_PREFIX/lib/x86_64-linux-gnu/gz-rendering-8/engine-plugins:${GZ_RENDERING_PLUGIN_PATH:-}"
  export GZ_PHYSICS_ENGINE_PATH="$HS_GZ_PREFIX/lib/x86_64-linux-gnu/gz-physics-7/engine-plugins:${GZ_PHYSICS_ENGINE_PATH:-}"
  export GZ_SIM_PHYSICS_ENGINE_PATH="$GZ_PHYSICS_ENGINE_PATH"
  export GZ_SIM_RENDER_ENGINE_PATH="$GZ_RENDERING_PLUGIN_PATH"
  export GZ_RENDERING_RESOURCE_PATH="$HS_GZ_PREFIX/share/gz/gz-rendering8"
  export OGRE2_RESOURCE_PATH="$HS_GZ_PREFIX/lib/x86_64-linux-gnu/OGRE-2.3/OGRE"
  export OGRE2_PLUGIN_PATH="$HS_GZ_PREFIX/lib/x86_64-linux-gnu/OGRE-2.3/OGRE"
fi
export GZ_SIM_SYSTEM_PLUGIN_PATH="$HS_STAGE1_ROOT/vendor/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="$HS_STAGE1_ROOT/models:$HS_STAGE1_ROOT/worlds:$HS_STAGE1_ROOT/vendor/ardupilot_gazebo/models:$HS_STAGE1_ROOT/vendor/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH:-}"
