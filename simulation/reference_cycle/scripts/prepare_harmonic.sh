#!/usr/bin/env bash
# Install Harmonic in this project without replacing Gazebo Classic / ROS.
set -euo pipefail
root=$(cd "$(dirname "$0")/.." && pwd)
if (( EUID!=0 )); then exec sudo bash "$0";fi
export DEBIAN_FRONTEND=noninteractive
mkdir -p "$root/results" "$root/downloads/harmonic" "$root/gz_prefix" "$root/runtime/gz_config"
curl -fsSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
printf '%s\n' 'deb [arch=amd64 signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main' > /etc/apt/sources.list.d/hs-gazebo-stable.list
apt-get update -qq
apt-get -s install --no-install-recommends libgz-sim8-dev libgz-physics7-dartsim libgz-rendering8-ogre1 libgz-rendering8-ogre2 gz-sim8-cli gz-transport13-cli rapidjson-dev libopencv-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl python3-gz-transport13 python3-gz-msgs10 ruby python3-venv ffmpeg > "$root/results/dependency_plan.log"
export HS_PREPARE_ROOT="$root"
python3 - <<'PY'
from pathlib import Path
import os,re,subprocess
b=Path(os.environ['HS_PREPARE_ROOT']);native=[];isolated=[]
for line in (b/'results/dependency_plan.log').read_text().splitlines():
    m=re.match(r'Inst (\S+)(?: \[[^]]+\])? \((\S+) (.*)\)',line)
    if m:
        name,version,source=m.groups()
        (isolated if 'OSRF:' in source else native).append(name+'='+version)
if native:subprocess.run(['apt-get','install','-y','--no-remove','--no-install-recommends',*native],check=True)
d=b/'downloads/harmonic';prefix=b/'gz_prefix'
if isolated:subprocess.run(['apt-get','download',*isolated],cwd=d,check=True)
for deb in sorted(d.glob('*.deb')):subprocess.run(['dpkg-deb','-x',str(deb),str(prefix)],check=True)
for f in (prefix/'usr/share/gz').glob('*.yaml'):
    (b/'runtime/gz_config'/f.name).write_text(f.read_text().replace('library_path: /usr/',f'library_path: {prefix}/usr/'))
owner=os.environ.get('SUDO_USER')
if owner:
    for path in ('downloads','gz_prefix','runtime','results'):
        subprocess.run(['chown','-R',f'{owner}:{owner}',str(b/path)],check=True)
print('Harmonic dependencies prepared in project directory')
PY
