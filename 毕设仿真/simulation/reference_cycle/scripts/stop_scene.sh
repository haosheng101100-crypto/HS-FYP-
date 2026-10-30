#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "$0")/env.sh"
gz topic -t /vehicle/cmd_vel -m gz.msgs.Twist -p 'linear: {x: 0} angular: {z: 0}' || true
python3 - <<'PY'
from pathlib import Path
import os,signal,time
base=Path(os.environ['HS_STAGE1_ROOT']);processes={};targets=set()
for d in Path('/proc').iterdir():
    if not d.name.isdigit():continue
    try:
        cmd=(d/'cmdline').read_bytes().split(b'\0')
        stat=(d/'stat').read_text().split(') ')[1].split()
        processes[int(d.name)]=(int(stat[1]),cmd)
        args=b' '.join(cmd)
        if str(base).encode() in args and (b'cooperation_demo.py' in args or b'attitude_demo.py' in args or cmd[0]==str(base/'vendor/arducopter').encode()):
            targets.add(int(d.name))
        if b'gz sim' in args and b'GZ_PARTITION=hs_ardupilot_stage1' in (d/'environ').read_bytes().split(b'\0'):
            targets.add(int(d.name))
    except (OSError,ValueError,IndexError):pass
for name in ('cooperation','attitude','arducopter','gazebo'):
    f=base/'results'/f'{name}.pid'
    if f.exists():
        pid=int(f.read_text())
        if pid in processes and str(base).encode() in b' '.join(processes[pid][1]):targets.add(pid)
while True:
    children={pid for pid,(parent,_) in processes.items() if parent in targets}
    if children<=targets:break
    targets.update(children)
targets.discard(os.getpid());targets.discard(os.getppid())
for pid in sorted(targets,reverse=True):
    try:os.kill(pid,signal.SIGTERM)
    except ProcessLookupError:pass
pending=list(targets)
deadline=time.monotonic()+8
while time.monotonic()<deadline:
    pending=[]
    for pid in targets:
        try:
            current=(Path('/proc')/str(pid)/'cmdline').read_bytes().split(b'\0')
            if current==processes[pid][1] and current[0]:pending.append(pid)
        except OSError:pass
    if not pending:break
    time.sleep(.2)
for pid in pending:
    try:
        current=(Path('/proc')/str(pid)/'cmdline').read_bytes().split(b'\0')
        if current==processes[pid][1]:os.kill(pid,signal.SIGKILL)
    except (OSError,ProcessLookupError):pass
print(f'Stopped {len(targets)} project simulation processes')
PY
