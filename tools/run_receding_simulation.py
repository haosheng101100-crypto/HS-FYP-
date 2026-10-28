"""Freeze a fresh version and launch the real VM experiment without overwriting evidence."""
from pathlib import Path
import argparse,json,re,subprocess,sys,socket,time

ROOT=Path(__file__).resolve().parents[1]
ALL=['surge','braking','fast_maneuvering','constant','varying','turning','maneuvering','noisy_maneuvering','switchback']


def check_previous(current):
    """Refuse an overlapping flight before advancing local staging/version."""
    code="""set -eu
python3 - <<'PY'
from pathlib import Path
import json
s=Path('/home/haosheng/hs_ardupilot_stage1/interactive_20261023/receding_session___VERSION__')
d=json.loads((s/'status.json').read_text())
if d.get('armed',True):raise RuntimeError('当前仿真仍在飞行或需要检查中断记录；没有创建新试验。')
if d.get('stage') not in ('COMPLETE','ERROR'):raise RuntimeError('当前仿真尚未结束；没有创建新试验。')
cycles=list(s.glob('cycle_*'))
if not cycles:raise RuntimeError('上一轮缺少完整记录，需要先检查；没有创建新试验。')
for c in cycles:
 if not (c/'results/cycle_closed.json').exists():raise RuntimeError('当前仿真尚未安全结束；没有创建新试验。')
print('上一轮已安全结束，可以启动新仿真。')
PY
""".replace('__VERSION__',f'{current:03d}')
    path=ROOT/'work'/f'check_previous_receding_{current:03d}.sh'
    path.write_text(code,encoding='utf-8')
    subprocess.run([sys.executable,'work/paper_key_remote.py','--script',str(path)],cwd=ROOT,check=True)

def ensure_vm():
    vmrun=Path('D:/ROS/vmrun.exe');vm=Path('D:/64/64.vmx')
    state=subprocess.run([str(vmrun),'list'],capture_output=True,text=True,check=True).stdout
    if str(vm).replace('/','\\').lower() not in state.lower():
        print('正在启动原仿真虚拟机……',flush=True)
        subprocess.run([str(vmrun),'start',str(vm),'nogui'],check=True)
    # This is the interactive simulator the person invoking this entry needs.
    subprocess.Popen(['D:/ROS/vmware.exe','-t',str(vm)],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
    deadline=time.monotonic()+180
    while True:
        try:
            with socket.create_connection(('192.168.159.131',22),timeout=2):break
        except OSError:
            if time.monotonic()>deadline:raise RuntimeError('虚拟机 SSH 尚未就绪；没有启动新的飞行。')
            time.sleep(2)

def main():
    parser=argparse.ArgumentParser(description='启动原虚拟机及真实 Gazebo/ArduCopter 自动仿真。')
    parser.add_argument('--scenario',choices=['all']+ALL,default='all')
    a=parser.parse_args()
    for pattern in ['test_receding_live.py','test_receding_entry_capture.py','test_receding_command_limits.py','test_receding_frame_reserve.py','test_receding_gain_speed.py','test_receding_heading_guard.py','test_receding_tracking_envelope.py','test_platform_filter.py','test_prediction_uncertainty.py','test_directional_landing.py','test_terminal_capture_window.py','test_landing_alignment.py']:
        subprocess.run([sys.executable,'-m','unittest','discover','-s','tests','-p',pattern,'-q'],cwd=ROOT,check=True)
    ensure_vm()
    supervisor=(ROOT/'work/operator_receding_v1/operator_supervisor.py').read_text(encoding='utf-8-sig')
    current=int(re.search(r"PKG=BASE/'paper_experiments/operator_receding_v(\d+)'",supervisor).group(1))
    check_previous(current)
    config=ROOT/'config/experiments/receding_live_v1.json';cfg=json.loads(config.read_text())
    cfg['validation_scenarios']=ALL if a.scenario=='all' else [a.scenario]
    config.write_text(json.dumps(cfg,indent=2,ensure_ascii=False))
    version=current+1
    subprocess.run([sys.executable,'work/promote_receding_trial.py','--previous',str(current),'--version',str(version)],cwd=ROOT,check=True)
    subprocess.run([sys.executable,'work/paper_key_remote.py','--script',f'work/launch_receding_operator_v{version}.sh','--upload','work/operator_receding_v1','--remote-version',f'operator_receding_v{version}'],cwd=ROOT,check=True)
    print(f'已启动独立仿真 receding_session_{version:03d}；Gazebo 窗口在 Ubuntu 虚拟机中。')
    print('结果由实际接触及支撑记录判定；请在试验结束后运行独立物理审计。')

if __name__=='__main__':main()
