"""Actual native Gazebo demonstrations; the accepted flight controller is unchanged.

Run inside the verified Linux container. Each output is immutable and new.
Recording is isolated to the Gazebo window, never the rest of the desktop.
"""
from pathlib import Path
import argparse,ctypes,ctypes.util,datetime,difflib,fcntl,gzip,hashlib,json,os,shutil,signal,socket,subprocess,sys,threading,time,uuid

CONTROL_COMMIT='748b27e28be42ad294769105c5c899d3e9ecfb25'
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def save(path,value):
 path=Path(path);tmp=path.with_suffix('.tmp');tmp.write_text(json.dumps(value,ensure_ascii=False,indent=2));tmp.replace(path)
def prepare_demo(root,output,axes):
 root=Path(root);output=Path(output)
 if not axes or len(set(axes))!=len(axes) or any(a not in ('x','y','xy') for a in axes):raise ValueError('Choose unique axes x, y, xy')
 if output.exists():raise FileExistsError('Preserve existing evidence; choose a new output')
 package=root/'simulation/operator_package'
 source=json.loads((package/'source_manifest.json').read_text())['source_sha256']
 for name,digest in source.items():
  if sha(package/name)!=digest:raise ValueError('Source mismatch: '+name)
 profile=root/'config/experiments/planar_reacquisition_v1.json'
 if sha(profile)!=source['config/experiments/planar_reacquisition_v1.json']:raise ValueError('Accepted root profile mismatch')
 cfg=json.loads(profile.read_text())
 cfg['validation_scenarios']=['planar_reacquire_'+a for a in axes]
 output.mkdir(parents=True,exist_ok=False)
 shutil.copytree(package,output/'operator',ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
 save(output/'profile.json',cfg)
 deployed={}
 for name,digest in source.items():
  if name.startswith(('landing/','tools/','config/airframes/')):deployed[name]=digest
  elif name in ('interactive_base.py','operator_backend.py','operator_clock.py','operator_legacy.py'):deployed['tools/'+name]=digest
 # Keep all flight/controller files byte-for-byte. Only supervisor presentation,
 # output attestation and a unique transport partition are adapted.
 p=output/'operator/operator_supervisor.py';original=p.read_text();code=original
 def replace(old,new):
  nonlocal code
  if code.count(old)!=1:raise ValueError('Unsupported supervisor presentation adapter')
  code=code.replace(old,new)
 replace('all_cycles=[]','all_cycles=[]\ntime.sleep(1)\nif panel.poll() is not None:raise RuntimeError("Panel startup failed before physics")')
 replace("GZ_PARTITION=f'hs_receding_20261023_{number:03d}'","GZ_PARTITION='hs_demo_'+os.environ['HS_DEMO_PARTITION']")
 replace("            pre=start('preflight'", "            declared=json.loads((SESSION.parent/'declared_protocol.json').read_text())\n            for name,digest in declared['source_sha256'].items():\n                assert hashlib.sha256((run/name).read_bytes()).hexdigest()==digest,name\n            assert json.loads((run/'config/experiments/receding_live_v1.json').read_text())==declared['control_configuration']\n            pre=start('preflight'")
 replace('            for row in reversed(rows):stop_group(row,run)','            # Presentation dwell only after the existing armed guard.\n            time.sleep(5)\n            for row in reversed(rows):stop_group(row,run)')
 gui=root/'simulation/environment_reference/demo_gui.config'
 shutil.copy2(gui,output/'operator/demo_gui.config')
 replace("        (run/'operator_gui.config').write_text(cfg)","        cfg=(PKG/'demo_gui.config').read_text()\n        (run/'operator_gui.config').write_text(cfg)")
 p.write_text(code)
 (output/'supervisor_adapter.diff').write_text(''.join(difflib.unified_diff(original.splitlines(True),code.splitlines(True),fromfile='accepted/operator_supervisor.py',tofile='demo/operator_supervisor.py')))
 save(output/'declared_protocol.json',dict(scope='毕设仿真 only; actual physical demo, not a repetition batch',control_commit=CONTROL_COMMIT,control_configuration=cfg,source_sha256=deployed,package_original_sha256=source,cases=[dict(scenario=s,expected='landing_after_reacquisition') for s in cfg['validation_scenarios']],thresholds_relaxed=False,initial_leg_gap_m=4.0))
 return output/'operator'

def verify_environment(root,base):
 root=Path(root);base=Path(base)
 fixed=json.loads((root/'simulation/environment_reference/source_snapshot.json').read_text())
 firmware=base/'vendor/arducopter'
 if not firmware.is_file() or sha(firmware)!=fixed['arducopter_sha256']:raise ValueError('Fixed ArduCopter firmware mismatch')
 for name,digest in fixed['files'].items():
  p=base/'vendor/ardupilot_gazebo'/name
  if not p.is_file() or sha(p)!=digest:raise ValueError('Fixed Gazebo plugin source mismatch: '+name)
 assets=json.loads((root/'simulation/environment_reference/demo_runtime_assets.json').read_text())
 plugin='build/libArduPilotPlugin.so'
 if plugin not in assets.get('vendor/ardupilot_gazebo',{}):raise ValueError('Loaded Gazebo plugin must be pinned')
 for location,files in assets.items():
  for name,digest in files.items():
   p=base/location/name
   if not p.is_file() or sha(p)!=digest:raise ValueError('Original model/runtime mismatch: '+str(p))
 return dict(firmware_sha256=sha(firmware),original_assets_verified=sum(map(len,assets.values())),plugin_files_verified=len(fixed['files']),loaded_plugin_sha256=sha(base/'vendor/ardupilot_gazebo'/plugin))

def acquire_lock(base):
 folder=Path(base)/'runtime';folder.mkdir(exist_ok=True)
 handle=(folder/'functional_demo.lock').open('a+')
 try:fcntl.flock(handle.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
 except BlockingIOError:
  handle.close();raise RuntimeError('Another demonstration owns the shared simulator deployment')
 return handle

def check_ports():
 # Ports owned by the fixed native SITL/Gazebo runtime. Fail before starting.
 for kind,ports in ((socket.SOCK_DGRAM,(9002,9003,14550)),(socket.SOCK_STREAM,(5760,5762,5763))):
  for port in ports:
   with socket.socket(socket.AF_INET,kind) as s:s.bind(('0.0.0.0',port))

def gazebo_window():
 """Find only the native Gazebo window on X11; return its ID, never root."""
 x=ctypes.CDLL(ctypes.util.find_library('X11'));P=ctypes.c_void_p;U=ctypes.c_ulong
 x.XOpenDisplay.argtypes=[ctypes.c_char_p];x.XOpenDisplay.restype=P
 d=x.XOpenDisplay(None)
 if not d:raise RuntimeError('Cannot open the current desktop display')
 x.XDefaultRootWindow.argtypes=[P];x.XDefaultRootWindow.restype=U
 x.XQueryTree.argtypes=[P,U,ctypes.POINTER(U),ctypes.POINTER(U),ctypes.POINTER(ctypes.POINTER(U)),ctypes.POINTER(ctypes.c_uint)]
 x.XFetchName.argtypes=[P,U,ctypes.POINTER(ctypes.c_char_p)]
 x.XFree.argtypes=[P];x.XCloseDisplay.argtypes=[P]
 try:
  root=U();parent=U();children=ctypes.POINTER(U)();count=ctypes.c_uint()
  x.XQueryTree(d,x.XDefaultRootWindow(d),ctypes.byref(root),ctypes.byref(parent),ctypes.byref(children),ctypes.byref(count))
  found=None
  for i in range(count.value):
   name=ctypes.c_char_p()
   if x.XFetchName(d,children[i],ctypes.byref(name)) and name.value:
    if name.value.decode(errors='replace')=='Gazebo Sim':found=int(children[i])
    x.XFree(ctypes.cast(name,P))
  if children:x.XFree(children)
  if found:
   x.XMoveResizeWindow.argtypes=[P,U,ctypes.c_int,ctypes.c_int,ctypes.c_uint,ctypes.c_uint]
   x.XRaiseWindow.argtypes=[P,U];x.XFlush.argtypes=[P]
   x.XMoveResizeWindow(d,found,0,0,1280,720);x.XRaiseWindow(d,found);x.XFlush(d)
  return found
 finally:x.XCloseDisplay(d)

def record_session(output,stop):
 session=output/'session_001';videos=output/'videos';videos.mkdir();timeline=(output/'recording_timeline.jsonl').open('x');active=None;proc=None;log=None;attempts=[]
 try:
  while not stop.is_set() or proc:
   try:cycle=json.loads((session/'active_cycle.json').read_text());ui=json.loads((session/'status.json').read_text())
   except (FileNotFoundError,json.JSONDecodeError):cycle={};ui={}
   run=Path(cycle['run']) if cycle else None
   if proc and (stop.is_set() or (run and str(run)!=active) or (Path(active)/'results/execution_status.json').exists()):
    if proc.poll() is None:proc.send_signal(signal.SIGINT)
    try:proc.wait(timeout=15)
    except subprocess.TimeoutExpired:proc.terminate();proc.wait(timeout=5)
    log.close();attempts[-1].update(exit_code=proc.returncode,finished_wall_unix_s=time.time());save(output/'recordings.json',attempts);proc=None;log=None
   if stop.is_set():break
   if run and str(run)!=active:
    wid=gazebo_window()
    if wid:
     active=str(run);time.sleep(.6)
     number=cycle['number'];axis=json.loads((output/'profile.json').read_text())['validation_scenarios'][number-1].rsplit('_',1)[1].upper()
     text=videos/f'{axis}_status.txt';text.write_text(f'{axis} | Gazebo physical simulation | wall-clock recording')
     video=videos/f'{axis}_raw.mp4';log=(videos/f'{axis}_ffmpeg.log').open('x')
     filt=f"pad=iw:ih+76:0:0:color=0x101820,drawtext=fontfile=/usr/local/share/fonts/hs-chinese/NotoSansCJK-Regular.ttc:textfile={text}:reload=1:fontcolor=white:fontsize=22:x=18:y=h-62"
     args=['ffmpeg','-nostdin','-hide_banner','-loglevel','warning','-n','-thread_queue_size','128','-f','x11grab','-draw_mouse','0','-framerate','10','-window_id',str(wid),'-i',os.environ['DISPLAY'],'-vf',filt,'-an','-c:v','libx264','-preset','ultrafast','-crf','23','-threads','1','-pix_fmt','yuv420p','-movflags','+frag_keyframe+empty_moov+default_base_moof','-g','30',str(video)]
     started=time.time();proc=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT)
     attempts.append(dict(axis=axis,cycle=str(run),video=str(video),window_id=wid,started_wall_unix_s=started,command=args));save(output/'recordings.json',attempts)
   if proc:
    if proc.poll() is not None:raise RuntimeError('Window recording ended unexpectedly; see ffmpeg log')
    stage=str(ui.get('stage','INITIALIZING'));height=ui.get('body_height_m');message=ui.get('message','')
    # Plain text avoids interpreting UI content as a filter/expression.
    info=text.with_suffix('.tmp');info.write_text(f'{axis}方向 | 实际物理仿真录屏（墙钟时间）\n阶段：{stage}  高度：{height if height is not None else "--"} m');info.replace(text)
    timeline.write(json.dumps(dict(wall_unix_s=time.time(),cycle=str(run),stage=stage,message=message,height_m=height,armed=ui.get('armed')),ensure_ascii=False)+'\n');timeline.flush()
   time.sleep(.25)
 except Exception as exc:
  save(output/'recording_error.json',dict(error=repr(exc)))
 finally:
  if proc:
   if proc.poll() is None:proc.send_signal(signal.SIGINT)
   proc.wait(timeout=15)
   if log:log.close()
   attempts[-1].update(exit_code=proc.returncode,finished_wall_unix_s=time.time());save(output/'recordings.json',attempts)
  timeline.close()

def inspect_completion(output,axes,record_requested,recorder_alive=False):
 output=Path(output);expected=['planar_reacquire_'+a for a in axes];trials=[]
 for run in sorted((output/'session_001').glob('cycle_*')):
  try:
   status=json.loads((run/'results/execution_status.json').read_text());closed=json.loads((run/'results/cycle_closed.json').read_text())
  except (FileNotFoundError,json.JSONDecodeError):status={};closed={}
  passed=bool(status.get('completed') is True and status.get('armed_at_save') is False and status.get('final_mode')=='LAND' and closed.get('all_owned_cycle_processes_stopped') is True)
  trials.append(dict(run=str(run),scenario=status.get('scenario'),landing_completed=passed,status_present=bool(status),closed=bool(closed)))
 seen=[r['scenario'] for r in trials];missing=[v for v in expected if v not in seen]
 physics=len(trials)==len(expected) and not missing and all(seen.count(v)==1 for v in expected) and all(r['landing_completed'] for r in trials)
 recordings=[]
 if record_requested:
  try:records=json.loads((output/'recordings.json').read_text())
  except (FileNotFoundError,json.JSONDecodeError):records=[]
  for axis in axes:
   selected=[v for v in records if v.get('axis')==axis.upper()];good=False;info={}
   if len(selected)==1:
    row=selected[0];video=Path(row['video'])
    if video.is_file() and row.get('exit_code') in (0,255) and row.get('finished_wall_unix_s'):
     # ffmpeg returns255 for the requested SIGINT finalization. Preserve that
     # exact status and independently require a playable, finalized video.
     try:
      probe=subprocess.run(['ffprobe','-v','error','-show_entries','stream=codec_type,width,height:format=duration','-of','json',str(video)],capture_output=True,text=True,timeout=15)
      info=json.loads(probe.stdout);good=bool(probe.returncode==0 and float(info.get('format',{}).get('duration',0))>.5 and any(v.get('codec_type')=='video' and v.get('width',0)>0 and v.get('height',0)>0 for v in info.get('streams',[])))
     except (OSError,ValueError,subprocess.TimeoutExpired):pass
   recordings.append(dict(axis=axis,usable=good,probe=info))
 recording_ok=not record_requested or (not recorder_alive and not (output/'recording_error.json').exists() and all(v['usable'] for v in recordings) and len(records)==len(axes))
 return dict(physics_complete=bool(physics),missing_scenarios=missing,actual_attempts=len(trials),trials=trials,recording_requested=record_requested,recording_complete=bool(recording_ok),recorder_still_running=bool(recorder_alive),recordings=recordings)

def audit_demo(root,output,axes,record_requested,recorder_alive=False):
 root=Path(root);output=Path(output);report=inspect_completion(output,axes,record_requested,recorder_alive);audits=[]
 if str(root) not in sys.path:sys.path.insert(0,str(root))
 from tools.run_repeatability_group_a import history_check
 declared=json.loads((output/'declared_protocol.json').read_text())
 for row in report['trials']:
  run=Path(row['run']);res=run/'results'
  if not row['closed']:
   audits.append(dict(run=str(run),passed=False,reason='No closed native cycle evidence'));continue
  integrity=all((run/n).is_file() and sha(run/n)==h for n,h in declared['source_sha256'].items())
  integrity=integrity and json.loads((run/'config/experiments/receding_live_v1.json').read_text())==declared['control_configuration']
  raw=res/'model_states.jsonl';packed=res/'model_states.jsonl.gz'
  if raw.exists() and not packed.exists():
   original=sha(raw)
   with raw.open('rb') as src,gzip.open(packed,'wb',compresslevel=1) as dst:shutil.copyfileobj(src,dst)
   digest=hashlib.sha256()
   with gzip.open(packed,'rb') as src:
    for block in iter(lambda:src.read(1048576),b''):digest.update(block)
   if digest.hexdigest()!=original:raise ValueError('Lossless raw-data verification failed')
   save(res/'raw_compression.json',dict(verified_lossless=True,sha256_uncompressed=original,bytes_uncompressed=raw.stat().st_size,bytes_compressed=packed.stat().st_size));raw.unlink()
  with (res/'demo_independent_audit.log').open('w') as log:
   proc=subprocess.run([sys.executable,str(root/'tools/audit_planar_reacquisition.py'),str(run)],cwd=root,stdout=log,stderr=subprocess.STDOUT)
  audit_path=res/'independent_planar_reacquisition_audit.json';audit=json.loads(audit_path.read_text()) if audit_path.exists() else {}
  history=history_check(run);save(res/'request_history_audit.json',history)
  audits.append(dict(run=str(run),scenario=row['scenario'],passed=bool(proc.returncode==0 and audit.get('passed') and integrity and history['passed']),audit_exit_code=proc.returncode,source_integrity=integrity,request_history_passed=history['passed'],physical_audit=audit))
 report.update(control_commit=CONTROL_COMMIT,strict_audits=audits,thresholds_relaxed=False,scope='Actual functional demonstrations; no large repetition matrix')
 report['passed']=bool(report['physics_complete'] and report['recording_complete'] and len(audits)==len(axes) and all(a['passed'] for a in audits))
 save(output/'demonstration_acceptance.json',report);return report

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--base',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--axes',nargs='+',choices=('x','y','xy'),default=['x','y','xy']);p.add_argument('--prepare-only',action='store_true');p.add_argument('--record',action='store_true');a=p.parse_args()
 root=Path(__file__).resolve().parents[1];output=a.output.resolve();base=a.base.resolve()
 environment_receipt={}
 if not a.prepare_only:
  deployment_lock=acquire_lock(base)
  check_ports();environment_receipt=verify_environment(root,base)
 package=prepare_demo(root,output,a.axes)
 if a.prepare_only:return 0
 for location in (base/'reference',base/'baseline',base/'venv/bin/python',base/'vendor/arducopter'):
  if not location.exists():raise FileNotFoundError(str(location))
 if a.record and not shutil.which('ffmpeg'):raise RuntimeError('ffmpeg is required for recording')
 env=dict(os.environ,HS_SIM_BASE=str(base),HS_OPERATOR_SESSION=str(output/'session_001'),HS_OPERATOR_PACKAGE=str(package),HS_LANDING_PROFILE=str(output/'profile.json'),HS_SIM_SOURCE=str(base/'reference'),HS_FROZEN_PACKAGE=str(base/'baseline'),HS_DEMO_PARTITION=uuid.uuid4().hex)
 # Guard the flight process from desktop/session signals. Native controls retain
 # responsibility for LAND/disarm; recording errors cannot terminate physics.
 state=dict(stage='RUNNING',control_commit=CONTROL_COMMIT,started_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),axes=a.axes,record_requested=a.record,environment_verified=environment_receipt);save(output/'run_receipt.json',state)
 stop=threading.Event();recorder=threading.Thread(target=record_session,args=(output,stop)) if a.record else None
 if recorder:recorder.start()
 with (output/'supervisor.log').open('x') as log:
  child=subprocess.Popen([sys.executable,str(package/'operator_supervisor.py')],cwd=package,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True,pass_fds=(deployment_lock.fileno(),))
  while child.poll() is None:
   try:ui=json.loads((output/'session_001/status.json').read_text())
   except (FileNotFoundError,json.JSONDecodeError):ui={}
   if ui.get('stage')=='ERROR' and not ui.get('armed',False):(output/'session_001/close.request').touch()
   time.sleep(.5)
 stop.set()
 if recorder:recorder.join(timeout=30)
 state.update(stage='AUDITING',supervisor_exit_code=child.returncode);save(output/'run_receipt.json',state)
 report=audit_demo(root,output,a.axes,a.record,bool(recorder and recorder.is_alive()))
 state.update(stage='COMPLETE' if report['passed'] else 'INCOMPLETE',acceptance_passed=report['passed'],finished_utc=datetime.datetime.now(datetime.timezone.utc).isoformat());save(output/'run_receipt.json',state)
 return 0 if child.returncode==0 and report['passed'] else 2
if __name__=='__main__':sys.exit(main())
