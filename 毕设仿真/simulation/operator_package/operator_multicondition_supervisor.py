"""Own the interactive simulation, native controls and resettable cycles."""
from pathlib import Path
import os,json,subprocess,time,signal,shutil,hashlib,traceback
from planar_runtime_setup import desktop_environment,prepare_world_start
BASE=Path(os.environ.get('HS_SIM_BASE','/home/haosheng/hs_ardupilot_stage1'))
SESSION=Path(os.environ.get('HS_OPERATOR_SESSION',str(BASE/'interactive_20261023/receding_session_083')))
PKG=Path(os.environ.get('HS_OPERATOR_PACKAGE',str(BASE/'paper_experiments/operator_receding_v83')))
FROZEN=Path(os.environ.get('HS_FROZEN_PACKAGE',str(BASE/'paper_experiments/moving_rapid_v1')))
SOURCE=Path(os.environ.get('HS_SIM_SOURCE',str(BASE/'reproduction_20261018/fixed_gimbal_terminal_004')))
PROFILE=Path(os.environ.get('HS_LANDING_PROFILE',str(PKG/'config/experiments/receding_live_v1.json')))
VALIDATION_SCENARIOS=json.loads(PROFILE.read_text()).get('validation_scenarios',[])
SESSION.mkdir(exist_ok=False,parents=True);(SESSION/'requests').mkdir();(SESSION/'processed').mkdir()
display_env=dict(os.environ,**desktop_environment(os.environ),HS_OPERATOR_SESSION=str(SESSION))
panel_log=(SESSION/'panel.log').open('x')
panel=subprocess.Popen(['python3',str(PKG/'operator_panel.py')],cwd=SESSION,env=display_env,stdout=panel_log,stderr=subprocess.STDOUT,start_new_session=True)
all_cycles=[]

def atomic_status(value):
    p=SESSION/'supervisor_status.tmp';p.write_text(json.dumps(value,ensure_ascii=False));p.replace(SESSION/'status.json')

def stop_group(row,run):
    members=[]
    for p in Path('/proc').iterdir():
        if not p.name.isdigit():continue
        try:
            if os.getpgid(int(p.name))!=row['pid'] or not (p/'cmdline').read_bytes():continue
            cwd=(p/'cwd').resolve();assert cwd==run or run in cwd.parents,(p.name,str(cwd))
            members.append(int(p.name))
        except (FileNotFoundError,ProcessLookupError):pass
    if members:
        os.killpg(row['pid'],signal.SIGTERM)
        deadline=time.monotonic()+4
        while time.monotonic()<deadline:
            live=[]
            for pid in members:
                try:
                    if os.getpgid(pid)==row['pid'] and (Path('/proc')/str(pid)/'cmdline').read_bytes():live.append(pid)
                except (FileNotFoundError,ProcessLookupError):pass
            if not live:return
            time.sleep(.1)
        os.killpg(row['pid'],signal.SIGKILL)

try:
    for number in range(1,len(VALIDATION_SCENARIOS)+1 if VALIDATION_SCENARIOS else 1001):
        if (SESSION/'close.request').exists():break
        atomic_status({'stage':'INITIALIZING','message':'初始化中，尚未起飞；请稍候按钮亮起','armed':False,'paused':False,'buttons':{},'cycle':number})
        run=SESSION/f'cycle_{number:03d}';run.mkdir()
        for name in ['scripts','tools','models','worlds','config','first_stage','control']:
            shutil.copytree(SOURCE/name,run/name,ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
        manifest=json.loads((FROZEN/'package_manifest.json').read_text())
        for name,digest in manifest['source_sha256'].items():assert hashlib.sha256((FROZEN/name).read_bytes()).hexdigest()==digest,name
        for p in FROZEN.rglob('*'):
            if p.is_file() and '__pycache__' not in p.parts:
                dest=run/p.relative_to(FROZEN);dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,dest)
        for name in ['vendor','gz_prefix','venv']:(run/name).symlink_to(BASE/name,target_is_directory=True)
        (run/'runtime').mkdir();(run/'results').mkdir();(run/'runtime/gz_config').symlink_to(BASE/'runtime/gz_config',target_is_directory=True)
        for name in ['interactive_base.py','operator_backend.py','operator_clock.py','operator_legacy.py']:
            shutil.copy2(PKG/name,run/'tools'/name)
        for name in ['landing/receding_landing.py','landing/receding_live.py','landing/swept_envelope.py','landing/platform_prediction.py','landing/platform_filter.py','landing/planar_landing.py','landing/planar_motion.py','landing/prediction_uncertainty.py','config/airframes/iris_swept_envelope_v1.json','tools/multicondition_scene.py','tools/multicondition_bridge.py','tools/receding_planner_worker.py','tools/build_planar_profile.py','config/experiments/receding_live_v1.json']:
            shutil.copy2(PKG/name,run/name)
        shutil.copy2(PKG/'operator_multicondition_backend.py',run/'tools/operator_backend.py')
        shutil.copy2(PROFILE,run/'config/experiments/receding_live_v1.json')
        recorder_path=run/'tools/record_model_states.py'
        recorder_code=recorder_path.read_text()
        recorder_code=recorder_code.replace('node=Node()',"node=Node()\nfrom gz.msgs10.world_stats_pb2 import WorldStatistics\nphysics_paused=False\ndef on_stats(message):\n    global physics_paused\n    physics_paused=bool(message.paused)\nnode.subscribe(WorldStatistics,'/world/uav_vehicle_stage1/stats',on_stats)")
        recorder_code=recorder_code.replace('    def callback(message):','    def callback(message):\n        if physics_paused:return')
        recorder_path.write_text(recorder_code)
        live_cfg=json.loads(PROFILE.read_text())
        if live_cfg.get('deck_half_extents'):
            import xml.etree.ElementTree as ET
            model_path=run/'models/landing_vehicle/model.sdf';tree=ET.parse(model_path)
            hx,hy=live_cfg['deck_half_extents']
            for kind in ('collision','visual'):
                node=tree.find('.//'+kind+'[@name="landing_deck"]/geometry/box/size')
                assert node is not None;node.text=f'{2*hx} {2*hy} 0.12'
            tree.write(model_path,encoding='unicode',xml_declaration=True)
        if live_cfg.get('platform_motion_model')=='force_driven_planar_guide':
            import importlib.util
            spec=importlib.util.spec_from_file_location('planar_fixture_builder',PKG/'tools/build_planar_profile.py')
            builder=importlib.util.module_from_spec(spec);spec.loader.exec_module(builder)
            model_path=run/'models/landing_vehicle/model.sdf'
            original_path=model_path.with_name('original_differential_drive.sdf')
            shutil.copy2(model_path,original_path)
            builder.build_planar_model(original_path,model_path)
        world=run/'worlds/uav_vehicle_stage1.sdf';world.write_text(prepare_world_start(world.read_text()))
        cfg=(BASE/'gz_prefix/usr/share/gz/gz-sim8/gui/gui.config').read_text().replace('show_again="true"','show_again="false"')
        cfg=cfg.replace('<camera_pose>-6 0 6 0 0.5 0</camera_pose>','<camera_pose>3.5 -8 5 0 0.463647609 1.570796327</camera_pose>').replace('<start_paused>true</start_paused>','<start_paused>false</start_paused>')
        cfg=(PKG/'demo_gui.config').read_text()
        (run/'operator_gui.config').write_text(cfg)
        raw=subprocess.check_output(['bash','-c','source ./scripts/env.sh;env -0'],cwd=run)
        env=dict(x.decode().split('=',1) for x in raw.split(b'\0') if b'=' in x)
        env.update({k:display_env[k] for k in ['DISPLAY','XAUTHORITY','XDG_RUNTIME_DIR','DBUS_SESSION_BUS_ADDRESS','HS_OPERATOR_SESSION']},GZ_PARTITION='hs_multi_'+SESSION.parent.name+'_'+str(number),LP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1')
        rows=[];handles=[];files=[]
        def start(name,args):
            if name=='gazebo_gui':args=['nice','-n','15',*args]
            elif name=='model_states':args=['nice','-n','5',*args]
            f=(run/'results'/f'{name}.log').open('x');files.append(f)
            p=subprocess.Popen(args,cwd=run,env=env,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
            rows.append({'name':name,'pid':p.pid});handles.append(p)
            (run/'results/managed_processes.json').write_text(json.dumps(rows,indent=2));return p
        reset=False
        try:
            pre=start('preflight',['./venv/bin/python','-c','from pathlib import Path;from landing.reference_loader import load_verified_reference;load_verified_reference(Path.cwd());print("Frozen geometry and runtime verified")'])
            assert pre.wait(timeout=30)==0
            gz=start('gazebo',['gz','sim','-s','-r','-v','2','./worlds/uav_vehicle_stage1.sdf'])
            gui=start('gazebo_gui',['gz','sim','-g','-v','2','--gui-config',str(run/'operator_gui.config')])
            recorder=start('model_states',['python3','./tools/record_model_states.py','./results'])
            fcu=start('arducopter',['bash','./scripts/run_fcu.sh'])
            scene=start('wait_scene',['./venv/bin/python','./tools/wait_scene.py','--world','uav_vehicle_stage1'])
            assert scene.wait(timeout=200)==0
            topic='/world/uav_vehicle_stage1/model/landing_vehicle/link/chassis/sensor/deck_contact/contact'
            q=subprocess.run(['gz','topic','-i','-t',topic],env=env,cwd=run,capture_output=True,text=True,timeout=15)
            assert 'gz.msgs.Contacts' in q.stdout.split('Subscribers')[0] and 'tcp://' in q.stdout.split('Subscribers')[0]
            env['HS_DECK_CONTACT_TOPIC']=topic
            env['HS_AUTONOMOUS_TRIAL']='1'
            if VALIDATION_SCENARIOS:env['HS_TRIAL_SCENARIO']=VALIDATION_SCENARIOS[number-1]
            env['HS_PLANNER_DEPS']=str(PKG/'linux_deps')
            worker=start('receding_planner',['./venv/bin/python','-u','./tools/receding_planner_worker.py'])
            deadline=time.monotonic()+120
            while not (run/'runtime/receding_planner/health.json').exists():
                if worker.poll() is not None:raise RuntimeError('New planner worker failed; see receding_planner.log')
                if time.monotonic()>deadline:raise RuntimeError('Planner warmup timed out')
                time.sleep(.2)
            backend=start('operator_backend',['./venv/bin/python','-u','./tools/operator_backend.py'])
            (SESSION/'active_cycle.json').write_text(json.dumps({'run':str(run),'number':number,'processes':rows},indent=2))
            close_sent=False
            while backend.poll() is None:
                if ((SESSION/'close.request').exists() or gui.poll() is not None or panel.poll() is not None) and not close_sent:
                    (SESSION/'requests'/f'{time.time_ns()}_close.json').write_text(json.dumps({'action':'close'}));close_sent=True
                time.sleep(.2)
            # The completed, unarmed scene may remain visible; its raw recorder must stop.
            final_status=json.loads((SESSION/'status.json').read_text())
            if not final_status['armed']:
                stop_group(next(row for row in rows if row['name']=='model_states'),run)
                recorder.wait(timeout=5)
            if backend.returncode==10:reset=True
            elif VALIDATION_SCENARIOS and not json.loads((SESSION/'status.json').read_text())['armed']:
                reset=number<len(VALIDATION_SCENARIOS)
            elif not (SESSION/'close.request').exists():
                # Keep an error/finished scene available until Reset or Close.
                while gui.poll() is None and not (SESSION/'close.request').exists():
                    requests=sorted((SESSION/'requests').glob('*.json'))
                    for p in requests:
                        if json.loads(p.read_text()).get('action')=='reset':
                            p.rename(SESSION/'processed'/(run.name+'__'+p.name));reset=True;break
                    if reset:break
                    time.sleep(.2)
        except Exception as exc:
            try:last_status=json.loads((SESSION/'status.json').read_text())
            except (FileNotFoundError,json.JSONDecodeError):last_status={}
            if last_status.get('armed',False):
                (SESSION/'requests'/f'{time.time_ns()}_close.json').write_text(json.dumps({'action':'close','source':'native_supervisor_error'}))
            (run/'results/supervisor_error.txt').write_text(traceback.format_exc())
            atomic_status({'stage':'ERROR','message':'运行异常，保留原生安全退出：'+str(exc),'armed':bool(last_status.get('armed',False)),'paused':True,'buttons':{'reset':True},'cycle':number})
            while not (SESSION/'close.request').exists():
                for p in sorted((SESSION/'requests').glob('*.json')):
                    if json.loads(p.read_text()).get('action')=='reset':p.rename(SESSION/'processed'/(run.name+'__'+p.name));reset=True;break
                if reset:break
                time.sleep(.2)
        finally:
            # Never destroy the simulation while the last backend status still
            # reports an armed aircraft. Retain the scene for native recovery.
            while (SESSION/'status.json').exists() and json.loads((SESSION/'status.json').read_text()).get('armed',False):
                time.sleep(.5)
            for row in reversed(rows):stop_group(row,run)
            for p in handles:
                try:p.wait(timeout=2)
                except subprocess.TimeoutExpired:pass
            for f in files:f.close()
            all_cycles.append({'number':number,'run':str(run),'reset_requested':reset})
            (run/'results/cycle_closed.json').write_text(json.dumps({'all_owned_cycle_processes_stopped':True,'reset_requested':reset},indent=2))
            (SESSION/'cycles.json').write_text(json.dumps(all_cycles,indent=2))
        if not reset:break
finally:
    if panel.poll() is None:panel.terminate()
    panel_log.close()
