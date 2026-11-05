"""Run existing raw physics auditors plus the new decision audit, preserving outputs."""
from pathlib import Path
import sys,subprocess,json,hashlib
R=Path(__file__).resolve().parents[1]
run=Path(sys.argv[1]).resolve();pkg=Path(sys.argv[2]).resolve() if len(sys.argv)>2 else R/'work/unified_decision_v1';res=run/'results'
s=json.loads((res/'execution_status.json').read_text())
assert s['completed'], 'Failed trials require failure-specific analysis; never force a success audit'
accel=s.get('accelerating_decision_trial',False)
commands=[]
def execute(name,*args):
 p=subprocess.run([sys.executable,'-X','utf8',str(R/'tools'/name),*map(str,args)],cwd=R,capture_output=True,text=True,encoding='utf-8')
 commands.append({'tool':name,'exit_code':p.returncode,'stdout':p.stdout,'stderr':p.stderr})
 print(json.dumps({'tool':name,'exit_code':p.returncode,'output_tail':p.stdout[-1200:],'stderr':p.stderr[-1000:]}),flush=True)
 assert p.returncode==0, name
if accel:
 execute('audit_acceleration_return.py',run,pkg,*([Path(sys.argv[3]).resolve()] if len(sys.argv)>3 else []))
 execute('audit_acceleration_interfaces.py',run)
 analysis=run/'decision_analysis'
else:
 execute('assemble_interrupted_descent_audit.py',run)
 execute('analyze_attitude_braking.py',run,'--status-override',run/'landing_analysis/assembled_execution_status.json',*(['--scenario-protocol',Path(sys.argv[3]).resolve()] if len(sys.argv)>3 else []))
 execute('audit_raw_continuous_motion.py',run)
 execute('audit_interrupted_descent.py',run,pkg)
 analysis=run/'landing_analysis'
if s.get('planning_handoff'):execute('audit_planning_handoff.py',run,pkg)
if s.get('direct_braking_plan',{}).get('stopping_governor',{}).get('enabled'):execute('audit_stopping_governor.py',run,pkg)
execute('audit_local_execution.py',run)
execute('audit_unified_descent_decision.py',run,pkg)
actual=json.loads((res/'attempt_manifest.json').read_text())['source_sha256']
profile=json.loads((R/'config/airframes/iris_fixed_gimbal_sim.json').read_text())
models={n:actual.get(n.replace('\\','/'))==h for n,h in profile['sources']['fixed_model_sha256'].items()}
old=json.loads((R/'outputs/model_audit/step23/run_003/results/dataflash_final/parameters.json').read_text())
new=json.loads((res/'dataflash_final/parameters.json').read_text())
gains={k:abs(v-new[k])<1e-8 for k,v in old.items() if k.startswith('ATC_')}
source={'passed':all(models.values()) and all(gains.values()),'original_airframe_hashes':models,'unchanged_native_attitude_parameters':gains}
(analysis/'source_consistency.json').write_text(json.dumps(source,indent=2))
checks={}
for name in ['verification.json','local_execution_verification.json','source_consistency.json']+(['interface_verification.json'] if accel else ['raw_motion_verification.json','interruption_verification.json']):
 checks[name]=json.loads((analysis/name).read_text())['passed']
if s.get('planning_handoff'):checks['planning_handoff_verification.json']=json.loads((analysis/'planning_handoff_verification.json').read_text())['passed']
if s.get('direct_braking_plan',{}).get('stopping_governor',{}).get('enabled'):checks['governor_verification.json']=json.loads((analysis/'governor_verification.json').read_text())['passed']
checks['unified_analysis']=json.loads((run/'unified_analysis/verification.json').read_text())['passed']
result={'passed':all(checks.values()),'case':'acceleration' if accel else 'braking','checks':checks,
        'commands':commands,'source_log_sha256':json.loads((res/'postflight_checks.json').read_text())['raw_sha256']}
target=run/'verification.json';assert not target.exists();target.write_text(json.dumps(result,indent=2))
print(json.dumps({k:v for k,v in result.items() if k!='commands'}))
