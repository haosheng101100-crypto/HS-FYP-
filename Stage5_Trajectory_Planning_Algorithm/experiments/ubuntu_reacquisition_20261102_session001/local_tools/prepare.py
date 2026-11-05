from pathlib import Path
import shutil,json,hashlib,difflib
w=Path(__file__).resolve().parents[2];p=w/'work/reacquisition_20261102';r=p/'repository/毕设仿真';d=p/'deployment';o=w/'outputs/重新跟随专项_20261102'
assert not r.exists() and not d.exists() and not o.exists()
shutil.copytree(w/'work/linux_handoff_20261102/repository/毕设仿真',r,ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
m=json.loads((r/'handoff/source_package_manifest.json').read_text())
for n,h in m['files_sha256'].items():assert hashlib.sha256((r/n).read_bytes()).hexdigest()==h,n
shutil.copytree(w/'work/planar_20261101/deployment',d,symlinks=True,ignore=shutil.ignore_patterns('operator','__pycache__','*.pyc'))
shutil.copytree(r/'simulation/operator_package',d/'operator',ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
profile=json.loads((r/'config/experiments/planar_reacquisition_v1.json').read_text())
sm=json.loads((d/'operator/source_manifest.json').read_text())
for n,h in sm['source_sha256'].items():assert hashlib.sha256((d/'operator'/n).read_bytes()).hexdigest()==h,n
names=['landing/receding_landing.py','landing/receding_live.py','landing/swept_envelope.py','landing/platform_prediction.py','landing/platform_filter.py','landing/planar_landing.py','landing/planar_motion.py','landing/prediction_uncertainty.py','config/airframes/iris_swept_envelope_v1.json','tools/receding_planner_worker.py','tools/build_planar_profile.py']
hashes={n:hashlib.sha256((d/'operator'/n).read_bytes()).hexdigest() for n in names}
for n in ['interactive_base.py','operator_backend.py','operator_clock.py','operator_legacy.py']:hashes['tools/'+n]=hashlib.sha256((d/'operator'/n).read_bytes()).hexdigest()
source=d/'operator/operator_supervisor.py';old=source.read_text();s=old.replace("GZ_PARTITION=f'hs_receding_20261023_{number:03d}'","GZ_PARTITION=f'hs_reacquisition_20261102_{number:03d}'")
needle="            pre=start('preflight'";guard="""            declared=json.loads((SESSION.parent/'declared_protocol.json').read_text())
            for name,digest in declared['source_sha256'].items():
                assert hashlib.sha256((run/name).read_bytes()).hexdigest()==digest,name
            assert json.loads((run/'config/experiments/receding_live_v1.json').read_text())==declared['control_configuration']
"""
assert s.count(needle)==1;s=s.replace(needle,guard+needle)
s=s.replace('all_cycles=[]','all_cycles=[]\ntime.sleep(1)\nif panel.poll() is not None:raise RuntimeError("Panel startup failed before physics")')
compile(s,'supervisor','exec');source.write_text(s)
o.mkdir();(o/'declared_protocol.json').write_text(json.dumps({'source_commit':'ca86ce7cf8c8ee1a392a23867296718b7583d59e','feature_commit':'58065a94335388d6b698f9463cf30185fc4d3cfa','control_configuration':profile,'source_sha256':hashes,'cases':[{'scenario':n,'expected':profile['validation_expectations'][n]} for n in profile['validation_scenarios']],'thresholds_relaxed':False,'initial_conditions_changed':'Independent 4m leg-clearance trial; not same-condition comparison to previous1.5m matrix','scope':'毕设仿真 only'},ensure_ascii=False,indent=2))
(o/'supervisor_adapter.diff').write_text(''.join(difflib.unified_diff(old.splitlines(True),s.splitlines(True))))
for filename in ['check_runtime.py','panel_probe.py']:
 s=(w/'work/planar_20261101'/filename).read_text().replace('work/planar_20261101','work/reacquisition_20261102').replace('outputs/二维平台仿真_20261101','outputs/重新跟随专项_20261102');(p/filename).write_text(s)
print('Pinned source and deployed source verified; three declared cases prepared')
