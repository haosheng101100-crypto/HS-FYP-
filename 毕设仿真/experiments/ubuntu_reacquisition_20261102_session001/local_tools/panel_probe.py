from pathlib import Path
import json,os,subprocess,time
out=Path('/workspace/outputs/重新跟随专项_20261102/panel_probe')
env=dict(os.environ,HS_OPERATOR_SESSION=str(out))
with (out/'panel.log').open('w') as log:
 p=subprocess.Popen(['/opt/hs-venv/bin/python','/workspace/work/reacquisition_20261102/deployment/operator/operator_panel.py'],env=env,stdout=log,stderr=subprocess.STDOUT)
 try:
  time.sleep(12);alive=p.poll() is None
  result={'alive_after_12_s':alive,'ipc_mode':'host','flight_commands_sent':False,'exit_code_before_cleanup':p.returncode}
  (out/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
  assert alive,'Panel failed: '+(out/'panel.log').read_text()
 finally:
  if p.poll() is None:p.terminate();p.wait(timeout=5)
