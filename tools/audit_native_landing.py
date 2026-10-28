"""Check native DataFlash mode/arming journal; no FCU commands."""
from pathlib import Path
import argparse,json,hashlib
from pymavlink import DFReader,mavutil
p=argparse.ArgumentParser();p.add_argument('run',type=Path);a=p.parse_args();run=a.run
files=sorted((run/'runtime/logs').glob('*.BIN'));assert files
events=[];arming=[];mode=None;arm_time=None;flight_errors=[];crash=[]
for path in files:
 reader=DFReader.DFReader_binary(str(path))
 while True:
  m=reader.recv_match(type=['MODE','ARM','ERR','MSG'])
  if m is None:break
  d=m.to_dict();kind=m.get_type()
  if kind=='MODE':mode=int(d.get('ModeNum',d.get('Mode')));events.append(d)
  elif kind=='ARM':
   d['mode_at_event']=mode;arming.append(d);events.append(d)
   if d['ArmState']==1:arm_time=d['TimeUS']
  elif kind=='ERR' and arm_time is not None and d['TimeUS']>=arm_time and d['ECode']!=0:flight_errors.append(d)
  elif kind=='MSG' and 'crash' in d.get('Message','').lower():crash.append(d)
armed=[d for d in arming if d['ArmState']==1];disarmed=[d for d in arming if d['ArmState']==0]
mode_names=mavutil.mode_mapping_acm
checks=dict(native_arming_recorded=bool(armed),native_disarming_recorded=bool(disarmed),
    all_arm_and_disarm_nonforced=bool(arming) and all(d.get('Forced')==0 for d in arming),
    disarm_after_arming=bool(armed and disarmed) and min(d['TimeUS'] for d in disarmed)>min(d['TimeUS'] for d in armed),
    all_disarming_in_native_land=bool(disarmed) and all(mode_names.get(d['mode_at_event'])=='LAND' for d in disarmed),
    no_native_flight_error=not flight_errors,no_crash_message=not crash)
report=dict(passed=all(checks.values()),checks=checks,native_events=events,flight_errors=flight_errors,crash_messages=crash,
    source_sha256={f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in files},
    interpretation='Native mode at each nonforced disarm is checked using installed pymavlink mode mapping. Numeric arming Method values are preserved without guessing their enum labels.')
out=run/'results/native_landing_audit.json';assert not out.exists();out.write_text(json.dumps(report,indent=2));print(json.dumps(dict(passed=report['passed'],checks=checks),indent=2))
