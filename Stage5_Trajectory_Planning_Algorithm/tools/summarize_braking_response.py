"""Compare recorded command and measured response; no fitted control model."""
import json,math,sys
from pathlib import Path
run=Path(sys.argv[1]);status=json.loads((run/'results/execution_status.json').read_text())
rows=[json.loads(l) for l in (run/'results/execution_feedback.jsonl').read_text().splitlines()]
detect=status.get('measured_braking_detection',{}).get('sim_t_s')
if detect is None:
 print(json.dumps({'completed':status['completed'],'failures':status.get('external_failures')}));raise SystemExit
samples=[]
for dt in [0,.12,.24,.4,.6,.8,1.,1.2,1.6,2.,2.4,2.68]:
 available=[r for r in rows if r['stage']=='DECK_LANDING' and r['sim_t_s']>=detect]
 if not available or available[-1]['sim_t_s']<detect+dt-.04:continue
 row=min(available,key=lambda r:abs(r['sim_t_s']-detect-dt));cmd=row.get('attitude_command')
 samples.append({'dt_s':round(row['sim_t_s']-detect,3),'speed_m_s':row['observation']['motion']['velocity_at_origin_world_m_s'][0],
  'measured_ax':row['fit']['body_acceleration_world_m_s2'][0],'actual_tilt_deg':row['tilt_deg'],
  'actual_pitch_rate_rad_s':-row['observation']['motion']['angular_velocity_world_rad_s'][1],
  'command_pitch_deg':math.degrees(cmd['pitch_command_rad']) if cmd else None,
  'command_rates':cmd.get('body_rate_frd_rad_s') if cmd else None,
  'position_error':row['tracking_position_error_m'],'speed_error':row['tracking_speed_error_m_s']})
print(json.dumps({'completed':status['completed'],'failures':status.get('external_failures'),
 'predictive_rejection':status.get('predictive_rejection',{}).get('guard'),'samples':samples},ensure_ascii=False))
