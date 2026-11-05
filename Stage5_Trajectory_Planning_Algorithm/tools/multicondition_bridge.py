"""Measured world-side scene inputs. No trajectory information enters prediction."""
def measured_scene_state(now,gap,vertical_speed,phase,events,contact_detected=False):
 return dict(sim_t_s=now,clearance_m=gap,vertical_speed_m_s=vertical_speed,
             phase='CONTACT_HANDOVER' if contact_detected else phase,
             reacquisitions=sum(v.get('event')=='PLANAR_REACQUIRE' for v in events),
             activated_plan_count=sum(v.get('event')=='PLAN_ACTIVATED' for v in events),
             contact_detected=bool(contact_detected),contact_complete=phase in ('CONTACT_COMPLETE','COMPLETE'))

def initialize_world_scene(case_file,identity,scenario,results):
 import json
 from pathlib import Path
 from tools.multicondition_scene import ScenarioProgram
 case=json.loads(Path(case_file).read_text())
 if case['scenario']!=scenario or case['case_id']!=identity['case_id'] or identity['scenario']!=scenario:
  raise ValueError('World-side case does not match declared scenario')
 program=ScenarioProgram(case)
 for name,value in [('world_case.json',case),('scene_maneuvers.json',[])]:
  with (Path(results)/name).open('x') as stream:json.dump(value,stream,ensure_ascii=False,indent=2)
 return program
