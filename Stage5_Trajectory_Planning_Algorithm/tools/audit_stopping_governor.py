"""Replay the declared distance governor and native actuation from actual inputs."""
from pathlib import Path
import sys,json,math,hashlib,ast,textwrap,inspect,datetime
R=Path(__file__).resolve().parents[1];run=Path(sys.argv[1]).resolve();pkg=Path(sys.argv[2]).resolve();sys.path[:0]=[str(pkg),str(R/'tools'),str(R)]
from verify_unified_decision_interface import observation,deck
from landing.reference_loader import load_verified_reference
from landing.braking_candidates import state_selected_braking_plan
from landing.stopping_governor import StopDistanceAttitudeController,_NativeHorizontalDemandHook
from landing.attitude_braking import AttitudeCaptureController
from landing.execution import BodyReference
res=run/'results';s=json.loads((res/'execution_status.json').read_text());_,g,_,profile=load_verified_reference(R)
o=observation(s['decision_observation']);d=deck(s['decision_deck']);curve,_,_=state_selected_braking_plan(g,profile,o,d,s['measured_braking_detection']['measured_acceleration_m_s2'])
rows=[json.loads(l) for l in (res/'execution_feedback.jsonl').read_text().splitlines()];active=[r for r in rows if r.get('attitude_command')];start=s['main_approach_start_sim_t_s']
limits=s['verified_native_climb_limits_m_s'];preview=s['direct_braking_plan']['terminal_coordination']['own_horizontal_preview_s']
c=StopDistanceAttitudeController(curve,start,limits['WP_SPD_UP'],limits['WP_SPD_DN'],rate_feedforward=True,native_rate_mode=True,braking_tail_guard=True,measured_thrust_mapping=True,own_reference_preview_s=preview,horizontal_velocity_gain=4.,vertical_velocity_feedback=.8)
errors=[];demand_errors=[];fresh=[]
def difference(x,y):
 if isinstance(x,dict):return max([difference(v,y[k]) for k,v in x.items()]+[0.]) if isinstance(y,dict) and x.keys()==y.keys() else math.inf
 if isinstance(x,(list,tuple)):return max([difference(a,b) for a,b in zip(x,y)]+[0.]) if isinstance(y,(list,tuple)) and len(x)==len(y) else math.inf
 if isinstance(x,bool) or isinstance(x,str) or x is None:return 0. if x==y else math.inf
 return abs(x-y) if isinstance(y,(float,int)) else math.inf
for r in active:
 expected=c.command(BodyReference(**r['body_reference']),observation(r['observation']),r['sim_t_s']);errors.append(difference(expected,r['attitude_command']))
 q=r['attitude_command']['stopping_governor'];motion=r['observation']['motion'];v=motion['velocity_at_origin_world_m_s'][0];target=curve.terminal_boundary.aircraft.pose.position_world_m[0];distance=target-motion['pose']['position_world_m'][0];usable=max(.02,distance-.2*max(0,v));a=-max(0,v)**2/(2*usable)
 demand_errors.append(max(abs(q['remaining_target_distance_m']-distance),abs(q['usable_stopping_distance_m']-usable),abs(q['raw_forward_demand_m_s2']-a),abs(q['target_body_x_m']-target)))
 fresh.append(0<=r['sim_t_s']-r['observation']['sim_t_s']<=.06 and not q['target_shift_used'] and not q['reserve_is_identified_bound'])
base=textwrap.dedent(inspect.getsource(AttitudeCaptureController.command));hook=textwrap.dedent(inspect.getsource(_NativeHorizontalDemandHook.command)).replace('    ax,governor=self.govern_horizontal(reference,observation,now,ax)\n','').replace("    command['stopping_governor']=governor\n",'')
protocols=[json.loads(p.read_text()) for p in run.parent.glob('experiment_protocol*.json') if any(x.get('index')==run.name.rsplit('_',1)[1] and x.get('package')==pkg.name for x in json.loads(p.read_text()).get('cases',[]))]
manifest=json.loads((res/'attempt_manifest.json').read_text());declared=len(protocols)==1 and protocols[0].get('stopping_governor',{}).get('response_reserve_s')==.2 and datetime.datetime.fromisoformat(protocols[0]['frozen_utc']).timestamp()<=manifest['created_unix']
module=Path(sys.modules[StopDistanceAttitudeController.__module__].__file__).resolve()
checks={'frozen_governor_module_used':module==(pkg/'landing/stopping_governor.py').resolve(),'native_method_exact_except_explicit_horizontal_hook':ast.dump(ast.parse(base))==ast.dump(ast.parse(hook)), 'governor_and_reserve_frozen_before_run':declared,'all_native_commands_replayed_from_actual_inputs':bool(errors) and max(errors)<1e-9,'distance_equations_recomputed_independently':bool(demand_errors) and max(demand_errors)<1e-10,'current_causal_inputs_no_target_shift_no_identified_bound_claim':bool(fresh) and all(fresh)}
result={'passed':all(checks.values()),'checks':checks,'commands_replayed':len(active),'maximum_command_replay_error':max(errors) if errors else None,'maximum_distance_equation_error':max(demand_errors) if demand_errors else None,'response_reserve_s':.2,'response_bound_identified':False,'source_feedback_sha256':hashlib.sha256((res/'execution_feedback.jsonl').read_bytes()).hexdigest()}
target=run/'landing_analysis/governor_verification.json';assert not target.exists();target.write_text(json.dumps(result,indent=2));print(json.dumps(result))
