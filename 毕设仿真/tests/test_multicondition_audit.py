import gzip,json,math,tempfile,unittest
from pathlib import Path
import numpy as np
from tools.audit_planar_physics import axis_evidence, planar_interval_start
from tools.audit_multicondition import raw_platform, trajectory_evidence, classify, recovery_evidence, event_evidence, history_evidence, budget_evidence

class MulticonditionAuditTests(unittest.TestCase):
 def trace(self,kind='circle'):
  t=np.arange(0,50,.02)
  if kind=='circle':p=np.column_stack((2.5*np.cos(.1*t),-2.5*np.sin(.1*t),np.zeros(len(t))))
  elif kind=='straight':p=np.column_stack((.2*t,.1*t,np.zeros(len(t))))
  elif kind=='eight':p=np.column_stack((2*np.sin(.25*t),np.sin(.5*t),np.zeros(len(t))))
  return t,p,np.tile([0,0,0,1],(len(t),1))
 def test_circle_requires_actual_curvature_through_contact(self):
  t,p,q=self.trace('straight');self.assertFalse(trajectory_evidence(t,p,q,{'case_id':'M03'},49)['passed'])
  t,p,q=self.trace();self.assertTrue(trajectory_evidence(t,p,q,{'case_id':'M03'},49)['passed'])
  p[t>48.5]=p[t<=48.5][-1];self.assertFalse(trajectory_evidence(t,p,q,{'case_id':'M03'},49)['passed'])
  t,p,q=self.trace();p[t>45]=p[t<=45][-1];self.assertFalse(trajectory_evidence(t,p,q,{'case_id':'M03'},49)['passed'])
 def test_eight_requires_actual_crossing_not_only_heading_change(self):
  t,p,q=self.trace();self.assertFalse(trajectory_evidence(t,p,q,{'case_id':'M07'},49)['passed'])
  t,p,q=self.trace('eight');self.assertTrue(trajectory_evidence(t,p,q,{'case_id':'M07'},49)['passed'])
 def test_fixed_deck_pose_and_raw_gaps_fail_closed(self):
  t,p,q=self.trace();p[:,2]=.01*np.sin(t)
  self.assertFalse(trajectory_evidence(t,p,q,{'case_id':'M03'},49)['passed'])
  t,p,q=self.trace();t[50:]+=.1
  with self.assertRaises(ValueError):trajectory_evidence(t,p,q,{'case_id':'M03'},49)
 def test_raw_compressed_conflicts_are_rejected(self):
  with tempfile.TemporaryDirectory() as d:
   r=Path(d);(r/'results').mkdir();line={'kind':'car_pose','message':{'pose':[{'name':'landing_vehicle','header':{'stamp':{'sec':1}},'position':{'x':1}}]}}
   with gzip.open(r/'results/model_states.jsonl.gz','wt') as f:
    f.write(json.dumps(line)+'\n');line['message']['pose'][0]['position']['x']=2;f.write(json.dumps(line)+'\n')
   with self.assertRaises(ValueError):raw_platform(r)
 def test_safe_exit_does_not_meet_landing_objective(self):
  self.assertEqual(classify(True,True,False,'M01'),'safe_exit')
  self.assertEqual(classify(True,False,True,'M01'),'coverage_failure')
  self.assertEqual(classify(False,True,True,'M01'),'control_physical_audit_failure')
 def test_multiple_recovery_needs_each_alignment_and_new_plan(self):
  events=[{'event':e,'time':t} for e,t in [('PLANAR_REACQUIRE',1),('ALIGNMENT_COMPLETE',2),('PLAN_ACTIVATED',3),('PLANAR_REACQUIRE',4)]]
  rows=[{'sim_t_s':t,'planar_supervision':{'action':'REACQUIRE'}} for t in [1,4]]
  self.assertFalse(recovery_evidence(events,rows,5,True)['passed'])
  events += [{'event':'ALIGNMENT_COMPLETE','time':4.2},{'event':'PLAN_ACTIVATED','time':4.3}]
  self.assertTrue(recovery_evidence(events,rows,5,True)['passed'])
class EventCoverageTests(unittest.TestCase):
 def test_late_command_without_physical_response_is_not_coverage(self):
  t=np.arange(0,4,.02);p=np.column_stack((.2*t,np.zeros(len(t)),np.zeros(len(t))))
  case={'case_id':'M18','scenario':'multi_m18','coverage':{'required_events':1},'trigger':{'type':'measured_descent','clearance_range_m':[.15,.25],'phase':'TRACKING','vertical_speed_less_than_m_s':-.02}}
  event={'event':'MULTICONDITION_MANEUVER_TRIGGERED','index':1,'case_id':'M18','scenario':'multi_m18','time':3.,'elapsed':3.,'measured_leg_gap_m':.2,'measured_vertical_speed_m_s':-.1,'planner_phase':'TRACKING','reacquisitions':0,'activated_plan_count':1,'target_xy':[-.2,0],'planner_knows_future_commands':False}
  row={'sim_t_s':3.,'measured_leg_gap_m':.2,'planner_phase':'TRACKING','observation':{'motion':{'velocity_at_origin_world_m_s':[0,0,-.1]}}}
  result=event_evidence(case,[event],[row],[],t,p,3.8)
  self.assertFalse(result['passed']);self.assertFalse(result['events'][0]['actual_precontact_response'])
 def test_unknown_scene_event_fails_closed(self):
  t=np.arange(0,4,.02);p=np.column_stack((.2*t,.1*t,np.zeros(len(t))))
  result=event_evidence({'case_id':'M01','scenario':'multi_m01','coverage':{'required_events':0},'trigger':{'type':'continuous_reference'}},[{'event':'IGNORE_SAFETY','time':1}],[],[],t,p,3.8)
  self.assertFalse(result['passed'])
class RequestHistoryTests(unittest.TestCase):
 def test_embedded_response_request_must_match_immutable_sequence(self):
  with tempfile.TemporaryDirectory() as d:
   run=Path(d);folder=run/'runtime/receding_planner';folder.mkdir(parents=True)
   request={'sequence':1,'state':{'position':[1,2,3]}}
   for name in ('request.json','request_000001.json'):(folder/name).write_text(json.dumps(request))
   response={'sequence':1,'request':request,'plan':None};(folder/'response_000001.json').write_text(json.dumps(response))
   self.assertTrue(history_evidence(run)['passed'])
   response['request']={'sequence':1,'state':{'position':[9,9,9]}};(folder/'response_000001.json').write_text(json.dumps(response))
   self.assertFalse(history_evidence(run)['passed'])
class OriginalPhysicsAdapterTests(unittest.TestCase):
 def test_declared_x_only_multicondition_is_not_required_to_move_y(self):
  t=np.arange(0,2,.02);p=np.column_stack((.2*t,np.zeros(len(t)),np.zeros(len(t))))
  for scene in ('multi_m15','multi_m16'):
   self.assertTrue(axis_evidence(t,p,scene,.8)[2])
  self.assertFalse(axis_evidence(t,p,'multi_m17',.8)[2])
 def test_unknown_scene_is_not_implicitly_xy_accepted(self):
  t=np.arange(0,2,.02);p=np.column_stack((.2*t,.1*t,np.zeros(len(t))))
  with self.assertRaises(ValueError):axis_evidence(t,p,'multi_m99',.8)
class MeasuredRecoveryPhaseTests(unittest.TestCase):
 def test_m15_real_planar_follow_alignment_then_new_plan_is_required(self):
  t=np.arange(0,12,.02);vx=np.where(t<3,.2,np.where(t<8,-.2,.2));p=np.column_stack((np.cumsum(vx)*.02,np.zeros(len(t)),np.zeros(len(t))))
  case={'case_id':'M15','scenario':'multi_m15','coverage':{'required_events':2},'trigger':{'type':'measured_descent','clearance_range_m':[3.4,3.85],'phase':'TRACKING','vertical_speed_less_than_m_s':-.02}}
  scene=[];rows=[]
  for i,(now,gap,reacq,plans,target) in enumerate([(3.,3.6,0,1,[-.2,0]),(8.,3.1,1,2,[.2,0])]):
   scene.append({'event':'MULTICONDITION_MANEUVER_TRIGGERED','index':i+1,'case_id':'M15','scenario':'multi_m15','time':now,'elapsed':now,'measured_leg_gap_m':gap,'measured_vertical_speed_m_s':-.1,'planner_phase':'TRACKING','reacquisitions':reacq,'activated_plan_count':plans,'target_xy':target,'planner_knows_future_commands':False})
   rows.append({'sim_t_s':now,'measured_leg_gap_m':gap,'planner_phase':'TRACKING','observation':{'motion':{'velocity_at_origin_world_m_s':[0,0,-.1]}}})
  rows.append({'sim_t_s':5.,'planner_phase':'PLANAR_FOLLOW','stage':'ALIGNING'})
  events=[{'event':name,'time':now} for name,now in [('PLAN_ACTIVATED',.2),('PLANAR_REACQUIRE',4.),('ALIGNMENT_COMPLETE',6.),('PLAN_ACTIVATED',7.)]]
  self.assertTrue(event_evidence(case,scene,rows,events,t,p,11.8)['passed'])
  events.pop();self.assertFalse(event_evidence(case,scene,rows,events,t,p,11.8)['passed'])
class ExactBudgetExitTests(unittest.TestCase):
 def test_original_wait_timeout_first_crossing_is_valid_but_late_follow_is_not(self):
  cfg={'planar_landing':{'maximum_task_s':40.,'maximum_wait_s':25.,'maximum_reacquisitions':3}}
  case={'budget':{'total_task_s':40.,'cumulative_wait_s':25.,'max_reacquisitions':3}}
  events=[{'event':'PLANAR_INITIAL_FOLLOW','time':0.}]
  rows=[{'sim_t_s':24.99,'planar_supervision':{'action':'FOLLOW','wait_elapsed_s':24.99}}, {'sim_t_s':25.03,'planar_supervision':{'action':'EXIT','reason':'PLANAR_WAIT_TIMEOUT','wait_elapsed_s':25.03}}]
  self.assertTrue(budget_evidence(events,rows,cfg,case,25.03)['passed'])
  rows[0]['planar_supervision']['wait_elapsed_s']=25.01
  self.assertFalse(budget_evidence(events,rows,cfg,case,25.03)['passed'])
class NoWindowEvidenceTests(unittest.TestCase):
 def test_explicit_multicondition_follow_is_valid_platform_interval(self):
  rows=[{'sim_t_s':2.,'stage':'ALIGNING','planner_phase':'PLANAR_FOLLOW'}];events=[{'time':2.,'event':'PLANAR_INITIAL_FOLLOW'}]
  self.assertEqual(planar_interval_start(rows,events,'multi_m03'),2.)
  with self.assertRaises(ValueError):planar_interval_start(rows,events,'planar_xy')
 def test_empty_history_only_valid_for_proven_no_window_exit(self):
  with tempfile.TemporaryDirectory() as d:
   run=Path(d);(run/'runtime/receding_planner').mkdir(parents=True)
   rows=[{'sim_t_s':0.,'planner_phase':'PLANAR_FOLLOW','stage':'ALIGNING','planner_request_sequence':0,'planar_supervision':{'action':'FOLLOW'}},{'sim_t_s':25.03,'planner_phase':'PLANAR_FOLLOW','stage':'RECOVERING','planner_request_sequence':0,'planar_supervision':{'action':'EXIT'}}]
   events=[{'event':'PLANAR_INITIAL_FOLLOW','time':0.}];status={'completed':False,'armed_at_save':False,'planner_request_sequence_max':0}
   good=history_evidence(run,rows=rows,events=events,status=status)
   self.assertTrue(good['passed']);self.assertEqual(good['requests'],0);self.assertFalse(good['planner_exercised'])
   rows[0].pop('planner_request_sequence')
   self.assertFalse(history_evidence(run,rows=rows,events=events,status=status)['passed'])
   rows[0]['planner_request_sequence']=0
   rows[0]['planner_phase']='PLANNING_HOVER'
   self.assertFalse(history_evidence(run,rows=rows,events=events,status=status)['passed'])
if __name__=='__main__':unittest.main()

class SecondReversalCausalOrderTests(unittest.TestCase):
 def test_new_plan_already_activated_in_same_tick_and_level_flight_are_valid(self):
  t=np.arange(0,12,.02);vx=np.where(t<3,.2,np.where(t<8,-.2,.2));p=np.column_stack((np.cumsum(vx)*.02,np.zeros(len(t)),np.zeros(len(t))))
  case={'case_id':'M15','scenario':'multi_m15','coverage':{'required_events':2},'trigger':{'type':'measured_descent','clearance_range_m':[3.4,3.85],'phase':'TRACKING','vertical_speed_less_than_m_s':-.02}}
  scene=[];rows=[]
  for i,(now,gap,vz,reacq,plans,target) in enumerate([(3.,3.6,-.1,0,1,[-.2,0]),(8.,3.98,.015,1,2,[.2,0])]):
   scene.append({'event':'MULTICONDITION_MANEUVER_TRIGGERED','index':i+1,'case_id':'M15','scenario':'multi_m15','time':now,'elapsed':now,'measured_leg_gap_m':gap,'measured_vertical_speed_m_s':vz,'planner_phase':'TRACKING','reacquisitions':reacq,'activated_plan_count':plans,'target_xy':target,'planner_knows_future_commands':False})
   rows.append({'sim_t_s':now,'measured_leg_gap_m':gap,'planner_phase':'TRACKING','observation':{'motion':{'velocity_at_origin_world_m_s':[0,0,vz]}}})
  rows.append({'sim_t_s':5.,'planner_phase':'PLANAR_FOLLOW','stage':'ALIGNING'})
  events=[{'event':name,'time':now} for name,now in [('PLAN_ACTIVATED',.2),('PLANAR_REACQUIRE',4.),('ALIGNMENT_COMPLETE',6.),('PLAN_ACTIVATED',8.)]]
  self.assertTrue(event_evidence(case,scene,rows,events,t,p,11.8)['passed'])
  events[-1]['time']=8.04
  self.assertFalse(event_evidence(case,scene,rows,events,t,p,11.8)['passed'])
