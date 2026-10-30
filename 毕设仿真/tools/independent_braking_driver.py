"""Scenario-only wheel command driver; no braking preview is sent to aircraft."""
from pathlib import Path
import json,time,threading,signal,sys
from dataclasses import asdict
from gz.transport13 import Node
from gz.msgs10.clock_pb2 import Clock
from gz.msgs10.twist_pb2 import Twist
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.boolean_pb2 import Boolean
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from landing.contact_geometry import Pose,RigidMotion,Deck,evaluate_airframe
from landing.reference_loader import load_verified_reference

cfg=json.loads((ROOT/'config/experiments/independent_braking_driver.json').read_text())
_,geometry,_,_=load_verified_reference(ROOT)
node=Node();lock=threading.RLock();running=True;clock=None;request=0.;active=False;uav=[];car=[];event=None;last=None;rows=[]
def stop(*args):
 global running
 running=False
signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
def at(p):return p.header.stamp.sec+p.header.stamp.nsec*1e-9
def pose(p):return Pose((p.position.x,p.position.y,p.position.z),(p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w))
def on_clock(m):
 global clock
 with lock:clock=m.sim.sec+m.sim.nsec*1e-9
def on_request(m):
 global request
 with lock:request=m.linear.x
def on_active(m):
 global active
 with lock:active=bool(m.data)
def on_uav(m):
 with lock:
  for p in m.pose:
   if p.name=='uav':uav.append((at(p),pose(p)));uav[:]=uav[-50:]
def on_car(m):
 with lock:
  for p in m.pose:
   if p.name=='landing_vehicle':car.append((at(p),pose(p)));car[:]=car[-50:]
callbacks=[on_clock,on_request,on_active,on_uav,on_car]
for kind,topic,callback in [(Clock,'/world/uav_vehicle_stage1/clock',on_clock),(Twist,'/scenario/cruise_request',on_request),(Boolean,'/scenario/normal_descent_started',on_active),(Pose_V,'/model/uav/pose',on_uav),(Pose_V,'/model/landing_vehicle/pose',on_car)]:assert node.subscribe(kind,topic,callback)
publisher=node.advertise('/vehicle/cmd_vel',Twist)
stream=(ROOT/'results/scenario_driver_commands.jsonl').open('x')
try:
 while running:
  with lock:
   t=clock
   if t is None or last is not None and t-last<.019999:pass
   else:
    checks=None
    if event is None and active and len(uav)>1 and len(car)>1:
     ut,up=uav[-1];cu,cp=car[-1];pt,pp=uav[-2];bt,bp=car[-2]
     if ut>pt and cu>bt and 0<=t-ut<.06 and 0<=t-cu<.06:
      vz=(up.position_world_m[2]-pp.position_world_m[2])/(ut-pt)
      vx=(cp.position_world_m[0]-bp.position_world_m[0])/(cu-bt)
      deck=Deck(RigidMotion.stationary(Pose(cp.transform((0,0,.8)),cp.orientation_xyzw)),(1,.7))
      feet=evaluate_airframe(geometry,RigidMotion.stationary(up),deck);gap=feet[0].clearance_m
      checks={'nominal_descent_active':active,'car_cruising':abs(vx-5)<.03,'aircraft_descending':vz<-.2,'gap_crossed':gap<=cfg['trigger_foot_gap_m']}
      if all(checks.values()):
       event={'command_sim_t_s':t,'aircraft_pose_sim_t_s':ut,'car_pose_sim_t_s':cu,'car_speed_m_s':vx,'aircraft_vertical_speed_m_s':vz,'lowest_foot_gap_m':gap,'checks':checks,'aircraft_pose':asdict(up),'car_pose':asdict(cp),'cruise_request_at_trigger_m_s':request}
       (ROOT/'results/scenario_brake_event.json').write_text(json.dumps(event,indent=2))
    command=0. if event is not None else request
    msg=Twist();msg.linear.x=command;publisher.publish(msg);last=t
    row={'publish_sim_t_s':t,'aircraft_cruise_request_m_s':request,'actual_vehicle_command_m_s':command,'normal_descent_active':active,'brake_latched':event is not None}
    stream.write(json.dumps(row)+'\n');stream.flush()
  time.sleep(.003)
finally:
 stream.close()
 (ROOT/'results/scenario_driver_summary.json').write_text(json.dumps({'scenario_only':True,'aircraft_receives_brake_preview':False,'config':cfg,'brake_event':event},indent=2))
