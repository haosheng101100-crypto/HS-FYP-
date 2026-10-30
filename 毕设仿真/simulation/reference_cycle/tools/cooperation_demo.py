"""Run the first-stage controller against actual Gazebo physics and ArduCopter.

Gazebo truth supplies initial-stage observations. Flight is driven by MAVLink
velocity commands; the vehicle is driven through DiffDrive wheel joints.
"""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import signal
import sys
import threading
import time

os.environ.setdefault('MAVLINK20','1')
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from first_stage.core import Config, Controller, Sample
from first_stage.integration import rotate_quaternion
from gz.transport13 import Node
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.contacts_pb2 import Contacts
from gz.msgs10.twist_pb2 import Twist
from pymavlink import mavutil

def yaw(q):
    x,y,z,w=q
    return math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))

def wrap(angle):
    return math.atan2(math.sin(angle),math.cos(angle))

class Cooperation:
    def __init__(self,args):
        self.args=args;self.node=Node();self.lock=threading.Lock();self.poses={}
        self.contact_sim_stamp=-math.inf;self.previous_observation=None;self.contact_stamp=-math.inf;self.telemetry={};self.running=True
        self.phase='CONNECT';self.vehicle_speed=0.;self.last_hb=0.;self.last_car=0.
        self.last_control=None;self.last_log=-math.inf;self.last_print=-math.inf
        self.command_enu=(0.,0.,0.);self.info={};self.events=[];self.rows=[]
        self.result={'task':args.task,'completed':False,'observations':'Gazebo ground truth; no observation noise or delay yet'}
        self.controller=Controller(Config(feedforward=True,landing_enabled=False,require_contact=True,
                                         hover_height=2.5,max_xy_speed=1.5,descent_speed=.18,
                                         align_error=.20,align_speed=.22,stable_time=2.))
        self.pub=self.node.advertise('/vehicle/cmd_vel',Twist)
        self.callbacks=[]
        for name in ('uav','landing_vehicle'):
            def callback(msg,name=name):
                with self.lock:
                    for p in msg.pose:
                        if p.name==name:
                            self.poses[name]={'t':p.header.stamp.sec+p.header.stamp.nsec*1e-9,
                                'p':(p.position.x,p.position.y,p.position.z),
                                'q':(p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w),
                                'received':time.monotonic()}
            self.callbacks.append(callback)
            if not self.node.subscribe(Pose_V,'/model/'+name+'/pose',callback):
                raise RuntimeError('Could not subscribe to '+name)
        def contacts(msg):
            # Require actual contact between the roof and this UAV.
            if any('uav' in c.collision1.name or 'uav' in c.collision2.name for c in msg.contact):
                with self.lock:
                    self.contact_stamp=time.monotonic()
                    self.contact_sim_stamp=msg.header.stamp.sec+msg.header.stamp.nsec*1e-9
        self.callbacks.append(contacts)
        self.node.subscribe(Contacts,'/vehicle/deck/contact',contacts)
        self.node.subscribe(Contacts,'/world/uav_vehicle_stage1/model/landing_vehicle/link/chassis/sensor/deck_contact/contact',contacts)
        deadline=time.monotonic()+90
        while True:
            try:
                self.m=mavutil.mavlink_connection('tcp:127.0.0.1:5760',source_system=255,source_component=190)
                break
            except ConnectionRefusedError:
                if time.monotonic()>deadline:raise
                time.sleep(1)
        signal.signal(signal.SIGTERM,self.stop)
        signal.signal(signal.SIGINT,self.stop)

    def stop(self,*_):self.running=False
    def event(self,message):
        self.events.append({'wall_time':time.time(),'phase':self.phase,'message':message})
        print(f'[{self.phase}] {message}',flush=True)
    def command(self,cmd,*params):
        self.m.mav.command_long_send(1,1,cmd,0,*(list(params)+[0]*7)[:7])
    def mode(self,name):self.m.set_mode(self.m.mode_mapping()[name])
    def armed(self):
        hb=self.telemetry.get('HEARTBEAT')
        return bool(hb and hb[0].base_mode&128)
    def snapshot(self):
        with self.lock:return {k:dict(v) for k,v in self.poses.items()},time.monotonic()-self.contact_stamp<.4

    def send_velocity(self,v,yaw_enu):
        # Official Iris uses world ENU -> flight-controller NED: N=y,E=x,D=-z.
        self.m.mav.set_position_target_local_ned_send(
            int(time.monotonic()*1000)&0xffffffff,1,1,mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            2503,0,0,0,v[1],v[0],-v[2],0,0,0,wrap(math.pi/2-yaw_enu),0)

    def velocity_target(self,sim_t,dt,poses,contact):
        return self.controller.step(sim_t,min(dt,.2),contact)

    def send_control(self,v,yaw_enu,sim_t,dt):
        self.send_velocity(v,yaw_enu)

    def pump(self,control=False):
        now=time.monotonic()
        if now-self.last_hb>=1:
            self.m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,mavutil.mavlink.MAV_AUTOPILOT_INVALID,0,0,0)
            self.m.mav.rc_channels_override_send(1,1,1500,1500,1000,1500,1800,1500,1500,1500)
            self.last_hb=now
        if now-self.last_car>=.1:
            msg=Twist();msg.linear.x=self.vehicle_speed;self.pub.publish(msg);self.last_car=now
        for _ in range(100):
            msg=self.m.recv_match(blocking=False)
            if msg is None:break
            kind=msg.get_type()
            if kind=='BAD_DATA':continue
            self.telemetry[kind]=(msg,now)
            if kind=='STATUSTEXT':self.event('ArduCopter: '+msg.text)
            if kind=='COMMAND_ACK':self.event(f'Command {msg.command}, result {msg.result}')
        poses,contact=self.snapshot()
        if len(poses)==2:
            sim_t=min(d['t'] for d in poses.values())
            if control and (self.last_control is None or sim_t-self.last_control>=.04):
                dt=.04 if self.last_control is None else sim_t-self.last_control
                self.last_control=sim_t
                for name,d in poses.items():
                    # Use the lowest landing-foot reference, not the body origin.
                    offset=(0,0,-.195) if name=='uav' else (0,0,.8)
                    local=rotate_quaternion(offset,d['q'])
                    position=tuple(a+b for a,b in zip(d['p'],local))
                    history=self.controller.uav if name=='uav' else self.controller.vehicle
                    history.push(Sample(d['t'],position))
                attitude=self.telemetry.get('ATTITUDE')
                heading_error=abs(wrap(yaw(poses['uav']['q'])-yaw(poses['landing_vehicle']['q'])))
                attitude_ready=bool(attitude and now-attitude[1]<1.5 and
                    max(abs(attitude[0].roll),abs(attitude[0].pitch))<math.radians(12) and heading_error<math.radians(15))
                self.controller.cfg.landing_enabled=self.phase=='LAND_DECK' and attitude_ready
                if self.controller.state=='DESCEND' and not attitude_ready:
                    self.controller.transition('TRACK',sim_t)
                if any(now-d['received']>1.5 for d in poses.values()):
                    self.command_enu=(0.,0.,0.);self.info={'state':'HOLD','valid':False}
                else:
                    self.command_enu,self.info=self.velocity_target(sim_t,dt,poses,contact)
                self.info.update(contact=contact,heading_error_deg=math.degrees(heading_error),attitude_ready=attitude_ready)
                self.send_control(self.command_enu,yaw(poses['landing_vehicle']['q']),sim_t,dt)
            if sim_t-self.last_log>=.1:
                row={'sim_t':sim_t,'phase':self.phase,'controller':self.controller.state,'armed':self.armed(),
                     'car_command':self.vehicle_speed,**self.info}
                for name,d in poses.items():
                    for axis,val in zip('xyz',d['p']):row[name+'_'+axis]=val
                    row[name+'_yaw']=yaw(d['q'])
                for axis,val in zip('xyz',self.command_enu):row['command_'+axis]=val
                for kind,fields in [('LOCAL_POSITION_NED',('x','y','z','vx','vy','vz')),('ATTITUDE',('roll','pitch','yaw'))]:
                    if kind in self.telemetry:
                        for field in fields:row['fcu_'+field]=getattr(self.telemetry[kind][0],field)
                # Independent observations keep updating after controller state LANDED.
                u=poses['uav'];car=poses['landing_vehicle']
                off=rotate_quaternion((0,0,-.195),u['q'])
                foot=tuple(a+b for a,b in zip(u['p'],off))
                rel=tuple(a-b for a,b in zip(u['p'],car['p']))
                cy=math.cos(yaw(car['q']));sy=math.sin(yaw(car['q']))
                row.update(observed_contact=contact,
                    contact_age_sim=sim_t-self.contact_sim_stamp,
                    observed_contact_recent=-.04<=sim_t-self.contact_sim_stamp<=.12,
                    relative_x_body=cy*rel[0]+sy*rel[1],
                    relative_y_body=-sy*rel[0]+cy*rel[1],
                    observed_xy_error=math.hypot(rel[0],rel[1]),
                    observed_foot_height=foot[2]-car['p'][2]-.8,
                    observed_heading_error_deg=math.degrees(abs(wrap(yaw(u['q'])-yaw(car['q'])))))
                x,y,z,w=u['q']
                row['observed_roll_deg']=math.degrees(math.atan2(2*(w*x+y*z),1-2*(x*x+y*y)))
                row['observed_pitch_deg']=math.degrees(math.asin(max(-1,min(1,2*(w*y-z*x)))))
                previous=self.previous_observation
                if previous and sim_t>previous[0]:
                    dt=sim_t-previous[0]
                    uv=tuple((a-b)/dt for a,b in zip(u['p'],previous[1]))
                    cv=tuple((a-b)/dt for a,b in zip(car['p'],previous[2]))
                    row['observed_car_speed']=math.hypot(cv[0],cv[1])
                    row['observed_relative_speed']=math.hypot(uv[0]-cv[0],uv[1]-cv[1])
                self.previous_observation=(sim_t,u['p'],car['p'])
                self.rows.append(row);self.last_log=sim_t
                (ROOT/'results/progress.json').write_text(json.dumps(row))
            if control and sim_t-self.last_print>=3:
                self.event(f"{self.controller.state}: error={self.info.get('xy_error_est',float('nan')):.2f}m, relative speed={self.info.get('relative_speed_est',float('nan')):.2f}m/s, height={self.info.get('height_est',float('nan')):.2f}m")
                self.last_print=sim_t
        time.sleep(.01)

    def wait(self,predicate,timeout,label,control=False):
        end=time.monotonic()+timeout
        while self.running and time.monotonic()<end:
            self.pump(control)
            if predicate():return
        raise RuntimeError('Interrupted or timed out: '+label)
    def sim_time(self):
        p,_=self.snapshot()
        return min((d['t'] for d in p.values()),default=0.)
    def sim_wait(self,seconds,control=True):
        start=self.sim_time();self.wait(lambda:self.sim_time()-start>=seconds,max(180,seconds*25),'simulation advance',control)

    def run(self):
        self.wait(lambda:'HEARTBEAT' in self.telemetry,60,'flight-controller connection')
        for mid in (30,32,33,245):self.command(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,mid,100000)
        self.wait(lambda:len(self.poses)==2 and 'LOCAL_POSITION_NED' in self.telemetry,180,'both model poses and position estimator')
        self.phase='READY';self.event('Official Iris and wheel-driven vehicle connected; first-stage controller imported')
        self.sim_wait(12,False)
        self.mode('GUIDED')
        self.wait(lambda:self.telemetry['HEARTBEAT'][0].custom_mode==4,30,'GUIDED mode')
        self.phase='ARM'
        for attempt in range(8):
            self.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,1)
            try:self.wait(self.armed,12,'normal arming');break
            except RuntimeError:
                if attempt==7 or not self.running:raise
        self.phase='TAKEOFF';self.event('Normal arming confirmed; taking off')
        self.command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,0,0,0,0,0,0,3.5)
        self.wait(lambda:self.poses.get('uav',{}).get('p',(0,0,0))[2]>3.1,180,'actual takeoff')
        self.phase='ALIGN';self.event('Tracking the stationary roof using first-stage PID')
        self.wait(lambda:self.info.get('xy_error_est',99)<.25 and self.info.get('relative_speed_est',99)<.25,240,'roof alignment',True)
        self.sim_wait(3)
        if self.args.task!='stationary-land':
            self.phase='FOLLOW';self.vehicle_speed=self.args.car_speed
            self.event(f'Vehicle driving at {self.vehicle_speed} m/s; UAV follows its measured motion')
            self.sim_wait(self.args.drive_seconds)
        if self.args.task=='moving-land':
            self.phase='LAND_DECK'
            self.event('Continue wheel motion while first-stage landing state machine descends')
        else:
            self.vehicle_speed=0.;self.phase='STOP_VEHICLE';self.sim_wait(4)
            self.phase='LAND_DECK';self.event('Vehicle stopped; descending onto roof with contact feedback')
        self.wait(lambda:self.controller.state=='LANDED',420,'physical roof contact',True)
        self.phase='LAND_CONFIRM'
        if self.args.task!='moving-land':self.vehicle_speed=0.
        self.event('First-stage landing state and roof contact reached; ArduCopter LAND handles motor shutdown')
        self.mode('LAND')
        self.wait(lambda:not self.armed(),180,'normal automatic disarm after landing')
        if self.args.task=='moving-land':
            self.phase='POSTCONTACT'
            self.event('Motors disarmed; keep platform moving for 5 simulation seconds to measure passive support')
            self.sim_wait(5,False)
            self.result['postcontact_drive_seconds']=5.
        self.vehicle_speed=0.
        self.phase='COMPLETE';self.result['completed']=True
        self.event('Scene demonstration completed; drone rests on vehicle roof')
        self.sim_wait(2,False)

    def save(self):
        result=ROOT/'results';result.mkdir(exist_ok=True)
        if self.rows:
            keys=sorted(set().union(*(r.keys() for r in self.rows)))
            with (result/'cooperation.csv').open('w',newline='') as f:
                w=csv.DictWriter(f,keys);w.writeheader();w.writerows(self.rows)
        self.result.update(final_phase=self.phase,events=self.events,transitions=self.controller.transitions)
        (result/'cooperation_status.json').write_text(json.dumps(self.result,indent=2,ensure_ascii=False))

def main():
    p=argparse.ArgumentParser()
    p.add_argument('--task',choices=['follow','stationary-land','moving-land'],default='follow')
    p.add_argument('--car-speed',type=float,default=.35)
    p.add_argument('--drive-seconds',type=float,default=12.)
    a=p.parse_args()
    if not 0<a.car_speed<=1 or not 0<a.drive_seconds<=60:p.error('car speed must be (0,1] and drive seconds (0,60]')
    d=Cooperation(a)
    try:d.run()
    except Exception as e:
        d.result['error']=str(e);d.event(str(e));d.phase='STOPPED'
        if d.armed():d.mode('LAND')
        raise
    finally:
        d.vehicle_speed=0.;msg=Twist();d.pub.publish(msg)
        d.save();d.m.close()

if __name__=='__main__':main()
