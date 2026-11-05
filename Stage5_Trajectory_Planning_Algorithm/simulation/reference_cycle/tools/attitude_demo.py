"""Gazebo-only flight through our angle PID and ArduCopter's rate inner loop."""
import argparse
import csv
from dataclasses import asdict
import json
import math
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from cooperation_demo import Cooperation,yaw,wrap
from control.attitude_pid import AttitudeCascade
from first_stage.core import PID
from gz.msgs10.twist_pb2 import Twist
from pymavlink import mavutil


class AttitudeDemo(Cooperation):
    def __init__(self,args):
        super().__init__(args)
        self.cascade=None;self.previous_guid_options=None;self.origin=None
        self.yaw_target=0.;self.takeoff_start=0.;self.pid_cycles=0
        self.hold_xy=[PID(1.0,.05,.08,.6) for _ in range(2)]
        self.controller.cfg.max_xy_speed=.8
        self.result.update(controller='Own attitude-angle PID + velocity PI + original position PID',
            inner_loop='ArduCopter body-rate control and motor mixer',
            interface='SET_ATTITUDE_TARGET mask=128, GUID_OPTIONS bit 3 enabled',
            simulation_only=True)

    def param(self,name):
        self.m.mav.param_request_read_send(1,1,name.encode(),-1)
        self.wait(lambda:self.telemetry.get('PARAM_VALUE') is not None and
                  self.telemetry['PARAM_VALUE'][0].param_id==name,10,'parameter '+name)
        return self.telemetry['PARAM_VALUE'][0].param_value

    def velocity_target(self,t,dt,poses,contact):
        if self.phase in ('TAKEOFF','HOVER','YAW_DEMO','YAW_RETURN'):
            actual=self.controller.uav.at(t)
            if actual is None:raise RuntimeError('No fresh position for takeoff control')
            z_ref=self.origin[2]+3.
            if self.phase=='TAKEOFF':z_ref=min(z_ref,self.origin[2]+.45*(t-self.takeoff_start))
            xy=tuple(pid.step(self.origin[k]-actual[k],min(dt,.2)) for k,pid in enumerate(self.hold_xy))
            vz=max(-.5,min(.5,1.1*(z_ref-actual[2])))
            return (*xy,vz),{'state':self.phase,'valid':True,'height_est':actual[2]-self.origin[2],
                'height_reference':z_ref-self.origin[2],
                'xy_error_est':math.hypot(self.origin[0]-actual[0],self.origin[1]-actual[1]),
                'relative_speed_est':math.hypot(*self.controller.uav.velocity()[:2])}
        self.yaw_target=wrap(math.pi/2-yaw(poses['landing_vehicle']['q']))
        return super().velocity_target(t,dt,poses,contact)

    def send_control(self,v,yaw_enu,t,dt):
        if not self.cascade or not self.armed():return
        now=time.monotonic()
        if dt>.2 or not self.info.get('valid'):
            raise RuntimeError('Flight observation or simulation interval expired')
        for kind in ('ATTITUDE','LOCAL_POSITION_NED'):
            if kind not in self.telemetry or now-self.telemetry[kind][1]>1:
                raise RuntimeError('Expired flight feedback: '+kind)
        a=self.telemetry['ATTITUDE'][0];position=self.telemetry['LOCAL_POSITION_NED'][0]
        body,thrust,info=self.cascade.step(v,(position.vx,position.vy,position.vz),
            (a.roll,a.pitch,a.yaw),(a.rollspeed,a.pitchspeed,a.yawspeed),self.yaw_target,dt)
        self.m.mav.set_attitude_target_send(int(t*1000)&0xffffffff,1,1,128,[1.,0.,0.,0.],*body,thrust)
        self.info.update(info);self.pid_cycles+=1

    def run(self):
        self.wait(lambda:'HEARTBEAT' in self.telemetry,60,'flight-controller connection')
        self.wait(lambda:len(self.poses)==2,90,'Gazebo UAV and vehicle')
        # A hard-coded loopback SITL connection and this simulator parameter exclude a hardware backend.
        self.param('SIM_SPEEDUP')
        self.previous_guid_options=int(self.param('GUID_OPTIONS'))
        hover=self.param('MOT_THST_HOVER')
        self.cascade=AttitudeCascade(hover)
        self.result.update(hover_thrust=hover,gains=asdict(self.cascade.g))
        self.m.mav.param_set_send(1,1,b'GUID_OPTIONS',float(self.previous_guid_options|8),mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        self.wait(lambda:self.telemetry.get('PARAM_VALUE') is not None and
            self.telemetry['PARAM_VALUE'][0].param_id=='GUID_OPTIONS' and
            int(self.telemetry['PARAM_VALUE'][0].param_value)==self.previous_guid_options|8,10,'pure-thrust option')
        for mid,interval in ((30,20000),(32,20000),(245,100000),(83,20000)):
            self.command(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,mid,interval)
        self.wait(lambda:'ATTITUDE' in self.telemetry and 'LOCAL_POSITION_NED' in self.telemetry,90,'flight feedback')
        self.phase='READY';self.event('Own attitude-angle PID connected through body-rate targets; simulator only')
        self.sim_wait(12,False)
        self.mode('GUIDED');self.wait(lambda:self.telemetry['HEARTBEAT'][0].custom_mode==4,30,'GUIDED mode')
        self.phase='ARM'
        for attempt in range(8):
            self.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,1)
            try:self.wait(self.armed,12,'normal arming');break
            except RuntimeError:
                if attempt==7 or not self.running:raise
        poses,_=self.snapshot();self.origin=poses['uav']['p']
        # The base Gazebo pose is the body origin; the control reference is the landing feet.
        self.origin=(self.origin[0],self.origin[1],self.origin[2]-.195)
        self.yaw_target=self.telemetry['ATTITUDE'][0].yaw;initial_yaw=self.yaw_target
        self.cascade.reset();self.phase='TAKEOFF';self.takeoff_start=self.sim_time()
        self.event('Taking off through own velocity/attitude PID and pure thrust; no NAV_TAKEOFF command')
        self.wait(lambda:self.info.get('height_est',0)>2.85,180,'PID controlled actual takeoff',True)
        self.phase='HOVER';self.event('Own PID holds hover height and attitude');self.sim_wait(3)
        self.phase='YAW_DEMO';self.yaw_target=wrap(initial_yaw+math.radians(20))
        self.event('Own yaw PID follows a 20 degree heading target');self.sim_wait(4)
        self.phase='YAW_RETURN';self.yaw_target=initial_yaw;self.sim_wait(4)
        self.phase='ALIGN';self.info={}
        self.event('First-stage position PID feeds our velocity and attitude PID')
        self.wait(lambda:self.info.get('xy_error_est',99)<.20 and self.info.get('relative_speed_est',99)<.22,240,'roof alignment',True)
        self.sim_wait(3)
        if self.args.task!='stationary-land':
            self.phase='FOLLOW';self.vehicle_speed=self.args.car_speed;self.event('Following wheel-driven vehicle through own attitude PID')
            self.sim_wait(self.args.drive_seconds)
        if self.args.task!='moving-land':
            self.vehicle_speed=0.;self.phase='STOP_VEHICLE';self.sim_wait(4)
        self.phase='LAND_DECK';self.event('Own PID controls descent until physical roof contact')
        self.wait(lambda:self.controller.state=='LANDED',420,'PID controlled roof contact',True)
        self.phase='LAND_CONFIRM';self.vehicle_speed=0.;self.mode('LAND')
        self.event('Roof contact confirmed; standard LAND completes motor shutdown')
        self.wait(lambda:not self.armed(),180,'normal automatic disarm')
        self.phase='COMPLETE';self.result['completed']=True;self.event('Own attitude PID flight completed')
        self.sim_wait(2,False)

    def save(self):
        result=ROOT/'results';result.mkdir(exist_ok=True)
        if self.rows:
            keys=sorted(set().union(*(r.keys() for r in self.rows)))
            with (result/'attitude_demo.csv').open('w',newline='') as f:
                w=csv.DictWriter(f,keys);w.writeheader();w.writerows(self.rows)
        self.result.update(final_phase=self.phase,events=self.events,pid_cycles=self.pid_cycles,
            transitions=self.controller.transitions)
        (result/'attitude_demo_status.json').write_text(json.dumps(self.result,indent=2,ensure_ascii=False))


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--task',choices=['follow','stationary-land','moving-land'],default='follow')
    p.add_argument('--car-speed',type=float,default=.25)
    p.add_argument('--drive-seconds',type=float,default=10.)
    a=p.parse_args()
    if not 0<a.car_speed<=.5 or not 0<a.drive_seconds<=60:p.error('Invalid demonstration speed or duration')
    d=AttitudeDemo(a)
    try:d.run()
    except Exception as e:
        d.result['error']=str(e);d.event(str(e));d.phase='STOPPED'
        if d.armed():d.mode('LAND')
        raise
    finally:
        d.vehicle_speed=0.;d.pub.publish(Twist())
        # Preserve the simulator's original option after flight has stopped.
        if d.previous_guid_options is not None and not d.armed():
            d.m.mav.param_set_send(1,1,b'GUID_OPTIONS',float(d.previous_guid_options),mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        d.save();d.m.close()


if __name__=='__main__':main()
