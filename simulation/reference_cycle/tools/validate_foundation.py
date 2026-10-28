"""Actual same-world foundation acceptance: wheel motion, flight, hover, landing.

No UAV pose-setting or vehicle position teleportation is used. This is not yet
the vehicle-following controller. Gazebo truth and FCU estimates are logged apart.
"""
import csv
import json
import math
import os
from pathlib import Path
import sys
import threading
import time

os.environ.setdefault('GZ_PARTITION','hs_ardupilot_stage1')
os.environ.setdefault('MAVLINK20','1')
sys.path.append('/usr/lib/python3/dist-packages')
from gz.transport13 import Node
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.twist_pb2 import Twist
from pymavlink import mavutil

ROOT=Path(__file__).resolve().parents[1]

class Foundation:
    def __init__(self):
        self.node=Node();self.lock=threading.Lock();self.truth={};self.state={};self.rows=[]
        self.pub=self.node.advertise('/vehicle/cmd_vel',Twist)
        for name,topic in [('vehicle','/model/landing_vehicle/pose'),('uav','/model/uav/pose')]:
            def callback(msg,name=name):
                with self.lock:
                    for pose in msg.pose:
                        if pose.name in ('landing_vehicle','uav'):
                            self.truth[name]={'x':pose.position.x,'y':pose.position.y,'z':pose.position.z,
                                'qw':pose.orientation.w,'qx':pose.orientation.x,'qy':pose.orientation.y,'qz':pose.orientation.z,
                                'sim_t':msg.header.stamp.sec+msg.header.stamp.nsec*1e-9,'wall_t':time.monotonic()}
            assert self.node.subscribe(Pose_V,topic,callback),topic
        self.m=mavutil.mavlink_connection('tcp:127.0.0.1:5760',source_system=255,source_component=190)
        self.phase='CONNECT';self.last_heartbeat=0;self.last_log=0;self.last_vehicle=0;self.vehicle_speed=0
        self.summary={'passed':False,'scope':'foundation only; flight and wheel motion, not cooperative following or deck landing','events':[]}

    def event(self,text):
        self.summary['events'].append(text);print(text,flush=True)

    def command(self,cmd,*params):
        values=list(params)+[0]*7
        self.m.mav.command_long_send(1,1,cmd,0,*values[:7])

    def mode(self,name):
        self.m.set_mode(self.m.mode_mapping()[name])

    def pump(self):
        now=time.monotonic()
        if now-self.last_heartbeat>=1:
            self.m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,mavutil.mavlink.MAV_AUTOPILOT_INVALID,0,0,0)
            self.m.mav.rc_channels_override_send(1,1,1500,1500,1000,1500,1800,1500,1500,1500)
            self.last_heartbeat=now
        if now-self.last_vehicle>=0.1:
            msg=Twist();msg.linear.x=self.vehicle_speed;self.pub.publish(msg);self.last_vehicle=now
        for _ in range(200):
            msg=self.m.recv_match(blocking=False)
            if msg is None:break
            kind=msg.get_type()
            if kind=='BAD_DATA':continue
            self.state[kind]=(msg,now)
            if kind=='STATUSTEXT':self.event('FCU: '+msg.text)
        if now-self.last_log>=0.1:
            row={'wall_t':now,'phase':self.phase,'vehicle_cmd_mps':self.vehicle_speed}
            with self.lock:
                truth={k:dict(v) for k,v in self.truth.items()}
            for name,d in truth.items():
                for key,val in d.items():row[name+'_'+key]=val
            for kind,keys in [('LOCAL_POSITION_NED',['x','y','z','vx','vy','vz']),('ATTITUDE',['roll','pitch','yaw','rollspeed','pitchspeed','yawspeed'])]:
                if kind in self.state:
                    msg,stamp=self.state[kind]
                    row[kind.lower()+'_age']=now-stamp
                    for key in keys:row['fcu_'+key]=getattr(msg,key)
            if 'HEARTBEAT' in self.state:
                hb=self.state['HEARTBEAT'][0];row['armed']=bool(hb.base_mode&128);row['custom_mode']=hb.custom_mode
            if 'EXTENDED_SYS_STATE' in self.state:row['landed_state']=self.state['EXTENDED_SYS_STATE'][0].landed_state
            self.rows.append(row);self.last_log=now
        time.sleep(0.02)

    def wait(self,predicate,timeout,label):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            self.pump()
            if predicate():return
        raise RuntimeError('Timed out: '+label)

    def state_fresh(self,kind):
        return kind in self.state and time.monotonic()-self.state[kind][1]<1.5

    def truth_fresh(self):
        with self.lock:return len(self.truth)==2 and all(time.monotonic()-d['wall_t']<1 for d in self.truth.values())

    def armed(self):
        return self.state_fresh('HEARTBEAT') and bool(self.state['HEARTBEAT'][0].base_mode&128)

    def run(self):
        self.wait(lambda:self.state_fresh('HEARTBEAT'),30,'heartbeat')
        self.event('FCU heartbeat connected')
        for msg_id in [30,32,33,245,1]:
            self.command(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,msg_id,100000)
        self.command(mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,148)
        self.wait(lambda:self.truth_fresh() and self.state_fresh('LOCAL_POSITION_NED'),90,'both world poses and FCU local position')
        self.phase='READY'
        self.event('Same world: UAV and wheel-driven vehicle poses available')
        # Let estimator finish startup before requesting a normal (unforced) arm.
        end=time.monotonic()+12
        while time.monotonic()<end:self.pump()
        self.mode('GUIDED')
        self.wait(lambda:self.state['HEARTBEAT'][0].custom_mode==4,10,'GUIDED')
        self.phase='ARM'
        for attempt in range(10):
            self.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,1)
            try:self.wait(self.armed,6,'normal arming');break
            except RuntimeError:
                if attempt==9:raise
        self.event('Normal arming confirmed')
        self.phase='TAKEOFF'
        self.command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,0,0,0,0,0,0,4)
        self.wait(lambda:self.state_fresh('LOCAL_POSITION_NED') and -self.state['LOCAL_POSITION_NED'][0].z>=3.5,60,'takeoff to 4 m')
        self.event('Takeoff confirmed')
        self.phase='HOVER_AND_VEHICLE'
        with self.lock:start=dict(self.truth['vehicle']);start_sim=self.truth['uav']['sim_t']
        self.vehicle_speed=0.5
        self.wait(lambda:self.truth_fresh() and self.truth['uav']['sim_t']-start_sim>=12,50,'12 simulation seconds of hovering and vehicle drive')
        self.vehicle_speed=0
        self.phase='STOP_VEHICLE'
        with self.lock:stop_sim=self.truth['uav']['sim_t']
        self.wait(lambda:self.truth_fresh() and self.truth['uav']['sim_t']-stop_sim>=3,15,'vehicle stopping')
        with self.lock:finish=dict(self.truth['vehicle'])
        distance=math.hypot(finish['x']-start['x'],finish['y']-start['y'])
        self.summary['vehicle']={'start':start,'finish':finish,'displacement_m':distance,'command_mps':0.5,'command_sim_seconds':12}
        hover=[r for r in self.rows if r['phase']=='HOVER_AND_VEHICLE' and 'fcu_roll' in r and r.get('local_position_ned_age',99)<1]
        assert hover,'No hover telemetry'
        altitude=[-r['fcu_z'] for r in hover];tilt=[math.degrees(max(abs(r['fcu_roll']),abs(r['fcu_pitch']))) for r in hover]
        xyz=[(r['uav_x'],r['uav_y'],r['uav_z']) for r in hover if 'uav_x' in r]
        self.summary['hover']={'samples':len(hover),'altitude_min_m':min(altitude),'altitude_max_m':max(altitude),'max_abs_roll_pitch_deg':max(tilt),'truth_height_min_m':min(p[2] for p in xyz),'truth_height_max_m':max(p[2] for p in xyz)}
        self.event(f'Vehicle physically moved {distance:.3f} m while UAV hovered')
        self.phase='LAND_GROUND'
        self.mode('LAND')
        self.wait(lambda:self.state_fresh('HEARTBEAT') and not self.armed(),90,'ground landing and actual disarm')
        self.wait(lambda:self.state_fresh('EXTENDED_SYS_STATE') and self.state['EXTENDED_SYS_STATE'][0].landed_state==mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,10,'landed state on ground')
        self.event('Ground landing and actual disarm confirmed')
        self.summary['checks']={'both_models_live':self.truth_fresh(),'normal_arm':True,'takeoff':True,'vehicle_moved':distance>2,'hover_height':min(altitude)>3.4 and max(altitude)<4.6,'hover_tilt':max(tilt)<10,'flight_truth':min(p[2] for p in xyz)>3.4,'ground_landed':True,'disarmed':not self.armed()}
        self.summary['passed']=all(self.summary['checks'].values())
        self.phase='COMPLETE';self.pump()

    def save(self):
        result=ROOT/'results';result.mkdir(exist_ok=True)
        if self.rows:
            keys=sorted(set().union(*(r.keys() for r in self.rows)))
            with (result/'foundation.csv').open('w',newline='') as f:
                writer=csv.DictWriter(f,fieldnames=keys);writer.writeheader();writer.writerows(self.rows)
        (result/'foundation_summary.json').write_text(json.dumps(self.summary,indent=2),encoding='utf-8')
        print(json.dumps(self.summary,indent=2),flush=True)

if __name__=='__main__':
    app=Foundation()
    try:app.run()
    except Exception as exc:
        app.summary['error']=str(exc);app.event('FAILED: '+str(exc));app.vehicle_speed=0
        if app.armed():app.mode('LAND')
        for _ in range(50):app.pump()
    finally:app.save();app.m.close()
    sys.exit(0 if app.summary['passed'] else 1)
