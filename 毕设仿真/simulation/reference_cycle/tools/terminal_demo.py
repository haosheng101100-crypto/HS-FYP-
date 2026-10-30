"""Static platform, simultaneous descent and horizontal braking in Gazebo/SITL."""
import argparse,json,math,sys,time
from pathlib import Path
from types import SimpleNamespace

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from cooperation_demo import Cooperation,yaw,wrap
from first_stage.terminal_trajectory import select_approach
from gz.msgs10.twist_pb2 import Twist
from pymavlink import mavutil

class TerminalDemo(Cooperation):
    def __init__(self,args):
        super().__init__(args)
        self.plan=None;self.plan_start=None;self.plan_info={};self.reference=None
        self.hold=None;self.origin_ned=None;self.touched_since=None
        self.first_touch=None;self.previous_feedback=None;self.feedback=[]
        self.observation_warmup_start=None
        self.result.update(controller='Sample-constrained quintic + ArduCopter position/velocity/acceleration tracking',method=args.method,
            task='static-terminal-braking',simulation_only=True,platform_command_speed=0.,
            contact_target={'horizontal_speed_m_s':0.,'descent_speed_m_s':.08,'level_attitude':True},
            caveat='Ground truth observations; sampled feasibility; no claim of time optimality or paper novelty')

    def velocity_target(self,t,dt,poses,contact):
        actual=self.controller.uav.at(t);vel=self.controller.uav.velocity();deck=self.controller.vehicle.at(t)
        if actual is None or deck is None:
            # Different pose rates can put the first common timestamp before
            # one history starts. Send a bounded hold while history fills.
            if self.plan is None:
                if self.observation_warmup_start is None:self.observation_warmup_start=t
                if 0 <= t-self.observation_warmup_start <= .5:
                    return (0.,0.,0.),{'state':'OBSERVATION_WARMUP','valid':False}
            raise RuntimeError('Position history unavailable beyond bounded startup warmup')
        error=math.hypot(actual[0]-deck[0],actual[1]-deck[1]);height=actual[2]-deck[2]
        if self.args.method=='sequential' and self.phase=='LAND_DECK':
            return super().velocity_target(t,dt,poses,contact)
        if self.plan is None:
            self.controller.state='PREPARE'
            v=tuple(max(-.5,min(.5,1.1*(a-b))) for a,b in zip(self.hold,actual)) if self.hold else (0,0,0)
            return v,{'state':'PREPARE','valid':True,'xy_error_est':error,'height_est':height,'relative_speed_est':math.hypot(*vel[:2])}
        tau=t-self.plan_start
        p,v,a,j=self.plan.at(tau)
        if tau>self.plan.T:
            # Terminal continuation gently closes residual tracking error at
            # the same deck position. Its duration is measured, never hidden.
            v=(0.,0.,-.05);a=(0.,0.,0.)
        self.reference=(p,v,a)
        self.phase='BRAKE_DESCEND' if tau>=self.plan_info['planned_braking_start_s'] else 'APPROACH_DESCEND'
        self.controller.state=self.phase
        tilt=0.
        if 'ATTITUDE' in self.telemetry:
            att=self.telemetry['ATTITUDE'][0];tilt=max(abs(att.roll),abs(att.pitch))*180/math.pi
        recent=-.04<=t-self.contact_sim_stamp<=.12
        entry={'sim_t':t,'elapsed':tau,'actual_foot':actual,'actual_velocity':vel,'contact':recent,'xy_error':error,'height':height,'tilt_deg':tilt,
               'reference_position':p,'reference_velocity':v,'reference_acceleration':a,'phase':self.phase}
        if self.previous_feedback and t>self.previous_feedback['sim_t']:
            sample_dt=t-self.previous_feedback['sim_t']
            entry['interval_velocity']=tuple((a-b)/sample_dt for a,b in zip(actual,self.previous_feedback['actual_foot']))
            entry['interval_dt']=sample_dt
        self.feedback.append(entry)
        if recent and self.first_touch is None:
            self.first_touch={'contact_sample':entry,'precontact_sample':self.previous_feedback}
            self.result['first_contact']=self.first_touch
            self.event(f'First physical contact: xy error {error:.4f} m; preceding velocity {self.previous_feedback["actual_velocity"] if self.previous_feedback else None}')
        self.previous_feedback=entry
        if recent and error<.18 and abs(height)<.06 and math.hypot(*vel[:2])<.16 and abs(vel[2])<.12 and tilt<8:
            if self.touched_since is None:self.touched_since=t
            if t-self.touched_since>=.4:
                self.controller.transition('LANDED',t);self.phase='CONTACT_SETTLED'
        else:self.touched_since=None
        if (height<.08 and error>.4) or height<-.08 or tilt>25:
            raise RuntimeError('Approach envelope violated; inspect saved flight data')
        if tau>self.plan.T+8:raise RuntimeError('Terminal correction exceeded 8 s; no valid contact')
        return v,{'state':self.controller.state,'valid':True,'xy_error_est':error,'height_est':height,'relative_speed_est':math.hypot(*vel[:2]),
            'trajectory_elapsed':tau,'trajectory_duration':self.plan.T,'reference_x':p[0],'reference_y':p[1],'reference_z':p[2],
            'reference_vx':v[0],'reference_vy':v[1],'reference_vz':v[2],'reference_ax':a[0],'reference_ay':a[1],'reference_az':a[2],
            'planned_tilt_deg':math.degrees(math.atan2(math.hypot(*a[:2]),9.80665+a[2])),
            'observed_vx':vel[0],'observed_vy':vel[1],'observed_vz':vel[2],'contact_recent_sim':recent}

    def send_control(self,v,yaw_enu,t,dt):
        if self.reference is None:return super().send_control(v,yaw_enu,t,dt)
        p,v,a=self.reference
        # The trajectory is a landing-foot reference, MAVLink targets body origin.
        body=(p[0],p[1],p[2]+.195)
        ned=(body[1]+self.origin_ned[0],body[0]+self.origin_ned[1],-body[2]+self.origin_ned[2])
        self.m.mav.set_position_target_local_ned_send(int(t*1000)&0xffffffff,1,1,mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            2048,*ned,v[1],v[0],-v[2],a[1],a[0],-a[2],wrap(math.pi/2-yaw_enu),0.)

    def run(self):
        self.wait(lambda:'HEARTBEAT' in self.telemetry,60,'flight controller')
        for mid in (30,32,33,245):self.command(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,mid,50000)
        self.wait(lambda:len(self.poses)==2 and 'LOCAL_POSITION_NED' in self.telemetry,180,'model poses and estimator')
        self.phase='READY';self.event('Static deck; a single timed trajectory will brake while descending')
        self.sim_wait(12,False)
        self.mode('GUIDED');self.wait(lambda:self.telemetry['HEARTBEAT'][0].custom_mode==4,30,'GUIDED')
        self.phase='ARM'
        for attempt in range(8):
            self.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,1)
            try:self.wait(self.armed,12,'normal arming');break
            except RuntimeError:
                if attempt==7 or not self.running:raise
        poses,_=self.snapshot();u=poses['uav']['p'];fcu=self.telemetry['LOCAL_POSITION_NED'][0]
        self.origin_ned=(fcu.x-u[1],fcu.y-u[0],fcu.z+u[2])
        self.result['initial_world_to_ned_offset']=self.origin_ned
        self.phase='TAKEOFF';self.event('Take off at launch point; no alignment or hover over platform')
        self.command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,0,0,0,0,0,0,3.5)
        self.wait(lambda:self.poses['uav']['p'][2]>3.1,180,'actual takeoff')
        self.hold=(u[0],u[1],3.1);self.phase='PREPARE';self.sim_wait(2,True)
        t=self.sim_time();p=self.controller.uav.at(t);vel=self.controller.uav.velocity();deck=self.controller.vehicle.at(t)
        self.plan_start=t;self.result['approach_start_sim_t']=t
        if self.args.method=='sequential':
            # Matched speed limits, same launch preparation and normal FCU.
            # Only the "align first, then descend" strategy is changed.
            self.controller.state='TRACK';self.controller.cfg.hover_height=p[2]-deck[2]
            self.controller.cfg.max_xy_speed=2.;self.controller.cfg.max_z_speed=.85
            self.controller.cfg.descent_speed=.85;self.controller.cfg.align_error=.12
            self.controller.cfg.align_speed=.12;self.controller.cfg.stable_time=.5
            self.phase='LAND_DECK'
            self.result['baseline_parameters']=vars(self.controller.cfg)
            self.event('Matched-limit sequential baseline: align over deck, then descend')
        else:
            self.plan,self.plan_info=select_approach(p,vel,deck)
            (ROOT/'results/trajectory_plan.json').write_text(json.dumps(self.plan_info,indent=2))
            self.result['plan']=self.plan_info
            self.event(f'Selected {self.plan.T:.2f} s approach; start braking at {self.plan_info["planned_braking_start_s"]:.2f} s while descending')
        self.wait(lambda:self.controller.state=='LANDED',180,'synchronized terminal contact',True)
        self.result['contact_settled_sim_t']=self.sim_time()
        self.phase='LAND_CONFIRM';self.mode('LAND');self.reference=None
        self.wait(lambda:not self.armed(),180,'normal automatic disarm',False)
        self.phase='COMPLETE';self.result['completed']=True
        self.event('Diagonal approach and braking complete; normal disarm confirmed')
        self.sim_wait(3,False)

    def save(self):
        super().save()
        (ROOT/'results/trajectory_feedback.json').write_text(json.dumps(self.feedback))

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--method',choices=['terminal','sequential'],default='terminal')
    args=parser.parse_args();args.task='static-terminal-braking';args.car_speed=0.;args.drive_seconds=0.
    d=TerminalDemo(args)
    try:d.run()
    except Exception as exc:
        d.result['error']=str(exc);d.event(str(exc));d.phase='STOPPED'
        if d.armed():d.mode('LAND')
        raise
    finally:
        d.vehicle_speed=0.;d.pub.publish(Twist());d.save();d.m.close()

if __name__=='__main__':main()
