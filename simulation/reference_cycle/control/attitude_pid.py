"""Own attitude-angle PID and velocity PI; no transport or simulator dependencies.

Feedback/commands: FRD body axes, NED world velocities, radians and seconds.
The output goes to ArduCopter's body-rate inner loop, not to raw motor outputs.
"""
from dataclasses import dataclass
import math


def bound(value,limit):
    return max(-limit,min(limit,value))


def wrap(angle):
    return math.atan2(math.sin(angle),math.cos(angle))


@dataclass
class Gains:
    angle_kp: float=4.0
    angle_ki: float=.10
    angle_kd: float=.08
    yaw_kp: float=2.5
    yaw_ki: float=.06
    yaw_kd: float=.05
    velocity_kp: float=2.0
    velocity_ki: float=.30
    vertical_kp: float=2.0
    vertical_ki: float=.80
    max_tilt_deg: float=15.
    max_rate_deg: float=60.
    max_yaw_rate_deg: float=40.
    max_horizontal_accel: float=2.0
    max_vertical_accel: float=2.5
    min_thrust: float=.12
    max_thrust: float=.65


class FeedbackPID:
    """D on filtered measured rate avoids a derivative kick on target changes."""
    def __init__(self,kp,ki,kd,limit,integral_limit):
        self.kp,self.ki,self.kd=kp,ki,kd
        self.limit,self.integral_limit=limit,integral_limit
        self.reset()

    def reset(self):
        self.integral=0.;self.filtered_rate=0.;self.terms={}

    def step(self,error,dt,measured_rate=0.):
        if not all(math.isfinite(v) for v in (error,dt,measured_rate)) or not 0<dt<=.2:
            raise ValueError('Invalid feedback or simulation control interval')
        self.filtered_rate+=dt/(.06+dt)*(measured_rate-self.filtered_rate)
        candidate=bound(self.integral+error*dt,self.integral_limit)
        p=self.kp*error;d=-self.kd*self.filtered_rate
        raw=p+self.ki*candidate+d
        if abs(raw)<=self.limit or raw*error<0:
            self.integral=candidate
        i=self.ki*self.integral
        self.terms={'p':p,'i':i,'d':d,'error':error}
        return bound(p+i+d,self.limit)


class AttitudeCascade:
    def __init__(self,hover_thrust=.39,gains=None):
        self.g=gains or Gains();self.hover=hover_thrust
        if not .1<=hover_thrust<=.6:raise ValueError('Invalid simulated hover thrust')
        g=self.g
        self.velocity=[FeedbackPID(g.velocity_kp,g.velocity_ki,0,g.max_horizontal_accel,2.) for _ in range(2)]
        self.vertical=FeedbackPID(g.vertical_kp,g.vertical_ki,0,g.max_vertical_accel,3.)
        self.angles=[FeedbackPID(g.angle_kp,g.angle_ki,g.angle_kd,math.radians(g.max_rate_deg),.5) for _ in range(2)]
        self.angles.append(FeedbackPID(g.yaw_kp,g.yaw_ki,g.yaw_kd,math.radians(g.max_yaw_rate_deg),.5))

    def reset(self):
        for p in (*self.velocity,self.vertical,*self.angles):p.reset()

    def step(self,velocity_enu,measured_velocity_ned,attitude,body_rates,yaw_target,dt):
        values=(*velocity_enu,*measured_velocity_ned,*attitude,*body_rates,yaw_target)
        if not all(math.isfinite(v) for v in values):raise ValueError('Non-finite flight state')
        roll,pitch,heading=attitude;p,q,r=body_rates;g=self.g
        if max(abs(roll),abs(pitch))>math.radians(30):raise ValueError('Attitude exceeds simulated operating envelope')
        # NED is (north,east,down); upward acceleration is positive here.
        desired=(velocity_enu[1],velocity_enu[0],-velocity_enu[2])
        a_n,a_e=[pid.step(desired[k]-measured_velocity_ned[k],dt) for k,pid in enumerate(self.velocity)]
        horizontal=math.hypot(a_n,a_e)
        factor=min(1.,g.max_horizontal_accel/max(horizontal,1e-9))
        a_n*=factor;a_e*=factor
        a_up=self.vertical.step(measured_velocity_ned[2]-desired[2],dt)
        forward=a_n*math.cos(heading)+a_e*math.sin(heading)
        right=-a_n*math.sin(heading)+a_e*math.cos(heading)
        vertical=9.80665+a_up
        target_pitch=math.atan2(-forward,vertical)
        target_roll=math.atan2(right*math.cos(target_pitch),vertical)
        tilt=math.hypot(target_roll,target_pitch);limit=math.radians(g.max_tilt_deg)
        factor=min(1.,limit/max(tilt,1e-9))
        target=(target_roll*factor,target_pitch*factor,wrap(yaw_target))
        # Body gyro rates -> Euler derivatives; Euler PID output -> body rates.
        sr,cr=math.sin(roll),math.cos(roll);cp=math.cos(pitch)
        euler_rates=(p+(q*sr+r*cr)*math.tan(pitch),q*cr-r*sr,(q*sr+r*cr)/cp)
        commands=[pid.step(wrap(goal-actual),dt,rate) for pid,goal,actual,rate in zip(self.angles,target,attitude,euler_rates)]
        rd,pd,yd=commands
        body=(rd-yd*math.sin(pitch),pd*cr+yd*sr*cp,-pd*sr+yd*cr*cp)
        body=tuple(bound(v,math.radians(g.max_rate_deg)) for v in body)
        # Guided pure-thrust mode retains ArduCopter's existing angle boost.
        thrust=max(g.min_thrust,min(g.max_thrust,self.hover*(1+a_up/9.80665)))
        info={'thrust':thrust,'accel_north':a_n,'accel_east':a_e,'accel_up':a_up,
              'set_attitude_target_mask':128}
        for k,(name,pid) in enumerate(zip(('roll','pitch','yaw'),self.angles)):
            info['target_'+name+'_deg']=math.degrees(target[k])
            info['actual_'+name+'_deg']=math.degrees(attitude[k])
            info['rate_'+name]=body[k]
            info[name+'_error_deg']=math.degrees(pid.terms['error'])
            for term in ('p','i','d'):info[name+'_'+term]=pid.terms[term]
        return body,thrust,info
