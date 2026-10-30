"""Sample-constrained quintic terminal approach; ENU landing-foot coordinates.

Shortest feasible candidate duration on a 50 ms grid, not a proof of global
time optimality. Position, velocity and acceleration share one terminal time.
"""
from dataclasses import dataclass,asdict
import math

@dataclass(frozen=True)
class Limits:
    xy_speed:float=2.0
    z_speed:float=.85
    xy_acceleration:float=1.6
    z_acceleration:float=1.6
    jerk:float=5.0
    tilt_deg:float=15.
    terminal_descent:float=.08
    penetration:float=.015

class Quintic:
    def __init__(self,p0,v0,a0,p1,v1,a1,duration):
        if duration<=0:raise ValueError('Positive duration required')
        self.T=duration;self.c=[]
        for i in range(3):
            c0=p0[i];c1=v0[i]*duration;c2=.5*a0[i]*duration**2
            b0=p1[i]-c0-c1-c2;b1=v1[i]*duration-c1-2*c2;b2=a1[i]*duration**2-2*c2
            self.c.append((c0,c1,c2,10*b0-4*b1+.5*b2,-15*b0+7*b1-b2,6*b0-3*b1+.5*b2))
    def at(self,t):
        s=max(0.,min(1.,t/self.T));out=[]
        for order in range(4):
            values=[]
            for c in self.c:
                values.append(sum(c[k]*math.factorial(k)/math.factorial(k-order)*s**(k-order) for k in range(order,6))/self.T**order)
            out.append(tuple(values))
        return tuple(out)

def select_approach(p0,v0,deck,limits=None):
    lim=limits or Limits()
    target=(deck[0],deck[1],deck[2]-lim.penetration)
    rejected=0
    for tick in range(40,401):
        T=tick*.05
        q=Quintic(p0,v0,(0,0,0),target,(0,0,-lim.terminal_descent),(0,0,0),T)
        extrema={'xy_speed':0.,'z_speed':0.,'xy_acceleration':0.,'z_acceleration':0.,'jerk':0.,'tilt_deg':0.}
        valid=True
        for n in range(301):
            p,v,a,j=q.at(T*n/300)
            upward=9.80665+a[2]
            values={'xy_speed':math.hypot(*v[:2]),'z_speed':abs(v[2]),'xy_acceleration':math.hypot(*a[:2]),
                    'z_acceleration':abs(a[2]),'jerk':math.sqrt(sum(x*x for x in j)),
                    'tilt_deg':math.degrees(math.atan2(math.hypot(*a[:2]),upward))}
            for k,value in values.items():extrema[k]=max(extrema[k],value)
            # Stay above the deck until the landing footprint fits over it.
            clear=abs(p[0]-deck[0])<=.73 and abs(p[1]-deck[1])<=.43
            if upward<3 or any(value>getattr(lim,k)+1e-9 for k,value in values.items()) or (not clear and p[2]<deck[2]+.15):
                valid=False;break
        if not valid:rejected+=1;continue
        # A tilted front foot can contact before the centre-foot reference.
        # Require the flatness proxy to be small throughout the last 8 cm.
        near=[q.at(T*n/300) for n in range(301) if q.at(T*n/300)[0][2]-deck[2]<=.08]
        if any(math.hypot(*v[:2])>.28 or math.degrees(math.atan2(math.hypot(*a[:2]),9.80665+a[2]))>4 for p,v,a,j in near):
            rejected+=1;continue
        peak=max(range(301),key=lambda n:math.hypot(*q.at(T*n/300)[1][:2]))
        brake_t=T*peak/300
        braking=q.at(brake_t)
        return q,{'duration_s':T,'limits':asdict(lim),'sampled_extrema':extrema,'samples_per_candidate':301,
                  'candidate_step_s':.05,'rejected_shorter_candidates':rejected,
                  'initial_foot_position':p0,'initial_velocity':v0,'target_foot_position':target,
                  'terminal_velocity':(0,0,-lim.terminal_descent),'terminal_acceleration':(0,0,0),
                  'planned_braking_start_s':brake_t,'planned_braking_start_position':braking[0],
                  'planned_braking_start_xy_speed':math.hypot(*braking[1][:2]),
                  'method':'Quintic boundary-value trajectory with sampled constraints and duration search'}
    raise RuntimeError('No feasible approach duration up to 20 s')
