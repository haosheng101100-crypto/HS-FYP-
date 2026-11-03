"""World-only velocity references; never import into prediction or landing policy.

Every returned target must pass through the original PlanarMotionCommand and
bounded-force joint controller. This module never changes pose, safety limits,
or controller state. Geometry formulas describe the *reference*, not physics.
"""
import copy
import math


def _smooth(x):
    x=max(0.,min(1.,x))
    return x*x*(3.-2.*x)


def _polar(speed,degrees):
    angle=math.radians(degrees)
    return speed*math.cos(angle),speed*math.sin(angle)


class ScenarioProgram:
    def __init__(self,case):
        self.case=copy.deepcopy(case)
        self.kind=case['kind'];self.p=self.case['parameters']
        if case.get('scenario')!=f"multi_m{int(case['case_id'][1:]):02d}" or not 1<=int(case['case_id'][1:])<=20:
            raise ValueError('Unknown multicondition case identity')
        self.last=None;self.fired=0;self.initial_reacquisitions=None;self.first_plan=None
        self.realigned=False;self.emitted=set()

    def _event(self,elapsed,m,target,reason):
        return dict(event='MULTICONDITION_MANEUVER_TRIGGERED',index=self.fired,
                    case_id=self.case['case_id'],scenario=self.case['scenario'],
                    time=float(m['sim_t_s']),elapsed=float(elapsed),
                    measured_leg_gap_m=float(m['clearance_m']),
                    measured_vertical_speed_m_s=float(m['vertical_speed_m_s']),
                    planner_phase=m['phase'],reacquisitions=int(m['reacquisitions']),
                    activated_plan_count=int(m['activated_plan_count']),
                    target_xy=list(target),trigger_reason=reason,
                    planner_knows_future_commands=False)

    def step(self,elapsed,measured):
        m=measured
        nums=(elapsed,m['sim_t_s'],m['clearance_m'],m['vertical_speed_m_s'],m['reacquisitions'],m['activated_plan_count'])
        if not all(math.isfinite(x) for x in nums) or elapsed<0 or (self.last is not None and elapsed<=self.last):
            raise ValueError('Finite increasing world elapsed clock and measured state required')
        self.last=elapsed;p=self.p;k=self.kind;events=[]
        terminal=m['phase'] in ('LAND','COMPLETE','ABORTED','FINISHED') or m.get('contact_detected',False) or m.get('contact_complete',False)
        if k in ('height_reversal','double_reversal'):
            trigger=self.case['trigger'];low,high=trigger['clearance_range_m']
            if self.fired and k=='double_reversal' and m['reacquisitions']>self.initial_reacquisitions and m['phase'] in ('PLANAR_FOLLOW','ALIGNING'):self.realigned=True
            reason=None
            if not terminal and self.fired==0 and m['phase']=='TRACKING' and low<=m['clearance_m']<=high and m['vertical_speed_m_s']<trigger['vertical_speed_less_than_m_s']:
                reason='measured_descent';self.first_plan=m['activated_plan_count'];self.initial_reacquisitions=m['reacquisitions']
            elif not terminal and k=='double_reversal' and self.fired==1 and self.realigned and m['phase']=='TRACKING' and m['reacquisitions']>self.initial_reacquisitions and m['activated_plan_count']>self.first_plan:
                reason='realigned_with_new_activated_plan'
            if reason:self.fired+=1
            initial=tuple(p['initial_target_xy']);changed=tuple(p.get('changed_target_xy',[-v for v in initial]))
            target=changed if self.fired%2 else initial
            if reason:events.append(self._event(elapsed,m,target,reason))
            return target,events
        if k=='constant':target=_polar(p['speed_m_s'],p['angle_deg'])
        elif k=='circle':
            w=p['angular_speed_rad_s'];theta=w*elapsed+p['phase_rad'];r=p['radius_m']
            target=(-r*w*math.sin(theta),r*w*math.cos(theta))
        elif k=='ellipse':
            a,b=p['semi_axes_m'];w=p['angular_speed_rad_s'];theta=w*elapsed+p['phase_rad']
            target=(-a*w*math.sin(theta),b*w*math.cos(theta))
        elif k=='s_curve':target=(p['forward_speed_m_s'],p['lateral_amplitude_m']*p['angular_speed_rad_s']*math.cos(p['angular_speed_rad_s']*elapsed+p['phase_rad']))
        elif k=='figure_eight':
            a,b=p['amplitudes_m'];w=p['angular_speed_rad_s'];px,py=p['phase_rad']
            target=(a*w*math.cos(w*elapsed+px),2*b*w*math.cos(2*w*elapsed+py))
        elif k=='ramp':
            blend=_smooth((elapsed-p['ramp_start_s'])/(p['ramp_end_s']-p['ramp_start_s']))
            target=_polar(p['speed_start_m_s']+(p['speed_end_m_s']-p['speed_start_m_s'])*blend,p['angle_deg'])
        elif k=='asynchronous':target=(.27+.12*math.sin(.22*elapsed),.12+.09*math.sin(.31*elapsed+.6))
        elif k=='stop':target=_polar(p['speed_m_s']*(1-_smooth((elapsed-p['stop_start_s'])/(p['stop_end_s']-p['stop_start_s']))),p['angle_deg'])
        elif k=='restart':
            if elapsed<p['stop_end_s']:target=_polar(p['speed_m_s']*(1-_smooth((elapsed-p['stop_start_s'])/(p['stop_end_s']-p['stop_start_s']))),p['initial_angle_deg'])
            else:target=_polar(p['speed_m_s']*_smooth((elapsed-p['restart_start_s'])/(p['restart_end_s']-p['restart_start_s'])),p['final_angle_deg'])
        elif k=='turn':target=_polar(p['speed_m_s'],p['initial_angle_deg']+p['turn_deg']*_smooth((elapsed-p['turn_start_s'])/(p['turn_end_s']-p['turn_start_s'])))
        elif k in ('two_turns','irregular_tail','irregular_continuous'):
            times=p['switch_times_s'];index=sum(elapsed>=t for t in times) if k=='two_turns' else max(0,sum(elapsed>=t for t in times)-1)
            target=tuple(p['targets_xy'][index])
        else:raise ValueError('Unknown world reference kind: '+str(k))
        # Scheduled maneuvers document only commands already sent; no future is
        # returned. All missed boundaries are recorded at the actual issue time.
        boundaries=p.get('switch_times_s',[])
        if k.startswith('irregular'):boundaries=boundaries[1:]
        if k=='restart':boundaries=[p['stop_start_s'],p['restart_start_s']]
        if k=='stop':boundaries=[p['stop_start_s']]
        if k=='turn':boundaries=[p['turn_start_s']]
        for i,boundary in enumerate(boundaries):
            if elapsed>=boundary and i not in self.emitted:
                self.emitted.add(i)
                if not terminal:
                    self.fired+=1;event=self._event(elapsed,m,target,'scheduled_motion_change');event['scheduled_elapsed_s']=boundary;events.append(event)
        return tuple(float(v) for v in target),events
