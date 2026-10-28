"""Planar point-mass approach references with physical foot contact boundaries.

Seventh-order COM position polynomials match p/v/a/jerk at both ends; the
ninth-order variant additionally matches endpoint snap. Pitch
and pitch rate follow thrust direction and its derivative, rather than being
an independently animated attitude. This is not full rotor/flight-controller
dynamics. The first version supports a stationary level deck aligned with
world x/y and roll=yaw=0; unsupported cases are rejected explicitly.
"""
from dataclasses import dataclass,asdict
import math
from .contact_geometry import Pose,RigidMotion,vector,add,sub,scale,cross,norm,evaluate_airframe,quaternion_from_rpy


class SepticScalar:
    def __init__(self,initial,terminal,duration_s):
        if len(initial)!=4 or len(terminal)!=4 or not all(math.isfinite(x) for x in (*initial,*terminal,duration_s)) or duration_s<=0:
            raise ValueError('Finite p/v/a/jerk endpoints and positive duration required')
        self.T=duration_s;self.initial=tuple(initial);self.terminal=tuple(terminal)
        p,v,a,j=initial;T=duration_s
        c0,c1,c2,c3=p,v*T,a*T*T/2,j*T**3/6
        b0=terminal[0]-c0-c1-c2-c3
        b1=terminal[1]*T-c1-2*c2-3*c3
        b2=terminal[2]*T*T-2*c2-6*c3
        b3=terminal[3]*T**3-6*c3
        self.coefficients=(c0,c1,c2,c3,35*b0-15*b1+2.5*b2-b3/6,
                           -84*b0+39*b1-7*b2+b3/2,70*b0-34*b1+6.5*b2-b3/2,
                           -20*b0+10*b1-2*b2+b3/6)

    def at(self,t,order=0):
        if not math.isfinite(t) or not isinstance(order,int) or not 0<=order<len(self.coefficients):
            raise ValueError('Finite time and derivative order within polynomial degree required')
        if t < -1e-12 or t > self.T+1e-12:
            raise ValueError('Reference ends at first planned contact; no hidden continuation')
        t=min(self.T,max(0.,t));s=t/self.T
        # Exact endpoint data avoids cancellation of normalized coefficients.
        if order<=3 and t==0:return self.initial[order]
        if order<=3 and t==self.T:return self.terminal[order]
        return sum(self.coefficients[k]*math.factorial(k)/math.factorial(k-order)*s**(k-order)
                   for k in range(order,len(self.coefficients)))/self.T**order


class NinthScalar(SepticScalar):
    """Match p/v/a/jerk/snap via an endpoint-vanishing degree-9 correction."""
    def __init__(self,initial,terminal,duration_s):
        if len(initial)!=5 or len(terminal)!=5 or not all(math.isfinite(x) for x in (*initial,*terminal)):
            raise ValueError('Finite p/v/a/jerk/snap endpoints required')
        super().__init__(initial[:4],terminal[:4],duration_s)
        first_snap=self.at(0,4);last_snap=self.at(duration_s,4)
        A=(initial[4]-first_snap)*duration_s**4/24
        B=(terminal[4]-last_snap)*duration_s**4/24-A
        coefficients=list(self.coefficients)+[0.,0.]
        for k,delta in enumerate([A,-4*A+B,6*A-4*B,-4*A+6*B,A-4*B,B],start=4):
            coefficients[k]+=delta
        self.coefficients=tuple(coefficients)
        self.first_snap=initial[4];self.last_snap=terminal[4]

    def at(self,t,order=0):
        if order==4 and t==0 and hasattr(self,'first_snap'):return self.first_snap
        if order==4 and t==self.T and hasattr(self,'last_snap'):return self.last_snap
        return super().at(t,order)


@dataclass(frozen=True)
class ApproachDesignLimits:
    max_horizontal_speed_m_s:float
    max_vertical_speed_m_s:float
    max_horizontal_acceleration_m_s2:float
    max_vertical_acceleration_m_s2:float
    max_jerk_m_s3:float
    max_snap_m_s4:float
    max_tilt_rad:float
    max_pitch_rate_rad_s:float
    max_pitch_acceleration_rad_s2:float
    minimum_upward_specific_force_m_s2:float
    edge_height_margin_m:float
    footprint_margin_m:float
    near_contact_height_m:float
    near_contact_tangential_speed_m_s:float
    near_contact_closing_speed_m_s:float
    near_contact_tilt_rad:float
    max_low_horizontal_speed_lead_s:float

    def __post_init__(self):
        if any(not math.isfinite(x) or x<=0 for x in asdict(self).values()):
            raise ValueError('All approach design bounds must be positive and finite')


class PlanarSepticApproach:
    def __init__(self,initial_com_position_m,initial_com_velocity_m_s,terminal_boundary,
                 com_offset_body_m,gravity_m_s2,mass_kg,duration_s):
        if any(not math.isfinite(v) or v<=0 for v in (gravity_m_s2,mass_kg)):
            raise ValueError('Gravity and simulation mass must be explicit and positive')
        self.com_offset_body_m=vector(com_offset_body_m)
        self.gravity=gravity_m_s2;self.mass=mass_kg;self.T=duration_s
        self.terminal_boundary=terminal_boundary
        initial_position=vector(initial_com_position_m);initial_velocity=vector(initial_com_velocity_m_s)
        end=terminal_boundary.aircraft
        if norm(end.angular_velocity_world_rad_s)>1e-10:
            raise ValueError('Version 1 requires zero terminal angular velocity')
        body_y=end.pose.rotate((0,1,0));body_z=end.pose.rotate((0,0,1))
        if norm(sub(body_y,(0,1,0)))>1e-9 or body_z[2]<=0:
            raise ValueError('Version 1 requires roll=yaw=0 and upright thrust')
        theta=-math.atan2(body_z[0],body_z[2])
        terminal_acceleration=(-gravity_m_s2*math.tan(theta),0.,0.)
        terminal_jerk=(0.,0.,0.)
        end_com=end.pose.transform(self.com_offset_body_m)
        end_com_velocity=end.velocity_at_world_point(end_com)
        if abs(initial_position[1]-end_com[1])>1e-9 or max(abs(initial_velocity[1]),abs(end_com_velocity[1]))>1e-9:
            raise ValueError('Version 1 is planar: constant COM y and zero y velocity required')
        self.axes=tuple(SepticScalar((initial_position[k],initial_velocity[k],0.,0.),
                                    (end_com[k],end_com_velocity[k],terminal_acceleration[k],terminal_jerk[k]),duration_s) for k in range(3))

    def at(self,t):
        p,v,a,j,snap=(tuple(axis.at(t,order) for axis in self.axes) for order in range(5))
        ax,az=a[0],a[2]+self.gravity
        jx,jz=j[0],j[2];sx,sz=snap[0],snap[2]
        denominator=ax*ax+az*az
        if denominator<1e-10 or az<=0:raise ValueError('Invalid point-mass thrust direction')
        theta=-math.atan2(ax,az)
        numerator=az*jx-ax*jz
        theta_rate=-numerator/denominator
        theta_acceleration=-((az*sx-ax*sz)*denominator-numerator*2*(ax*jx+az*jz))/denominator**2
        q=quaternion_from_rpy(0,-theta,0)
        com_pose=Pose(p,q)
        omega=(0.,-theta_rate,0.);alpha=(0.,-theta_acceleration,0.)
        body=RigidMotion.from_com(com_pose,v,omega,self.com_offset_body_m)
        lever=com_pose.rotate(self.com_offset_body_m)
        body_acceleration=sub(a,add(cross(alpha,lever),cross(omega,cross(omega,lever))))
        return {'motion':body,'com_position_world_m':p,'com_velocity_world_m_s':v,
                'com_acceleration_world_m_s2':a,'com_jerk_world_m_s3':j,'com_snap_world_m_s4':snap,
                'body_acceleration_world_m_s2':body_acceleration,
                'nose_up_rad':theta,'nose_up_rate_rad_s':theta_rate,'nose_up_acceleration_rad_s2':theta_acceleration,
                'point_mass_required_thrust_N':self.mass*math.sqrt(denominator)}


class PlanarSnapMatchedApproach(PlanarSepticApproach):
    """Degree-9 variant: preserve contact boundary and tune terminal COM snap."""
    def __init__(self,*args,terminal_com_snap_world_m_s4,**kwargs):
        super().__init__(*args,**kwargs)
        terminal_snap=vector(terminal_com_snap_world_m_s4)
        if abs(terminal_snap[1])>1e-12:raise ValueError('Planar reference requires zero y snap')
        self.axes=tuple(NinthScalar((*axis.initial,0.),(*axis.terminal,terminal_snap[k]),self.T) for k,axis in enumerate(self.axes))
        self.terminal_com_snap_world_m_s4=terminal_snap


def check_height_band_entries(approach,geometry,deck,limits,sample_count=401):
    """Refine sampled height-band crossings and check both sides.

    This closes a conditional-constraint gap at detected band entries. A grid
    can still miss unbracketed crossings; this is not continuous certification.
    """
    if not isinstance(sample_count,int) or sample_count<3:raise ValueError('At least 3 samples required')
    def feet_at(t):
        return {f.name:f for f in evaluate_airframe(geometry,approach.at(t)['motion'],deck)}
    events=[];previous_t=0.;previous=feet_at(0.)
    for tick in range(1,sample_count):
        t=approach.T*tick/(sample_count-1);current=feet_at(t)
        for name in previous:
            for height in [limits.near_contact_height_m,limits.edge_height_margin_m]:
                if (previous[name].clearance_m-height)*(current[name].clearance_m-height)>0:continue
                lo=previous_t;hi=t;flo=previous[name].clearance_m-height
                for _ in range(40):
                    mid=(lo+hi)/2;fm=feet_at(mid)[name].clearance_m-height
                    if flo*fm<=0:hi=mid
                    else:lo=mid;flo=fm
                event_time=(lo+hi)/2
                for probe_t in [max(0.,event_time-1e-7),event_time,min(approach.T,event_time+1e-7)]:
                    foot=feet_at(probe_t)[name]
                    near=foot.clearance_m<=limits.near_contact_height_m
                    low=foot.clearance_m<limits.edge_height_margin_m
                    checks={
                        'near_tangential_speed':not near or foot.tangential_speed_upper_bound_m_s<=limits.near_contact_tangential_speed_m_s+1e-9,
                        'near_closing_speed':not near or foot.closing_speed_upper_bound_m_s<=limits.near_contact_closing_speed_m_s+1e-9,
                        'near_tilt':not near or abs(approach.at(probe_t)['nose_up_rad'])<=limits.near_contact_tilt_rad+1e-9,
                        'low_footprint_margin':not low or min(foot.whole_cylinder_footprint_margins_m)>=limits.footprint_margin_m-1e-9}
                    events.append({'foot':name,'band_height_m':height,'boundary_t_s':event_time,'probe_t_s':probe_t,
                        'clearance_m':foot.clearance_m,'tangential_speed_upper_bound_m_s':foot.tangential_speed_upper_bound_m_s,
                        'closing_speed_upper_bound_m_s':foot.closing_speed_upper_bound_m_s,'checks':checks,'passed':all(checks.values())})
        previous_t=t;previous=current
    return {'passed':all(e['passed'] for e in events),'events':events,'bracketing_samples':sample_count}


def check_approach_reference(approach,geometry,deck,limits,sample_count=501):
    if not isinstance(sample_count,int) or sample_count<3:raise ValueError('At least 3 samples required')
    if norm(deck.motion.velocity_at_origin_world_m_s)+norm(deck.motion.angular_velocity_world_rad_s)>1e-10 or norm(sub(deck.motion.pose.rotate((0,0,1)),(0,0,1)))>1e-10:
        raise ValueError('Version 1 requires a stationary level platform')
    extrema={key:0. for key in ['horizontal_speed_m_s','vertical_speed_m_s','horizontal_acceleration_m_s2',
        'vertical_acceleration_m_s2','jerk_m_s3','snap_m_s4','tilt_rad','pitch_rate_rad_s','pitch_acceleration_rad_s2']}
    limit_values={
        'horizontal_speed_m_s':limits.max_horizontal_speed_m_s,'vertical_speed_m_s':limits.max_vertical_speed_m_s,
        'horizontal_acceleration_m_s2':limits.max_horizontal_acceleration_m_s2,'vertical_acceleration_m_s2':limits.max_vertical_acceleration_m_s2,
        'jerk_m_s3':limits.max_jerk_m_s3,'snap_m_s4':limits.max_snap_m_s4,
        'tilt_rad':limits.max_tilt_rad,'pitch_rate_rad_s':limits.max_pitch_rate_rad_s,
        'pitch_acceleration_rad_s2':limits.max_pitch_acceleration_rad_s2}
    minimum_upward=math.inf;thrust_min=math.inf;thrust_max=0.;minimum_precontact_gap=math.inf
    last_high_speed_time=None;rows=[]
    near_extrema={'tangential_speed_upper_bound_m_s':0.,'closing_speed_upper_bound_m_s':0.,'tilt_rad':0.}
    for tick in range(sample_count):
        t=approach.T*tick/(sample_count-1)
        try:state=approach.at(t)
        except ValueError as error:return {'passed':False,'reason':str(error),'t_s':t,'sample_count':sample_count}
        motion=state['motion'];velocity=state['com_velocity_world_m_s'];acceleration=state['com_acceleration_world_m_s2']
        values={'horizontal_speed_m_s':math.hypot(*velocity[:2]),'vertical_speed_m_s':abs(velocity[2]),
            'horizontal_acceleration_m_s2':math.hypot(*acceleration[:2]),'vertical_acceleration_m_s2':abs(acceleration[2]),
            'jerk_m_s3':norm(state['com_jerk_world_m_s3']),'snap_m_s4':norm(state['com_snap_world_m_s4']),
            'tilt_rad':abs(state['nose_up_rad']),'pitch_rate_rad_s':abs(state['nose_up_rate_rad_s']),
            'pitch_acceleration_rad_s2':abs(state['nose_up_acceleration_rad_s2'])}
        for key,value in values.items():
            extrema[key]=max(extrema[key],value)
            if value>limit_values[key]+1e-9:return {'passed':False,'reason':key+' design bound exceeded','t_s':t,'value':value,'limit':limit_values[key],'sample_count':sample_count}
        minimum_upward=min(minimum_upward,acceleration[2]+approach.gravity)
        if acceleration[2]+approach.gravity<limits.minimum_upward_specific_force_m_s2:
            return {'passed':False,'reason':'upward specific force design margin','t_s':t,'sample_count':sample_count}
        if motion.velocity_at_origin_world_m_s[0]<-1e-4:
            return {'passed':False,'reason':'horizontal reversal before contact','t_s':t,'sample_count':sample_count}
        if motion.velocity_at_origin_world_m_s[2]>.01:
            return {'passed':False,'reason':'climb during descending approach','t_s':t,'sample_count':sample_count}
        feet=evaluate_airframe(geometry,motion,deck);gap=feet[0].clearance_m
        if tick<sample_count-1:
            minimum_precontact_gap=min(minimum_precontact_gap,gap)
            if gap< -1e-8:return {'passed':False,'reason':'leg penetrates deck plane before terminal time','t_s':t,'sample_count':sample_count}
        for foot in feet:
            if foot.clearance_m<limits.edge_height_margin_m and min(foot.whole_cylinder_footprint_margins_m)<limits.footprint_margin_m-1e-9:
                return {'passed':False,'reason':'leg/deck edge clearance design margin','t_s':t,'sample_count':sample_count}
            if foot.clearance_m<=limits.near_contact_height_m:
                near_extrema['tangential_speed_upper_bound_m_s']=max(near_extrema['tangential_speed_upper_bound_m_s'],foot.tangential_speed_upper_bound_m_s)
                near_extrema['closing_speed_upper_bound_m_s']=max(near_extrema['closing_speed_upper_bound_m_s'],foot.closing_speed_upper_bound_m_s)
                near_extrema['tilt_rad']=max(near_extrema['tilt_rad'],abs(state['nose_up_rad']))
                if foot.tangential_speed_upper_bound_m_s>limits.near_contact_tangential_speed_m_s+1e-9:
                    return {'passed':False,'reason':'near-contact foot tangential speed','t_s':t,'sample_count':sample_count,
                            'foot':foot.name,'value':foot.tangential_speed_upper_bound_m_s,'limit':limits.near_contact_tangential_speed_m_s}
                if foot.closing_speed_upper_bound_m_s>limits.near_contact_closing_speed_m_s+1e-9:
                    return {'passed':False,'reason':'near-contact foot closing speed','t_s':t,'sample_count':sample_count,
                            'foot':foot.name,'value':foot.closing_speed_upper_bound_m_s,'limit':limits.near_contact_closing_speed_m_s}
                if abs(state['nose_up_rad'])>limits.near_contact_tilt_rad+1e-9:
                    return {'passed':False,'reason':'near-contact tilt','t_s':t,'sample_count':sample_count}
        if math.hypot(*motion.velocity_at_origin_world_m_s[:2])>=.12:last_high_speed_time=t
        thrust=state['point_mass_required_thrust_N'];thrust_min=min(thrust_min,thrust);thrust_max=max(thrust_max,thrust)
        rows.append({'t_s':t,'nose_up_rad':state['nose_up_rad'],'nose_up_rate_rad_s':state['nose_up_rate_rad_s'],
                     'body_position_world_m':motion.pose.position_world_m,'body_velocity_world_m_s':motion.velocity_at_origin_world_m_s,
                     'minimum_leg_clearance_m':gap,'required_thrust_N':thrust})
    final=approach.at(approach.T)['motion'];target=approach.terminal_boundary.aircraft
    peak_index=max(range(len(rows)),key=lambda index:rows[index]['body_velocity_world_m_s'][0])
    minimum_after_peak=rows[peak_index]['body_velocity_world_m_s'][0]
    for row in rows[peak_index:]:
        value=row['body_velocity_world_m_s'][0]
        if value-minimum_after_peak>.02:
            return {'passed':False,'reason':'reacceleration after main braking peak','t_s':row['t_s'],'sample_count':sample_count}
        minimum_after_peak=min(minimum_after_peak,value)
    final_feet=evaluate_airframe(geometry,final,deck)
    position_error=norm(sub(final.pose.position_world_m,target.pose.position_world_m))
    velocity_error=norm(sub(final.velocity_at_origin_world_m_s,target.velocity_at_origin_world_m_s))
    tilt_error=norm(sub(final.pose.rotate((0,0,1)),target.pose.rotate((0,0,1))))
    angle_rate_error=norm(sub(final.angular_velocity_world_rad_s,target.angular_velocity_world_rad_s))
    lead=approach.T-(last_high_speed_time if last_high_speed_time is not None else 0.)
    if lead>limits.max_low_horizontal_speed_lead_s:
        return {'passed':False,'reason':'low horizontal speed too early before contact','lead_s':lead,'sample_count':sample_count}
    checks={'terminal_position_matches_boundary':position_error<1e-8,'terminal_velocity_matches_boundary':velocity_error<1e-8,
            'terminal_attitude_matches_boundary':tilt_error<1e-8,'terminal_pitch_rate_matches_boundary':angle_rate_error<1e-8,
            'minimum_terminal_leg_gap_zero':abs(final_feet[0].clearance_m)<1e-8,
            'precontact_sampled_leg_gaps_nonnegative':minimum_precontact_gap>=-1e-8}
    height_entries=check_height_band_entries(approach,geometry,deck,limits,min(sample_count,501))
    if not height_entries['passed']:
        return {'passed':False,'reason':'height-band entry constraint','sample_count':sample_count,
                'failed_events':[e for e in height_entries['events'] if not e['passed']]}
    checks['detected_height_band_entries_pass']=True
    return {'passed':all(checks.values()),'checks':checks,'sample_count':sample_count,'duration_s':approach.T,
            'sampled_extrema':extrema,'point_mass_thrust_demand_range_N':[thrust_min,thrust_max],
            'near_contact_sampled_extrema':near_extrema,'no_reacceleration_after_peak_above_m_s':.02,
            'height_band_entry_checks':height_entries,
            'minimum_upward_specific_force_m_s2':minimum_upward,'low_horizontal_speed_lead_s':lead,
            'terminal_residuals':{'position_m':position_error,'velocity_m_s':velocity_error,'thrust_axis_vector_error':tilt_error,'angular_velocity_rad_s':angle_rate_error},
            'terminal_active_legs':[f.name for f in final_feet if abs(f.clearance_m)<1e-8],
            'minimum_precontact_sample_gap_m':minimum_precontact_gap,'samples':rows,
            'scope':'Sampled planar COM point-mass reference only; rotor/FCU tracking and impact stability unverified'}
