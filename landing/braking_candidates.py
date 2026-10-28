"""State-dependent touchdown-time search for a bounded SITL development case.

Keep native execution and contact gates unchanged. Sampled reference checks
are screening, never a guarantee of closed-loop or contact feasibility.
"""
import math,time
from .attitude_braking import attitude_capture_plan,AttitudeCaptureController
from .direct_braking import predict_stopped_deck,MonotoneBrakeAxis
from .septic_approach import NinthScalar
from .contact_geometry import Pose,RigidMotion,Deck,evaluate_airframe
from .touchdown_boundary import TouchdownRequest,ContactDesignLimits,solve_touchdown_boundary
from .descent_decision import state_metrics,SCHEMA
from .cached_reference import CachedScalar,cached_attitude_capture_plan
from .actuator_braking import CubicBrakeAxis

DURATIONS=(2.5,2.7,3.,3.3,3.6,4.)

def state_selected_braking_plan(geometry,profile,observation,deck,acceleration):
    started=time.perf_counter()
    stopped=predict_stopped_deck(deck,acceleration)
    offset=profile['nominal_com_in_body_m'];com=observation.motion.pose.transform(offset)
    velocity=observation.motion.velocity_at_world_point(com)
    y=com[1]-offset[1]-stopped.motion.pose.position_world_m[1]
    if abs(y)>.02:raise ValueError('Planar candidate entry outside lateral range')
    boundary=solve_touchdown_boundary(geometry,stopped,TouchdownRequest.planar(0.,0.,(0.,y),(0.,0.,-.45)),
        ContactDesignLimits(math.radians(5),math.radians(5),.05,.25,.65,.02))
    end=boundary.aircraft.pose.transform(offset)
    axes={}
    for shape,ramp in [('quintic_smoothstep',.4),('cubic_smoothstep',.44),('cubic_smoothstep',.48)]:
        key=shape+'_'+str(ramp)
        cls=MonotoneBrakeAxis if shape=='quintic_smoothstep' else CubicBrakeAxis
        axes[key]=cls(com[0],velocity[0],end[0],4.,ramp_in=ramp,ramp_out=.6)
    choices=[]
    for T in DURATIONS:
        z=CachedScalar(NinthScalar((com[2],velocity[2],0.,0.,0.),(end[2],-.45,0.,0.,0.),T))
        down=[-z.at(T*i/40,1) for i in range(41)]
        az=[z.at(T*i/40,2) for i in range(41)];peak=max(abs(v) for v in az)
        min_up=9.80665+min(az);jz=max(abs(z.at(T*i/40,3)) for i in range(41))
        for key,axis in axes.items():
            shape,ramp=key.rsplit('_',1);ramp=float(ramp)
            jerk=axis.peak_deceleration*(1.875 if shape=='quintic_smoothstep' else 1.5)/min(ramp,.6)
            rate=math.degrees(jerk/min_up+axis.peak_deceleration*jz/min_up**2) if min_up>0 else math.inf
            checks={'horizontal_release_reserve':axis.stop_s<T-.25,'reference_braking_acceleration':axis.peak_deceleration<=3.4,
                    'vertical_descent_design_reserve':min(down)>=.20,'vertical_speed_design_reserve':max(down)<=1.10,
                    'sampled_vertical_force_positive':min_up>6.,'nominal_pitch_rate_budget':rate<=70.}
            choices.append({'duration_s':T,'axis_key':key,'braking_shape':shape,'ramp_in_s':ramp,'ramp_out_s':.6,
                        'checks':checks,'minimum_reference_downward_speed_m_s':min(down),'maximum_reference_downward_speed_m_s':max(down),
                        'peak_reference_vertical_acceleration_m_s2':peak,'sampled_vertical_extrema_pitch_rate_bound_deg_s':rate,
                        'score':peak+.02*T+.0005*rate,'score_definition':'Nominal vertical acceleration + .02 * time + .0005 * nominal pitch-rate bound; design preference in restricted family'})
    eligible=sorted((r for r in choices if all(r['checks'].values())),key=lambda r:(r['score'],r['duration_s']))
    if not eligible:raise ValueError('No candidate with declared continuous-descent and release reserves')
    for selected in eligible:
        T=selected['duration_s'];axis=axes[selected['axis_key']]
        try:curve,terminal,plan=cached_attitude_capture_plan(geometry,profile,observation,deck,acceleration,T,stopped,boundary,axis)
        except ValueError as exc:
            selected['full_reference_rejection']=str(exc);continue
        minimum_gap=math.inf;coverage=math.inf;max_rate=0.;max_tilt=0.
        vx=deck.motion.velocity_at_origin_world_m_s[0];stop=-vx/acceleration[0];p=deck.motion.pose.position_world_m
        for i in range(81):
            t=T*i/80;u=min(t,stop)
            target=Deck(RigidMotion.stationary(Pose((p[0]+vx*u+.5*acceleration[0]*u*u,p[1],p[2]),deck.motion.pose.orientation_xyzw)),deck.half_extents_m)
            state=curve.at(t);feet=evaluate_airframe(geometry,state['motion'],target)
            max_rate=max(max_rate,math.degrees(abs(state['nose_up_rate_rad_s'])));max_tilt=max(max_tilt,math.degrees(abs(state['nose_up_rad'])))
            if i<80:minimum_gap=min(minimum_gap,min(f.clearance_m for f in feet))
            coverage=min(coverage,min(min(f.whole_cylinder_footprint_margins_m) for f in feet))
        checks={'sampled_no_early_foot_penetration':minimum_gap>=-1e-7,'sampled_moving_deck_footprint':coverage>=.02,
                'nominal_pitch_rate_within_command_slew':max_rate<=70.,'nominal_tilt_within_virtual_command_limit':max_tilt<=19.5}
        selected['full_path_checks']=checks
        if not all(checks.values()):continue
        plan['candidate_search']={'durations_s':DURATIONS,'candidates':choices,'selected_duration_s':T,
            'selected_score':selected['score'],'selected_path_checks':checks,'nominal_minimum_preterminal_gap_m':minimum_gap,
            'minimum_nominal_footprint_margin_m':coverage,'computation_wall_s':time.perf_counter()-started,
            'selected_braking_shape':selected['braking_shape'],'selected_ramp_in_s':selected['ramp_in_s'],'selected_ramp_out_s':.6,
            'maximum_nominal_pitch_rate_deg_s':max_rate,'maximum_nominal_tilt_deg':max_tilt,
            'scope':'Sampled current-state reference search in a restricted family; physical execution still required',
            'closed_loop_or_contact_guaranteed':False,'future_scenario_input_used':False}
        return curve,terminal,plan
    raise ValueError('All full-reference candidates rejected')


def assess_state_selected_braking(geometry,observation,deck,acceleration,now,plan):
    m=state_metrics(geometry,observation,deck,acceleration,now);search=plan.get('candidate_search',{})
    checks={'armed_guided':observation.armed and observation.flight_mode=='GUIDED',
        'development_high_gap_entry':1.9<=m['lowest_foot_gap_m']<=2.05,
        'measured_braking_entry':-2.55<=acceleration[0]<-2.2,
        'aircraft_speed_entry':4.7<=m['aircraft_vx_m_s']<=5.1,
        'vehicle_speed_entry':4.7<=m['platform_vx_m_s']<=5.05,
        'downward_speed_entry':.5<=m['downward_speed_m_s']<=.75,
        'position_entry':abs(m['relative_body_position_world_m'][0])<=.15 and abs(m['relative_body_position_world_m'][1])<=.02,
        'attitude_entry':m['tilt_deg']<=2 and m['angular_speed_deg_s']<=8,
        'whole_footprint_entry':m['whole_footprint_margin_m']>=.02,
        'nominal_arrest_reserve':m['nominal_arrest_clearance_reserve_m']>=0,
        'candidate_reference_checks':bool(plan.get('reference_checks')) and all(plan['reference_checks'].values()),
        'candidate_path_checks':bool(search.get('selected_path_checks')) and all(search['selected_path_checks'].values()),
        'duration_selected_from_declared_grid':plan.get('duration_s')==search.get('selected_duration_s') and plan.get('duration_s') in DURATIONS}
    permitted=all(checks.values())
    return {'schema':SCHEMA,'algorithm_revision':'state_selected_braking_v1','metrics':m,'decision_sim_t_s':now,
        'assessment':'CAN_PROPOSE_DEVELOPMENT_CANDIDATE' if permitted else 'UNKNOWN',
        'action':'EXECUTE_BRAKING_DESCENT_CANDIDATE' if permitted else 'REQUIRE_RECOVERY_ASSESSMENT',
        'execution_permitted_for_simulation':permitted,'platform_landing_completed':False,'candidate_duration_s':plan.get('duration_s'),
        'checks':checks,'reasons':['CURRENT_STATE_AND_SEARCH_PASS_DECLARED_DEVELOPMENT_GATES','ACTUAL_TOUCHDOWN_AND_SUPPORT_REQUIRE_AUDIT'] if permitted else [k for k,v in checks.items() if not v],
        'scope':'First high-gap SITL development admission; no continuous safe set, arbitrary-height admission or optimality claim',
        'global_infeasibility_claim':False,'hardware_deployment_allowed':False}


class StateSelectedAttitudeController(AttitudeCaptureController):
    """Add measured vertical-speed damping; retain original native limits."""
    def __init__(self,*args,vertical_velocity_feedback=.8,**kwargs):
        super().__init__(*args,**kwargs)
        if not math.isfinite(vertical_velocity_feedback) or not 0<vertical_velocity_feedback<=1.:
            raise ValueError('Explicit vertical velocity feedback must stay in (0,1]')
        self.vertical_velocity_feedback=vertical_velocity_feedback
        self.vertical_feedback_blend=0.

    def command(self,reference,observation,now):
        dt=.04 if self.last_t is None else now-self.last_t
        command=super().command(reference,observation,now)
        error=reference.velocity_world_m_s[2]-observation.motion.velocity_at_origin_world_m_s[2]
        tilt=math.degrees(math.acos(max(-1.,min(1.,observation.motion.pose.rotate((0,0,1))[2]))))
        released=now-self.start>=self.curve.axes[0].stop_s and abs(observation.motion.velocity_at_origin_world_m_s[0])<=.1 and tilt<=3.
        self.vertical_feedback_blend=min(1.,self.vertical_feedback_blend+dt/.2) if released else 0.
        correction=self.vertical_feedback_blend*self.vertical_velocity_feedback*error
        original=command['climb_rate_command_m_s']
        climb=max(-1.2,min(.5,original+correction))
        command.update(climb_rate_command_m_s=climb,normalized_climb_field=.5+climb/(2*(self.up if climb>=0 else self.down)),
            vertical_velocity_tracking={'reference_vz_m_s':reference.velocity_world_m_s[2],
             'measured_vz_m_s':observation.motion.velocity_at_origin_world_m_s[2],
             'error_m_s':error,'feedback_gain':self.vertical_velocity_feedback,'correction_m_s':correction,
             'unmodified_climb_command_m_s':original,'identified_response_model':False,
             'horizontal_braking_released_and_measured_settled':released,'feedback_blend':self.vertical_feedback_blend,
             'blend_ramp_s':.2,'release_speed_bound_m_s':.1,'release_tilt_bound_deg':3.})
        return command
