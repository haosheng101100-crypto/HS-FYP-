"""Conservative admission screen for interrupted descent, not an impossibility proof."""
from dataclasses import dataclass,asdict
import math
from .contact_geometry import evaluate_airframe,norm

@dataclass(frozen=True)
class AccelerationAdmissionLimits:
    contact_tilt_deg:float=5.
    actual_flight_tilt_deg:float=20.
    horizontal_command_m_s2:float=3.5
    relative_contact_speed_m_s:float=.05
    contact_matching_window_s:float=.2
    minimum_abort_clearance_m:float=.3
    abort_reaction_reserve_s:float=.2
    assumed_vertical_arrest_acceleration_m_s2:float=.8

def assess_accelerating_descent(geometry,observation,deck,acceleration,now,flat_return_corridor_known,limits=AccelerationAdmissionLimits()):
    if len(acceleration)!=3 or not all(math.isfinite(a) for a in acceleration):raise ValueError('Finite current measured platform acceleration required')
    if not 0<=now-observation.sim_t_s<=.06:raise ValueError('Fresh causal aircraft observation required')
    if type(flat_return_corridor_known) is not bool:raise ValueError('Explicit simulated route evidence required')
    a=acceleration[0];p=observation.motion.pose.position_world_m;v=observation.motion.velocity_at_origin_world_m_s
    dp=deck.motion.pose.position_world_m;dv=deck.motion.velocity_at_origin_world_m_s
    feet=evaluate_airframe(geometry,observation.motion,deck);gap=min(f.clearance_m for f in feet)
    down=max(0.,-v[2]);loss=down*limits.abort_reaction_reserve_s+down*down/(2*limits.assumed_vertical_arrest_acceleration_m_s2)
    required=math.degrees(math.atan2(abs(a),9.80665))
    tilt_capacity=9.80665*math.tan(math.radians(limits.contact_tilt_deg))
    acceleration_slack=limits.relative_contact_speed_m_s/limits.contact_matching_window_s
    sustained=abs(a)<=tilt_capacity+acceleration_slack
    candidates=[]
    for T in (2.5,3.,3.5):
        target=dp[0]+dv[0]*T+.5*a*T*T;speed=dv[0]+a*T
        candidates.append({'duration_s':T,'predicted_deck_x_m':target,'predicted_deck_vx_m_s':speed,
            'average_required_aircraft_acceleration_m_s2':(speed-v[0])/T,
            'optimistic_velocity_budget':abs(speed-v[0])<=limits.horizontal_command_m_s2*T,
            'optimistic_position_budget':abs(target-p[0]-v[0]*T)<=.5*limits.horizontal_command_m_s2*T*T,
            'sustained_precontact_matching_screen':sustained,'physics_validated':False})
    checks={'armed_guided':observation.armed and observation.flight_mode=='GUIDED',
        'finite_footprint_at_decision':all(min(f.whole_cylinder_footprint_margins_m)>=.02 for f in feet),
        'positive_acceleration_observed':a>.05,'sustained_precontact_matching':sustained,
        'nominal_vertical_arrest_reserve':gap-loss>=limits.minimum_abort_clearance_m,
        'simulated_flat_return_corridor_known':flat_return_corridor_known}
    reasons=[]
    if not sustained:reasons.append('SUSTAINED_CONTACT_MATCH_OUTSIDE_CURRENT_NEAR_LEVEL_ENVELOPE')
    # No new positively accelerating touchdown / native LAND support certificate
    # exists. A kinematic candidate alone never authorises physical touchdown.
    reasons.append('NO_MATCHED_ACCELERATING_TOUCHDOWN_AND_SUPPORT_VALIDATION')
    action='ABORT_DESCENT_VALIDATE_RETURN_CORRIDOR' if checks['nominal_vertical_arrest_reserve'] and flat_return_corridor_known else 'ABORT_DESCENT_HOLD_FOR_REASSESSMENT'
    if not checks['armed_guided']:action='REQUIRE_FLIGHT_STATE_RECOVERY'
    return {'action':action,'platform_landing_authorised':False,'landing_physics_validated':False,
        'reasons':reasons,'checks':checks,'limits':asdict(limits),'decision_sim_t_s':now,
        'measured_acceleration_m_s2':acceleration,'relative_body_position_world_m':tuple(x-y for x,y in zip(p,dp)),
        'relative_body_velocity_world_m_s':tuple(x-y for x,y in zip(v,dv)),
        'lowest_foot_gap_m':gap,'downward_speed_m_s':down,'required_acceleration_matching_tilt_deg':required,
        'near_level_horizontal_acceleration_capacity_m_s2':tilt_capacity,
        'allowed_relative_acceleration_over_matching_window_m_s2':acceleration_slack,
        'predicted_vertical_arrest_loss_m':loss,'nominal_arrest_clearance_reserve_m':gap-loss-limits.minimum_abort_clearance_m,
        'candidates':candidates,'global_infeasibility_claim':False,
        'screen_scope':'Current near-level contact envelope plus declared 0.2 s matching window; not a proof that momentary touchdown or another tilted-contact policy is impossible.',
        'abort_arrest_model_identified':False,'battery_or_real_route_validated':False,'hardware_deployment_allowed':False}
