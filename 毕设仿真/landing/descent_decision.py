"""Common causal decision interface for two bounded SITL regression branches.

Declared entry gates restrict this first integration, not a certified safe set.
An admitted candidate still needs actual flight/contact/support verification.
"""
import math
from .contact_geometry import evaluate_airframe, norm
from .acceleration_decision import assess_accelerating_descent

SCHEMA = 'interrupted_descent_decision_v1'


def state_metrics(geometry, observation, deck, acceleration, now):
    if len(acceleration) != 3 or not all(math.isfinite(x) for x in acceleration):
        raise ValueError('Finite measured platform acceleration required')
    if not 0 <= now - observation.sim_t_s <= .06:
        raise ValueError('Fresh causal aircraft observation required')
    p = observation.motion.pose.position_world_m
    v = observation.motion.velocity_at_origin_world_m_s
    dp = deck.motion.pose.position_world_m
    dv = deck.motion.velocity_at_origin_world_m_s
    values = (*p, *v, *dp, *dv, *observation.motion.angular_velocity_world_rad_s)
    if not all(math.isfinite(x) for x in values):
        raise ValueError('Finite current flight and platform state required')
    feet = evaluate_airframe(geometry, observation.motion, deck)
    gap = min(f.clearance_m for f in feet)
    down = max(0., -v[2])
    # Deliberately labelled design assumptions; not identified arrest guarantees.
    loss = down * .2 + down * down / (2 * .8)
    return {
        'sample_sim_t_s': observation.sim_t_s, 'decision_sim_t_s': now,
        'observation_age_s': now - observation.sim_t_s,
        'relative_body_position_world_m': [x-y for x,y in zip(p,dp)],
        'relative_body_velocity_world_m_s': [x-y for x,y in zip(v,dv)],
        'platform_acceleration_m_s2': list(acceleration),
        'aircraft_vx_m_s': v[0], 'platform_vx_m_s': dv[0],
        'lowest_foot_gap_m': gap, 'downward_speed_m_s': down,
        'whole_footprint_margin_m': min(min(f.whole_cylinder_footprint_margins_m) for f in feet),
        'tilt_deg': math.degrees(math.acos(max(-1., min(1., observation.motion.pose.rotate((0,0,1))[2])))),
        'angular_speed_deg_s': math.degrees(norm(observation.motion.angular_velocity_world_rad_s)),
        'nominal_arrest_clearance_reserve_m': gap-loss-.3,
        'abort_arrest_model_identified': False,
        'arrest_design_assumptions': {'response_reserve_s': .2, 'upward_acceleration_m_s2': .8, 'retained_gap_m': .3},
    }


def assess_interrupted_descent(geometry, observation, deck, acceleration, now,
                               flat_return_corridor_known, braking_candidate=None):
    if type(flat_return_corridor_known) is not bool:
        raise ValueError('Explicit simulated route evidence required')
    m = state_metrics(geometry, observation, deck, acceleration, now)
    a = acceleration[0]
    result = {'schema': SCHEMA, 'metrics': m, 'decision_sim_t_s': now,
              'assessment': 'UNKNOWN', 'action': 'REQUIRE_RECOVERY_ASSESSMENT',
              'execution_permitted_for_simulation': False, 'platform_landing_completed': False,
              'candidate_duration_s': None, 'reasons': [], 'checks': {},
              'global_infeasibility_claim': False, 'hardware_deployment_allowed': False,
              'scope': 'Two bounded SITL regression branches; no certified continuous safe set or general landing classifier'}
    if not observation.armed or observation.flight_mode != 'GUIDED':
        result['reasons'] = ['FLIGHT_STATE_NOT_ARMED_GUIDED']
        return result
    if a > 2.2:
        screen = assess_accelerating_descent(geometry, observation, deck, acceleration, now, flat_return_corridor_known)
        result.update(assessment='REJECT_CURRENT_LANDING_POLICY', action=screen['action'],
                      reasons=screen['reasons'], checks=screen['checks'], acceleration_screen=screen)
        # The old screen's HOLD wording cannot imply that insufficient arrest
        # reserve makes hover feasible. Keep that case explicitly unvalidated.
        if not screen['checks']['nominal_vertical_arrest_reserve']:
            result['action'] = 'REQUIRE_RECOVERY_ASSESSMENT'
            result['reasons'].append('ARREST_RESERVE_NOT_ESTABLISHED')
        return result
    if a < -2.2:
        checks = {
            'declared_acceleration_entry': -2.55 <= a < -2.2,
            'declared_aircraft_speed_entry': 4.7 <= m['aircraft_vx_m_s'] <= 5.1,
            'declared_platform_speed_entry': 4.7 <= m['platform_vx_m_s'] <= 5.05,
            'declared_gap_entry': 1.45 <= m['lowest_foot_gap_m'] <= 1.55,
            'declared_downward_speed_entry': .5 <= m['downward_speed_m_s'] <= .75,
            'declared_position_entry': abs(m['relative_body_position_world_m'][0]) <= .15 and abs(m['relative_body_position_world_m'][1]) <= .02,
            'declared_attitude_entry': m['tilt_deg'] <= 2. and m['angular_speed_deg_s'] <= 8.,
            'whole_footprint_at_entry': m['whole_footprint_margin_m'] >= .02,
            'nominal_arrest_reserve': m['nominal_arrest_clearance_reserve_m'] >= 0.,
            'candidate_present': isinstance(braking_candidate, dict),
        }
        if isinstance(braking_candidate, dict):
            refchecks = braking_candidate.get('reference_checks', {})
            checks['candidate_reference_checks'] = bool(refchecks) and all(v is True for v in refchecks.values())
            checks['declared_candidate_duration'] = braking_candidate.get('duration_s') == 3.
            result['candidate_duration_s'] = braking_candidate.get('duration_s')
        result['checks'] = checks
        if all(checks.values()):
            result.update(assessment='CAN_PROPOSE_BOUNDED_REGRESSION_CANDIDATE',
                          action='EXECUTE_BRAKING_DESCENT_CANDIDATE', execution_permitted_for_simulation=True,
                          reasons=['CURRENT_STATE_AND_CANDIDATE_PASS_DECLARED_REGRESSION_GATES', 'ACTUAL_TOUCHDOWN_AND_SUPPORT_STILL_REQUIRE_INDEPENDENT_AUDIT'])
        else:
            result['reasons'] = [k for k,v in checks.items() if not v]
        return result
    result.update(action='CONTINUE_EXISTING_NOMINAL_REFERENCE',
                  reasons=['NO_COVERED_STRONG_ACCELERATION_EVENT'])
    return result
