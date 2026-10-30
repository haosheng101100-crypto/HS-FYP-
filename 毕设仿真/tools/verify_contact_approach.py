"""Rebuild the saved candidate, densify it and inspect height-band entries.

This addresses a conditional sampling concern: the 8 cm speed condition may
first apply between the original sample times. It is not continuous proof.
"""
from pathlib import Path
from types import SimpleNamespace
import hashlib
import json
import math
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from landing.contact_geometry import AirframeGeometry,Pose,RigidMotion,Deck,evaluate_airframe
from landing.septic_approach import ApproachDesignLimits,PlanarSnapMatchedApproach,check_approach_reference,check_height_band_entries

OUT=ROOT/'outputs/model_audit/step07'
plan=json.loads((OUT/'approach_plan.json').read_text(encoding='utf-8'))
evidence=json.loads((OUT/'verification.json').read_text(encoding='utf-8'))
cfg=json.loads((ROOT/'config/experiments/snap_approach_v2.json').read_text())
bcfg=json.loads((ROOT/cfg['boundary_config']).read_text())
geometry=AirframeGeometry.load(ROOT/bcfg['airframe_profile'])
assert geometry.simulation_only and not cfg['hardware_deployment_allowed']
data=plan['boundary']['aircraft']
motion=RigidMotion(Pose(**data['pose']),data['velocity_at_origin_world_m_s'],data['angular_velocity_world_rad_s'])
boundary=SimpleNamespace(aircraft=motion)
curve=PlanarSnapMatchedApproach(plan['initial_com_position_world_m'],plan['initial_com_velocity_world_m_s'],boundary,
    plan['com_offset_body_m'],plan['gravity_m_s2'],plan['simulation_mass_kg'],plan['duration_s'],
    terminal_com_snap_world_m_s4=tuple(axis[4] for axis in plan['terminal_com_jet']))
deck=Deck(RigidMotion.stationary(Pose(bcfg['deck_top_center_world_m'],bcfg['deck_orientation_xyzw'])),bcfg['deck_half_extents_m'])
values=dict(cfg['design_limits'])
for a,b in [('max_tilt_deg','max_tilt_rad'),('max_pitch_rate_deg_s','max_pitch_rate_rad_s'),
            ('max_pitch_acceleration_deg_s2','max_pitch_acceleration_rad_s2'),('near_contact_tilt_deg','near_contact_tilt_rad')]:
    values[b]=math.radians(values.pop(a))
limits=ApproachDesignLimits(**values)
dense=check_approach_reference(curve,geometry,deck,limits,20001)

events=check_height_band_entries(curve,geometry,deck,limits,1001)['events']

hashes_ok=all(hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==expected for name,expected in evidence['input_sha256'].items())
coeff_error=max(abs(a-b) for axis,stored in zip(curve.axes,plan['normalized_polynomial_coefficients']) for a,b in zip(axis.coefficients,stored))
same_limits=cfg['design_limits']==json.loads((ROOT/'config/experiments/septic_approach_v1.json').read_text())['design_limits']
result={'passed':dense['passed'] and all(e['passed'] for e in events) and hashes_ok and coeff_error<1e-10 and same_limits,
    'saved_input_hashes_match':hashes_ok,'maximum_rebuilt_coefficient_error':coeff_error,
    'design_limits_unchanged_from_septic_v1':same_limits,
    'dense_check':{k:v for k,v in dense.items() if k!='samples'},'height_band_checks':events,
    'scope':'Offline sampled and bracketed-height-event checks only; not continuous certification or new physics flight.',
    'verification_script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
(OUT/'boundary_densification_check.json').write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
print(json.dumps({k:v for k,v in result.items() if k not in ['height_band_checks','dense_check']},ensure_ascii=False,indent=2))
print(json.dumps({'dense_passed':dense['passed'],'dense_reason':dense.get('reason'),'height_probes':len(events),
    'failed_height_probes':[e for e in events if not e['passed']]},ensure_ascii=False,indent=2))
assert result['passed'],'Inspect boundary_densification_check.json; do not promote a failed candidate'
