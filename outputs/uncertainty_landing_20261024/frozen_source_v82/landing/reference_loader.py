"""Rebuild the step07 simulation candidate from pinned configuration/evidence."""
import hashlib
import json
import math
from pathlib import Path
from .contact_geometry import AirframeGeometry,Deck,Pose,RigidMotion,norm,sub
from .touchdown_boundary import ContactDesignLimits,TouchdownRequest,solve_touchdown_boundary
from .septic_approach import PlanarSnapMatchedApproach


def load_verified_reference(root):
    root=Path(root);cfg=json.loads((root/'config/experiments/snap_approach_v2.json').read_text())
    evidence_root=root/'outputs/model_audit/step07'
    evidence=json.loads((evidence_root/'verification.json').read_text())
    dense=json.loads((evidence_root/'boundary_densification_check.json').read_text())
    plan=json.loads((evidence_root/'approach_plan.json').read_text())
    if not all([cfg['simulation_only'],evidence['passed'],dense['passed'],plan['passed']]) or cfg['hardware_deployment_allowed']:
        raise ValueError('Passing simulation-only reference evidence required')
    for name,digest in evidence['input_sha256'].items():
        relative=Path(name.replace('\\','/'))
        if relative.is_absolute() or '..' in relative.parts or ':' in name:raise ValueError('Expected workspace-relative source path')
        if hashlib.sha256((root/relative).read_bytes()).hexdigest()!=digest:raise ValueError('Reference source changed: '+name)
    bcfg=json.loads((root/cfg['boundary_config']).read_text())
    profile=json.loads((root/bcfg['airframe_profile']).read_text())
    geometry=AirframeGeometry.load(root/bcfg['airframe_profile'])
    if not geometry.simulation_only or profile['hardware_deployment_allowed']:raise ValueError('Simulation profile required')
    deck=Deck(RigidMotion.stationary(Pose(bcfg['deck_top_center_world_m'],bcfg['deck_orientation_xyzw'])),bcfg['deck_half_extents_m'])
    b=bcfg['design_limits']
    limits=ContactDesignLimits(math.radians(b['max_relative_tilt_deg']),math.radians(b['max_relative_angular_speed_deg_s']),
        b['max_tangential_speed_m_s'],b['min_closing_speed_m_s'],b['max_closing_speed_m_s'],b['footprint_margin_m'])
    boundary=solve_touchdown_boundary(geometry,deck,TouchdownRequest.planar(math.radians(cfg['primary_contact_angle_deg']),
        math.radians(cfg['contact_pitch_rate_deg_s']),bcfg['target_support_centroid_deck_xy_m'],
        bcfg['target_support_centroid_relative_velocity_deck_m_s']),limits)
    T=evidence['primary_duration_s']
    snap=tuple(evidence['terminal_com_snap_world_m_s4'])
    offset=profile['nominal_com_in_body_m']
    initial=(cfg['initial_com_x_m'],offset[1],cfg['initial_com_z_m'])
    curve=PlanarSnapMatchedApproach(initial,cfg['initial_com_velocity_world_m_s'],boundary,offset,cfg['reference_gravity_m_s2'],
        profile['mass_kg'],T,terminal_com_snap_world_m_s4=snap)
    stored=plan['normalized_polynomial_coefficients']
    if len(stored)!=3 or any(len(c)!=10 for c in stored):raise ValueError('Expected three degree-nine axes')
    if any(not math.isfinite(x) for c in stored for x in c):raise ValueError('Finite stored coefficients required')
    if max(abs(a-b) for axis,c in zip(curve.axes,stored) for a,b in zip(axis.coefficients,c))>1e-10:
        raise ValueError('Stored coefficients do not match pinned reference')
    if abs(plan['duration_s']-T)>1e-12 or norm(sub(plan['initial_com_position_world_m'],initial))>1e-12:
        raise ValueError('Saved reference state mismatch')
    return curve,geometry,deck,profile
