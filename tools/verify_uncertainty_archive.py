"""Verify final physical cases, source identity and local native/raw evidence."""
from pathlib import Path
import json,hashlib,gzip,argparse
from audit_guided_motion import audit as audit_guided_motion
ROOT=Path(__file__).resolve().parents[1]
base=ROOT/'outputs/uncertainty_landing_20261024'
p=argparse.ArgumentParser();p.add_argument('--protocol',default='validation_protocol_v18.json');p.add_argument('--uncertainty-audit',default='independent_uncertainty_audit_v13.json');p.add_argument('--output',default='verification.json');args=p.parse_args()
protocol_path=base/args.protocol
protocol=json.loads(protocol_path.read_text())

def digest(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for data in iter(lambda:f.read(1048576),b''):h.update(data)
    return h.hexdigest()

def load(path):return json.loads(path.read_text(encoding='utf-8'))

ref=protocol['reference_case']
reference=base/f"batch_{ref['session']:03d}"/f"cycle_{ref['cycle']:03d}"
shared=['landing/horizontal_feedback.py','landing/mavlink_reference.py','landing/moving_platform.py',
        'landing/execution.py','config/airframes/iris_fixed_gimbal_sim.json',
        'config/airframes/iris_swept_envelope_v1.json','config/experiments/execution_v1.json']
shared_hashes={name:digest(reference/name) for name in shared}
original=base/'batch_065/cycle_003'
original_contract=['config/copter.parm','config/gazebo-iris.parm','config/stage1.parm','scripts/run_fcu.sh',
    'config/experiments/execution_v1.json','config/airframes/iris_fixed_gimbal_sim.json',
    'config/airframes/iris_swept_envelope_v1.json','landing/contact_geometry.py']
original_hashes={name:digest(original/name) for name in original_contract}
for name,expected in original_hashes.items():assert digest(reference/name)==expected,('Original model/native contract changed',name)
cases=[];largest_offset=0.;frame_reserve_rejections=0;heading_rejections=0
for declared in protocol['cases']:
    run=base/f"batch_{declared['session']:03d}"/f"cycle_{declared['cycle']:03d}";r=run/'results'
    assert load(r/'cycle_closed.json')['all_owned_cycle_processes_stopped']
    st=load(r/'execution_status.json');assert not st['armed_at_save']
    assert st['scenario']==declared['scenario']
    for name,expected in protocol['source_sha256'].items():assert digest(run/name)==expected,(run,name)
    for name,expected in shared_hashes.items():assert digest(run/name)==expected,(run,name)
    for name,expected in original_hashes.items():assert digest(run/name)==expected,(run,'Original contract',name)
    cfg=load(run/'config/experiments/receding_live_v1.json');cfg.pop('validation_scenarios')
    assert cfg==protocol['control_configuration'],run
    outcome='landing' if st['completed'] else 'safe_exit'
    assert declared['expected'] in (outcome,'landing_or_safe_exit'),(run,declared['expected'],outcome)
    physics='independent_contact_audit.json' if st['completed'] else ('independent_maneuver_exit_audit.json' if st['scenario']=='switchback' else 'independent_exit_audit_v3.json')
    reports={name:load(r/name) for name in [physics,'native_landing_audit.json',
        'independent_command_audit.json',args.uncertainty_audit]}
    reports['independent_guided_motion_audit.json']=audit_guided_motion(run)
    assert all(v['passed'] for v in reports.values()),run
    largest_offset=max(largest_offset,reports[args.uncertainty_audit]['largest_landing_point_offset_m'])
    frame_reserve_rejections+=reports[args.uncertainty_audit]['frame_velocity_reserve_rejections']
    heading_rejections+=reports[args.uncertainty_audit]['heading_guard_rejections']
    h=hashlib.sha256()
    with gzip.open(r/'model_states.jsonl.gz','rb') as f:
        for data in iter(lambda:f.read(1048576),b''):h.update(data)
    raw_sha=h.hexdigest();assert raw_sha==load(r/'raw_compression.json')['sha256_uncompressed']
    assert reports['independent_guided_motion_audit.json']['raw_uncompressed_sha256']==raw_sha
    audit=reports[physics]
    assert raw_sha==(audit['source_sha256']['model_states.jsonl'] if st['completed'] else audit['raw_source_sha256'])
    if st['completed']:
        for name,expected in audit['source_sha256'].items():
            if name!='model_states.jsonl':assert digest(r/name)==expected,(run,name)
    native=reports['native_landing_audit.json']
    for name,expected in native['source_sha256'].items():assert digest(run/'runtime/logs'/name)==expected,(run,name)
    if declared['scenario']=='noisy_maneuvering':
        assert cfg['synthetic_platform_observation_stress']['delay_s']>0
        assert cfg['synthetic_platform_observation_stress']['position_sigma_m']>0
        with (r/'receding_control.jsonl').open() as f:
            first=json.loads(next(f))
        assert first['synthetic_platform_observation_stress']==cfg['synthetic_platform_observation_stress']
        assert first['platform_fit_input']
    cases.append(dict(**declared,outcome=outcome,passed=True,physics_audit=physics,
        raw_uncompressed_sha256=raw_sha,guided_motion_audit='independent_guided_motion_audit.json',terminal_guard_samples=reports[args.uncertainty_audit]['terminal_guard_samples']))
unit=load(base/protocol['unit_test_report']);assert unit['passed']
assert unit['source_sha256']==protocol['source_sha256']
assert unit['control_configuration']==protocol['control_configuration']
assert largest_offset>1e-7,'Landing-point optimization did not produce a nonzero offset in any selected case'
assert heading_rejections>0,'The new early turning recovery branch was not physically exercised'
interruptions=[]
for version in (38,46,47):
    session=base/f'interrupted_{version:03d}'
    review=load(session/'interruption_review.json')
    assert review['cause']=='host_restart'
    assert review['all_previous_owned_processes_absent']
    assert digest(session/'status.json')==review['status_sha256']
    records=[]
    for cycle in sorted(session.glob('cycle_*')):
        path=cycle/'results/interruption_record.json'
        if not path.exists():continue
        record=load(path)
        assert record['completed'] is False and record['cause']=='host_restart'
        assert record['normal_disarm_and_support_not_verified']
        assert not (cycle/'results/cycle_closed.json').exists()
        for name,expected in record['source_sha256'].items():
            assert digest(cycle/'results'/name)==expected,(cycle,name)
        assert record['last_recorded_status']==load(session/'status.json')
        records.append(dict(cycle=cycle.name,last_recorded_armed=record['last_recorded_status']['armed'],counted_as_success=False,source_hashes_verified=True))
    assert records,session
    interruptions.append(dict(session=version,original_status_preserved=True,records=records))
preserved_failures=[]
for version,cycle in ((48,5),(55,1)):
    failed=base/f'batch_{version:03d}/cycle_{cycle:03d}/results'
    failed_exit=load(failed/'independent_maneuver_exit_audit.json')
    assert failed_exit['passed'] is False and failed_exit['checks']['positive_raw_clearance_through_return'] is False
    assert failed_exit['minimum_exit_leg_clearance_m']<.05
    h=hashlib.sha256()
    with gzip.open(failed/'model_states.jsonl.gz','rb') as f:
        for data in iter(lambda:f.read(1048576),b''):h.update(data)
    assert h.hexdigest()==failed_exit['raw_source_sha256']==load(failed/'raw_compression.json')['sha256_uncompressed']
    preserved_failures.append(dict(session=version,cycle=cycle,counted_as_success=False,raw_hash_verified=True,
        minimum_leg_clearance_m=failed_exit['minimum_exit_leg_clearance_m']))
external_failures=[]
for excluded in protocol.get('excluded_external_cases',[]):
    r=base/f"batch_{excluded['session']:03d}/cycle_{excluded['cycle']:03d}/results"
    assert excluded['counted_as_success'] is False
    st=load(r/'execution_status.json');assert not st['completed'] and not st['armed_at_save']
    assert st['external_failures']==[excluded['reason']]
    assert load(r/'cycle_closed.json')['all_owned_cycle_processes_stopped']
    for name,sha in excluded['source_sha256'].items():assert digest(r/name)==sha
    h=hashlib.sha256()
    with gzip.open(r/'model_states.jsonl.gz','rb') as f:
        for data in iter(lambda:f.read(1048576),b''):h.update(data)
    assert h.hexdigest()==excluded['raw_uncompressed_sha256']
    native=load(r/'native_landing_audit.json');assert native['passed']
    for name,sha in native['source_sha256'].items():assert digest(r.parent/'runtime/logs'/name)==sha
    external_failures.append(dict(**excluded,original_evidence_verified=True,normal_native_recovery_verified=True))
report=dict(passed=True,cases=cases,source_sha256=protocol['source_sha256'],
    preserved_external_simulator_failures=external_failures,
    original_native_parameters_and_contact_geometry_preserved=True,original_contract_sha256=original_hashes,
    shared_native_interface_and_geometry_sha256=shared_hashes,
    same_control_configuration=True,unit_tests_passed=True,local_archive_hashes_verified=True,
    joint_landing_point_optimization_exercised=True,largest_landing_point_offset_m=largest_offset,
    early_measured_turning_recovery_exercised=True,heading_acceleration_rejections=heading_rejections,
    frame_velocity_reserve_rejections=frame_reserve_rejections,
    preserved_host_interruptions=interruptions,
    preserved_late_exit_failures=preserved_failures,
    protocol_sha256=digest(protocol_path),scope=protocol['scope'],
    statistical_coverage_proved=False,continuous_collision_guarantee=False,hardware_validated=False)
out=base/args.output;assert not out.exists()
out.write_text(json.dumps(report,indent=2),encoding='utf-8')
print('Verified all declared physical cases, identical control sources/configuration, causal guards, lossless raw logs and native flight records.')
