"""Read-only M17 raw replay. Does not change controller, cases or old audits."""
from pathlib import Path
import json, gzip, hashlib, math, statistics, sys

W = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
SOURCE = W/'outputs/HS-FYP-/毕设仿真'
sys.path.insert(0, str(SOURCE))

def inspect(relative):
    attempt = W/relative
    result = json.loads((attempt/'result.json').read_text())
    res = attempt/'session_001/cycle_001/results'
    rows = [json.loads(s) for s in (res/'receding_control.jsonl').read_text().splitlines()]
    trigger = json.loads((res/'scene_maneuvers.json').read_text())[0]
    stamp = trigger['time']
    exit_row = next(r for r in rows if (r.get('planar_supervision') or {}).get('action') == 'EXIT')
    end = exit_row['sim_t_s']
    raw = res/'model_states.jsonl'
    stream = raw.open('rb') if raw.exists() else gzip.open(raw.with_suffix('.jsonl.gz'), 'rb')
    samples = {}; digest = hashlib.sha256()
    with stream:
        for line in stream:
            digest.update(line); d = json.loads(line)
            if d['kind'] != 'car_pose': continue
            for pose in d['message'].get('pose', []):
                if pose['name'] != 'landing_vehicle': continue
                s = pose['header']['stamp']; t = float(s.get('sec', 0))+float(s.get('nsec', 0))*1e-9
                if stamp-1 <= t <= stamp+7:
                    p = pose.get('position', {})
                    samples[t] = (float(p.get('x', 0)), float(p.get('y', 0)))
    expected_digest = json.loads((res/'raw_compression.json').read_text())['sha256_uncompressed']
    assert digest.hexdigest() == expected_digest
    times = sorted(samples); intervals = []
    for a,b in zip(times,times[1:]):
        assert b>a
        v = tuple((samples[b][i]-samples[a][i])/(b-a) for i in (0,1))
        intervals.append((a,b,v))
    before = [v for a,b,v in intervals if a>=stamp-.4 and b<stamp-.05]
    baseline = tuple(statistics.median(v[i] for v in before) for i in (0,1))
    speed = math.hypot(*baseline); unit = tuple(v/speed for v in baseline)
    target = trigger['target_xy']; delta = tuple(target[i]-baseline[i] for i in (0,1)); n = math.hypot(*delta)
    def qualifies(v):
        return sum((v[i]-baseline[i])*delta[i]/n for i in (0,1))>.01 and sum(v[i]*baseline[i] for i in (0,1))<-.5*speed*math.hypot(*v) and math.hypot(*v)>.04
    hits = []
    for i in range(len(intervals)-2):
        window = intervals[i:i+3]
        if window[0][0]>stamp+.05 and all(qualifies(v) for a,b,v in window):
            hits.append(window); break
    assert hits, 'No actual reverse response found, even after exit'
    first = hits[0]
    velocity_at_exit = min(intervals, key=lambda x:abs((x[0]+x[1])/2-end))
    audit = result['multicondition_audit']
    assert result['category']=='coverage_failure' and audit['safety']['passed'] and audit['physics']['passed']
    assert first[0][0] > end, 'Unexpected pre-exit reverse; original failure requires deeper review'
    minimum_gap = min(f['clearance_m'] for f in exit_row['feet'])
    report = dict(original_attempt=relative, control_commit=result['control_commit'], original_category=result['category'],
        original_safety_passed=True, original_physics_passed=True, original_coverage_passed=False,
        trigger_sim_s=stamp, exit_sim_s=end, response_window_before_exit_s=end-stamp,
        trigger_clearance_m=trigger['measured_leg_gap_m'], exit_clearance_m=minimum_gap,
        exit_reason=exit_row['planar_supervision']['reason'], baseline_raw_velocity_m_s=baseline,
        baseline_speed_m_s=speed, raw_velocity_near_exit_m_s=velocity_at_exit[2],
        first_reverse_three_raw_intervals=[dict(start_s=a,end_s=b,velocity_xy_m_s=v) for a,b,v in first],
        first_reverse_delay_after_trigger_s=(first[0][0]+first[0][1])/2-stamp,
        first_reverse_delay_after_exit_s=(first[0][0]+first[0][1])/2-end,
        command_acceleration_cap_m_s2=.12,
        ideal_command_time_to_zero_lower_bound_s=speed/.12,
        ideal_collinear_command_time_to_minus_004_lower_bound_s=(speed+.04)/.12,
        lower_bound_scope='Ideal scalar command bound neglecting jerk and servo lag; explanatory, not a bound on all possible physical disturbances.',
        raw_uncompressed_sha256=digest.hexdigest(),
        original_reports_unchanged=True, no_new_physical_flight=True)
    plot = [dict(start_s=a,end_s=b,sim_s=(a+b)/2,velocity_xy_m_s=v,
        projected_velocity_m_s=sum(v[i]*unit[i] for i in (0,1))) for a,b,v in intervals]
    return report, plot

reports = {}; plots = {}
for name, path in [('original_20_case','outputs/二维多工况_20261103/batch_002/cases/M17/attempt_001'),
                   ('terminal_v2','outputs/近地退出连续性修复_20261103/batch_003/cases/M17/attempt_001')]:
    reports[name], plots[name] = inspect(path)

# Minimal deterministic command replay under exactly the original acceleration/jerk limits.
from landing.planar_motion import PlanarMotionCommand
import numpy as np
command = PlanarMotionCommand(); initial=np.array([math.sqrt(3)/8, .125])
command.v=initial.copy(); command.a=np.zeros(2); command.last=0.
replay=[]
for i in range(1,701):
    t=i*.01; v,a=command.step(t,-initial); replay.append((t,math.hypot(*v),float(np.dot(v,initial)/np.linalg.norm(initial))))
assert all(speed<=.8+1e-8 for _,speed,_ in replay)
first_reverse=next(t for t,s,p in replay if p<-.04)
assert first_reverse>reports['terminal_v2']['response_window_before_exit_s']
bundle=dict(analysis='M17 full reversal occurs after safety exit in both recorded physical attempts. No audit is relaxed; post-exit motion does not satisfy descent coverage.',
    attempts=reports, deterministic_command_replay=dict(first_opposite_speed_over_004_s=first_reverse, acceleration_cap_m_s2=.12,jerk_cap_m_s3=.3,physical_flight=False),
    original_M17_coverage_still_incomplete=True, new_physical_attempts=0,
    next_step='Keep original M17 coverage_failure. Any experiment seeking full reversal before exit needs a separately declared condition or a justified controller redesign, rather than waiting past a safety exit or renaming the old outcome.')
(OUT/'M17诊断结果.json').write_text(json.dumps(bundle,ensure_ascii=False,indent=2))
(OUT/'M17_raw_velocity_windows.json').write_text(json.dumps(plots,ensure_ascii=False,indent=2))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig, axes=plt.subplots(2,1,figsize=(10,7),layout='constrained')
for ax,(name,r) in zip(axes,reports.items()):
    data=plots[name]
    ax.plot([x['sim_s']-r['trigger_sim_s'] for x in data],[x['projected_velocity_m_s'] for x in data],label='Actual raw platform velocity along initial direction')
    ax.axvline(0,color='black',ls=':',label='Reversal command')
    ax.axvline(r['response_window_before_exit_s'],color='red',ls='--',label='Aircraft safety EXIT')
    ax.axvline(r['first_reverse_delay_after_trigger_s'],color='green',ls='--',label='First 3 actual reverse intervals')
    ax.axhline(0,color='gray',lw=.6);ax.axhline(-.04,color='gray',ls=':',lw=.6)
    ax.set_title(name);ax.set_xlabel('Seconds after measured reversal trigger');ax.set_ylabel('Velocity (m/s)');ax.grid(alpha=.2);ax.legend(fontsize=8)
fig.savefig(OUT/'M17实际换向与安全退出.png',dpi=150)
print(json.dumps(bundle,ensure_ascii=False,indent=2))
