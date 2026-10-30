"""Lagged point-mass numerical checks, explicitly not physical flight evidence."""
from receding_landing_demo import ROOT, build
from dataclasses import replace
import json
import math
import time
import numpy as np
from landing.receding_landing import FlightState, PlatformState, RecedingLandingSession
from landing.contact_geometry import Pose, RigidMotion, quaternion_from_rpy, evaluate_airframe


def run(name):
    planner = build(); planner.limits = replace(planner.limits,audit_samples=101)
    platform = PlatformState(0.,(.5,0.,.8),(.3,0.,0.),(0.,0.,0.))
    state = FlightState(0.,(0.,0.,2.),(0.,0.,0.),(0.,0.,0.))
    # Compile the template before measuring updates; no actuator is attached.
    planner.solve(state,platform,0.,4.)
    session = RecedingLandingSession(planner,(3.,4.,5.),max_compute_s=5.)
    dt=.02; next_update=0.; rows=[]; decisions=[]; outcome='TIMEOUT'; maximum_speed=0.; maximum_accel=0.
    for k in range(1001):
        now=k*dt
        if k:
            a=np.zeros(3)
            if name=='speed_change': a[0]=.12 if 1.<=now<2.5 else -.08 if 2.5<=now<4. else 0.
            if name=='turning':
                omega=.08 if now<2.5 else -.06
                a=np.array([-omega*platform.velocity[1],omega*platform.velocity[0],0.])
            v=np.array(platform.velocity); p=np.array(platform.position)
            platform=replace(platform,stamp=now,position=tuple(p+dt*v+.5*dt*dt*a),
                velocity=tuple(v+dt*a),acceleration=tuple(a),
                yaw=platform.yaw+(omega*dt if name=='turning' else 0.),
                yaw_rate=omega if name=='turning' else 0.)
        if now>=next_update-1e-9:
            decision=session.update(state,platform,now,landing_requested=True)
            decisions.append({'time_s':now,'action':decision['action'],
                'reason':decision.get('reason'),'report':decision.get('report')})
            next_update=now+.4
            if decision['reference'] is None:
                outcome=decision['action'];break
        plan=session.plan
        if now>plan.arrival:
            outcome='REFERENCE_EXPIRED_WITHOUT_CONTACT';break
        ref=plan.at(now-plan.start)
        p=np.array(state.position);v=np.array(state.velocity);a=np.array(state.acceleration)
        # Known surrogate actuator response: inverse-lag feedforward. This time
        # constant is part of this numerical plant, not an identified FCU value.
        command=ref['acceleration']+.12*ref['jerk']+2.*(ref['position']-p)+2.5*(ref['velocity']-v)
        length=np.linalg.norm(command);command*=min(1.,planner.limits.acceleration/max(length,1e-12))
        jerk=(command-a)/.12
        jerk*=min(1.,planner.limits.jerk/max(np.linalg.norm(jerk),1e-12))
        # Constant jerk integration with an explicit 120ms acceleration response.
        pn=p+dt*v+.5*dt*dt*a+dt**3*jerk/6
        vn=v+dt*a+.5*dt*dt*jerk;an=a+dt*jerk
        state=FlightState(now+dt,tuple(pn),tuple(vn),tuple(an))
        pitch=math.atan2(an[0],9.80665+an[2]);roll=-math.atan2(an[1],math.hypot(an[0],9.80665+an[2]))
        pose=Pose((0,0,0),quaternion_from_rpy(roll,pitch,0.))
        body=RigidMotion(Pose(tuple(pn-np.array(pose.rotate(planner.offset))),pose.orientation_xyzw),tuple(vn),(0,0,0))
        aligned=platform.predict(dt);feet=evaluate_airframe(planner.geometry,body,aligned.deck())
        gap=min(f.clearance_m for f in feet)
        maximum_speed=max(maximum_speed,float(np.linalg.norm(vn)));maximum_accel=max(maximum_accel,float(np.linalg.norm(an)))
        rows.append({'time_s':now+dt,'com_position':pn.tolist(),'platform_position':list(aligned.position),
            'speed_m_s':float(np.linalg.norm(vn)),'acceleration_m_s2':float(np.linalg.norm(an)),
            'foot_gap_m':gap,'planned_arrival_s':plan.arrival})
        if gap<=0:
            # These are surrogate crossing metrics; there are no contact forces.
            centre=np.mean([f.point_world_m for f in feet],axis=0)
            error=float(np.linalg.norm((centre-np.array(aligned.position))[:2]))
            outcome='SURROGATE_PLANE_CROSSING'
            break
    return {'scenario':name,'outcome':outcome,'crossing_xy_error_m':error if outcome=='SURROGATE_PLANE_CROSSING' else None,
        'maximum_speed_m_s':maximum_speed,'maximum_acceleration_m_s2':maximum_accel,
        'physics_validated':False,'contact_dynamics_included':False,
        'angular_material_foot_speed_validated':False,'rows':rows,'decisions':decisions}


if __name__=='__main__':
    folder=ROOT/'outputs/receding_landing_20261023';folder.mkdir(parents=True,exist_ok=True)
    results=[]
    for name in ('constant_velocity','speed_change','turning'):
        result=run(name);results.append(result)
        print(name,result['outcome'],result['crossing_xy_error_m'],flush=True)
        (folder/(name+'_closed_loop.json')).write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    (folder/'closed_loop_summary.json').write_text(json.dumps([{k:v for k,v in r.items() if k not in ('rows','decisions')}
        for r in results],ensure_ascii=False,indent=2),encoding='utf-8')
