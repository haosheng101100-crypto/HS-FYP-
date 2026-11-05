"""Prepare a measured 5 m/s descending scene, brake car, record, exit and LAND.

This intentionally does not solve sudden-braking interception or land on the car.
"""
import json,math,time,sys
from dataclasses import asdict
from scene_live_io import LiveApproach as LiveIO,ROOT,Twist,mavutil
from landing.contact_geometry import Pose,RigidMotion,Deck,add,sub,scale,norm,evaluate_airframe
from landing.execution import BodyReference,ExecutionDecision
from landing.mavlink_reference import dispatch,LocalNedAlignment
from landing.sudden_brake_scene import SceneReferences,reference_from_axes


class PhysicalScene(LiveIO):
    def __init__(self):
        super().__init__();topic=self.result['verified_contact_publisher_topic']
        self.refs=SceneReferences();self.spec=self.refs.spec
        self.mission_start=None;self.descent_start=None;self.brake_epoch=None
        self.exit_start=None;self.exit_axes=None;self.hold_since=None;self.scene_done=False
        self.car_samples=[]
        self.result={'simulation_only':True,'scope':'One physical sudden-braking scene and snapshots; no landing decision algorithm',
            'completed':False,'landing_on_braking_platform_verified':False,'decision_algorithm_implemented':False,
            'pose_teleportation_used':False,'force_disarm_used':False,'hardware_deployment_allowed':False,
            'verified_contact_publisher_topic':topic,'scenario_specification':asdict(self.spec),
            'observations':'Causal fits of actual Gazebo physics poses; no vision, noise or real radio',
            'normal_disarm_observed':False}

    def deck_at(self,t):
        with self.lock:
            wall=time.monotonic();obs=self.car_fit.estimate(t,wall,False,'SIM_DECK',wall)
            self.car_fit_info=dict(self.car_fit.last_fit)
        a=tuple(self.car_fit_info['body_acceleration_world_m_s2']);dt=t-obs.sim_t_s
        assert 0<=dt<=.08
        top=add(obs.motion.pose.transform((0,0,.8)),add(scale(obs.motion.velocity_at_origin_world_m_s,dt),scale(a,.5*dt*dt)))
        velocity=add(obs.motion.velocity_at_origin_world_m_s,scale(a,dt))
        self.current_acceleration=a
        if not self.car_samples or obs.sim_t_s>self.car_samples[-1]['sample_sim_t_s']:
            self.car_samples.append({'sample_sim_t_s':obs.sim_t_s,'position_m':obs.motion.pose.transform((0,0,.8)),
                'velocity_m_s':obs.motion.velocity_at_origin_world_m_s,'acceleration_m_s2':a,'source':'causal raw physics fit; no clipping'})
        return Deck(RigidMotion(Pose(top,obs.motion.pose.orientation_xyzw),velocity,obs.motion.angular_velocity_world_rad_s),(1.,.7))

    def publish_car(self,t):
        cmd=Twist();cmd.linear.x=self.car_speed;self.car_pub.publish(cmd);self.last_car=t

    def pump(self):
        wall=time.monotonic()
        if wall-self.last_hb>=1:
            self.m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,mavutil.mavlink.MAV_AUTOPILOT_INVALID,0,0,0)
            self.m.mav.rc_channels_override_send(1,1,1500,1500,1000,1500,1800,1500,1500,1500);self.last_hb=wall
        if self.sim_t()-self.last_car>=.02:self.publish_car(self.sim_t())
        for _ in range(100):
            msg=self.m.recv_match(blocking=False)
            if msg is None:break
            if msg.get_type()=='BAD_DATA':continue
            self.fcu_capture.record(msg,time.monotonic(),self.clock[0] if self.clock else None)
            self.telemetry[msg.get_type()]=(msg,wall)
            if msg.get_type()=='HEARTBEAT':
                with self.lock:self.heartbeat_sim_t=min(self.sim_t(),self.clock[0]) if self.clock else self.sim_t()
            if msg.get_type()=='STATUSTEXT':self.event('FCU: '+msg.text)
        if self.enabled:
            if self.callback_error:raise RuntimeError('Observation callback: '+self.callback_error)
            with self.lock:latest=self.latest;clock=self.clock
            if latest is None or clock is None or wall-latest.received_monotonic_s>.5 or wall-clock[1]>1.:
                raise RuntimeError('Physical clock/pose reception stalled')
            t=min(latest.sim_t_s,clock[0])
            if self.last_control is None or t-self.last_control>=.039999:
                self.last_control=t;hb=self.telemetry.get('HEARTBEAT')
                if hb is None or self.heartbeat_sim_t is None or not 0<=t-self.heartbeat_sim_t<=1.5:
                    raise RuntimeError('Missing/stale mode heartbeat')
                with self.lock:
                    obs=self.history.estimate(t,time.monotonic(),self.armed(),self.mode(),hb[1])
                    fit=dict(self.history.last_fit);frame,event=self.contacts.poll(t,self.limits.contact_age_s)
                self.deck=self.deck_at(t);feet=evaluate_airframe(self.geometry,obs.motion,self.deck)
                tilt=math.degrees(math.acos(max(-1.,min(1.,obs.motion.pose.rotate((0,0,1))[2]))))
                if frame and frame.active or event and event.active:raise RuntimeError('Unexpected deck contact during scene preparation')
                if self.stage=='HOVER':
                    ref=BodyReference((0,0,4),(0,0,0),(0,0,0),0.)
                    ready=norm(sub(obs.motion.pose.position_world_m,ref.position_world_m))<.05 and norm(obs.motion.velocity_at_origin_world_m_s)<.05 and tilt<1.
                    self.hold_since=t if ready and self.hold_since is None else self.hold_since if ready else None
                    if self.hold_since is not None and t-self.hold_since>=.4:
                        self.mission_start=t;self.car_speed=5.;self.publish_car(t);self.stage='MATCH_SPEED';self.hold_since=None
                        self.event('Start physical 0-to-5 m/s preparation for aircraft and wheel-driven vehicle')
                elif self.stage in ('MATCH_SPEED','DESCENDING','BRAKE_OBSERVE'):
                    ref=self.refs.reference(t-self.mission_start,None if self.descent_start is None else t-self.descent_start)
                    dx=obs.motion.pose.position_world_m[0]-self.deck.motion.pose.position_world_m[0]
                    vx=obs.motion.velocity_at_origin_world_m_s[0];car_v=self.deck.motion.velocity_at_origin_world_m_s[0]
                    if self.stage=='MATCH_SPEED' and t-self.mission_start>=self.spec.acceleration_duration_s+self.spec.minimum_cruise_s:
                        if abs(vx-5)<.08 and abs(car_v-5)<.02 and abs(dx)<.2 and abs(obs.motion.velocity_at_origin_world_m_s[2])<.06 and tilt<1.5:
                            self.descent_start=t;self.stage='DESCENDING';self.event('Measured matched 5 m/s; begin actual descending approach')
                        if t-self.mission_start>12:raise RuntimeError('Unable to establish measured matched-speed scene')
                    if self.stage=='DESCENDING':
                        if feet[0].clearance_m<=1.52:
                            checks={'foot_gap':1.45<=feet[0].clearance_m<=1.52,'aircraft_speed':abs(vx-5)<.08,
                                'vehicle_speed':abs(car_v-5)<.02,'relative_x':abs(dx)<.2,
                                'descending':abs(obs.motion.velocity_at_origin_world_m_s[2]+.4)<.08,'upright':tilt<2.}
                            if not all(checks.values()):raise RuntimeError('Trigger state outside declared scene: '+str(checks))
                            self.brake_epoch=t;self.car_speed=0.;self.publish_car(t);self.stage='BRAKE_OBSERVE'
                            self.result['scenario_trigger']={'sim_t_s':t,'checks':checks,'aircraft':asdict(obs),'deck':asdict(self.deck),
                                'feet':[asdict(f) for f in feet],'body_reference':asdict(ref),'vehicle_brake_command_m_s':0.,
                                'vehicle_brake_limit_m_s2':-2.5,'aircraft_does_not_receive_brake_preview':True}
                            self.event('Measured 5 m/s descending scene reached: trigger physical car brake; keep original aircraft reference for 0.4 s observation')
                    if self.stage=='BRAKE_OBSERVE' and t-self.brake_epoch>=self.spec.observe_after_brake_s:
                        self.exit_axes=self.refs.fixed_exit(ref);self.exit_start=t;self.stage='FIXED_EXIT'
                        self.result['fixed_exit_start_sim_t_s']=t
                        self.event('Scene captured: execute predetermined clearance climb/deceleration, no interception decision')
                elif self.stage=='FIXED_EXIT':
                    u=min(self.spec.fixed_exit_duration_s,t-self.exit_start)
                    ref=reference_from_axes(self.exit_axes,u)
                    if u>=self.spec.fixed_exit_duration_s:
                        ready=norm(sub(obs.motion.pose.position_world_m,ref.position_world_m))<.12 and norm(obs.motion.velocity_at_origin_world_m_s)<.08
                        self.hold_since=t if ready and self.hold_since is None else self.hold_since if ready else None
                        if self.hold_since is not None and t-self.hold_since>=.5:self.scene_done=True
                    if t-self.exit_start>12:raise RuntimeError('Fixed scene exit did not settle')
                else:raise RuntimeError('Unknown active scene stage: '+self.stage)
                error=norm(sub(obs.motion.pose.position_world_m,ref.position_world_m))
                speed_error=norm(sub(obs.motion.velocity_at_origin_world_m_s,ref.velocity_world_m_s))
                # Takeoff enters HOVER while still ascending. The preparation
                # phase must settle before the steady-flight error gate applies.
                if (self.stage!='HOVER' and (error>.35 or speed_error>.6)) or tilt>20.:
                    raise RuntimeError('Physical reference tracking envelope exceeded')
                if self.stage=='HOVER' and t-self.prepare_start>60.:
                    raise RuntimeError('Hover preparation did not settle')
                if self.brake_epoch is not None and feet[0].clearance_m<.8:raise RuntimeError('Scene clearance below 0.8 m')
                decision=ExecutionDecision('SCENE_REFERENCE',False,reference=ref)
                sent=self.xy_feedback.update(decision,obs,t);dispatch(sent,self.m,self.request_land,self.alignment,t)
                row={'sim_t_s':t,'stage':self.stage,'observation':asdict(obs),'fit':fit,'measured_deck':asdict(self.deck),
                    'car_fit':self.car_fit_info,'car_command_m_s':self.car_speed,'shared_acceleration_m_s2':self.current_acceleration,
                    'feet':[asdict(f) for f in feet],'minimum_leg_gap_m':feet[0].clearance_m,'body_reference':asdict(ref),
                    'sent_reference':asdict(sent.reference),'xy_feedback':self.xy_feedback.diagnostics,
                    'contact_frame':asdict(frame) if frame else None,'new_contact_event':asdict(event) if event else None,
                    'tracking_position_error_m':error,'tracking_speed_error_m_s':speed_error,'tilt_deg':tilt}
                self.rows.append(row);(ROOT/'results/progress.json').write_text(json.dumps(row,allow_nan=False))
                if t-self.last_print>=2:
                    self.event('p='+str(tuple(round(v,3) for v in obs.motion.pose.position_world_m))+' v='+str(tuple(round(v,3) for v in obs.motion.velocity_at_origin_world_m_s))+' gap='+str(round(feet[0].clearance_m,3)))
                    self.last_print=t
        time.sleep(.005)

    def run(self):
        self.wait(lambda:'HEARTBEAT' in self.telemetry,60,'heartbeat')
        self.m.mav.param_set_send(1,1,b'GUID_OPTIONS',32.,mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        self.wait(lambda:'PARAM_VALUE' in self.telemetry and self.telemetry['PARAM_VALUE'][0].param_id=='GUID_OPTIONS' and self.telemetry['PARAM_VALUE'][0].param_value==32.,30,'GUID_OPTIONS acknowledgement')
        self.result['guid_options_confirmed']=32
        for mid in (30,32,33,245,83,85,62):self.command(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,mid,50000)
        self.wait(lambda:self.latest and self.car_history and 'LOCAL_POSITION_NED' in self.telemetry,180,'physical estimator')
        initial_t=self.sim_t();self.wait(lambda:self.sim_t()-initial_t>=12.,180,'sensor settling')
        self.m.set_mode(self.m.mode_mapping()['GUIDED']);self.wait(lambda:self.mode()=='GUIDED',30,'GUIDED')
        self.stage='ARM'
        for attempt in range(8):
            self.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,1)
            try:self.wait(self.armed,12,'normal arming');break
            except RuntimeError:
                if attempt==7:raise
        local=self.telemetry['LOCAL_POSITION_NED'][0]
        self.alignment=LocalNedAlignment.from_body_measurement(self.latest.pose.position_world_m,(local.x,local.y,local.z))
        self.stage='TAKEOFF';self.command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,0,0,0,0,0,0,4.)
        self.wait(lambda:self.latest.pose.position_world_m[2]>3.7,180,'physical takeoff')
        self.stage='HOVER';self.enabled=True;self.prepare_start=self.sim_t();self.event('Hold before physical scene preparation; no pose or velocity reset')
        self.wait(lambda:self.scene_done,500,'measured scene, brake observation and fixed exit')
        self.enabled=False;self.stage='GROUND_LAND';self.request_land()
        self.wait(lambda:not self.armed(),180,'normal ground LAND and automatic disarm')
        self.result['normal_disarm_observed']=True
        disarm_t=self.sim_t();self.wait(lambda:self.sim_t()-disarm_t>=3.,60,'post-disarm observation')
        self.result.update(completed=True,mission_start_sim_t_s=self.mission_start,descent_start_sim_t_s=self.descent_start,
            braking_start_sim_t_s=self.brake_epoch,final_phase='SCENE_COMPLETE',planned_ground_land_used=True,
            ground_landing_body_position_m=self.latest.pose.position_world_m)
        self.stage='COMPLETE'

    def save(self):
        self.result.update(external_failures=self.external_failures,events=self.events,armed_at_save=self.armed(),final_mode=self.mode())
        (ROOT/'results/execution_status.json').write_text(json.dumps(self.result,indent=2,allow_nan=False))
        with (ROOT/'results/execution_feedback.jsonl').open('w') as f:
            for row in self.rows:f.write(json.dumps(row,allow_nan=False)+'\n')
        (ROOT/'results/shared_platform_states.json').write_text(json.dumps(self.car_samples,indent=2,allow_nan=False))
        self.fcu_capture.save(ROOT/'results');self.car_speed=0.;self.car_pub.publish(Twist());self.m.close()


if __name__=='__main__':
    demo=PhysicalScene()
    try:demo.run()
    except Exception as exc:demo.emergency(exc)
    finally:demo.save()
    sys.exit(0 if demo.result['completed'] else 2)
