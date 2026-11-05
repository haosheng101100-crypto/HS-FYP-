"""Continuous one-way braking descent to actual deck contact.

No airborne hold, backward recenter or second descent. Actual physics must verify the reference.
"""
import json,math,time,sys
from collections import deque
from dataclasses import asdict
from scene_live_io import LiveApproach as LiveIO,ROOT,Twist,mavutil,stamp
from landing.contact_geometry import Pose,RigidMotion,Deck,add,sub,scale,norm,evaluate_airframe
from landing.execution import BodyReference,ExecutionDecision
from landing.mavlink_reference import dispatch,LocalNedAlignment
from landing.sudden_brake_scene import SceneReferences,reference_from_axes
from landing.attitude_braking import attitude_capture_plan,AttitudeCaptureController,send_attitude
from landing.abort_recovery import ForwardClearRecovery,predictive_tracking_guard
from landing.outcome_audit import classify_disarm
from landing.centre_landing import centre_metrics
from landing.platform_imu import ForwardAccelerationShare
from gz.msgs10.imu_pb2 import IMU
from landing.horizontal_feedback import HorizontalFeedback as ReactiveFeedback
from landing.accelerating_feedback import CaptureAwareFeedback
from landing.moving_platform import MovingLandingExecutor,MovingContactHandover


class PhysicalLandingCase(LiveIO):
    def __init__(self):
        super().__init__();topic=self.result['verified_contact_publisher_topic']
        self.refs=SceneReferences();self.spec=self.refs.spec;self.centre_observations=deque(maxlen=512)
        self.platform_imu=ForwardAccelerationShare();self.platform_imu_rows=[]
        def vehicle_imu_callback(msg):
            try:
                at=stamp(msg);a=(msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z)
                with self.lock:
                    car=next((p for p in reversed(self.car_history) if p.sim_t_s<=at),None)
                    if car is None:return
                    # The world-x component of rotated specific force has no
                    # gravity contribution. No gravity constant is fabricated.
                    ax=car.pose.rotate(a)[0];self.platform_imu.push(at,ax)
                    self.platform_imu_rows.append({'sample_sim_t_s':at,'specific_force_sensor_m_s2':a,'forward_acceleration_world_m_s2':ax})
            except Exception as exc:self.callback_error='Vehicle IMU: '+repr(exc)
        self.callbacks.append(vehicle_imu_callback)
        assert self.node.subscribe(IMU,'/vehicle/physical_imu',vehicle_imu_callback)

        self.car_fit.span=.103;self.attitude_controller=None;self.native_speed_limits={};self.attitude_command=None
        self.mission_start=None;self.descent_start=None;self.brake_epoch=None
        self.exit_start=None;self.exit_axes=None;self.hold_since=None;self.scene_done=False
        self.car_samples=[];self.reaction=None;self.reaction_t=None;self.landing_start=None;self.landing_executor=None;self.last_alignment_checks=None
        self.result={'simulation_only':True,'scope':'Same 5 m/s scene; direct native-attitude braking descent; deck-centre target; measured body-centre tolerance 0.15 m; no hover/reverse',
            'completed':False,'landing_on_braking_platform_verified':False,'decision_algorithm_implemented':True,'policy':'detect measured braking; estimate stopping point; direct continuous braking descent; no backward recovery',
            'pose_teleportation_used':False,'force_disarm_used':False,'hardware_deployment_allowed':False,
            'verified_contact_publisher_topic':topic,'scenario_specification':asdict(self.spec),
            'observations':'Causal fits of actual Gazebo physics poses; no vision, noise or real radio',
            'normal_disarm_observed':False}

    def deck_at(self,t):
        with self.lock:
            wall=time.monotonic();obs=self.car_fit.estimate(t,wall,False,'SIM_DECK',wall)
            self.car_fit_info=dict(self.car_fit.last_fit)
            imu_t,ax=self.platform_imu.sample_at(t)
            fitted=self.car_fit_info['body_acceleration_world_m_s2']
            if abs(ax)<.05 and abs(fitted[0])<.05:
                self.platform_imu.anchor(obs.sim_t_s,obs.motion.velocity_at_origin_world_m_s[0])
            vx=self.platform_imu.velocity(t);vpose=self.platform_imu.velocity(obs.sim_t_s)
        a=(ax,fitted[1],fitted[2]);dt=t-obs.sim_t_s
        assert 0<=dt<=.08
        v=(vpose,obs.motion.velocity_at_origin_world_m_s[1],obs.motion.velocity_at_origin_world_m_s[2])
        top=add(obs.motion.pose.transform((0,0,.8)),add(scale(v,dt),scale(a,.5*dt*dt)))
        velocity=(vx,v[1]+a[1]*dt,v[2]+a[2]*dt);self.current_acceleration=a
        if not self.car_samples or t>self.car_samples[-1]['query_sim_t_s']:
            self.car_samples.append({'query_sim_t_s':t,'sample_sim_t_s':imu_t,'pose_sample_sim_t_s':obs.sim_t_s,
                'position_m':top,'velocity_m_s':velocity,'acceleration_m_s2':a,'pose_fit_acceleration_m_s2':fitted,
                'source':'Physical vehicle IMU world-x shared causally, integrated vx with steady pose-fit anchors; y/z from causal pose fit'})
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
            if msg.get_type()=='PARAM_VALUE':self.native_speed_limits[msg.param_id]=msg.param_value
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
                self.deck=self.deck_at(t);self.latest_motion_for_centre=obs.motion;self.centre_observations.append((obs,self.deck));feet=evaluate_airframe(self.geometry,obs.motion,self.deck)
                tilt=math.degrees(math.acos(max(-1.,min(1.,obs.motion.pose.rotate((0,0,1))[2]))))
                if self.landing_executor is None and ((frame and frame.active) or (event and event.active)):raise RuntimeError('Unexpected deck contact before planned landing')
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
                                'feet':[asdict(f) for f in feet],'body_reference':asdict(ref) if ref else None,'vehicle_brake_command_m_s':0.,
                                'vehicle_brake_limit_m_s2':-2.5,'aircraft_does_not_receive_brake_preview':True}
                            self.event('Measured 5 m/s descending scene reached: trigger physical car brake; await measured braking detection then execute reactive deck landing')
                    if self.stage=='BRAKE_OBSERVE' and self.current_acceleration[0]<-2.2 and car_v<5.05:
                        self.reaction_t=t
                        self.result['preplan_measured_braking']={'sim_t_s':t,'delay_since_driver_command_s':t-self.brake_epoch,'deck':asdict(self.deck),'measured_acceleration_m_s2':self.current_acceleration}
                        self.curve,terminal,plan=attitude_capture_plan(self.geometry,self.profile,obs,self.deck,self.current_acceleration)
                        boundary=self.curve.terminal_boundary
                        self.landing_executor=MovingLandingExecutor(self.curve,self.geometry,terminal,self.entry,self.limits,None)
                        self.landing_executor.handover=MovingContactHandover(self.geometry,boundary,terminal,self.limits,self.deck_at)
                        self.attitude_controller=AttitudeCaptureController(self.curve,t,self.native_speed_limits['WP_SPD_UP'],self.native_speed_limits['WP_SPD_DN'],rate_feedforward=True,native_rate_mode=True,braking_tail_guard=True,measured_thrust_mapping=True)
                        self.landing_start=t;self.stage='DECK_LANDING';ref=None
                        self.result['measured_braking_detection']={'sim_t_s':t,'delay_since_driver_command_s':t-self.brake_epoch,
                            'measured_acceleration_m_s2':self.current_acceleration,'measured_vehicle_speed_m_s':car_v,
                            'policy':'Only current causal state; no trigger timestamp used by trajectory planner'}
                        plan['rate_feedforward_enabled']=True
                        plan['measured_thrust_mapping']={'enabled':True,'horizontal_command_limit_m_s2':3.5,'virtual_tilt_limit_deg':19.5,'original_actual_tilt_deg':20.,'original_actual_rate_deg_s':80.}
                        plan['terminal_coordination']={'own_horizontal_preview_s':.10,'braking_tail_preview_s':.10,'residual_forward_speed_m_s':.008,'native_release_time_s':.14,'continuous_descent_duration_s':3.,'identified_model':False}
                        plan['native_rate_outer_loop']={'enabled':True,'attitude_gain_per_s':4.5,'rate_damping':.3,'rate_limit_deg_s':70.,'type_mask':128,'native_gains_modified':False}
                        self.result['direct_braking_plan']=plan
                        self.result['landing_start']={'sim_t_s':t,'deck':asdict(self.deck),'terminal_boundary':asdict(boundary),'duration_s':self.curve.T}
                        self.event('Measured braking: begin ONE continuous descent/braking trajectory; no hold or backward recenter')
                elif self.stage=='DECK_LANDING':
                    ref=None
                else:raise RuntimeError('Unknown active scene stage: '+self.stage)
                decision=ExecutionDecision('SCENE_REFERENCE',False,reference=ref)
                if self.stage=='DECK_LANDING':
                    decision=self.landing_executor.step(t,obs,frame,event);ref=decision.reference
                    if decision.phase=='WAIT_ENTRY':raise RuntimeError('Attitude capture entry mismatch; reject instead of waiting airborne')
                    if self.landing_executor.start_s is not None:self.attitude_controller.start=self.landing_executor.start_s
                    h=self.landing_executor.handover
                    if h.first_contact and 'measured_centre_before_contact' not in self.result:
                        ct=h.first_contact['observation_sim_t_s']
                        co,cd=next((o,d) for o,d in reversed(self.centre_observations) if abs(o.sim_t_s-ct)<1e-9)
                        self.result['measured_centre_before_contact']={'observation_sim_t_s':ct,**centre_metrics(self.geometry,co.motion,cd)}
                    if h.phase=='WAIT_DISARM' and not self.land_authority_requested:
                        assert h.first_contact and all(h.last_support_checks.values()) and self.mode()=='LAND'
                        self.m.mav.param_set_send(1,1,b'PSC_ANGLE_MAX',5.,mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
                        self.land_authority_requested=True
                        self.result['supported_land_authority_transition']={'sim_t_s':t,'maximum_position_control_tilt_deg':5.,
                            'support_checks':dict(h.last_support_checks),'support_dwell_s':t-h.stable_since,'parameter_acknowledged':False}
                    if self.land_authority_requested and 'PARAM_VALUE' in self.telemetry:
                        param=self.telemetry['PARAM_VALUE'][0]
                        if param.param_id=='PSC_ANGLE_MAX' and abs(param.param_value-5.)<1e-5:self.result['supported_land_authority_transition']['parameter_acknowledged']=True
                    if h.phase=='FAILED' and obs.armed:raise RuntimeError('Deck contact/support verification failed: '+str(h.failures))
                    if not obs.armed:
                        if self.disarmed_since is None:self.disarmed_since=t
                    else:self.disarmed_since=None
                    if self.landing_executor.experiment_can_finish(t,obs.armed,self.disarmed_since):
                        self.scene_done=True;self.result['completed']=decision.complete
                error=None if ref is None else norm(sub(obs.motion.pose.position_world_m,ref.position_world_m))
                speed_error=None if ref is None else norm(sub(obs.motion.velocity_at_origin_world_m_s,ref.velocity_world_m_s))
                if ref is not None and self.stage=='DECK_LANDING' and self.landing_executor.handover.started is None:
                    guard=predictive_tracking_guard(ref,obs.motion)
                    if guard['reject_now']:
                        self.result['predictive_rejection']={'sim_t_s':t,'guard':guard,'observation':asdict(obs),'foot_gap_m':feet[0].clearance_m}
                        raise RuntimeError('Predictive tracking margin exhausted; abort before hard tracking limit')
                if ref is not None:
                    if self.stage!='HOVER' and (error>.35 or speed_error>.6):raise RuntimeError('Physical reference tracking envelope exceeded')
                if tilt>20.:raise RuntimeError('Actual tilt exceeds 20 degree simulation envelope')
                if self.stage=='HOVER' and t-self.prepare_start>60.:raise RuntimeError('Hover preparation did not settle')
                if self.stage=='DECK_LANDING' and self.landing_executor.handover.started is None:
                    if norm(obs.motion.angular_velocity_world_rad_s)>math.radians(80):raise RuntimeError('Actual angular speed exceeds new interface test bound 80 deg/s')
                    if obs.motion.velocity_at_origin_world_m_s[0]<-.05:raise RuntimeError('One-way braking violated: actual backward flight')
                    if obs.motion.velocity_at_origin_world_m_s[2]>-.15:raise RuntimeError('Continuous descent violated: actual airborne stop/climb')
                self.attitude_command=None
                if self.attitude_controller is not None and decision.reference is not None and not decision.stop_reference:
                    self.attitude_command=self.attitude_controller.command(decision.reference,obs,t)
                    send_attitude(self.attitude_command,self.m,t);sent=decision
                else:
                    sent=self.xy_feedback.update(decision,obs,t);dispatch(sent,self.m,self.request_land,self.alignment,t)
                row={'sim_t_s':t,'stage':self.stage,'observation':asdict(obs),'fit':fit,'measured_deck':asdict(self.deck),
                    'car_fit':self.car_fit_info,'car_command_m_s':self.car_speed,'shared_acceleration_m_s2':self.current_acceleration,
                    'feet':[asdict(f) for f in feet],'minimum_leg_gap_m':feet[0].clearance_m,'body_reference':asdict(ref) if ref else None,
                    'sent_reference':asdict(sent.reference) if sent.reference else None,'xy_feedback':self.xy_feedback.diagnostics if self.attitude_controller is None else None,'attitude_command':self.attitude_command,
                    'contact_frame':asdict(frame) if frame else None,'new_contact_event':asdict(event) if event else None,
                    'tracking_position_error_m':error,'tracking_speed_error_m_s':speed_error,'tilt_deg':tilt,'decision':asdict(decision),'alignment_checks':self.last_alignment_checks,'support_checks':self.landing_executor.handover.last_support_checks if self.landing_executor else None}
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
        for name,expected in [('WP_SPD_UP',2.5),('WP_SPD_DN',1.5)]:
            self.m.mav.param_request_read_send(1,1,name.encode(),-1)
            self.wait(lambda:name in self.native_speed_limits,20,'verify native climb speed '+name)
            if abs(self.native_speed_limits[name]-expected)>1e-5:raise RuntimeError('Native climb field scale mismatch: '+name)
        self.result['verified_native_climb_limits_m_s']={name:self.native_speed_limits[name] for name in ['WP_SPD_UP','WP_SPD_DN']}
        self.result['attitude_interface_firmware_reference']='ArduCopter 4.7.1 dbe79216'
        self.result['pva_jerk_snap_envelope_does_not_describe_new_interface']=True

        for mid in (30,32,33,245,83,85,62):self.command(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,mid,50000)
        self.wait(lambda:self.latest and self.car_history and self.platform_imu.samples and 'LOCAL_POSITION_NED' in self.telemetry,180,'physical aircraft/vehicle IMU estimator')
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
        self.wait(lambda:self.scene_done,650,'reactive braking, actual deck contact, LAND, normal disarm and support')
        h=self.landing_executor.handover
        self.result.update(normal_disarm_observed=not self.armed(),first_contact=h.first_contact,failures=h.failures,
            landing_on_braking_platform_verified=bool(self.result['completed']),mission_start_sim_t_s=self.mission_start,
            descent_start_sim_t_s=self.descent_start,braking_start_sim_t_s=self.brake_epoch,
            main_approach_start_sim_t_s=self.landing_executor.start_s,final_phase=h.phase,
            contact_limits=asdict(self.limits),planned_ground_land_used=False,
            continuous_braking_descent_required=True)
        self.result['measured_centre_after_support']=centre_metrics(self.geometry,self.latest_motion_for_centre,self.deck)
        self.result['completed']=self.result['completed'] and self.result['measured_centre_after_support']['body_centre_passed'] and self.result['measured_centre_before_contact']['body_centre_passed']
        self.enabled=False;self.stage='COMPLETE' if self.result['completed'] else 'FAILED'

    def emergency(self,error):
        self.running=True;self.enabled=False;self.car_speed=0.;self.external_failures.append(str(error));self.result['completed']=False
        self.result['recovery_platform_braking_used']=True;self.result['recovery_rows']=[]
        self.event('Rejected landing: '+str(error)+'; recovery is not deck-landing success')
        if self.armed():
            try:
                self.stage='RECOVERY_CLEARANCE';t0=self.sim_t();last=None;recovery=None;ready=False
                while self.armed() and self.sim_t()-t0<35.:
                    self.pump();t=self.sim_t()
                    if last is not None and t-last<.039999:continue
                    hb=self.telemetry.get('HEARTBEAT');wall=time.monotonic()
                    if hb is None:raise RuntimeError('Missing recovery flight heartbeat')
                    with self.lock:obs=self.history.estimate(t,wall,self.armed(),self.mode(),hb[1])
                    d=self.deck_at(t)
                    tilt=math.degrees(math.acos(max(-1.,min(1.,obs.motion.pose.rotate((0,0,1))[2]))))
                    gap=evaluate_airframe(self.geometry,obs.motion,d)[0].clearance_m
                    if recovery is None:
                        if gap<.5 or tilt>25.:raise RuntimeError('Not inside airborne recovery entry envelope')
                        recovery=ForwardClearRecovery(obs.motion)
                    cmd,checks=recovery.step(t,obs.motion,d.motion.pose.position_world_m[0]);send_attitude(cmd,self.m,t);last=t
                    self.result['recovery_rows'].append({'sim_t_s':t,'observation':asdict(obs),'deck':asdict(d),'command':cmd,'checks':checks,'foot_gap_m':gap})
                    if obs.motion.pose.position_world_m[2]>5. or tilt>25.:raise RuntimeError('Recovery flight envelope exceeded')
                    if checks['ready_for_native_land']:ready=True;break
                self.result['recovery_clearance_completed']=ready
                if not ready:raise RuntimeError('Recovery did not reach clear stable native-LAND entry')
            except Exception as exc:self.external_failures.append('recovery clearance: '+str(exc))
            self.stage='RECOVERY_LAND';self.request_land()
            try:self.wait(lambda:not self.armed(),180,'recovery LAND disarm, independently check no crash')
            except Exception as exc:self.external_failures.append('recovery LAND: '+str(exc))
        self.result.update(classify_disarm(self.armed(),self.events,self.mode()))
        self.result['completed']=False

    def save(self):
        self.result.update(classify_disarm(self.armed(),self.events,self.mode()))
        (ROOT/'results/vehicle_imu_shared_packets.json').write_text(json.dumps(self.platform_imu_rows,allow_nan=False))
        if self.landing_executor is not None:
            self.result.update(first_contact=self.landing_executor.handover.first_contact,failures=self.landing_executor.handover.failures,main_approach_start_sim_t_s=self.landing_executor.start_s,contact_limits=asdict(self.limits))
        self.result.update(external_failures=self.external_failures,events=self.events,armed_at_save=self.armed(),final_mode=self.mode())
        (ROOT/'results/execution_status.json').write_text(json.dumps(self.result,indent=2,allow_nan=False,default=lambda v:asdict(v)))
        with (ROOT/'results/execution_feedback.jsonl').open('w') as f:
            for row in self.rows:f.write(json.dumps(row,allow_nan=False)+'\n')
        (ROOT/'results/shared_platform_states.json').write_text(json.dumps(self.car_samples,indent=2,allow_nan=False))
        self.fcu_capture.save(ROOT/'results');self.car_speed=0.;self.car_pub.publish(Twist());self.m.close()


if __name__=='__main__':
    demo=PhysicalLandingCase()
    try:demo.run()
    except Exception as exc:demo.emergency(exc)
    finally:demo.save()
    sys.exit(0 if demo.result['completed'] else 2)
