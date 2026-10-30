"""Actual Gazebo -> asynchronous planner -> MAVLink -> native contact handover."""
from pathlib import Path
import sys,os,json,time,math
from dataclasses import asdict
from types import SimpleNamespace
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'tools')]
from operator_legacy import OperatorApproach as LegacyOperator,SESSION
from interactive_base import LiveApproach
from operator_clock import allowed_actions
from landing.execution import BodyReference,ExecutionDecision,HandoverLimits,assess_entry,attitude_error
from landing.contact_geometry import Pose,Deck,RigidMotion,norm,sub,evaluate_airframe,quaternion_from_rpy
from landing.swept_envelope import SweptEnvelope
from landing.horizontal_feedback import HorizontalFeedback
from landing.platform_prediction import estimate_platform,delayed_noisy_platform,synthetic_pose_samples
from landing.platform_filter import CausalPlatformKalman
from landing.planar_landing import PlanarLandingSupervisor,inflate_forecast_tube
from landing.planar_motion import PlanarMotionCommand,desired_planar_velocity
from landing.prediction_uncertainty import DelayedForecastMonitor
from landing.receding_landing import FlightState,PlatformState
from landing.receding_live import moving_alignment_required,initial_plan_durations,native_capture_time_guard,tracking_contact_envelope
from landing.receding_live import FilePlannerClient,AsyncLandingReference,RecoveryReference,FinalCommandLimiter,atomic_json,unpack_plan,terminal_body_prediction,braking_clearance_design,measured_frame_reference,terminal_feedback_gains,frame_velocity_reserve_check,contact_heading_acceleration_guard,terminal_capture_window,DeckAlignmentReference,execution_feedback_gains
from landing.moving_platform import MovingContactHandover,MovingLandingExecutor
from landing.touchdown_boundary import TouchdownRequest,ContactDesignLimits,solve_touchdown_boundary
from landing.mavlink_reference import dispatch,LocalNedAlignment
from pymavlink import mavutil
from gz.msgs10.twist_pb2 import Twist
from gz.msgs10.world_control_pb2 import WorldControl
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.double_pb2 import Double


class OperatorApproach(LegacyOperator):
    def __init__(self):
        self.measured_paused=None;self.pause_report_wall=-math.inf
        self.live_control=False;self.reference_session=None;self.recovery=None;self.attempt=0
        self.scenario='constant';self.scenario_start=None;self.car_omega=0.;self.last_car_sim=None
        self.planner_message='后台优化器已预热，等待起飞与降落指令'
        self.terminal_prediction=None
        self.scene_gap=None;self.surge_start=None
        self.pending_preparation=False;self.attempt_history=[];self.control_stream=None
        self.alignment_reference=None;self.alignment_started=None
        super().__init__()
        # Restore the original contact limits; the old rapid demonstration
        # changed the allowed closing speed and must not leak into this runtime.
        self.cfg=json.loads((ROOT/'config/experiments/execution_v1.json').read_text())
        self.live_cfg=json.loads((ROOT/'config/experiments/receding_live_v1.json').read_text())
        self.planar_design=self.live_cfg.get('planar_landing')
        self.planar_supervisor=None;self.last_sent_reference=None;self.planar_report=None
        self.planar_motion=PlanarMotionCommand() if self.planar_design else None
        if self.planar_design:
            self.planar_pubs=[self.node.advertise('/vehicle/planar_'+axis+'/cmd_vel',Double) for axis in ('x','y')]
        if self.live_cfg.get('tracking_error_feedback') and not self.live_cfg.get('measured_frame_terminal_control'):
            raise ValueError('Tracking envelope feedback requires the actually corrected platform frame')
        self.forecast_monitor=DelayedForecastMonitor(**self.live_cfg.get('prediction_uncertainty',{})) if self.live_cfg.get('prediction_uncertainty') else None
        self.forecast_tube=None
        self.platform_filter=CausalPlatformKalman(**self.live_cfg['platform_filter']) if self.live_cfg.get('platform_filter') else None
        self.scenario=os.environ.get('HS_TRIAL_SCENARIO',self.live_cfg['default_scenario'])
        self.result['scenario']=self.scenario
        self.envelope=SweptEnvelope.load(ROOT/self.live_cfg['envelope_profile']) if self.live_cfg.get('envelope_profile') else None
        if self.live_cfg.get('horizontal_feedback'):
            self.xy_feedback=HorizontalFeedback(**self.live_cfg['horizontal_feedback'])
            terminal_feedback_gains(self.live_cfg['horizontal_feedback'],self.live_cfg.get('terminal_horizontal_feedback'))
            self.result['horizontal_feedback_design']=self.live_cfg['horizontal_feedback']
        self.limits=HandoverLimits(**self.cfg['handover_limits'])
        self.hold_reference=BodyReference.from_state(self.lead.at(0))
        self.result.update(controller='asynchronous three-dimensional Bezier SOCP; causal platform acceleration; native LAND contact',
                           flight_validation_status='NOT_YET_FLIGHT_VALIDATED',contact_limits=asdict(self.limits))
        self.client=FilePlannerClient(ROOT/'runtime/receding_planner')
        self.command_limiter=FinalCommandLimiter(self.live_cfg['planner_limits'],self.live_cfg['control_interval_s'],self.live_cfg.get('final_command_acceleration_reserve_m_s2',0.))
        warm=unpack_plan(json.loads((ROOT/'runtime/receding_planner/warm_plan.json').read_text()))
        warm.body_reference_at(warm.start+.1)  # load geometry kernels before arming
        self.last_cycle_wall_s=0.
        self.control_stream=(ROOT/'results/receding_control.jsonl').open('x',buffering=1)

    def world_stats(self,msg):
        self.measured_paused=bool(msg.paused);self.pause_report_wall=time.monotonic()
        self.operator_clock.set_paused(self.measured_paused)

    def pause_world(self,pause):
        started=time.monotonic()
        ok,response=self.node.request('/world/uav_vehicle_stage1/control',WorldControl(pause=pause),WorldControl,Boolean,5000)
        confirmed=ok and response.data
        if not confirmed:
            # A slow GUI can delay the service response even after physics has
            # changed. Require a fresh actual WorldStatistics acknowledgement.
            deadline=time.monotonic()+10.
            while time.monotonic()<deadline:
                confirmed=self.measured_paused==pause and self.pause_report_wall>=started
                if confirmed:break
                time.sleep(.02)  # let transport callback threads publish the ack
        if not confirmed:raise RuntimeError('Gazebo pause/resume not confirmed by service or fresh world stats')
        self.operator_clock.set_paused(pause);self.event('Operator confirmed physics '+('PAUSE' if pause else 'RESUME'))

    def process_actions(self):
        for p in sorted((SESSION/'requests').glob('*.json')):
            req=json.loads(p.read_text())
            if req.get('action')=='scenario':
                if self.operator_stage in ('READY','HOVER') and req.get('value') in self.live_cfg.get('scenarios',{}):
                    self.scenario=req['value'];self.operator_message='已选择平台运动方式'
                p.rename(SESSION/'processed'/(ROOT.name+'__'+p.name))
        super().process_actions()

    def deck_at(self,t):
        d=super().deck_at(t)
        return Deck(d.motion,tuple(self.live_cfg.get('deck_half_extents',(1.,.7))))

    def publish_status(self,force=False):
        now=time.monotonic()
        if not force and now-self.last_status<.2:return
        self.last_status=now
        atomic_json(SESSION/'status.json',dict(stage=self.operator_stage,message=self.operator_message,armed=self.armed(),
            paused=self.operator_clock.paused,sim_t_s=self.sim_t(),body_height_m=self.latest.pose.position_world_m[2] if self.latest else None,
            buttons=allowed_actions(self.operator_stage,self.armed(),self.operator_clock.paused),cycle=ROOT.name,run=str(ROOT),
            real_wall_unix_s=time.time(),manual_takeoff_required=os.environ.get('HS_AUTONOMOUS_TRIAL')!='1',manual_landing_required=os.environ.get('HS_AUTONOMOUS_TRIAL')!='1',
            planner_message=self.planner_message,scenario=self.scenario,algorithm='ASYNC_BEZIER_SOCP',
            planner_phase=self.reference_session.phase if self.reference_session else None))

    def pump(self):
        self.process_actions()
        self.enabled=False
        self.last_car=math.inf  # only update_vehicle owns the full x/yaw command
        LiveApproach.pump(self)  # telemetry, GCS heartbeat; never old landing execution
        if self.live_control and not self.operator_clock.paused:self.control_step()
        self.publish_status()
        if self.close_requested:raise RuntimeError('Operator closed control panel')

    def make_reference_session(self,hold,moving_start=None):
        return AsyncLandingReference(self.client,self.curve.com_offset_body_m,hold,lead_s=self.live_cfg['future_activation_lead_s'],maximum_extension_s=self.live_cfg.get('arrival_extension_budget_s',1.),extension_window_s=self.live_cfg.get('arrival_extension_window_s'),initial_durations=initial_plan_durations(self.live_cfg,aligned=moving_start is not None),early_extension_s=self.live_cfg.get('early_arrival_extension_budget_s',0.),moving_hold_start_s=moving_start)

    def observation_stress(self):
        # The measurement producer owns this metadata; preparation consumes
        # exactly the same quality description as the causal pose predictor.
        return self.live_cfg['synthetic_platform_observation_stress'] if self.scenario=='noisy_maneuvering' else None

    def begin_attempt(self,t,obs,deck,platform):
        self.attempt+=1;self.land_requested=False;self.pending_preparation=False
        self.reference_session=self.make_reference_session(self.hold_reference)
        self.planar_supervisor=PlanarLandingSupervisor(self.planar_design) if self.planar_design else None
        closing=self.live_cfg['planner_limits']['contact_speed']
        req=TouchdownRequest(quaternion_from_rpy(0,0,-platform.yaw),(0,0,-platform.yaw_rate),(0,0),(0,0,-closing))
        boundary=solve_touchdown_boundary(self.geometry,deck,req,ContactDesignLimits(.01,1.01,.05,.5*closing,1.5*closing,.02))
        handover=MovingContactHandover(self.geometry,boundary,deck,self.limits,self.deck_at)
        # Compatibility fields used by the recorder and original finish policy.
        self.executor=SimpleNamespace(handover=handover,start_s=t,approach_entry_report=self.latest_entry)
        self.scenario_start=t;self.last_car_sim=t;self.operator_stage='LANDING';self.stage='RECEDING_EXECUTION'
        self.operator_message='新算法执行中：预测移动平台，更新拦截与下降轨迹'
        self.result['completed']=False;self.disarmed_since=None;self.xy_feedback.last_t=None
        self.result['execution_mode']='user_authorized_autonomous_simulation_trial' if os.environ.get('HS_AUTONOMOUS_TRIAL')=='1' else 'manual_panel'
        quality=self.observation_stress()
        alignment_required=bool(self.planar_design) or moving_alignment_required(self.live_cfg.get('moving_alignment'),quality)
        self.reference_session.events.append(dict(time=t,event='PREPARATION_POLICY',observation_stress=quality,moving_alignment_required=alignment_required))
        if alignment_required:
            cfg=self.live_cfg['moving_alignment']
            self.alignment_reference=DeckAlignmentReference(self.hold_reference,self.curve.com_offset_body_m,cfg['minimum_leg_gap_m'],cfg['dwell_s'])
            self.alignment_started=t;self.reference_session.phase='ALIGNING';self.operator_stage='ALIGNING'
            self.operator_message='先在安全高度追近并跟稳平台，实测收敛后再下降'
            if self.planar_design:self.start_planar_follow(t,'PLANAR_INITIAL_FOLLOW')
            self.reference_session.events.append(dict(time=t,event='ALIGNMENT_STARTED',initial_reference=asdict(self.hold_reference)))
        self.event('New async planner activated by explicit user landing request; scenario='+self.scenario)

    def platform_state(self,t,deck):
        q=deck.motion.pose.orientation_xyzw;x,y,z,w=q
        yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
        # Fixed-height deck model: measure and gate vertical deviations rather
        # than passing unsupported vertical motion to the planar predictor.
        if abs(deck.motion.velocity_at_origin_world_m_s[2])>.02 or abs(deck.motion.pose.rotate((0,0,1))[2]-1.)>1e-4:
            raise RuntimeError('Measured deck left fixed-height level model')
        acc=list(self.car_fit.last_fit['body_acceleration_world_m_s2']);acc[2]=0.
        velocity=list(deck.motion.velocity_at_origin_world_m_s);velocity[2]=0.
        if self.live_cfg.get('short_horizon_cubic_prediction'):
            with self.lock:samples=list(self.car_history)
            if self.platform_filter:
                stress=self.observation_stress()
                cutoff=t-(stress['delay_s'] if stress else 0.);span=self.live_cfg.get('platform_fit_span_s',.65)
                rows=[s for s in samples if cutoff-span<=s.sim_t_s<=cutoff]
                self.last_platform_fit_input=[asdict(s) for s in rows]
                if stress:
                    rows=synthetic_pose_samples(rows,cutoff,span,stress['position_sigma_m'],stress['yaw_sigma_rad'],stress['seed'])
                return self.platform_filter.update(rows,t,deck.half_extents_m)
            if self.observation_stress():
                stress=self.observation_stress()
                cutoff=t-stress['delay_s'];span=self.live_cfg.get('platform_fit_span_s',.65)
                self.last_platform_fit_input=[asdict(s) for s in samples if cutoff-span<=s.sim_t_s<=cutoff]
                return delayed_noisy_platform(samples,t,deck.half_extents_m,
                    span=span,**stress)
            return estimate_platform(samples,t,deck.half_extents_m,span=self.live_cfg.get('platform_fit_span_s',.35))
        return PlatformState(t,deck.motion.pose.position_world_m,tuple(velocity),tuple(acc),yaw,deck.motion.angular_velocity_world_rad_s[2],deck.half_extents_m)

    def update_vehicle(self,t):
        elapsed=0. if self.scenario_start is None else t-self.scenario_start
        if self.planar_motion is not None:
            target=(0.,0.) if self.scenario_start is None else desired_planar_velocity(self.scenario,elapsed)
            velocity,acceleration=self.planar_motion.step(t,target)
            for publisher,value in zip(self.planar_pubs,velocity):publisher.publish(Double(data=value))
            self.planar_command=dict(velocity_world_xy_m_s=velocity,acceleration_world_xy_m_s2=acceleration,planner_knows_future_commands=False)
            self.car_speed=math.hypot(*velocity);self.car_omega=0.;self.last_car_sim=t
            self.last_car=self.operator_clock.monotonic()
            return
        acceleration_cap=.12
        if self.scenario_start is None:target=0.;omega=0.
        else:
            target=.3;omega=0.
            if self.scenario=='varying':target=.3+.12*math.sin(.6*elapsed)
            if self.scenario=='turning':omega=.025*min(1.,elapsed/3.)
            if self.scenario in ('maneuvering','noisy_maneuvering'):
                target=.3+.12*math.sin(.6*elapsed+.7)
                omega=.025*math.sin(.5*elapsed)
            if self.scenario=='fast_maneuvering':
                target=.65+.15*math.sin(.4*elapsed)
                omega=.035*math.sin(.5*elapsed)
            if self.scenario=='switchback':
                target=.45 if int(elapsed/2.)%2==0 else .25
                omega=.025 if int(elapsed/3.)%2==0 else -.025
            if self.scenario=='braking':
                target=.6
                if self.surge_start is None and self.scene_gap is not None and self.scene_gap<=1.:
                    self.surge_start=t
                    atomic_json(ROOT/'results/scene_event.json',dict(event='BRAKE',sim_t_s=t,measured_leg_gap_m=self.scene_gap,trigger_gap_m=1.,command_acceleration_cap_m_s2=.3,target_speed_m_s=0.,planner_knows_future_commands=False))
                if self.surge_start is not None:target=0.;acceleration_cap=.3
            if self.scenario=='surge':
                if self.surge_start is None and self.scene_gap is not None and self.scene_gap<=.8:
                    self.surge_start=t
                    atomic_json(ROOT/'results/scene_event.json',dict(event='SURGE',sim_t_s=t,measured_leg_gap_m=self.scene_gap,command_acceleration_cap_m_s2=.8,target_speed_m_s=2.5,planner_knows_future_commands=False))
                if self.surge_start is not None:target=2.5;acceleration_cap=.8
        dt=0. if self.last_car_sim is None else max(0.,t-self.last_car_sim)
        self.car_speed+=max(-acceleration_cap*dt,min(acceleration_cap*dt,target-self.car_speed));self.car_omega=omega;self.last_car_sim=t
        cmd=Twist();cmd.linear.x=self.car_speed;cmd.angular.z=omega;self.car_pub.publish(cmd)
        # Stop the legacy telemetry pump from overwriting angular velocity.
        self.last_car=self.operator_clock.monotonic()

    def start_recovery(self,t,obs,reason):
        self.alignment_reference=None
        self.recovery_started=t
        self.reference_session.cancel(reason);self.recovery=RecoveryReference(obs,home=self.hold_reference.position_world_m)
        self.operator_stage='RECOVERING';self.operator_message='本次降落退出：正在制止下降、升高并返回起飞点悬停'
        self.planner_message='退出原因：'+reason
        self.result['completed']=False;self.event('Failed landing attempt; brake/climb/return: '+reason)

    def start_planar_follow(self,t,reason):
        """Invalidate pending descent and continue from the actually sent PVA."""
        initial=self.last_sent_reference or self.hold_reference
        old=self.reference_session
        old.cancel(reason)
        self.reference_session=self.make_reference_session(initial)
        self.reference_session.events=old.events+[dict(time=t,event=reason,initial_reference=asdict(initial))]
        cfg=self.live_cfg['moving_alignment']
        self.alignment_reference=DeckAlignmentReference(initial,self.curve.com_offset_body_m,cfg['minimum_leg_gap_m'],cfg['dwell_s'],start_s=t,reserve_vertical=True)
        self.alignment_started=t;self.reference_session.phase='PLANAR_FOLLOW';self.operator_stage='ALIGNING'
        self.terminal_prediction=None

    def update_planar_policy(self,t,obs,platform,active_contact):
        if self.planar_supervisor is None or self.recovery is not None:return None
        state=FlightState.from_observation(obs,self.curve.com_offset_body_m)
        descending=self.alignment_reference is None
        report=self.planar_supervisor.update(t,state,platform,self.scene_gap,
            aligned=descending,descending=descending,
            contact=bool(active_contact or self.executor.handover.started is not None),braking_reference=self.last_sent_reference)
        report['inputs']=dict(aligned=descending,descending=descending,gap_m=self.scene_gap,
            contact=bool(active_contact or self.executor.handover.started is not None),
            braking_reference=asdict(self.last_sent_reference) if self.last_sent_reference is not None else None)
        if report['action']=='EXIT':self.start_recovery(t,obs,report['reason'])
        elif report['action']=='REACQUIRE':self.start_planar_follow(t,'PLANAR_REACQUIRE')
        return report

    def control_step(self):
        cycle_started=time.perf_counter()
        wall=self.operator_clock.monotonic()
        with self.lock:latest=self.latest;clock=self.clock
        if latest is None or clock is None or wall-latest.received_monotonic_s>.5 or wall-clock[1]>1.:
            raise RuntimeError('Live simulation clock/body reception stalled')
        t=min(latest.sim_t_s,clock[0])
        if self.last_control is not None and t-self.last_control<.039999:return
        if self.last_control is not None and t-self.last_control>.080001:
            raise RuntimeError(f'Live control update gap: sim={t-self.last_control:.6f}, previous_cycle_wall={self.last_cycle_wall_s:.6f}')
        self.last_control=t
        if self.heartbeat_sim_t is None or not 0<=t-self.heartbeat_sim_t<=1.5:raise RuntimeError('Mode heartbeat stale')
        with self.lock:
            hb=self.telemetry['HEARTBEAT'];sample_wall=self.operator_clock.monotonic()
            obs=self.history.estimate(t,sample_wall,self.armed(),self.mode(),hb[1])
            frame,event=self.contacts.poll(t,self.limits.contact_age_s)
            fit=dict(self.history.last_fit)
        deck=self.deck_at(t);self.deck=deck;platform=self.platform_state(t,deck)
        if self.forecast_monitor:self.forecast_tube=self.forecast_monitor.update(platform)
        self.scene_gap=min(f.clearance_m for f in evaluate_airframe(self.geometry,obs.motion,deck))
        if self.land_requested:
            self.pending_preparation=True;self.land_requested=False
            self.preparation_requested_at=t
            self.operator_stage='PREPARING';self.operator_message='等待飞机稳定悬停，随后启动新规划器'
        if self.pending_preparation:
            self.latest_entry=assess_entry(t,obs,self.lead.at(0),self.curve.com_offset_body_m,self.entry,frame)
            if self.latest_entry['ready']:self.begin_attempt(t,obs,deck,platform)
            elif t-self.preparation_requested_at>60.:raise RuntimeError('Measured preparation did not settle within 60 simulation seconds')
        decision=ExecutionDecision('USER_HOVER',False,self.hold_reference)
        nominal_reference=None;terminal_frame_correction=None;frame_velocity_guard=dict(active=False,passed=True);heading_guard=dict(active=False,passed=True);capture_window=dict(active=False,ready=False);alignment_report=None
        tracking_envelope=None
        active_contact=event and event.active or frame and frame.active
        if self.reference_session:
            hand=self.executor.handover
            self.planar_report=self.update_planar_policy(t,obs,platform,active_contact)
            if self.planar_report:
                self.forecast_tube=inflate_forecast_tube(self.forecast_tube,self.planar_report.get('motion'))
            # The near-level contact/support design cannot track sustained deck
            # acceleration requiring more tilt than its original contact gate.
            # Reject early while the brake/climb option still has clearance.
            contact_acceleration=9.80665*math.tan(self.limits.contact_attitude_error_rad)
            if not active_contact and hand.started is None and self.recovery is None and self.scene_gap<=1. and obs.motion.velocity_at_origin_world_m_s[2]<0:
                heading_guard=contact_heading_acceleration_guard(platform,self.live_cfg['maximum_contact_yaw_acceleration_rad_s2'])
                if not heading_guard['passed']:self.start_recovery(t,obs,'CONTACT_YAW_ACCELERATION_RESERVE')
            if not active_contact and hand.started is None and self.recovery is None and self.scene_gap<=1. and obs.motion.velocity_at_origin_world_m_s[2]<0 and math.hypot(*platform.acceleration[:2])>contact_acceleration:
                self.start_recovery(t,obs,'CONTACT_ACCELERATION_RESERVE')
            if not active_contact and hand.started is None and self.recovery is None and self.scene_gap<=1. and obs.motion.velocity_at_origin_world_m_s[2]<0 and self.reference_session.plan is not None and self.live_cfg.get('measured_frame_terminal_control') and self.live_cfg.get('maximum_frame_velocity_correction_m_s'):
                prior=self.reference_session.plan
                if 0<prior.arrival-t<5.:
                    _,correction=measured_frame_reference(prior,self.reference_session.model,platform,t,prior.body_reference_at(t))
                    frame_velocity_guard=frame_velocity_reserve_check(correction,self.live_cfg['maximum_frame_velocity_correction_m_s'])
                    if not frame_velocity_guard['passed']:self.start_recovery(t,obs,'FRAME_VELOCITY_RESERVE')
            if (active_contact or hand.started is not None) and self.envelope and self.envelope.clearance(obs.motion,deck)<self.limits.footprint_margin_m:
                hand.fail('whole_aircraft_outside_deck_margin')
            # First process real contact, before expiry and background results.
            if active_contact:
                self.reference_session.touch()
                if self.recovery:hand.fail('contact_during_failed_attempt_recovery');self.recovery=None
            if hand.started is not None or active_contact:
                decision=hand.step(t,obs,frame,event)
            elif self.alignment_reference is not None:
                local_center=self.envelope.centered_body_xy_in_deck(platform.deck()) if self.envelope else (0.,0.)
                center=platform.deck().motion.pose.rotate((*local_center,0.))
                ref=self.alignment_reference.step(t,platform,center)
                state=FlightState.from_observation(obs,self.curve.com_offset_body_m)
                angle=attitude_error(obs.motion.pose,Pose((0,0,0),(0,0,0,1)))
                alignment_report=self.alignment_reference.readiness(t,state,platform,center,angle,norm(obs.motion.angular_velocity_world_rad_s))
                target=dict(motion=RigidMotion(Pose(ref.position_world_m,(0,0,0,1)),ref.velocity_world_m_s,(0,0,0)),com_acceleration_world_m_s2=ref.acceleration_world_m_s2)
                entry=assess_entry(t,obs,target,self.curve.com_offset_body_m,self.entry,frame)
                alignment_report['original_entry']=entry
                planar_ready=self.planar_report is None or self.planar_report.get('descent_eligible',False)
                if alignment_report['ready'] and entry['ready'] and planar_ready:
                    prior_events=self.reference_session.events
                    self.reference_session=self.make_reference_session(ref,t)
                    self.reference_session.events=prior_events+[dict(time=t,event='ALIGNMENT_COMPLETE',report=alignment_report,initial_reference=asdict(ref))]
                    self.executor.start_s=t;self.latest_entry=entry;self.executor.approach_entry_report=entry
                    self.alignment_reference=None;self.operator_stage='LANDING'
                    self.operator_message='已跟稳平台，开始优化最后下降轨迹'
                elif not self.planar_design and t-self.alignment_started>self.live_cfg['moving_alignment']['maximum_time_s']:
                    self.start_recovery(t,obs,'MOVING_ALIGNMENT_TIMEOUT')
                    ref,_=self.recovery.step(t,obs)
                decision=ExecutionDecision('ALIGNING' if self.recovery is None else 'FAILED_ATTEMPT_RETURN',False,ref)
                hand.step(t,obs,frame,event)
            elif self.recovery:
                if t-self.recovery_started>40.:raise RuntimeError('Recovery did not settle within 40 seconds; normal native LAND fallback')
                hand.step(t,obs,frame,event)  # retain every causal pre-contact sample
                ref,done=self.recovery.step(t,obs);decision=ExecutionDecision('FAILED_ATTEMPT_RETURN',False,ref)
                if done:
                    self.attempt_history.append(dict(attempt=self.attempt,success=False,reason=self.reference_session.failure,events=self.reference_session.events))
                    atomic_json(ROOT/'results/receding_attempts.json',self.attempt_history)
                    self.recovery=None;self.reference_session=None;self.scenario_start=None
                    self.operator_stage='HOVER';self.operator_message='本次降落未完成，已回到起飞点悬停；可重新选择场景并点击降落'
            else:
                state=FlightState.from_observation(obs,self.curve.com_offset_body_m)
                frame_guard=False
                prior_plan=self.reference_session.plan
                if self.live_cfg.get('tracking_error_feedback'):
                    tracking_envelope=tracking_contact_envelope(prior_plan,state,obs,platform,t,self.envelope,**self.live_cfg['tracking_error_feedback'])
                if prior_plan is not None and self.envelope and self.live_cfg.get('measured_frame_terminal_control') and 0<prior_plan.arrival-t<5.:
                    candidate=prior_plan.body_reference_at(t)
                    candidate,_=measured_frame_reference(prior_plan,self.reference_session.model,platform,t,candidate)
                    preview=terminal_body_prediction(prior_plan,state,obs,platform,self.envelope,t,self.limits.footprint_margin_m,reference_override=candidate,frame_model=self.reference_session.model,uncertainty=self.forecast_tube)
                    frame_guard=bool(preview['passed'] and norm(candidate.velocity_world_m_s)<=self.live_cfg['planner_limits']['speed'] and norm(candidate.acceleration_world_m_s2)<=self.live_cfg['planner_limits']['acceleration'])
                ref=self.reference_session.step(t,state,platform,landing_requested=True,measured_frame_guard=frame_guard,continue_measured_frame=self.live_cfg.get('measured_frame_terminal_control',False),uncertainty=self.forecast_tube,tracking_envelope=tracking_envelope)
                if self.reference_session.plan:
                    hand.expected_contact_s=self.reference_session.plan.arrival
                    remaining=self.reference_session.plan.arrival-t
                    if 0<=remaining<=self.limits.contact_time_error_s:
                        checks=MovingLandingExecutor.terminal_capture_checks(self.executor,t,obs)
                        body_margin=self.envelope.clearance(obs.motion,deck) if self.envelope else self.limits.footprint_margin_m
                        native_guard=None
                        if self.live_cfg.get('maximum_early_native_freeflight_s'):
                            fs=evaluate_airframe(self.geometry,obs.motion,deck);gap=fs[0].clearance_m
                            closest=[f for f in fs if f.clearance_m<=gap+.001]
                            closing=min(2*f.closing_speed_m_s-f.closing_speed_upper_bound_m_s for f in closest)
                            native_guard=native_capture_time_guard(gap,closing,self.live_cfg['maximum_early_native_freeflight_s'])
                        capture_window=terminal_capture_window(remaining,self.limits.contact_time_error_s,checks,body_margin,self.limits.footprint_margin_m,frame_guard,native_time_guard=native_guard)
                if capture_window['ready'] or self.reference_session.phase=='TERMINAL_EXPIRED':
                    # Reuse the original strict final 1 cm capture gate.
                    checks=MovingLandingExecutor.terminal_capture_checks(self.executor,t,obs)
                    body_margin=self.envelope.clearance(obs.motion,deck) if self.envelope else self.limits.footprint_margin_m
                    if all(checks.values()) and body_margin>=self.limits.footprint_margin_m:
                        if capture_window['ready']:
                            self.reference_session.events.append(dict(time=t,event='MEASURED_TERMINAL_CAPTURE',arrival=hand.expected_contact_s,checks=checks))
                        if self.planar_design:self.reference_session.events.append(dict(time=t,event='NATIVE_CAPTURE_STARTED'))
                        hand.begin(t);self.reference_session.touch();decision=hand.step(t,obs,frame,event)
                        self.native_contact_deadline=t+self.limits.contact_time_error_s
                    else:self.start_recovery(t,obs,'TERMINAL_CAPTURE_REJECTED')
                elif ref is None:self.start_recovery(t,obs,self.reference_session.failure or 'NO_REFERENCE')
                else:
                    nominal_reference=asdict(ref)
                    model=self.reference_session.model if self.reference_session.plan else None
                    if model is not None and self.live_cfg.get('measured_frame_terminal_control'):
                        ref,terminal_frame_correction=measured_frame_reference(self.reference_session.plan,model,platform,t,ref)
                        if norm(ref.velocity_world_m_s)>self.live_cfg['planner_limits']['speed'] or norm(ref.acceleration_world_m_s2)>self.live_cfg['planner_limits']['acceleration']:
                            self.start_recovery(t,obs,'COMPENSATED_COMMAND_BOUND')
                    decision=ExecutionDecision(self.reference_session.phase,False,ref)
                    if self.envelope and self.reference_session.plan:
                        use_frame=self.live_cfg.get('measured_frame_terminal_control')
                        self.terminal_prediction=terminal_body_prediction(self.reference_session.plan,state,obs,platform,self.envelope,t,self.limits.footprint_margin_m,reference_override=ref if use_frame else None,frame_model=model if use_frame else None,uncertainty=self.forecast_tube)
                        if not self.terminal_prediction['passed']:
                            braking=braking_clearance_design(state)
                            gap=min(f.clearance_m for f in evaluate_airframe(self.geometry,obs.motion,deck))
                            self.terminal_prediction.update(braking=braking,measured_leg_gap_m=gap)
                            # Keep replanning only while a separate brake/climb
                            # option retains its declared clearance allowance.
                            if gap<=braking['required_clearance_m']+.12:self.start_recovery(t,obs,'PREDICTED_WHOLE_AIRCRAFT_MARGIN')
                    self.planner_message={'PLANNING_HOVER':'后台求解中，飞机保持悬停','TRACKING':'正在跟踪并滚动重规划','TERMINAL_TRACKING':'末段接近甲板，核对落点与接触速度'}.get(self.reference_session.phase,self.reference_session.phase)
                if self.recovery:
                    ref,_=self.recovery.step(t,obs);decision=ExecutionDecision('FAILED_ATTEMPT_RETURN',False,ref)
                # Build causal pre-contact history even while airborne.
                if hand.started is None:hand.step(t,obs,frame,event)
            if getattr(self,'native_contact_deadline',None) is not None and hand.first_contact is None and t>self.native_contact_deadline:
                hand.fail('native_contact_completion_timeout')
            if not obs.armed:
                if self.disarmed_since is None:self.disarmed_since=t
            else:self.disarmed_since=None
            if hand.phase=='COMPLETE' or hand.phase=='FAILED' and self.disarmed_since is not None and t-self.disarmed_since>=3.:
                passed=hand.phase=='COMPLETE' and not hand.failures
                self.result['completed']=passed;self.operator_stage='COMPLETE' if passed else 'ERROR'
                self.operator_message='实际接触、正常停转与支撑检查完成，可复位' if passed else '本轮未通过，已记录接触与退出情况，可复位'
                self.planner_message='已完成实际物理流程' if passed else '本轮失败，查看物理日志'
                self.live_control=False
        contact_owns_disarm=self.reference_session is not None and self.executor.handover.started is not None
        if not obs.armed and self.operator_stage not in ('COMPLETE','ERROR') and not contact_owns_disarm:
            raise RuntimeError('Unexpected disarm during flight')
        self.update_vehicle(t)
        remaining=None
        if self.reference_session and self.reference_session.plan and self.recovery is None:
            remaining=self.reference_session.plan.arrival-t
        gain_function=execution_feedback_gains if self.live_cfg.get('stable_non_descent_feedback') else None
        if gain_function:gains,feedback_schedule=gain_function(self.live_cfg['horizontal_feedback'],self.live_cfg.get('terminal_horizontal_feedback'),remaining,math.hypot(*platform.velocity[:2]),decision.phase)
        else:gains,feedback_schedule=terminal_feedback_gains(self.live_cfg['horizontal_feedback'],self.live_cfg.get('terminal_horizontal_feedback'),remaining,math.hypot(*platform.velocity[:2]))
        self.xy_feedback.kp=gains['position_gain'];self.xy_feedback.kd=gains['velocity_gain'];self.xy_feedback.lead=gains['acceleration_lead_s']
        sent=self.xy_feedback.update(decision,obs,t)
        sent=self.command_limiter.update(sent,t)
        if sent.reference is not None:self.last_sent_reference=sent.reference
        dispatch(sent,self.m,self.request_land,self.alignment,t)
        feet=evaluate_airframe(self.geometry,obs.motion,deck)
        row=dict(sim_t_s=t,stage=self.operator_stage,observation=asdict(obs),fit=fit,measured_deck=asdict(deck),platform=asdict(platform),
                 decision=asdict(decision),sent_decision=asdict(sent),xy_feedback=self.xy_feedback.diagnostics,command_limits=self.command_limiter.diagnostics,
                 contact_frame=asdict(frame) if frame else None,first_new_contact_event=asdict(event) if event else None,
                 feet=[asdict(f) for f in feet],car_command_m_s=self.car_speed,car_turn_command_rad_s=self.car_omega,
                 planner_phase=self.reference_session.phase if self.reference_session else None,
                 support_checks=self.executor.handover.last_support_checks,terminal_capture_window=capture_window,alignment_readiness=alignment_report)
        row['tracking_contact_envelope']=tracking_envelope
        if self.planar_design:
            row['planar_supervision']=self.planar_report
            row['planar_command']=getattr(self,'planar_command',None)
            row['filter_innovation']=self.platform_filter.innovation_report if self.platform_filter else None
        if self.envelope:row['whole_aircraft_deck_margin_m']=self.envelope.clearance(obs.motion,deck)
        row['terminal_body_prediction']=self.terminal_prediction
        row['horizontal_feedback_schedule']=feedback_schedule
        row['frame_velocity_guard']=frame_velocity_guard
        row['heading_acceleration_guard']=heading_guard
        row['forecast_uncertainty']=asdict(self.forecast_tube) if self.forecast_tube else None
        if self.platform_filter:row['platform_fit_input']=self.last_platform_fit_input
        if self.observation_stress():
            row['synthetic_platform_observation_stress']=self.observation_stress()
            row['platform_fit_input']=self.last_platform_fit_input
        row['nominal_reference']=nominal_reference;row['terminal_frame_correction']=terminal_frame_correction
        self.rows.append(row);self.control_stream.write(json.dumps(row,allow_nan=False)+'\n')
        atomic_json(ROOT/'results/progress.json',row)
        if self.reference_session:
            atomic_json(ROOT/'results/planner_execution.json',dict(phase=self.reference_session.phase,failure=self.reference_session.failure,events=self.reference_session.events))
        self.last_cycle_wall_s=time.perf_counter()-cycle_started

    def run(self):
        self.wait(lambda:'HEARTBEAT' in self.telemetry,60,'heartbeat')
        self.m.mav.param_set_send(1,1,b'GUID_OPTIONS',32.,mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        self.wait(lambda:'PARAM_VALUE' in self.telemetry and self.telemetry['PARAM_VALUE'][0].param_id=='GUID_OPTIONS' and self.telemetry['PARAM_VALUE'][0].param_value==32.,30,'GUID_OPTIONS acknowledgement')
        for mid in (30,32,33,245,83,85,62):self.command(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,mid,50000)
        self.wait(lambda:self.latest and self.car_history and 'LOCAL_POSITION_NED' in self.telemetry,180,'estimator initialization')
        t=self.sim_t();self.wait(lambda:self.sim_t()-t>=12.,180,'sensor settling')
        health=json.loads((ROOT/'runtime/receding_planner/health.json').read_text());assert health['ready']
        self.m.set_mode(self.m.mode_mapping()['GUIDED']);self.wait(lambda:self.mode()=='GUIDED',30,'GUIDED')
        self.operator_stage='READY';self.stage='WAIT_USER_TAKEOFF';self.operator_message='就绪：新规划器已接入。先选择平台运动，再点击起飞'
        automatic=os.environ.get('HS_AUTONOMOUS_TRIAL')=='1'
        if automatic:self.takeoff_requested=True
        else:self.pause_world(True)
        self.publish_status(True)
        while self.running and not self.takeoff_requested:self.pump()
        if self.reset_requested:return
        if self.operator_clock.paused:self.pause_world(False)
        self.operator_stage='TAKING_OFF';self.operator_message='正在起飞，随后悬停等待你点击降落'
        for attempt in range(8):
            self.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,1)
            try:self.wait(self.armed,12,'normal arming');break
            except RuntimeError:
                if attempt==7:raise
        local=self.telemetry['LOCAL_POSITION_NED'][0]
        self.alignment=LocalNedAlignment.from_body_measurement(self.latest.pose.position_world_m,(local.x,local.y,local.z))
        self.command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,0,0,0,0,0,0,4.)
        self.wait(lambda:self.latest.pose.position_world_m[2]>3.7,180,'physical takeoff')
        self.live_control=True;self.operator_stage='HOVER';self.operator_message='悬停中：点击“降落到小车”才开始新算法'
        self.last_control=None
        if automatic:self.land_requested=True
        while self.running and not self.reset_requested:
            self.pump()
            if automatic and self.operator_stage in ('COMPLETE','ERROR'):
                self.publish_status(True);break
            if automatic and self.operator_stage=='HOVER' and self.attempt>0:
                raise RuntimeError('Automatic trial returned home after failed attempt')

    def save(self):
        if self.reference_session:
            atomic_json(ROOT/'results/receding_session.json',dict(phase=self.reference_session.phase,failure=self.reference_session.failure,events=self.reference_session.events,last_report=self.reference_session.last_report))
        if self.control_stream:self.control_stream.close()
        super().save()

    def emergency(self,error):
        # A close request must not re-raise on every telemetry pump while LAND
        # waits for native disarm. Keep the original failure in the result.
        self.close_requested=False
        self.operator_stage='RECOVERY_LAND';self.operator_message='本轮退出，正在通过原生 LAND 正常落地'
        super().emergency(error)


if __name__=='__main__':
    demo=OperatorApproach();code=0
    try:
        demo.run()
        if demo.reset_requested:code=10
    except Exception as exc:
        demo.live_control=False
        if demo.operator_clock.paused:demo.pause_world(False)
        demo.emergency(exc);demo.operator_stage='ERROR';demo.operator_message='已退出：'+str(exc);demo.publish_status(True);code=2
    finally:demo.save()
    sys.exit(code)
