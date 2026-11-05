"""Live Gazebo/ArduCopter bridge for the step07/08 simulation prototype."""
import json,math,os,signal,sys,threading,time
from pathlib import Path
from dataclasses import asdict
os.environ.setdefault('MAVLINK20','1')
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from google.protobuf.json_format import MessageToDict
from gz.transport13 import Node
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.contacts_pb2 import Contacts
from gz.msgs10.clock_pb2 import Clock
from gz.msgs10.twist_pb2 import Twist
from pymavlink import mavutil
from landing.contact_geometry import Pose,RigidMotion,Deck,add,scale,norm,sub,dot,evaluate_airframe
from landing.accelerating_feedback import CaptureAwareFeedback as HorizontalFeedback
from landing.reference_loader import load_verified_reference
from landing.rapid_contact import load_rapid_reference
from landing.moving_rapid import plan_moving_rapid
from landing.accelerating_platform import SharedDeckState,SharedStateReceiver,predict_deck
from landing.decelerating_platform import restore_decelerating_plan as restore_accelerating_plan,HoverEntry,require_moving_brake_window
from landing.tracking_compensation import terminal_height_feedforward
from landing.startup import VerticalLeadIn
from landing.moving_platform import plan_interception,MovingContactHandover,MovingLandingExecutor,translate_deck
from landing.execution import (ContactHistory,EntryLimits,HandoverLimits,LandingExecutor,BodyReference,
    ExecutionDecision,assess_entry,parse_deck_contacts,attitude_error)
from landing.observations import CausalPoseHistory,PoseSample,ObservationUnavailable
from landing.fcu_recording import FcuDiagnosticCapture
from landing.mavlink_reference import dispatch,LocalNedAlignment


def stamp(message):
    value=message.header.stamp;return value.sec+value.nsec*1e-9


class LiveApproach:
    def __init__(self):
        self.curve,self.geometry,self.deck,self.profile=load_verified_reference(ROOT)
        pinned=json.loads((ROOT/'outputs/model_audit/step19/reference_preflight.json').read_text())
        self.accel_cfg=json.loads((ROOT/'config/experiments/decelerating_platform_v1.json').read_text())
        self.curve=restore_accelerating_plan(self.geometry,self.profile,pinned)
        self.rapid_plan=pinned;self.pre_duration=self.curve.T;self.precheck=pinned
        self.shared_receiver=SharedStateReceiver();self.shared_packets=[];self.shared_sequence=0
        self.accel_epoch=None;self.accel_ready=None;self.current_acceleration=(0.,0.,0.)
        self.land_authority_requested=False
        self.cfg=json.loads((ROOT/'config/experiments/execution_v1.json').read_text())
        lo,hi=.25,.65
        self.cfg['handover_limits']['contact_minimum_closing_speed_m_s']=lo
        self.cfg['handover_limits']['contact_normal_speed_m_s']=hi
        self.lead=HoverEntry(self.curve)
        self.entry=EntryLimits(**self.cfg['entry_limits']);self.limits=HandoverLimits(**self.cfg['handover_limits'])
        self.executor=LandingExecutor(self.curve,self.geometry,self.deck,self.entry,self.limits,None)
        # SITL heartbeat is scheduled in simulation time. A slow real-time
        # factor must not make a fresh simulated heartbeat look stale. Check
        # both 1.5 simulation seconds below and a 15 s wall-clock upper bound.
        self.lock=threading.Lock();self.history=CausalPoseHistory(self.curve.com_offset_body_m,heartbeat_max_wall_age_s=15.)
        self.car_fit=CausalPoseHistory((0,0,0),span_s=.181,minimum_samples=4,minimum_span_s=.08,maximum_sample_gap_s=.05,maximum_sim_age_s=.08,maximum_wall_age_s=1.);self.car_speed=0.;self.plan_epoch=None
        self.contacts=ContactHistory();self.car_history=[];self.latest=None;self.clock=None;self.callback_error=None
        self.telemetry={};self.running=True;self.stage='CONNECT';self.enabled=False;self.alignment=None
        self.heartbeat_sim_t=None
        self.last_hb=-math.inf;self.last_car=-math.inf;self.last_control=None;self.last_print=-math.inf
        self.prepare_start=None;self.prepare_ready=None;self.disarmed_since=None;self.callbacks=[]
        self.rows=[];self.events=[];self.latest_entry=None;self.external_failures=[]
        self.fcu_capture=FcuDiagnosticCapture();self.xy_feedback=HorizontalFeedback(**self.accel_cfg['horizontal_feedback'])
        self.result={'simulation_only':True,'completed':False,'controller':'decelerating-deck constrained snap optimization + causal shared acceleration + rolling deadline corrections; native LAND',
            'observations':'Gazebo body truth with causal quadratic pose fits; no sensor noise/delay simulation',
            'reference_approach_duration_s':self.curve.T,'reference_preparation_duration_s':self.lead.T,
            'hardware_deployment_allowed':False,'pose_teleportation_used':False,'force_disarm_used':False}
        contact_topic=os.environ['HS_DECK_CONTACT_TOPIC']
        self.result['verified_contact_publisher_topic']=contact_topic
        self.node=Node();self.car_pub=self.node.advertise('/vehicle/cmd_vel',Twist)
        def pose_callback(msg):
            try:
                for item in msg.pose:
                    if item.name!='uav':continue
                    sample=PoseSample(stamp(item),Pose((item.position.x,item.position.y,item.position.z),
                        (item.orientation.x,item.orientation.y,item.orientation.z,item.orientation.w)),time.monotonic())
                    with self.lock:self.history.push(sample);self.latest=sample
            except Exception as exc:self.callback_error=repr(exc)
        def car_callback(msg):
            try:
                for item in msg.pose:
                    if item.name!='landing_vehicle':continue
                    sample=PoseSample(stamp(item),Pose((item.position.x,item.position.y,item.position.z),
                        (item.orientation.x,item.orientation.y,item.orientation.z,item.orientation.w)),time.monotonic())
                    with self.lock:
                        self.car_fit.push(sample);self.car_history.append(sample);self.car_history=self.car_history[-128:]
            except Exception as exc:self.callback_error=repr(exc)
        mapping={self.cfg['leg_collision_prefix']+leg.name:leg.name for leg in self.geometry.legs}
        def contact_callback(msg):
            try:
                frame=parse_deck_contacts(MessageToDict(msg,preserving_proto_field_name=True),self.cfg['deck_collision'],mapping,self.cfg['uav_model_prefix'])
                with self.lock:self.contacts.push(frame)
            except Exception as exc:self.callback_error=repr(exc)
        def clock_callback(msg):
            with self.lock:self.clock=(msg.sim.sec+msg.sim.nsec*1e-9,time.monotonic())
        for kind,topic,callback in [(Pose_V,'/model/uav/pose',pose_callback),(Pose_V,'/model/landing_vehicle/pose',car_callback),
            (Contacts,contact_topic,contact_callback),(Clock,'/world/uav_vehicle_stage1/clock',clock_callback)]:
            self.callbacks.append(callback)
            if not self.node.subscribe(kind,topic,callback):raise RuntimeError('Subscription failed: '+topic)
        signal.signal(signal.SIGTERM,self.stop);signal.signal(signal.SIGINT,self.stop)
        deadline=time.monotonic()+90
        while True:
            try:self.m=mavutil.mavlink_connection('tcp:127.0.0.1:5760',source_system=255,source_component=190);break
            except ConnectionRefusedError:
                if time.monotonic()>deadline:raise
                time.sleep(1)

    def stop(self,*args):self.running=False
    def event(self,text):
        row={'wall_unix_s':time.time(),'stage':self.stage,'message':text};self.events.append(row);print(json.dumps(row),flush=True)
    def command(self,command,*params):self.m.mav.command_long_send(1,1,command,0,*(list(params)+[0]*7)[:7])
    def armed(self):return bool('HEARTBEAT' in self.telemetry and self.telemetry['HEARTBEAT'][0].base_mode&128)
    def mode(self):return mavutil.mode_string_v10(self.telemetry['HEARTBEAT'][0]) if 'HEARTBEAT' in self.telemetry else 'UNKNOWN'
    def request_land(self):self.m.set_mode(self.m.mode_mapping()['LAND']);self.event('Requested normal LAND; no force-disarm')
    def sim_t(self):return self.latest.sim_t_s if self.latest else 0.

    def deck_at(self,t):
        with self.lock:
            wall=time.monotonic();obs=self.car_fit.estimate(t,wall,False,'SIM_DECK',wall)
            acc=self.car_fit.last_fit['body_acceleration_world_m_s2']
        top=obs.motion.pose.transform((0.,0.,.8))
        # Gazebo truth is the source in this model. This JSON exchange is the
        # causal sharing interface; it is not yet an actual vehicle radio.
        if self.shared_receiver.latest is None or obs.sim_t_s>self.shared_receiver.latest.sample_sim_t_s:
            self.shared_sequence+=1
            # Startup wheel transients are not admitted to the landing planner.
            a=(max(-1.,min(1.,acc[0])),0.,0.)
            packet=SharedDeckState(self.shared_sequence,obs.sim_t_s,top,obs.motion.velocity_at_origin_world_m_s,a)
            packet=SharedDeckState(**json.loads(json.dumps(asdict(packet))))
            self.shared_receiver.accept(packet);self.shared_packets.append(asdict(packet))
        # Handover can request the sample immediately BEFORE first contact.
        # Receiver arrival order stays monotone; retrospective evaluation must
        # select the latest packet whose sampling timestamp is <= that query.
        packet=self.shared_receiver.at(t)
        self.current_acceleration=packet.acceleration_m_s2
        return packet.deck_at(t)

    def prepare_interception(self,t):
        measured=self.deck_at(t)
        if not 1.<=measured.motion.pose.position_world_m[0]<=6.:raise RuntimeError('Measured car outside preflight planning range')
        self.curve=restore_accelerating_plan(self.geometry,self.profile,self.precheck)
        brake_intent=(-.5,0.,0.)
        require_moving_brake_window(measured,brake_intent,self.curve.T)
        self.curve.retarget(0.,measured,brake_intent,force=True)
        self.lead=HoverEntry(self.curve)
        self.planned_deck=measured
        static=Deck(RigidMotion.stationary(measured.motion.pose),measured.half_extents_m)
        self.executor=MovingLandingExecutor(self.curve,self.geometry,static,self.entry,self.limits,None)
        terminal=predict_deck(measured,brake_intent,self.curve.T)
        self.executor.handover=MovingContactHandover(self.geometry,self.curve.terminal_boundary,terminal,self.limits,self.deck_at)
        self.plan_epoch=t
        report=dict(self.precheck);report.update(actual_deck_at_planning=asdict(measured),planning_sim_t_s=t,
            actual_acceleration_at_planning_m_s2=self.current_acceleration,acceleration_start=self.result['acceleration_start'],
            brake_intent_m_s2=brake_intent,initial_prediction_source='Declared wheel brake command; causal acceleration fit admitted after 0.30 s')
        self.rapid_plan=report;self.result['rapid_plan']=report;self.result['moving_plan']=report
        self.result['reference_approach_duration_s']=self.curve.T;self.result['contact_limits']=asdict(self.limits)
        self.event('Acceleration-aware intercept: measured a='+str(self.current_acceleration)+'; fixed T='+str(self.curve.T))

    def pump(self):
        wall=time.monotonic()
        if wall-self.last_hb>=1:
            self.m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,mavutil.mavlink.MAV_AUTOPILOT_INVALID,0,0,0)
            self.m.mav.rc_channels_override_send(1,1,1500,1500,1000,1500,1800,1500,1500,1500);self.last_hb=wall
        if self.sim_t()-self.last_car>=.02:
            cmd=Twist();cmd.linear.x=self.car_speed;self.car_pub.publish(cmd);self.last_car=self.sim_t()
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
                raise RuntimeError('Live simulation clock/body reception stalled')
            t=min(latest.sim_t_s,clock[0])
            if self.last_control is None or t-self.last_control>=.039999:
                self.last_control=t
                hb=self.telemetry.get('HEARTBEAT')
                if hb is None:raise RuntimeError('Missing flight-state heartbeat')
                if self.heartbeat_sim_t is None or not 0<=t-self.heartbeat_sim_t<=1.5:raise RuntimeError('Mode heartbeat stale in simulation time')
                with self.lock:
                    # Read wall time after taking the lock: a callback may
                    # have supplied a new pose since this pump iteration began.
                    sample_wall=time.monotonic()
                    obs=self.history.estimate(t,sample_wall,self.armed(),self.mode(),hb[1])
                    car=next((item for item in reversed(self.car_history) if item.sim_t_s<=t),None)
                    frame,event=self.contacts.poll(t,self.limits.contact_age_s)
                    fit=dict(self.history.last_fit)
                if car is None or wall-car.received_monotonic_s>1. or t-car.sim_t_s>.12:raise RuntimeError('Static deck observation stale')
                self.deck=self.deck_at(t)
                if attitude_error(car.pose,Pose((0,0,0),(0,0,0,1)))>.01:
                    raise RuntimeError('Moving deck orientation outside planar scene gate')
                if self.plan_epoch is not None and self.executor.handover.started is None:
                    if t-self.accel_epoch>=.30 and abs(self.current_acceleration[0]+.5)>.12:raise RuntimeError('Shared acceleration outside declared braking scene')
                    if self.executor.start_s is not None:
                        elapsed=t-self.executor.start_s
                        if t-self.accel_epoch>=.30 and 0.<=elapsed<self.curve.T:
                            require_moving_brake_window(self.deck,self.current_acceleration,self.curve.T-elapsed)
                            if self.curve.retarget(elapsed,self.deck,self.current_acceleration):
                                h=self.executor.handover;terminal=predict_deck(self.deck,self.current_acceleration,self.curve.T-elapsed)
                                h.target_offset=sub(self.curve.terminal_boundary.aircraft.pose.position_world_m,terminal.motion.pose.position_world_m)
                                h.target_q=self.curve.terminal_boundary.aircraft.pose.orientation_xyzw
                if self.stage=='PREPARE':
                    decision=ExecutionDecision('PREPARE_HOLD',False,BodyReference.from_state(self.lead.at(0)))
                    self.latest_entry=assess_entry(t,obs,self.lead.at(0),self.curve.com_offset_body_m,self.entry,frame)
                    if self.latest_entry['ready'] and self.accel_epoch is None:self.car_speed=3.
                    if self.latest_entry['ready'] and self.accel_epoch is None and abs(self.deck.motion.velocity_at_origin_world_m_s[0]-3.)<.005 and abs(self.current_acceleration[0])<.03:
                        if self.prepare_ready is None:self.prepare_ready=t
                        if t-self.prepare_ready>=.25:
                            self.accel_epoch=t;self.car_speed=0.
                            self.result['acceleration_start']={'sim_t_s':t,'deck':asdict(self.deck),'measured_initial_speed_m_s':self.deck.motion.velocity_at_origin_world_m_s[0]}
                            self.result['braking_start']=self.result['acceleration_start']
                            self.prepare_interception(t)
                            cmd=Twist();cmd.linear.x=0.;self.car_pub.publish(cmd);self.last_car=t
                            self.stage='EXECUTION';decision=self.executor.step(t,obs,frame,event)
                            self.result['preparation_reference_start_sim_t_s']=t
                            self.event('Measured 3 m/s: synchronized physical wheel braking and direct hover-entry interception; no pose reset')
                    if t-self.prepare_start>60.:raise RuntimeError('Preparation did not match strict measured entry within 60 simulation seconds')
                else:
                    decision=self.executor.step(t,obs,frame,event)
                    if decision.reference is not None:
                        error=norm(sub(obs.motion.pose.position_world_m,decision.reference.position_world_m))
                        speed_error=norm(sub(obs.motion.velocity_at_origin_world_m_s,decision.reference.velocity_world_m_s))
                        tilt=math.acos(max(-1.,min(1.,obs.motion.pose.rotate((0,0,1))[2])))
                        if error>.35 or speed_error>.5 or tilt>math.radians(20):
                            self.executor.handover.begin(t,'live_tracking_envelope_exceeded')
                            decision=ExecutionDecision('FAILSAFE_LAND',True,request_land=True,pending_land_velocity_world_m_s=(0,0,0),
                                failures=tuple(self.executor.handover.failures))
                elapsed=None if self.executor.start_s is None else t-self.executor.start_s
                if self.executor.start_s is not None:
                    self.xy_feedback.deadline=self.executor.start_s+self.curve.T
                    self.xy_feedback.approach=self.curve;self.xy_feedback.start_s=self.executor.start_s
                h=self.executor.handover
                if h.phase=='WAIT_DISARM' and not self.land_authority_requested:
                    assert h.first_contact and all(h.last_support_checks.values()) and self.mode()=='LAND'
                    value=self.accel_cfg['supported_land_maximum_position_control_tilt_deg']
                    self.m.mav.param_set_send(1,1,b'PSC_ANGLE_MAX',value,mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
                    self.land_authority_requested=True
                    self.result['supported_land_authority_transition']={'sim_t_s':t,'maximum_position_control_tilt_deg':value,
                        'support_checks':dict(h.last_support_checks),'support_dwell_s':t-h.stable_since,'parameter_acknowledged':False,
                        'policy':'After stable multi-leg support in LAND, limit lateral attitude demand; keep FCU land detector and automatic normal disarm. No force-disarm.'}
                    self.event('Stable support confirmed; requesting LAND lateral authority cap '+str(value)+' deg')
                if self.land_authority_requested and 'PARAM_VALUE' in self.telemetry:
                    param=self.telemetry['PARAM_VALUE'][0]
                    if param.param_id=='PSC_ANGLE_MAX' and abs(param.param_value-self.accel_cfg['supported_land_maximum_position_control_tilt_deg'])<1e-5:
                        self.result['supported_land_authority_transition']['parameter_acknowledged']=True
                if h.phase=='FAILED' and obs.armed:raise RuntimeError('Failed contact/support handover; recover with physical platform braking')
                sent_decision=self.xy_feedback.update(decision,obs,t)
                dispatch(sent_decision,self.m,self.request_land,self.alignment,t)
                if not obs.armed and self.stage=='EXECUTION':
                    if self.disarmed_since is None:self.disarmed_since=t
                else:self.disarmed_since=None
                feet=evaluate_airframe(self.geometry,obs.motion,self.deck)
                row={'sim_t_s':t,'stage':self.stage,'observation':asdict(obs),'fit':fit,'decision':asdict(decision),
                    'sent_decision':asdict(sent_decision),'xy_feedback':self.xy_feedback.diagnostics,'contact_frame':asdict(frame) if frame else None,'first_new_contact_event':asdict(event) if event else None,
                    'measured_deck':asdict(self.deck),'car_fit':dict(self.car_fit.last_fit),'shared_acceleration_m_s2':self.current_acceleration,'shared_state_sequence':self.shared_receiver.latest.sequence,'car_command_m_s':self.car_speed,'minimum_leg_gap_m':feet[0].clearance_m,'feet':[asdict(f) for f in feet],
                    'entry_report':self.latest_entry if self.stage=='PREPARE' else self.executor.approach_entry_report,
                    'support_checks':self.executor.handover.last_support_checks}
                if 'LOCAL_POSITION_NED' in self.telemetry:
                    local,received=self.telemetry['LOCAL_POSITION_NED']
                    row['fcu_local_position_ned']={key:getattr(local,key) for key in ['x','y','z','vx','vy','vz','time_boot_ms']}
                    row['fcu_position_wall_age_s']=time.monotonic()-received
                self.rows.append(row)
                (ROOT/'results/progress.json').write_text(json.dumps(row,allow_nan=False))
                if t-self.last_print>=2:
                    self.event('phase='+decision.phase+' p='+str(tuple(round(x,3) for x in obs.motion.pose.position_world_m))+
                        ' v='+str(tuple(round(x,3) for x in obs.motion.velocity_at_origin_world_m_s))+' failures='+str(decision.failures))
                    self.last_print=t
                if decision.complete:self.result['completed']=True
        time.sleep(.005)

    def wait(self,predicate,timeout,label):
        deadline=time.monotonic()+timeout
        while self.running and time.monotonic()<deadline:
            self.pump()
            if predicate():return
        raise RuntimeError('Interrupted/timed out: '+label)

    def run(self):
        self.wait(lambda:'HEARTBEAT' in self.telemetry,60,'heartbeat')
        self.m.mav.param_set_send(1,1,b'GUID_OPTIONS',32.,mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        self.wait(lambda:'PARAM_VALUE' in self.telemetry and self.telemetry['PARAM_VALUE'][0].param_id=='GUID_OPTIONS' and self.telemetry['PARAM_VALUE'][0].param_value==32.,30,'GUID_OPTIONS 32 acknowledgement')
        self.result['horizontal_control']={'native_guided_options_confirmed':32,**self.accel_cfg['horizontal_feedback'],'observations':'causal Gazebo truth','scope':'External XY feedback; native vertical and attitude control retained; simulation tuning, not hardware identification'}
        for mid in (30,32,33,245,83,85,62):self.command(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,mid,50000)
        self.wait(lambda:self.latest and self.car_history and 'LOCAL_POSITION_NED' in self.telemetry,180,'body/deck/FCU estimator')
        initial_t=self.sim_t();self.wait(lambda:self.sim_t()-initial_t>=12.,180,'sensor settling')
        self.m.set_mode(self.m.mode_mapping()['GUIDED']);self.wait(lambda:self.mode()=='GUIDED',30,'GUIDED acknowledgement')
        self.stage='ARM'
        for attempt in range(8):
            self.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,1)
            try:self.wait(self.armed,12,'normal arming');break
            except RuntimeError:
                if attempt==7:raise
        local=self.telemetry['LOCAL_POSITION_NED'][0]
        self.alignment=LocalNedAlignment.from_body_measurement(self.latest.pose.position_world_m,(local.x,local.y,local.z))
        self.result['alignment_offset_ned_m']=self.alignment.offset_ned_m
        self.stage='TAKEOFF';self.command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,0,0,0,0,0,0,4.0)
        self.wait(lambda:self.latest.pose.position_world_m[2]>3.7,180,'physical takeoff')
        self.stage='PREPARE';self.enabled=True;self.prepare_start=self.sim_t()
        self.event('Holding above launch point for strict measured preparation entry; no pose reset')
        self.wait(lambda:self.executor.experiment_can_finish(self.sim_t(),self.armed(),self.disarmed_since),600,'reference, contact, LAND and normal disarm')
        self.result['failures']=list(self.executor.handover.failures)
        self.result['first_contact']=self.executor.handover.first_contact
        self.result['main_approach_start_sim_t_s']=self.executor.start_s
        self.result['main_entry_report']=self.executor.approach_entry_report
        self.result['native_contact_completion']=getattr(self.executor,'native_contact_completion',None)
        self.result['final_phase']=self.executor.handover.phase
        self.result['normal_disarm_observed']=not self.armed()
        self.enabled=False;self.stage='COMPLETE' if self.result['completed'] else 'FAILED'

    def emergency(self,error):
        self.running=True
        self.enabled=False;self.car_speed=0.;self.result['recovery_platform_braking_used']=True;self.external_failures.append(str(error));self.event('Stopped external reference: '+str(error))
        if self.armed():
            self.stage='RECOVERY_LAND';self.request_land()
            try:self.wait(lambda:not self.armed(),180,'normal recovery LAND disarm')
            except Exception as exc:self.external_failures.append('recovery: '+str(exc))
        self.result['normal_disarm_observed']=not self.armed()
        self.result['completed']=False

    def save(self):
        self.result.update(external_failures=self.external_failures,events=self.events,final_mode=self.mode(),armed_at_save=self.armed(),
            first_contact=self.executor.handover.first_contact,failures=list(self.executor.handover.failures),
            main_approach_start_sim_t_s=self.executor.start_s,main_entry_report=self.executor.approach_entry_report)
        (ROOT/'results/execution_status.json').write_text(json.dumps(self.result,indent=2,allow_nan=False))
        with (ROOT/'results/execution_feedback.jsonl').open('w') as stream:
            for row in self.rows:stream.write(json.dumps(row,allow_nan=False)+'\n')
        self.result['rolling_updates']=self.curve.updates
        (ROOT/'results/execution_status.json').write_text(json.dumps(self.result,indent=2,allow_nan=False))
        (ROOT/'results/shared_platform_states.json').write_text(json.dumps(self.shared_packets,indent=2,allow_nan=False))
        self.fcu_capture.save(ROOT/'results')
        self.car_speed=0.;self.car_pub.publish(Twist());self.m.close()


if __name__=='__main__':
    demo=LiveApproach()
    try:demo.run()
    except Exception as exc:demo.emergency(exc)
    finally:demo.save()
    sys.exit(0 if demo.result['completed'] else 2)
