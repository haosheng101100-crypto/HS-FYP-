"""Direct attitude/climb-rate braking controller for the fixed SITL airframe.

Interface semantics pinned to ArduCopter 4.7.1 dbe79216. Native attitude,
altitude, motors and Gazebo dynamics remain active. Unlike the PVA prototype,
old trajectory jerk/snap limits are not advertised as the envelope of this
new interface. Commands have explicit tilt and angular-slew limits; actual
motion and unchanged touchdown gates must be checked in physics.
"""
import math
from dataclasses import replace
from .direct_braking import MonotoneBrakeAxis,predict_stopped_deck
from .septic_approach import PlanarSnapMatchedApproach
from .contact_geometry import Pose,RigidMotion,Deck,quaternion_from_rpy,evaluate_airframe
from .touchdown_boundary import TouchdownRequest,ContactDesignLimits,solve_touchdown_boundary


def attitude_capture_plan(geometry,profile,observation,deck,acceleration,duration=3.0):
    terminal=predict_stopped_deck(deck,acceleration)
    offset=profile['nominal_com_in_body_m'];com=observation.motion.pose.transform(offset)
    v=observation.motion.velocity_at_world_point(com)
    y=com[1]-offset[1]-terminal.motion.pose.position_world_m[1]
    if abs(y)>.02:raise ValueError('Planar centre approach requires measured lateral position within 2 cm')
    # Declare the deck centre before flight. Earlier forward landing points
    # remain in their immutable executed packages, never relabelled as centred.
    target_x=0.
    boundary=solve_touchdown_boundary(geometry,terminal,TouchdownRequest.planar(0.,0.,(target_x,y),(0.,0.,-.45)),
        ContactDesignLimits(math.radians(5),math.radians(5),.05,.25,.65,.02))
    curve=PlanarSnapMatchedApproach(com,(v[0],0,v[2]),boundary,offset,9.80665,profile['mass_kg'],duration,
        terminal_com_snap_world_m_s4=(0,0,0))
    axis=MonotoneBrakeAxis(com[0],v[0],curve.axes[0].terminal[0],duration,ramp_in=.4,ramp_out=.6)
    curve.axes=(axis,*curve.axes[1:]);samples=[curve.at(duration*i/200) for i in range(201)]
    checks={'reference_never_reverses':min(s['com_velocity_world_m_s'][0] for s in samples)>=-1e-8,
        'reference_keeps_descending':max(s['com_velocity_world_m_s'][2] for s in samples)<-.15,
        'reference_braking_acceleration':axis.peak_deceleration<=3.4,
        'reference_vertical_speed':max(abs(s['com_velocity_world_m_s'][2]) for s in samples)<=1.2,
        'finite_deck_terminal_footprint':all(min(f.whole_cylinder_footprint_margins_m)>=.02 for f in evaluate_airframe(geometry,boundary.aircraft,terminal)),
        'braking_release_before_contact':axis.stop_s<duration-.25}
    if not all(checks.values()):raise ValueError('Attitude braking candidate rejected: '+str(checks))
    return curve,terminal,{'reference_checks':checks,'measured_deck':deck,'measured_acceleration_m_s2':acceleration,
        'predicted_stop':terminal,'duration_s':duration,'horizontal_braking_duration_s':axis.stop_s,
        'peak_reference_deceleration_m_s2':axis.peak_deceleration,'horizontal_pieces':axis.coefficients,
        'vertical_coefficients':curve.axes[2].coefficients,'target_contact_x_on_deck_m':target_x,
        'centre_landing_contract':{'maximum_body_horizontal_distance_m':.15,'target_xy_on_deck_m':[0.,y],
            'lateral_strategy':'Preserve measured lateral position within 2 cm for the planar approach'},
        'control_interface':'SET_ATTITUDE_TARGET with GUID_OPTIONS bit 3 clear: normalized climb-rate field',
        'reference_deceleration_limit_m_s2':3.4,
        'command_limits':{'horizontal_acceleration_m_s2':3.5,'tilt_deg':19.5,'angular_slew_deg_s':70.,
                          'climb_rate_m_s':[-1.2,.5]},
        'actual_flight_limits':{'tilt_deg':20.,'angular_speed_deg_s':80.},
        'no_claim_of_old_PVA_jerk_snap_envelope':True,'physics_verification_required':True}


class AttitudeCaptureController:
    def __init__(self,curve,start,up_speed=2.5,down_speed=1.5,rate_feedforward=False,braking_tail_guard=False,native_rate_mode=False,measured_thrust_mapping=False,own_reference_preview_s=None,horizontal_velocity_gain=2.4):
        if up_speed<=0 or down_speed<=0:raise ValueError('Positive verified native climb limits required')
        self.curve=curve;self.start=start;self.up=up_speed;self.down=down_speed
        self.last_t=None;self.roll=0.;self.pitch=0.;self.diagnostics=None
        self.rate_feedforward=rate_feedforward
        self.braking_tail_guard=braking_tail_guard
        self.native_rate_mode=native_rate_mode
        self.measured_thrust_mapping=measured_thrust_mapping
        if own_reference_preview_s is not None and not 0<=own_reference_preview_s<=.2:
            raise ValueError('Own-reference preview must be explicitly bounded to 0..0.2 s')
        self.own_reference_preview_s=own_reference_preview_s
        if not math.isfinite(horizontal_velocity_gain) or not 0<horizontal_velocity_gain<=4.:
            raise ValueError('Horizontal velocity feedback must stay within 0..4 per second')
        self.horizontal_velocity_gain=horizontal_velocity_gain
    def command(self,reference,observation,now):
        if now<observation.sim_t_s or now-observation.sim_t_s>.06:raise ValueError('Fresh causal state required')
        dt=.04 if self.last_t is None else now-self.last_t
        if not 0<dt<=.08:raise ValueError('Attitude update interval exceeded')
        p=observation.motion.pose.position_world_m;v=observation.motion.velocity_at_origin_world_m_s
        preview=.10 if self.native_rate_mode else .35 if self.braking_tail_guard else .18
        if self.own_reference_preview_s is not None:preview=self.own_reference_preview_s
        t=max(0.,min(self.curve.T,now-self.start));future=self.curve.at(min(self.curve.T,t+preview))
        ax=[]
        # Own-reference preview is causal; measured state drives correction.
        capture=max(0.,min(1.,(t-(self.curve.T-1.2))/1.2));kp=1.5*(1-capture)+.2*capture
        for k in range(2):
            a=future['body_acceleration_world_m_s2'][k]+kp*(reference.position_world_m[k]-p[k])+self.horizontal_velocity_gain*(reference.velocity_world_m_s[k]-v[k])
            ax.append(a)
        bound=3.5 if self.measured_thrust_mapping else 3.2
        length=math.hypot(*ax);factor=min(1.,bound/max(length,1e-12));ax=[a*factor for a in ax]
        # Prevent commanding a backward recovery; actual no-reversal is audited.
        tail=None
        if self.braking_tail_guard:
            # Declared conservative preview, not an identified time constant
            # or a hardware guarantee. Current physical acceleration includes
            # thrust/attitude lag: releasing the target does not release the
            # actual braking force instantly. Reserve velocity for this tail.
            acceleration=observation.com_acceleration_world_m_s2
            if acceleration is None or not all(math.isfinite(x) for x in acceleration):raise ValueError('Measured acceleration required for braking-tail guard')
            tail=v[0]+(.10 if self.native_rate_mode else .32)*min(0.,acceleration[0])
            residual=.008 if self.native_rate_mode else .015
            release=.14 if self.native_rate_mode else .22
            ax[0]=max(ax[0],min(1.,-(tail-residual)/release))
        else:ax[0]=max(ax[0],-max(v[0],0.)/.18)
        length=math.hypot(*ax);factor=min(1.,bound/max(length,1e-12));ax=[a*factor for a in ax]
        upforce=9.80665+reference.acceleration_world_m_s2[2]
        if self.measured_thrust_mapping:
            acceleration=observation.com_acceleration_world_m_s2
            if acceleration is None or not all(math.isfinite(x) for x in acceleration):raise ValueError('Measured COM acceleration required for thrust-direction mapping')
            upforce=9.80665+acceleration[2]
            if not 6.<upforce<14.:raise ValueError('Measured vertical force outside this simulation mapping envelope')
        target_pitch=-math.atan2(ax[0],upforce)
        target_roll=-math.atan2(ax[1],math.hypot(ax[0],upforce))
        maximum=math.radians(19.5 if self.measured_thrust_mapping else 19.);slew=math.radians(70)*dt
        scale=min(1.,maximum/max(math.hypot(target_roll,target_pitch),1e-12))
        target_roll*=scale;target_pitch*=scale
        dr=target_roll-self.roll;dp=target_pitch-self.pitch
        scale=min(1.,slew/max(math.hypot(dr,dp),1e-12))
        roll=self.roll+scale*dr;pitch=self.pitch+scale*dp
        # Native vertical velocity controller, with an external height
        # correction and modest preview of our own planned vertical acceleration.
        climb=reference.velocity_world_m_s[2]+1.5*(reference.position_world_m[2]-p[2])+.18*reference.acceleration_world_m_s2[2]
        climb=max(-1.2,min(.5,climb))
        thrust_field=.5+climb/(2*(self.up if climb>=0 else self.down))
        q=quaternion_from_rpy(roll,pitch,math.pi/2)
        command={'quaternion_ned_wxyz':(q[3],q[0],q[1],q[2]),'type_mask':7,
                 'normalized_climb_field':thrust_field,'climb_rate_command_m_s':climb,
                 'roll_command_rad':roll,'pitch_command_rad':pitch,'horizontal_acceleration_command_m_s2':ax}
        command['vertical_force_acceleration_for_attitude_m_s2']=upforce
        if self.braking_tail_guard:command.update(predicted_forward_speed_after_braking_tail_m_s=tail,braking_tail_preview_s=.10 if self.native_rate_mode else .32)
        if self.rate_feedforward:
            # Euler derivatives -> FRD body rates at fixed NED yaw. Sending
            # quaternion plus consistent rates uses native feedforward without
            # changing the native attitude/motor gains or command-slew bound.
            rd=(roll-self.roll)/dt;pd=(pitch-self.pitch)/dt
            command.update(type_mask=0,body_rate_frd_rad_s=(rd,pd*math.cos(roll),-pd*math.sin(roll)))
        if self.native_rate_mode:
            # Close the slower attitude loop here; retain native body-rate,
            # motors and climb-rate dynamics. No firmware/gain modification.
            pose=observation.motion.pose
            bx=pose.rotate((1,0,0));by=pose.rotate((0,1,0));bz=pose.rotate((0,0,1))
            actual_roll=math.atan2(by[2],bz[2]);actual_pitch=math.asin(max(-1.,min(1.,bx[2])))
            actual_yaw=math.pi/2-math.atan2(bx[1],bx[0])
            omega=observation.motion.angular_velocity_world_rad_s
            actual_rates=tuple(sign*sum(a*b for a,b in zip(axis,omega)) for sign,axis in zip((1,-1,-1),(bx,by,bz)))
            yaw_error=(math.pi/2-actual_yaw+math.pi)%(2*math.pi)-math.pi
            errors=(roll-actual_roll,pitch-actual_pitch,yaw_error)
            ff=command.get('body_rate_frd_rad_s',(0.,0.,0.))
            rates=[f+4.5*e-.3*(w-f) for f,e,w in zip(ff,errors,actual_rates)]
            magnitude=math.sqrt(sum(r*r for r in rates));factor=min(1.,math.radians(70)/max(magnitude,1e-12))
            command.update(type_mask=128,body_rate_frd_rad_s=tuple(r*factor for r in rates),
                native_rate_mode=True,attitude_error_frd_rad=errors,measured_body_rate_frd_rad_s=actual_rates)
        self.roll=roll;self.pitch=pitch;self.last_t=now;self.diagnostics=command
        return command


def send_attitude(command,transport,sim_t):
    transport.mav.set_attitude_target_send(int(sim_t*1000)&0xffffffff,1,1,command['type_mask'],
        command['quaternion_ned_wxyz'],*command.get('body_rate_frd_rad_s',(0.,0.,0.)),command['normalized_climb_field'])
