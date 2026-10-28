"""Distance-based forward stopping demand with unchanged native actuation.

The 0.20 s distance reserve is a design assumption, not an identified lag
bound. This controller requires new physical validation. The local native
command method is copied from the pinned baseline with only the explicit
horizontal-demand hook and diagnostics added; its limits are kept intact.
"""
import math
from .attitude_braking import AttitudeCaptureController
from .braking_candidates import StateSelectedAttitudeController
from .contact_geometry import quaternion_from_rpy


def stopping_demand(position,velocity,target,reserve_s=.20):
    if not all(math.isfinite(x) for x in (position,velocity,target,reserve_s)) or not 0<=reserve_s<=.3:
        raise ValueError('Finite state and declared stopping reserve required')
    distance=target-position;forward=max(0.,velocity)
    usable=max(.02,distance-reserve_s*forward)
    acceleration=-forward*forward/(2*usable)
    return acceleration,{'remaining_target_distance_m':distance,'measured_forward_speed_m_s':velocity,
        'response_distance_reserve_m':reserve_s*forward,'usable_stopping_distance_m':usable,
        'response_reserve_s':reserve_s,'reserve_is_identified_bound':False,'raw_forward_demand_m_s2':acceleration,
        'target_body_x_m':target,'target_shift_used':False,'global_feasibility_claim':False}


class _NativeHorizontalDemandHook(AttitudeCaptureController):
    """Retain the baseline native command calculation after demand selection."""
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
        ax,governor=self.govern_horizontal(reference,observation,now,ax)
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
        command['stopping_governor']=governor
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

class StopDistanceAttitudeController(StateSelectedAttitudeController,_NativeHorizontalDemandHook):
    """MRO keeps the existing late vertical-feedback wrapper around the hook."""
    def govern_horizontal(self,reference,observation,now,ax):
        target=self.curve.terminal_boundary.aircraft.pose.position_world_m[0]
        demand,diagnostics=stopping_demand(observation.motion.pose.position_world_m[0],
            observation.motion.velocity_at_origin_world_m_s[0],target)
        return [demand,ax[1]],diagnostics
