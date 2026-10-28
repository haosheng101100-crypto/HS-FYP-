"""One-way cubic smoothstep braking axis with explicit finite ramp times."""
from .direct_braking import AccelerationPiece

class CubicBrakeAxis:
    def __init__(self,p,v,target,total_duration,ramp_in=.44,ramp_out=.6):
        if v<=0 or target<=p:raise ValueError('Forward stopping distance required')
        self.T=total_duration;self.target=target;self.ramp_in=ramp_in;self.ramp_out=ramp_out
        self.shape='cubic_smoothstep'
        def build(hold):
            A=v/(hold+(ramp_in+ramp_out)/2)
            a=AccelerationPiece(p,v,[0.,0.,-3*A/ramp_in**2,2*A/ramp_in**3],ramp_in)
            b=AccelerationPiece(a.at(a.T),a.at(a.T,1),[-A],hold)
            c=AccelerationPiece(b.at(b.T),b.at(b.T,1),[-A,0.,3*A/ramp_out**2,-2*A/ramp_out**3],ramp_out)
            return A,[(0.,a),(ramp_in,b),(ramp_in+hold,c)],c.at(c.T)
        lo=0.;hi=total_duration-ramp_in-ramp_out
        if hi<=0 or not build(lo)[2]<=target<=build(hi)[2]:raise ValueError('No cubic stopping solution in horizon')
        for _ in range(60):
            mid=(lo+hi)/2
            if build(mid)[2]<target:lo=mid
            else:hi=mid
        self.peak_deceleration,self.pieces,_=build((lo+hi)/2)
        self.stop_s=ramp_in+(lo+hi)/2+ramp_out
        self.coefficients=[{'start_s':t,'duration_s':a.T,'position_coefficients_seconds':a.c} for t,a in self.pieces]
    def at(self,t,order=0):
        if t < -1e-9 or t>self.T+1e-9:raise ValueError('No continuation beyond planned contact')
        if t>=self.stop_s:return self.target if order==0 else 0.
        start,piece=next((s,a) for s,a in reversed(self.pieces) if t>=s)
        return piece.at(t-start,order)
