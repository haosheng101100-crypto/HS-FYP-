"""Causal forward acceleration sharing with steady-speed velocity anchors.

The input is a measured physical IMU acceleration in world x. Only past
samples are integrated. This initial simulation has no added sensor noise;
the class does not represent a real radio or a validated real-world INS.
"""
from collections import deque
import math,bisect

class ForwardAccelerationShare:
    def __init__(self,max_age=.05):
        self.samples=deque(maxlen=4096);self.integrals=deque(maxlen=4096);self.anchors=deque(maxlen=512);self.max_age=max_age
    def push(self,t,ax):
        if not all(math.isfinite(x) for x in (t,ax)):raise ValueError('Finite IMU packet required')
        if self.samples and t<self.samples[-1][0]:raise ValueError('IMU time reversal')
        if self.samples and t==self.samples[-1][0]:return
        integral=0. if not self.samples else self.integrals[-1][1]+(self.samples[-1][1]+ax)*.5*(t-self.samples[-1][0])
        self.samples.append((t,ax));self.integrals.append((t,integral))
    def sample_at(self,t):
        packet=next((p for p in reversed(self.samples) if p[0]<=t+1e-10),None)
        if packet is None or not -1e-10<=t-packet[0]<=self.max_age:raise ValueError('No fresh causal vehicle IMU packet')
        return packet
    def anchor(self,t,vx):
        self.sample_at(t)
        if not math.isfinite(vx):raise ValueError('Finite velocity anchor required')
        if not self.anchors or t>self.anchors[-1][0]:self.anchors.append((t,vx,self.integral_at(t)))
    def integral_at(self,t):
        at,ax=self.sample_at(t)
        integral=next(v for stamp,v in reversed(self.integrals) if stamp==at)
        return integral+ax*(t-at)
    def velocity(self,t):
        self.sample_at(t)
        anchor=next((p for p in reversed(self.anchors) if p[0]<=t+1e-10),None)
        if anchor is None:raise ValueError('No causal steady-speed anchor')
        begin,v,integral_at_anchor=anchor
        return v+self.integral_at(t)-integral_at_anchor
