"""Causal joint command limiting with discrete braking distance near joint bounds."""
from lab_common import configure
configure()
import numpy as np


class CommandLimiter:
    def __init__(self,q,lo,hi,dt=1/30,speed=4.,acceleration=20.,brake_at_target=False):
        self.q=q.copy();self.v=np.zeros_like(q);self.lo=lo;self.hi=hi
        self.dt=dt;self.speed=speed;self.acceleration=acceleration
        self.brake_at_target=brake_at_target

    def stoppable_speed(self,distance):
        # Includes the next commanded step and subsequent maximum-braking steps.
        low=np.zeros_like(distance);high=np.full_like(distance,self.speed)
        step=self.acceleration*self.dt
        for _ in range(35):
            mid=(low+high)*.5;n=np.ceil(mid/step)
            travel=self.dt*(n*mid-step*n*(n-1)*.5)
            feasible=travel<=np.maximum(distance,0)
            low=np.where(feasible,mid,low);high=np.where(feasible,high,mid)
        return low

    def update(self,target):
        dv=self.acceleration*self.dt
        lower=np.maximum(self.v-dv,-self.stoppable_speed(self.q-self.lo))
        upper=np.minimum(self.v+dv,self.stoppable_speed(self.hi-self.q))
        if np.max(lower-upper)>1e-8:raise RuntimeError('Command state cannot brake before a joint bound')
        error=target-self.q
        desired=np.sign(error)*self.stoppable_speed(abs(error)) if self.brake_at_target else error/self.dt
        velocity=np.clip(desired,lower,upper)
        command=self.q+velocity*self.dt
        if np.any(command<self.lo-1e-9) or np.any(command>self.hi+1e-9):raise RuntimeError('Command joint bound violated')
        self.q=command;self.v=velocity
        return command.copy()
