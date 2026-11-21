"""Causal position-measurement observer for the identified edge dynamics.

No plant velocities, contact state, ball positions or future measurements enter
the estimate. Recent issued commands are known inputs, not observations.
"""
import numpy as np


class EdgeObserver:
    def __init__(self, A, B, c, *, attitude_std_deg=.05, body_process_std_deg_s=2.,
                 head_process_std=10., head_measurement_std=.01, use_head_rate=False):
        values=[attitude_std_deg,body_process_std_deg_s,head_process_std,head_measurement_std]
        if not all(np.isfinite(v) and v>0 for v in values):
            raise ValueError('Observer deviations must be positive and finite')
        self.A=A;self.B=B;self.c=c;self.x=None
        self.indices=list(range(6))+([8,9,10,11] if use_head_rate else [])
        self.previous_position=None;self.use_head_rate=use_head_rate
        self.R=np.diag(np.square([attitude_std_deg]*2+[head_measurement_std]*4+([head_measurement_std/.02]*4 if use_head_rate else [])))
        # Position and velocity share the same acceleration uncertainty over
        # the 20 ms control interval. Command history is deterministic.
        G=np.zeros((20,6));G[:6]=.02*np.eye(6);G[6:12]=np.eye(6)
        self.Q=(G*np.square([body_process_std_deg_s]*2+[head_process_std]*4))@G.T
        self.P=np.zeros((20,20));self.P[:6,:6]=self.R[:6,:6]
        self.P[6:12,6:12]=np.diag([100.]*2+[400.]*4)

    def update(self, position, previous_action, history):
        head_rate=np.zeros(4) if self.previous_position is None else (position[2:]-self.previous_position[2:])/.02
        self.previous_position=position.copy();measurement=np.r_[position,head_rate] if self.use_head_rate else position
        if self.x is None:
            self.x=np.r_[position,np.zeros(6),history[0],history[1]]
        else:
            self.x=self.A@self.x+self.B@previous_action+self.c
            self.P=self.A@self.P@self.A.T+self.Q
            selected=self.indices;gain=np.linalg.solve(self.P[np.ix_(selected,selected)]+self.R,self.P[selected,:]).T
            self.x+=gain@(measurement-self.x[selected])
            residual=np.eye(20);residual[:,selected]-=gain
            self.P=residual@self.P@residual.T+gain@self.R@gain.T
            self.P=(self.P+self.P.T)/2
        self.x[12:16]=history[0];self.x[16:20]=history[1]
        return self.x[:12].copy()
