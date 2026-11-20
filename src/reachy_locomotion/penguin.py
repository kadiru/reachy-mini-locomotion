"""Compose balance-controlled edge motions without resetting physical state."""
import math
from collections import deque
import numpy as np

from .balance import BalanceController,mirrored


class PenguinController:
    def __init__(self,recipe,feedback,cycles=3,min_step_s=2.4,max_step_s=5.,start_side=1,checked_ik=None,
                 target_step_turn_deg=0.,max_repeats=1,pivot_only=False,heading_noise_deg=0.,heading_seed=0,
                 fallback_policy=None,switch_below_deg=3.,settle_s=.14,post_catch_s=1.2,head_return_margin_s=.2,
                 head_settle_yaw_deg=0.,head_settle_rate_deg_s=15.,delayed_sensor_fallback=None,
                 rescue_after_weak_fallback_deg=0.):
        if type(cycles) is not int or not 1<=cycles<=8 or start_side not in (-1,1):
            raise ValueError('Invalid cycle count or starting side')
        if not .8<=min_step_s<=max_step_s<=8 or type(max_repeats) is not int or not 1<=max_repeats<=4:
            raise ValueError('Invalid settling/repetition budget')
        if not .04<=settle_s<=.4 or not .4<=post_catch_s<=2 or not 0<=head_return_margin_s<=.5:
            raise ValueError('Invalid phase-completion timing')
        if not 0<=head_settle_yaw_deg<=5 or not 5<=head_settle_rate_deg_s<=30:
            raise ValueError('Invalid measured head settling criterion')
        if not 0<=target_step_turn_deg<=30 or not 0<=heading_noise_deg<=1 or not 0<=switch_below_deg<=10:
            raise ValueError('Invalid heading feedback')
        if fallback_policy and fallback_policy['feedback'].get('return_x_after_catch_s',-1)<0:
            raise ValueError('Fallback primitives require a physical neutral return')
        if delayed_sensor_fallback and (not fallback_policy or delayed_sensor_fallback['feedback'].get('return_x_after_catch_s',-1)<0):
            raise ValueError('Delayed-sensor fallback requires the same neutral-return contract')
        if not 0<=rescue_after_weak_fallback_deg<=10:raise ValueError('Invalid fallback progress threshold')
        self.recipe=dict(recipe);self.feedback=dict(feedback)
        self.cycles=cycles;self.min_step_s=min_step_s;self.max_step_s=max_step_s
        self.settle_s=settle_s;self.post_catch_s=post_catch_s;self.head_return_margin_s=head_return_margin_s
        self.head_settle_yaw_deg=head_settle_yaw_deg;self.head_settle_rate_deg_s=head_settle_rate_deg_s
        self.measured_head_history=deque([0.,0.],maxlen=2);self.measured_head_yaw=0.;self.measured_head_rate=0.
        self.start_side=start_side;self.side=start_side
        self.checked_ik=checked_ik
        self.child=BalanceController(mirrored(recipe,self.side),feedback,checked_ik=checked_ik)
        self.started=0.;self.ready_since=None;self.steps=[];self.heading=0.
        self.catch_trigger=None;self.yaw_trigger=None;self.finished=False;self.timed_out=False
        self.target_turn=target_step_turn_deg;self.max_repeats=max_repeats;self.pivot_only=pivot_only
        self.requested_steps=1 if pivot_only else 2*cycles
        if target_step_turn_deg and feedback.get('return_x_after_catch_s',-1)<0:
            raise ValueError('Repeated primitives require a physical neutral return')
        self.bursts=[];self.repeats=0;self.half_started=0.;self.goal_missed=False
        self.heading_queue=deque([0.,0.],maxlen=2);self.sensed_heading=0.
        self.heading_noise=heading_noise_deg;self.heading_rng=np.random.default_rng(heading_seed)
        self.fallback_policy=fallback_policy;self.fallback_active=False;self.switch_below_deg=switch_below_deg
        self.delayed_sensor_fallback=delayed_sensor_fallback;self.active_policy='primary'
        self.rescue_threshold=rescue_after_weak_fallback_deg;self.rescue_active=False
        self.burst_start_heading=0.

    def observe_heading(self,t,heading_deg):
        self.heading=heading_deg
        self.heading_queue.append(heading_deg+float(self.heading_rng.normal(0,self.heading_noise)))
        self.sensed_heading=self.heading_queue[0]

    def observe_head(self,t,yaw_deg):
        self.measured_head_history.append(yaw_deg)
        delayed=self.measured_head_history[0]
        raw=(delayed-self.measured_head_yaw)/.02 if t else 0.
        self.measured_head_rate+=(1-math.exp(-.02/.04))*(raw-self.measured_head_rate)
        self.measured_head_yaw=delayed
        self.child.observe_head(t,yaw_deg)

    def target(self,t,roll_deg):
        local=t-self.started
        pose,stage,sensed=self.child.target(local,roll_deg)
        if self.steps and self.child.f.return_x_after_catch_s<0:
            # Previous primitive ends at this forward head offset. Keep it,
            # rather than introducing an artificial reset at each new step.
            pose[0,3]=self.recipe['x_mm']/1000
        if self.child.catch_trigger is not None and self.catch_trigger is None:
            self.catch_trigger=t-local+self.child.catch_trigger
        if self.child.yaw_trigger is not None and self.yaw_trigger is None:
            self.yaw_trigger=t-local+self.child.yaw_trigger
        ready=(local>=self.min_step_s and abs(sensed)<2 and abs(self.child.rate)<10
               and self.child.catch_trigger is not None
               and local>self.child.catch_trigger+self.post_catch_s
               and (self.child.yaw_trigger is None or local>self.child.yaw_trigger+self.child.p.yaw_s+self.child.p.yaw_hold_s+self.child.p.yaw_return_s))
        if self.child.f.state_dependent_yaw and self.child.yaw_trigger is not None:
            ready=ready and self.child.yaw_return_start is not None and local>self.child.yaw_return_start+self.child.p.yaw_return_s
        if self.child.f.return_x_after_catch_s>=0 and self.child.catch_trigger is not None:
            ready=ready and local>self.child.catch_trigger+self.child.f.return_x_after_catch_s+self.child.f.return_x_duration_s+self.head_return_margin_s
        if self.head_settle_yaw_deg:
            ready=ready and abs(self.measured_head_yaw)<self.head_settle_yaw_deg and abs(self.measured_head_rate)<self.head_settle_rate_deg_s
        if ready:
            if self.ready_since is None:self.ready_since=t
        else:self.ready_since=None
        if not self.finished and self.ready_since is not None and t-self.ready_since>=self.settle_s-1e-9:
            self.repeats+=1
            progress=-self.side*(self.sensed_heading-self.burst_start_heading)
            self.bursts.append(dict(side=self.side,start_s=self.started,end_s=t,heading_deg=self.heading,
                                   observed_turn_progress_deg=progress,policy=self.active_policy,
                                   emergency_recovery=self.child.emergency_trigger is not None))
            if self.active_policy=='fallback' and self.rescue_threshold and progress<self.rescue_threshold:
                self.rescue_active=True
            if self.fallback_policy and progress<self.switch_below_deg:self.fallback_active=True
            target=-self.start_side*self.target_turn if len(self.steps)%2==0 else 0.
            achieved=not self.target_turn or -self.side*(self.sensed_heading-target)>=-2
            half_complete=achieved or self.repeats>=self.max_repeats
            if half_complete:
                self.goal_missed|=not achieved
                self.steps.append(dict(index=len(self.steps),side=self.side,start_s=self.half_started,end_s=t,
                                       heading_deg=self.heading,target_heading_deg=target,repeats=self.repeats,turn_goal_met=achieved))
                if len(self.steps)>=self.requested_steps or not achieved:self.finished=True
                if not self.finished:self.side=-self.side
                self.repeats=0;self.half_started=t+.02
            if not self.finished:
                previous=self.child
                self.started=t+.02;self.ready_since=None
                alternate=self.fallback_policy
                self.active_policy='fallback' if self.fallback_active else 'primary'
                if (self.fallback_active and self.delayed_sensor_fallback and self.feedback.get('sensor_delay_frames',1)>=2
                    and (not self.rescue_threshold or self.rescue_active)):
                    alternate=self.delayed_sensor_fallback;self.active_policy='fallback_delayed_sensor'
                g=alternate['recipe'] if self.fallback_active else self.recipe
                f=alternate['feedback'] if self.fallback_active else self.feedback
                self.child=BalanceController(mirrored(g,self.side),f,checked_ik=self.checked_ik)
                self.burst_start_heading=self.sensed_heading
                self.child.history=previous.history.copy();self.child.previous_roll=previous.previous_roll;self.child.rate=previous.rate
                self.child.rng=previous.rng
                self.child.head_history=previous.head_history.copy();self.child.head_yaw=previous.head_yaw;self.child.head_rate=previous.head_rate
        if not self.finished and local>self.max_step_s:self.timed_out=True
        return pose,('L ' if self.side<0 else 'R ')+stage,sensed

    def diagnostics(self):
        return dict(self.child.diagnostics(),step_index=len(self.steps),active_side=self.side,
                    completed_pair_count=len(self.steps)//2,primitive_bursts=len(self.bursts),sensed_heading_deg=self.sensed_heading,
                    measured_head_yaw_deg=self.measured_head_yaw,measured_head_rate_deg_s=self.measured_head_rate)

    def report(self):
        headings=[s['heading_deg'] for i,s in enumerate(self.steps) if i%2]
        return dict(completed_steps=len(self.steps),requested_steps=self.requested_steps,steps=self.steps,bursts=self.bursts,
                    completed=self.finished and not self.timed_out and not self.goal_missed,
                    max_completed_cycle_heading_deg=max(map(abs,headings),default=0.),
                    cycle_end_headings_deg=headings,physical_state_resets_between_steps=False,
                    target_step_turn_deg=self.target_turn,heading_sensor_delay_s=.02,heading_sensor_noise_deg=self.heading_noise,
                    fallback_activated=self.fallback_active,
                    sequencing=dict(min_step_s=self.min_step_s,settle_s=self.settle_s,post_catch_s=self.post_catch_s,
                                    head_return_margin_s=self.head_return_margin_s,head_settle_yaw_deg=self.head_settle_yaw_deg,
                                    head_settle_rate_deg_s=self.head_settle_rate_deg_s))
