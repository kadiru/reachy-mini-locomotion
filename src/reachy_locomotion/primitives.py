"""Isolated simulation curriculum: lean, tip, catch and head-driven edge pivot.

Unlike walking evaluation, these diagnostic trials may cross 35 degrees and
terminate at first nonfoot contact or 80-degree tilt. Never used by hardware.
"""
import math
from collections import deque
from dataclasses import asdict, dataclass

import mujoco
import numpy as np

from .geometry import euler, rotation
from .sway import SwaySimulator, relative_head_pose


def blend(u):
    u = float(np.clip(u, 0, 1))
    return u*u*u*(10+u*(-15+6*u))


def stroke(u, brake):
    u = float(np.clip(u, 0, 1)); accel = .2; area = 1-(accel+brake)/2
    if u < accel:return u*u/(2*accel*area)
    if u <= 1-brake:return (u-accel/2)/area
    return 1-(1-u)**2/(2*brake*area)


@dataclass
class Primitive:
    preload_roll_deg: float = -20.
    preload_y_mm: float = 0.
    preload_s: float = .5
    roll_deg: float = 35.
    y_mm: float = 0.
    swing_s: float = .12
    pitch_deg: float = 0.
    x_mm: float = 0.
    z_mm: float = 0.
    pump_cycles: int = 0
    pump_hz: float = 3.
    pump_y_phase_deg: float = 0.
    pump_sharpness: float = 0.
    catch_at_deg: float = 0.  # zero means no catch: retain the launch target
    catch_delay_s: float = 0.
    catch_roll_deg: float = 0.
    catch_y_mm: float = 0.
    catch_s: float = .12
    catch_hold_s: float = .2
    yaw_deg: float = 0.
    yaw_at_roll_deg: float = 0.
    yaw_delay_s: float = 0.
    yaw_s: float = .12
    yaw_hold_s: float = .12
    yaw_return_s: float = .3
    yaw_brake_fraction: float = .05

    def __post_init__(self):
        if not all(math.isfinite(x) for x in asdict(self).values()):raise ValueError('Nonfinite primitive')
        if max(abs(self.preload_roll_deg),abs(self.roll_deg),abs(self.catch_roll_deg),abs(self.pitch_deg))>40:
            raise ValueError('Head angle outside existing exploration bounds')
        if max(abs(self.preload_y_mm),abs(self.y_mm),abs(self.catch_y_mm),abs(self.z_mm),abs(self.x_mm))>25 or abs(self.yaw_deg)>60:
            raise ValueError('Head translation/yaw outside existing bounds')
        if any(not .04<=x<=1.5 for x in [self.preload_s,self.swing_s,self.catch_s,self.yaw_s,self.yaw_return_s]):
            raise ValueError('Invalid motion duration')
        if not 0<=self.catch_at_deg<=60 or not 0<=self.yaw_at_roll_deg<=60:raise ValueError('Invalid trigger')
        if any(not 0<=x<=1 for x in [self.catch_delay_s,self.catch_hold_s,self.yaw_delay_s,self.yaw_hold_s]):
            raise ValueError('Invalid hold/delay')
        if not .025<=self.yaw_brake_fraction<=.4:raise ValueError('Invalid braking fraction')
        if type(self.pump_cycles) is not int or not 0<=self.pump_cycles<=20 or not .5<=self.pump_hz<=5 or not 0<=self.pump_sharpness<=4:
            raise ValueError('Invalid pumping waveform')


class PrimitiveController:
    """20 ms delayed simulated roll sensing; no contact/COM oracle in control."""
    def __init__(self, recipe):
        self.p=recipe;self.roll_history=deque([0.],maxlen=2)
        self.catch_trigger=None;self.catch_origin=None;self.yaw_trigger=None

    def target(self,t,roll_deg):
        p=self.p;self.roll_history.append(roll_deg);sensed=self.roll_history[0]
        start=.2;launch=start+p.preload_s
        preload=np.array([p.preload_roll_deg,p.preload_y_mm,p.pitch_deg,p.z_mm])
        goal=np.array([p.roll_deg,p.y_mm,p.pitch_deg,p.z_mm])
        if t<launch:
            v=blend((t-start)/p.preload_s)*preload;stage='preload' if t>=start else 'neutral'
        else:
            v=preload+blend((t-launch)/p.swing_s)*(goal-preload);stage='swing' if t<launch+p.swing_s else 'hold'
        if p.pump_cycles and t>=launch:
            hz=50/max(10,round(50/p.pump_hz));phase=2*math.pi*hz*(t-launch)
            if phase<(p.pump_cycles+.5)*2*math.pi:
                def wave(a):
                    x=-math.cos(a)
                    return math.tanh(p.pump_sharpness*x)/math.tanh(p.pump_sharpness) if p.pump_sharpness else x
                v=(preload+goal)/2+(goal-preload)/2*wave(phase)
                phased=(preload[1]+goal[1])/2+(goal[1]-preload[1])/2*wave(phase+math.radians(p.pump_y_phase_deg))
                v[1]+=(phased-v[1])*blend((t-launch)/.2);stage='pump'
            else:v=goal.copy();stage='hold'
        if p.catch_at_deg and t>=launch and self.catch_trigger is None and abs(sensed)>=p.catch_at_deg:
            self.catch_trigger=t
        if self.catch_trigger is not None and t>=self.catch_trigger+p.catch_delay_s:
            if self.catch_origin is None:self.catch_origin=v.copy()
            elapsed=t-self.catch_trigger-p.catch_delay_s
            catch=np.array([p.catch_roll_deg,p.catch_y_mm,p.pitch_deg,p.z_mm])
            v=self.catch_origin+blend(elapsed/p.catch_s)*(catch-self.catch_origin);stage='catch'
            if elapsed>=p.catch_s+p.catch_hold_s:
                v=catch*(1-blend((elapsed-p.catch_s-p.catch_hold_s)/.4));stage='return/settle'
        if p.yaw_deg and t>=launch+p.yaw_delay_s and self.yaw_trigger is None and abs(sensed)>=p.yaw_at_roll_deg:
            self.yaw_trigger=t
        yaw=0.
        if self.yaw_trigger is not None:
            elapsed=t-self.yaw_trigger
            yaw=p.yaw_deg*stroke(elapsed/p.yaw_s,p.yaw_brake_fraction)
            if elapsed>=p.yaw_s+p.yaw_hold_s:
                yaw=p.yaw_deg*(1-blend((elapsed-p.yaw_s-p.yaw_hold_s)/p.yaw_return_s))
        pose=np.eye(4);pose[:3,:3]=rotation(np.radians([v[0],v[2],yaw]));pose[:3,3]=[p.x_mm*blend((t-start)/p.preload_s)/1000,v[1]/1000,.177+v[3]/1000]
        return pose,stage,sensed


def classify_trial(reason,tail,peak_roll):
    fell=reason in ('nonfoot_floor_contact','tilt_reached_80_deg')
    complete=reason is None
    settled=bool(tail) and all(x['tilt_deg']<3 and x['base_angular_speed_rad_s']<.2 and x['base_speed_m_s']<.02 and x['normal_n']>.5 for x in tail)
    return dict(fell=fell,completed=complete,model_valid=complete or fell,
                settled=complete and settled,recovered=complete and settled and peak_roll>=6)


class PrimitiveSimulator(SwaySimulator):
    def run_primitive(self,recipe,seconds=3.6,waist_mode='off',record=False,
                      controller=None,tilt_limit_deg=80,finish_settle_s=None,diagnostic_initial_roll_deg=None,
                      diagnostic_initial_pitch_deg=None):
        if not isinstance(recipe,Primitive):recipe=Primitive(**recipe)
        if waist_mode not in ('off','hold_neutral'):raise ValueError('Unknown waist mode')
        if not 1<=seconds<=600 or self.servos is None:raise ValueError('Require bounded datasheet simulation')
        if tilt_limit_deg not in (35,80):raise ValueError('Use walking or diagnostic tilt guard')
        if finish_settle_s is not None and not .6<=finish_settle_s<=2:
            raise ValueError('Require at least 0.6 seconds of physical finish settling')
        m=self.model;d=mujoco.MjData(m);d.qpos[:]=self.initial_qpos;d.qvel[:]=self.initial_qvel;d.ctrl[:]=self.initial_ctrl
        self.servos.reset(self.initial_targets)
        waist=m.actuator('yaw_body').id;torso=m.body('body_down_3dprint').id
        head_act=np.array([i for i in range(m.nu) if m.actuator(i).name.startswith('stewart')])
        if waist_mode=='off':d.ctrl[waist]=0
        # Excluded settling uses the same waist mode as the measured trial.
        for _ in range(round(.5/self.dt)):
            self.servos.apply(d)
            if waist_mode=='off':d.ctrl[waist]=0
            mujoco.mj_step(m,d)
        mujoco.mj_forward(m,d);d.time=0.
        # Curriculum-only reset, explicitly marked in the result. Never used
        # as evidence that the head can launch/capture from upright.
        if diagnostic_initial_roll_deg is not None and diagnostic_initial_pitch_deg is not None:
            raise ValueError('Choose one diagnostic reset axis')
        reset_angle=diagnostic_initial_roll_deg if diagnostic_initial_roll_deg is not None else diagnostic_initial_pitch_deg
        if reset_angle is not None:
            if not math.isfinite(reset_angle) or not 20<=abs(reset_angle)<=33:
                raise ValueError('Diagnostic edge reset must be between 20 and 33 degrees')
            angle=math.radians(reset_angle);qa=m.jnt_qposadr[m.joint('floating_foot').id]
            d.qpos[qa+2]=self.config['foot_radius_m']*abs(math.sin(angle))-.00002
            d.qpos[qa+3:qa+7]=[math.cos(angle/2),math.sin(angle/2) if diagnostic_initial_roll_deg is not None else 0,
                              math.sin(angle/2) if diagnostic_initial_pitch_deg is not None else 0,0]
            d.qvel[:]=0.;mujoco.mj_forward(m,d)
        if any(w.number for w in d.warning):raise ValueError('Initialization warning')
        start=d.xpos[self.base].copy();waist_qadr=self.servos.qadr[waist];waist_start=float(d.qpos[waist_qadr])
        bodies=[self.base,torso];previous=[math.atan2(d.xmat[i][3],d.xmat[i][0]) for i in bodies]
        angles=np.zeros(2);initial_yaw=previous[0];controller=controller or PrimitiveController(recipe)
        trace=[];trajectory=[];reason=None;first35=None;peak_roll=peak_tilt=peak_lift=peak_slip=0.
        peak_torque=np.zeros(m.nu);waist_work=head_braking=0.;joint_margin=1e9;max_closure=0.
        edge_time=edge_yaw=slip_distance=0.;last_roll=last_pitch=0.;previous_edge=False
        total_frames=round(seconds/.02);base_velocity=np.zeros(6)
        finished_at=None;duration_budget=seconds;early_finish=False
        for frame in range(total_frames+1):
            t=frame*.02;R=d.xmat[self.base].reshape(3,3);rpy=euler(R);roll=math.degrees(rpy[0])
            tilt=math.degrees(math.acos(float(np.clip(R[2,2],-1,1))))
            yaw_step=0.
            for j,body in enumerate(bodies):
                yaw=math.atan2(d.xmat[body][3],d.xmat[body][0]);delta=(yaw-previous[j]+math.pi)%(2*math.pi)-math.pi
                angles[j]+=delta;previous[j]=yaw
                if j==0:yaw_step=delta
            if hasattr(controller,'observe_heading'):controller.observe_heading(t,math.degrees(angles[0]))
            if hasattr(controller,'observe_head'):
                controller.observe_head(t,math.degrees(euler(relative_head_pose(m,d))[2]))
            if hasattr(controller,'observe_attitude'):
                controller.observe_attitude(t,math.degrees(rpy[1]))
            if hasattr(controller,'observe_head_state'):
                controller.observe_head_state(t,relative_head_pose(m,d))
            roll_rate=(roll-last_roll)/.02 if frame else 0.;last_roll=roll
            pitch=math.degrees(rpy[1]);pitch_rate=(pitch-last_pitch)/.02 if frame else 0.;last_pitch=pitch
            forces=np.zeros(2);foreaft_forces=np.zeros(2);points=[];weights=[]
            for ci,c in enumerate(d.contact):
                if {int(c.geom1),int(c.geom2)}=={self.foot,self.floor} and c.efc_address>=0:
                    wrench=np.zeros(6);mujoco.mj_contactForce(m,d,ci,wrench);normal=abs(float(wrench[0]))
                    local=R.T@(c.pos-d.xpos[self.base]);forces[int(local[1]>=0)]+=normal
                    foreaft_forces[int(local[0]>=0)]+=normal
                    if normal>.01:points.append(c.pos.copy());weights.append(normal)
            rims=[d.xpos[self.base]+R@np.array([0,sign*self.config['foot_radius_m'],0]) for sign in [-1,1]]
            lift=abs(float(rims[1][2]-rims[0][2]))*1000
            edge=bool(min(forces)<.1 and max(forces)>.5 and lift>=10)
            foreaft_lift=abs(2*self.config['foot_radius_m']*float(R[2,0]))*1000
            foreaft_edge=bool(min(foreaft_forces)<.1 and max(foreaft_forces)>.5 and foreaft_lift>=10)
            slip=0.;contact=None
            if points:
                contact=np.average(np.asarray(points),axis=0,weights=weights)
                # Velocity of the material foot point currently at the loaded
                # contact; movement of the contact centroid is NOT a slip proxy.
                jac=np.zeros((3,m.nv));mujoco.mj_jac(m,d,jac,None,contact,self.base)
                slip=float(np.linalg.norm((jac@d.qvel)[:2]))
            if edge and frame:
                edge_time+=.02;slip_distance+=slip*.02
                if previous_edge:edge_yaw+=yaw_step
            previous_edge=edge;peak_slip=max(peak_slip,slip)
            mujoco.mj_objectVelocity(m,d,mujoco.mjtObj.mjOBJ_XBODY,self.base,base_velocity,0)
            row=dict(time_s=t,roll_deg=roll,pitch_deg=pitch,roll_rate_deg_s=roll_rate,pitch_rate_deg_s=pitch_rate,tilt_deg=tilt,
                     foreaft_edge_support=foreaft_edge,front_back_height_difference_mm=foreaft_lift,
                     foot_front_back_normal_forces_n_back_front=foreaft_forces.tolist(),
                     side_height_difference_mm=lift,foot_yaw_deg=math.degrees(angles[0]),torso_yaw_deg=math.degrees(angles[1]),
                     waist_angle_change_deg=math.degrees(float(d.qpos[waist_qadr])-waist_start),
                     normal_n=float(sum(forces)),edge_support=edge,contact_slip_m_s=slip,
                     base_angular_speed_rad_s=float(np.linalg.norm(base_velocity[:3])),base_speed_m_s=float(np.linalg.norm(base_velocity[3:])),
                     com_world_m=d.subtree_com[self.base].tolist(),contact_world_m=contact.tolist() if contact is not None else None,
                     foot_xyz_m=d.xpos[self.base].tolist())
            pose,stage,sensed=controller.target(t,roll);row.update(stage=stage,sensed_roll_deg=sensed);trace.append(row)
            if hasattr(controller,'diagnostics'):row.update(controller.diagnostics())
            if finish_settle_s is not None and getattr(controller,'finished',False):
                if finished_at is None:finished_at=t
                if t-finished_at>=finish_settle_s-1e-9:
                    early_finish=True;seconds=t
            peak_roll=max(peak_roll,abs(roll));peak_tilt=max(peak_tilt,tilt);peak_lift=max(peak_lift,lift)
            if record:
                trajectory.append(dict(time_s=t,measuring=True,qpos=d.qpos.tolist(),qvel=d.qvel.tolist(),
                                       foot_xyz_m=d.xpos[self.base].tolist(),foot_rpy_rad=rpy.tolist(),
                                       head_rpy_rad=euler(relative_head_pose(m,d)).tolist(),target_head_rpy_rad=euler(pose).tolist(),
                                       heading_from_startup_rad=float(angles[0]),foot_side_normal_forces_n_right_left=forces.tolist(),
                                       foot_front_back_normal_forces_n_back_front=foreaft_forces.tolist(),front_back_height_difference_mm=foreaft_lift,
                                       primitive_stage=stage,side_height_difference_mm=lift,torso_yaw_deg=math.degrees(angles[1]),
                                       waist_torque_nm=float(d.actuator_force[waist])))
            if frame==total_frames or early_finish:break
            try:target=self.checked_ik(pose)
            except ValueError as error:reason='invalid_head_target: '+str(error);break
            targets=self.initial_targets.copy();targets[self.act]=target;targets[waist]=self.initial_targets[waist]
            self.servos.command(targets)
            for _ in range(self.substeps):
                self.servos.apply(d)
                if waist_mode=='off':d.ctrl[waist]=0.
                mujoco.mj_step(m,d);mujoco.mj_forward(m,d)
                if not np.isfinite(d.qpos).all() or not np.isfinite(d.qvel).all() or any(w.number for w in d.warning):reason='numerical_warning';break
                tilt=math.degrees(math.acos(float(np.clip(d.xmat[self.base][8],-1,1))))
                peak_tilt=max(peak_tilt,tilt)
                peak_roll=max(peak_roll,abs(math.degrees(math.atan2(d.xmat[self.base][7],d.xmat[self.base][8]))))
                peak_lift=max(peak_lift,abs(2*self.config['foot_radius_m']*float(d.xmat[self.base][7]))*1000)
                if tilt>=35 and first35 is None:first35=float(d.time)
                if tilt>=tilt_limit_deg:reason=f'tilt_reached_{tilt_limit_deg}_deg'
                closure=float(np.max(np.linalg.norm(d.site_xpos[self.closing_a]-d.site_xpos[self.closing_b],axis=1)))
                max_closure=max(max_closure,closure)
                if closure>.001:reason='loop_closure_error_exceeded_1_mm'
                q=d.qpos[self.joint_qpos];margin=float(min(np.min(q-self.command_limits[:,0]),np.min(self.command_limits[:,1]-q)))
                joint_margin=min(joint_margin,margin)
                if margin<-.005:reason='actual_joint_limit_exceeded'
                for ci,c in enumerate(d.contact):
                    pair={int(c.geom1),int(c.geom2)}
                    if self.floor in pair and self.foot not in pair and c.efc_address>=0:
                        wrench=np.zeros(6);mujoco.mj_contactForce(m,d,ci,wrench)
                        if abs(wrench[0])>.5 and reason in (None,f'tilt_reached_{tilt_limit_deg}_deg'):reason='nonfoot_floor_contact'
                peak_torque=np.maximum(peak_torque,np.abs(d.actuator_force))
                waist_work+=abs(float(d.actuator_force[waist]*d.actuator_velocity[waist]))*self.dt
                head_braking+=float(np.maximum(-d.actuator_force[head_act]*d.actuator_velocity[head_act],0).sum())*self.dt
                if reason:break
            if reason:break
        tail=[x for x in trace if x['time_s']>=seconds-.6]
        flags=classify_trial(reason,tail,peak_roll)
        terminal=dict(time_s=float(d.time),foot_rpy_rad=euler(d.xmat[self.base].reshape(3,3)).tolist(),
                      qpos=d.qpos.tolist(),qvel=d.qvel.tolist(),com_world_m=d.subtree_com[self.base].tolist())
        feedback=getattr(getattr(controller,'child',controller),'f',None)
        sensor_delay=.02*feedback.sensor_delay_frames if feedback is not None else .02
        result=dict(evidence='uncalibrated_simulation',hardware_ready=False,primitive_trial=True,recipe=asdict(recipe),config=self.config,
                    requested_seconds=seconds,elapsed_seconds=float(d.time),termination=reason or 'completed',**flags,
                    max_duration_seconds=duration_budget,stopped_after_controller_finish=early_finish,
                    diagnostic_falls_allowed=tilt_limit_deg==80,tilt_limit_deg=tilt_limit_deg,walking_fall_guard_unchanged=True,waist_mode=waist_mode,
                    diagnostic_initial_roll_deg=diagnostic_initial_roll_deg,diagnostic_initial_pitch_deg=diagnostic_initial_pitch_deg,
                    natural_upright_start=reset_angle is None,
                    waist_peak_torque_nm=float(peak_torque[waist]),waist_absolute_motor_work_j=waist_work,
                    commanded_waist_position_range_rad=[float(self.initial_targets[waist])]*2,
                    measured_waist_angle_range_deg=[min(x['waist_angle_change_deg'] for x in trace),max(x['waist_angle_change_deg'] for x in trace)],
                    peak_motor_torque_nm=peak_torque.tolist(),head_braking_work_j=head_braking,min_joint_margin_rad=joint_margin,
                    max_closure_error_m=max_closure,peak_roll_deg=peak_roll,peak_tilt_deg=peak_tilt,first_35deg_time_s=first35,
                    peak_side_height_difference_mm=peak_lift,edge_support_seconds=edge_time,edge_supported_foot_yaw_deg=math.degrees(edge_yaw),
                    edge_contact_slip_distance_mm=slip_distance*1000,peak_contact_slip_m_s=peak_slip,
                    final_foot_yaw_deg=math.degrees(angles[0]),final_torso_yaw_deg=math.degrees(angles[1]),
                    planar_displacement_m=(d.xpos[self.base]-start)[:2].tolist(),
                    catch_trigger_s=controller.catch_trigger,yaw_trigger_s=controller.yaw_trigger,
                    peak_sampled_roll_rate_deg_s=max(abs(x['roll_rate_deg_s']) for x in trace),terminal_state=terminal,
                    control_rate_hz=50,roll_sensor_delay_s=sensor_delay,roll_sensor_assumption='simulated base roll; physical estimator not calibrated',
                    external_forces_applied=False,upstream_xml_sha256=self.source_sha256,kinematics_sha256=self.ik.source_sha256,
                    trace=trace,motor_diagnostics=self.servos.report(),render_forward_frame='startup')
        if waist_mode=='off':assert result['waist_peak_torque_nm']==0 and waist_work==0
        if record:result['trajectory']=trajectory
        return result
