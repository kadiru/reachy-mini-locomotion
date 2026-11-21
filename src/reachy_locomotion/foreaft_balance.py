"""Front/back edge control in a rotated observation/command frame.

Only coordinates are rotated. The physical robot, mass, contacts, motor limits
and waist servo are unchanged. Fore-aft dynamics are identified independently.
"""
import math
import numpy as np

from .edge_linear import LinearEdgeController
from .geometry import euler,rotation

# Virtual x=physical y; virtual y=physical -x. Positive virtual roll is
# physical forward pitch, supported on the front (+physical x) rim.
Q=np.array([[0.,-1.,0.],[1.,0.,0.],[0.,0.,1.]])


def physical_pose(virtual):
    pose=np.eye(4);pose[:3,:3]=Q@virtual[:3,:3]@Q.T;pose[:3,3]=Q@virtual[:3,3]
    return pose


def virtual_pose(physical):
    pose=np.eye(4);pose[:3,:3]=Q.T@physical[:3,:3]@Q;pose[:3,3]=Q.T@physical[:3,3]
    return pose


class ForeAftController:
    def __init__(self,recipe,feedback,side,checked_ik,linear=None,excitation_seed=None):
        self.physical_ik=checked_ik;self.pitch=0.;self.side=side
        self.child=LinearEdgeController(recipe,feedback,side,self.checked_virtual_ik,linear,excitation_seed)

    def __getattr__(self,name):return getattr(self.child,name)

    def checked_virtual_ik(self,pose):
        physical=physical_pose(pose)
        if max(abs(np.degrees(euler(physical)[:2])))>40+1e-9:raise ValueError('Physical head angle exceeds exploration envelope')
        return self.physical_ik(physical)

    def observe_attitude(self,t,pitch_deg):self.pitch=pitch_deg

    def observe_head_state(self,t,pose):self.child.observe_head_state(t,virtual_pose(pose))

    def observe_angular_velocity(self,t,body_angular_velocity_rad_s):
        self.child.observe_angular_velocity(t,Q.T@body_angular_velocity_rad_s)

    def target(self,t,roll_deg):
        body=rotation(np.radians([roll_deg,self.pitch,0]))
        angles=np.degrees(euler(Q.T@body@Q))
        self.child.observe_attitude(t,float(angles[1]))
        pose,stage,sensed=self.child.target(t,float(angles[0]))
        return physical_pose(pose),'fore-aft '+stage,sensed

    def diagnostics(self):
        data=self.child.diagnostics()
        result={('virtual_'+k if not k.startswith('edge_') else k):v for k,v in data.items()}
        result.update(sensed_roll_deg=None,foreaft_sensed_axis_deg=float(self.child.sensed[0]),
                      controller_coordinate_frame='virtual x=physical y; virtual y=physical -x; z unchanged')
        return result


def foreaft_metrics(result,side,target):
    rows=result['trace'];previous=False;last_time=None;continuous=best=soft=0.;flags=[]
    for row in rows:
        error=abs(side*row['pitch_deg']-target);forces=row['foot_front_back_normal_forces_n_back_front']
        loaded=1 if side==1 else 0
        ok=(row['foreaft_edge_support'] and forces[loaded]>.5 and forces[1-loaded]<.1 and error<=2
            and abs(row['pitch_rate_deg_s'])<=5 and abs(row['roll_deg'])<=5)
        continuous=continuous+row['time_s']-last_time if ok and previous else 0.
        best=max(best,continuous);previous=ok;last_time=row['time_s'];flags.append(ok)
        if row['foreaft_edge_support'] and forces[loaded]>.5:
            soft+=.02*math.exp(-.5*(error/5)**2-.5*(row['pitch_rate_deg_s']/35)**2-.5*(row['roll_deg']/5)**2)
    tail=rows[-251:]
    return dict(longest_strict_hold_s=round(best,5),soft_hold_s=soft,
                final_five_seconds_pass=bool(result['completed'] and len(tail)==251 and all(flags[-251:])),
                target_pitch_deg=side*target,final_pitch_deg=rows[-1]['pitch_deg'],
                tail_pitch_range_deg=[min(x['pitch_deg'] for x in tail),max(x['pitch_deg'] for x in tail)],
                tail_max_pitch_rate_deg_s=max(abs(x['pitch_rate_deg_s']) for x in tail),
                peak_front_back_height_difference_mm=max(x['front_back_height_difference_mm'] for x in rows),
                peak_sampled_pitch_deg=max(abs(x['pitch_deg']) for x in rows),
                strict_definition='Requested front/back edge loaded; pitch error <=2 deg; |pitch rate| <=5 deg/s; |roll| <=5 deg')
