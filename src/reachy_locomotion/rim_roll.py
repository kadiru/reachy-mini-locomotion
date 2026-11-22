"""Measure winding of the loaded foot material and the ground path during each lap."""
import math
import numpy as np


def circle(points):
    """Circle fit in millimetres; straight or open paths cannot qualify as loops."""
    xy = np.asarray(points) - np.mean(points, axis=0)
    a = np.column_stack((2*xy, np.ones(len(xy))))
    center = np.linalg.lstsq(a, (xy*xy).sum(axis=1), rcond=None)[0][:2]
    rad = np.linalg.norm(xy-center, axis=1)
    radius = rad.mean()
    xy_angle = np.unwrap(np.arctan2(xy[:,1]-center[1], xy[:,0]-center[0]))
    closed = (radius > .2 and abs(xy_angle[-1]-xy_angle[0]) > 1.5*math.pi
              and np.linalg.norm(xy[-1]-xy[0]) < radius and rad.std() < .35*radius)
    return 2*float(radius) if closed else 0.


def measures(rows, radius_m, sign=None, start_s=1.5):
    active = [r for r in rows if r['time_s'] >= start_s]
    if len(active) < 2:
        active = rows[-2:]
    t = np.array([r['time_s'] for r in active])
    cop = np.array([r['contact_foot_xy_m'] or [0.,0.] for r in active])
    lean = np.array([r['tilt_deg'] for r in active])
    xy = np.array([r['foot_xyz_m'][:2] for r in active])*1000
    floor = np.array([(r['contact_world_m'] or [0.,0.])[:2] for r in active])*1000
    valid = (np.linalg.norm(cop,axis=1) >= .85*radius_m) & (lean>=5.) & np.array([r['normal_n']>.5 for r in active])
    a = np.arctan2(cop[:,1],cop[:,0])
    da = np.angle(np.exp(1j*np.diff(a)))
    keep = valid[:-1] & valid[1:] & (np.abs(da) < np.pi/2)
    direction = sign if sign in (-1,1) else (1 if da[keep].sum()>=0 else -1)
    signed = da*direction
    usable = np.where(keep,signed,0.)
    diameters, floor_diameters = [], []
    progress = best = net = 0.
    first = 0
    for i,delta in enumerate(signed):
        if not keep[i]:
            progress = net = 0.
            first = i+1
            continue
        progress += delta
        net += delta
        best = max(best, net)
        if progress < 0:
            progress = 0.
            first = i+1
        if progress >= 2*np.pi:
            # Retain endpoint for the next lap; no angle credited across gaps.
            diameters.append(circle(xy[first:i+2]))
            floor_diameters.append(circle(floor[first:i+2]))
            first = i+1
            progress = 0.
    duration = max(.02,t[-1]-t[0])
    slip_mm = 1000*np.array([r['contact_slip_m_s'] for r in active])
    fraction = float(np.mean(keep)) if len(keep) else 0.
    slip = float(np.sum((slip_mm[:-1]+slip_mm[1:])*.5*np.diff(t)*keep))
    arc = radius_m*1000*np.abs(da[keep]).sum()
    return dict(direction=int(direction), loaded_fraction=fraction, laps=len(diameters),
                signed_rim_turns=float(usable.sum()/(2*np.pi)), longest_rim_turns=float(best/(2*np.pi)),
                coherent_fraction=float(np.maximum(usable,0).sum()/max(1e-9,np.abs(usable).sum())),
                median_lean_deg=float(np.median(lean)), low_lean_deg=float(np.percentile(lean,10)),
                peak_lean_deg=float(lean.max()), closed_center_laps=sum(d>0 for d in diameters),
                center_diameter_mm=float(np.median(diameters)) if diameters else 0.,
                contact_diameter_mm=float(np.median(floor_diameters)) if floor_diameters else 0.,
                slip_per_rim_arc=float(slip/max(1.,arc)), center_excursion_mm=float(np.linalg.norm(xy-xy[0],axis=1).max()),
                span_xy_mm=np.ptp(xy,axis=0).tolist(), rim_turns_per_s=float(usable.sum()/(2*np.pi*duration)))


class TiltPhase:
    """Advance an orbit using delayed foot roll and pitch. Contact is evaluation-only."""
    catch_trigger = yaw_trigger = None

    def __init__(self, head, feedback, delay, noise, seed):
        from collections import deque
        self.head = head
        self.policy = feedback
        self.pitch = 0.
        self.buffer = deque([(0.,0.)]*(delay+1),maxlen=delay+1)
        self.rng = np.random.default_rng(seed)
        self.noise = noise
        self.phase = 0.
        self.cache = {}

    def observe_attitude(self,t,pitch):
        self.pitch=pitch

    def target(self,t,roll):
        p = self.policy
        sensed = np.array([self.head.mirror*roll,self.pitch])+self.rng.normal(0,self.noise,2)
        self.buffer.append(sensed)
        r,q = self.buffer[0]
        lean = math.hypot(r,q)
        nominal = 2*math.pi/self.head.orbit.period_frames
        if t:
            lead = p['lead_deg']+p['gain_per_deg']*(p['target_lean_deg']-min(45.,lean))
            observed = p['sense']*math.atan2(q,r)+math.radians(lead)
            phase_error = math.atan2(math.sin(observed-self.phase),math.cos(observed-self.phase))
            blend = min(1.,lean/8.,t/1.2)
            self.phase += nominal + np.clip(blend*p['lock_gain']*phase_error,-.85*nominal,1.5*nominal)
        tick = round(self.phase*120/(2*math.pi))%120
        ramp = min(1.,t)
        if ramp<1. or tick not in self.cache:
            target=self.head.pose_for_phase(tick*2*math.pi/120,ramp*ramp*(3-2*ramp))
            if ramp==1.:self.cache[tick]=target
        else:
            target=self.cache[tick]
        return target,'tilt-timed head orbit',float(r)


def ground_course(rows, foot_diameter_mm=112., accept_loop=None):
    """Find full floor loops separately from the rapid wobble in each rim lap."""
    t=np.array([r['time_s'] for r in rows])
    xy=np.array([r['foot_xyz_m'][:2] for r in rows])*1000
    # Average over 1 s and sample at 5 Hz to suppress within-lap wobble.
    if len(t)<201:return dict(best_loop=None, arc=None)
    avg=np.column_stack([np.convolve(xy[:,k],np.ones(51)/51,mode='valid') for k in range(2)])
    times=t[25:-25:10]
    pos=avg[::10]
    def fit(i,j):
        pts=pos[i:j]
        mean=pts.mean(axis=0)
        pts=pts-mean
        c=np.linalg.lstsq(np.column_stack((2*pts,np.ones(len(pts)))),(pts**2).sum(axis=1),rcond=None)[0][:2]
        rel=pts-c
        radii=np.linalg.norm(rel,axis=1)
        radius=radii.mean()
        if radius<2:return None
        d=np.diff(np.unwrap(np.arctan2(rel[:,1],rel[:,0])))
        angle=np.degrees(d.sum())
        return dict(start_s=float(times[i]),end_s=float(times[j-1]),center_xy_mm=(c+mean).tolist(),
                    diameter_mm=float(2*radius),sweep_deg=float(angle),radial_cv=float(radii.std()/radius),
                    coherence=float(abs(d.sum())/max(1e-8,np.abs(d).sum())),
                    endpoint_gap_mm=float(np.linalg.norm(pts[-1]-pts[0])),
                    period_s=float(360*(times[j-1]-times[i])/max(.001,abs(angle))))
    start=int(np.searchsorted(times,2.5))
    if len(times)-start<11:return dict(best_loop=None,arc=None)
    whole=fit(start,len(times))
    loops=[]
    for i in range(start,len(times)-40,15):
        for j in range(i+40,len(times)+1,5):
            loop=fit(i,j)
            if loop and (350<=abs(loop['sweep_deg'])<=385 and loop['coherence']>=.93
                         and loop['radial_cv']<=.18 and loop['endpoint_gap_mm']<=.175*loop['diameter_mm']):
                if accept_loop is None or accept_loop(loop):
                    loops.append(loop)
    best=max(loops,key=lambda c:c['diameter_mm'],default=None)
    # Count disjoint circles of at least one foot diameter. Overlapping windows
    # which describe the same revolution can only contribute a single lap.
    wide=[]
    for c in sorted(loops,key=lambda item:item['end_s']):
        if c['diameter_mm']>=foot_diameter_mm and (not wide or (c['start_s']>=wide[-1]['end_s']
                         and c['sweep_deg']*wide[-1]['sweep_deg']>0)):
            wide.append(c)
    return dict(best_loop=best,arc=whole,disjoint_wide_loops=wide)
