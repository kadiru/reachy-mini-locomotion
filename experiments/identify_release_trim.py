"""Bracket the stationary neutral-head tipping pose independently on each CAD edge."""
import numpy as np
from reachy_locomotion.primitives import Primitive
from reachy_locomotion.release_plane import ReleasePlane
from tilted_release import plant


def probe(spec):
    side,angle=spec
    sim,neutral=plant(True,.000125,False)
    sim.initial_targets=neutral.copy()
    ctl=ReleasePlane(side,16071,sim.checked_ik,dict(interval=5,amplitude=np.zeros(2)),excitation=True)
    r=sim.run_primitive(Primitive(),seconds=1.8,controller=ctl,waist_mode='hold_neutral',
                        diagnostic_initial_roll_deg=side*angle,tilt_limit_deg=None,observe_landing=True)
    rows=r['trace']
    end=side*rows[-1]['roll_deg']
    change=end-side*rows[-21]['roll_deg']
    direction=1 if end>40 else -1 if end<25 else np.sign(change)
    middle=[row for row in rows if .62<=row['time_s']<=1.00]
    state=np.median([row['plane_state'] for row in middle],axis=0)
    return dict(side=side,angle=angle,end_roll_deg=end,late_change_deg=float(change),outward=bool(direction>0),
                trim_state=state.tolist(),median_abs_rate_deg_s=float(np.median([abs(row['roll_rate_deg_s']) for row in middle])),
                middle_edge_frames=sum(row['edge_support'] for row in middle),middle_frames=len(middle),
                elapsed=r['elapsed_seconds'])


def identify(pool):
    sides=(1,-1)
    rows=list(pool.map(probe,[(s,a) for s in sides for a in (30.,35.)]))
    if any(r['outward']!=(r['angle']==35.) for r in rows):
        raise ValueError('Neutral target release must bracket inward and outward rotation')
    limits={s:[30.,35.] for s in sides}
    for _ in range(17):
        current=list(pool.map(probe,[(s,sum(limits[s])/2) for s in sides]))
        rows+=current
        for r in current:
            limits[r['side']][int(r['outward'])]=r['angle']
    trims={}
    for s in sides:
        candidates=[r for r in rows if r['side']==s and r['middle_edge_frames']==r['middle_frames'] and abs(r['trim_state'][0])<3.]
        best=min(candidates,key=lambda r:r['median_abs_rate_deg_s']+.02*min(10.,abs(r['late_change_deg'])))
        state=best['trim_state'][:3]+[0.]*7
        trims[str(s)]=dict(initial_bracket_deg=limits[s],state=state,release=best)
    return dict(rollouts=rows,trims=trims)
