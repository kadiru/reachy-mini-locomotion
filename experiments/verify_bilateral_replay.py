"""Compare an independent replay with every saved physical frame in the video."""
import json
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
import numpy as np
from reachy_locomotion.balance import BalanceController,mirrored
from reachy_locomotion.penguin import PenguinController
from reachy_locomotion.primitives import PrimitiveSimulator


def verify(name):
    path=Path('experiments/generated/bilateral')/(name+'.json')
    old=json.loads(path.read_text());sim=PrimitiveSimulator('experiments/upstream-reachy-mini',old['config'])
    if name.startswith('recovery'):
        p=json.loads(Path('configs/balance-recovery-trained.json').read_text())
        recipe=mirrored(p['recipe'],old['side']);controller=BalanceController(recipe,p['feedback'],checked_ik=sim.checked_ik)
    else:
        p=json.loads(Path('configs/penguin-feedback-experimental.json').read_text());g=p['primary']['recipe']
        recipe=mirrored(g,1)
        controller=PenguinController(g,p['primary']['feedback'],cycles=2,min_step_s=3.,start_side=1,checked_ik=sim.checked_ik,
                    target_step_turn_deg=p['target_step_turn_deg'],max_repeats=p['max_repeats'],heading_noise_deg=.1,heading_seed=42,
                    fallback_policy=p['fallback'],switch_below_deg=p['switch_below_deg'])
    new=sim.run_primitive(recipe,seconds=old['requested_seconds'],waist_mode='hold_neutral',record=True,controller=controller,tilt_limit_deg=35)
    assert old['termination']==new['termination']=='completed'
    assert len(old['trajectory'])==len(new['trajectory'])
    errors={}
    for field in ['qpos','qvel','time_s','foot_xyz_m','heading_from_startup_rad']:
        errors[field]=float(np.max(np.abs(np.array([r[field] for r in new['trajectory']])-np.array([r[field] for r in old['trajectory']]))))
    assert max(errors.values())<1e-9,errors
    return dict(recording=str(path),frames=len(new['trajectory']),max_absolute_errors=errors,passed=True)

if __name__=='__main__':
    out=Path('experiments/reports/motor-training-bilateral-replay-2026-11-19.json')
    if out.exists():raise FileExistsError(out)
    with ProcessPoolExecutor(max_workers=3) as pool:checks=list(pool.map(verify,['recovery-left','recovery-right','penguin-two-cycles']))
    out.write_text(json.dumps(dict(completed=True,checks=checks),indent=2)+'\n');print(json.dumps(checks))
