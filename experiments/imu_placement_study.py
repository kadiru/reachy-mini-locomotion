"""Replay three frozen policies with the same packets measured on the foot or head."""
import argparse
from concurrent.futures import ProcessPoolExecutor
import json
from unittest.mock import patch

import numpy as np

from imu_frequency_study import WORK, core_case
from imu_skill_cases import motion_case
from multirate_imu import Packets, feed_archived_runner
from reachy_locomotion.contact_probe import BASE
from reachy_locomotion.geometry import euler


def trial(job):
    name, mount, rate, repeat = job
    errors = []
    sample = Packets.sample

    def measured(observer, model, data, t):
        attitude, gyro = sample(observer, model, data, t)
        truth = np.degrees(euler(data.body(BASE).xmat.reshape(3,3))[:2])
        errors.append((attitude - truth + 180.) % 360. - 180.)
        return attitude, gyro

    # All cached plants in this pool include both sites, even on foot-control runs.
    with patch.object(Packets, 'sample', measured), feed_archived_runner(rate, repeat*.5, mount, include_head=True):
        if name == 'original release':
            r = core_case(name,repeat)
        else:
            r = motion_case(name,repeat)
    return dict(skill=name,mount=mount,rate=rate,repeat=repeat,
                roll_rmse_deg=float(np.sqrt(np.mean(np.square(np.array(errors)[:,0])))),**r)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workers',type=int,default=4)
    args = parser.parse_args()
    jobs = [(s,m,hz,i) for m in ('head','head+FK','base') for s in ('original release','40 degree catch','shuffle')
            for hz in (50,200,1000) for i in (0,1)]
    WORK.mkdir(parents=True,exist_ok=True)
    out = WORK/'placement.jsonl'
    done = {tuple(json.loads(line)[k] for k in ('skill','mount','rate','repeat'))
            for line in out.read_text().splitlines()} if out.exists() else set()
    with ProcessPoolExecutor(args.workers) as pool, out.open('a') as f:
        for row in pool.map(trial,[j for j in jobs if j not in done]):
            print(json.dumps(row,allow_nan=False),file=f,flush=True)


if __name__ == '__main__':
    main()
