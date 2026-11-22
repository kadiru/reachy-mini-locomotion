"""On a banked table replay the head poses that closed-loop balance requested on level ground."""
from concurrent.futures import ProcessPoolExecutor

import numpy as np

from moving_table import REPORT, SEEDS, case, simulate, write

FEEDBACK_REPORT = REPORT.with_name("moving-table-feedback.json")
from qualify_moving_table import edge_pass


class HeadCommands:
    def __init__(self, controller, capture, reference=None):
        self.controller = controller
        self.capture = capture
        self.reference = reference

    def __getattr__(self, name):
        return getattr(self.controller, name)

    def target(self, t, roll):
        pose, stage, observation = self.controller.target(t, roll)
        index = len(self.capture)
        self.capture.append(pose.copy())
        target = pose if self.reference is None else self.reference[index]
        return target, stage, observation


def replay_level(job, recording=False):
    reference = []
    simulate(dict(job, flat=True), wrap_controller=lambda c: HeadCommands(c, reference))
    return simulate(job, recording=recording, wrap_controller=lambda c: HeadCommands(c, [], reference))


def compare(seed):
    moving = case(1, 10., seed=seed)
    level, closed = [], []
    baseline = simulate(dict(moving, flat=True), wrap_controller=lambda c: HeadCommands(c, level))
    feedback = simulate(moving, wrap_controller=lambda c: HeadCommands(c, closed))
    replay = simulate(moving, wrap_controller=lambda c: HeadCommands(c, [], level))
    first = round(3./.02)
    if not np.array_equal(level[:first], closed[:first]):
        raise ValueError('Head commands diverged before the deck moved')
    a, b = np.asarray(closed[first:]), np.asarray(level[first:])
    relative = np.swapaxes(a[:,:3,:3],1,2) @ b[:,:3,:3]
    angle = np.degrees(np.arccos(np.clip((np.trace(relative,axis1=1,axis2=2)-1)/2, -1, 1)))
    shift = np.linalg.norm(a[:,:3,3]-b[:,:3,3],axis=1)*1000
    return dict(seed=seed, level=baseline, feedback=feedback, level_command_replay=replay,
                max_head_command_difference_deg=float(angle.max()), max_head_command_difference_mm=float(shift.max()))


if __name__=='__main__':
    with ProcessPoolExecutor(3) as pool:
        pairs=list(pool.map(compare,SEEDS))
    write(FEEDBACK_REPORT,pairs)
    for r in pairs:
        print(r['seed'],edge_pass(r['level']),edge_pass(r['feedback']),edge_pass(r['level_command_replay']))
