"""Assemble final checks, explicitly distinguishing replay from equivalent paths.

The sole final controller change is a larger tolerance on a progressing final
retry. Every previously successful half already met the tighter tolerance;
therefore it takes the identical branch under the selected policy. Only the
two unsuccessful repeated-walk paths need a new rollout at each timestep.
"""
import copy,json
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
PREFIX='experiments/reports/motor-training-sustained-'
DATE='2026-11-20'
def read(name):return json.loads((ROOT/f'{PREFIX}{name}-{DATE}.json').read_text())

if __name__=='__main__':
    policy=json.loads((ROOT/'configs/penguin-sustained-trained.json').read_text())
    assert policy['exhaustion_tolerance_deg']==6
    checks=[];jobs=[];sources=[];superseded=[]
    for name in ['development-final','fine-final','long-final']:
        r=read(name);assert r['completed'];expected=copy.deepcopy(r['frozen_policy'])
        expected['exhaustion_tolerance_deg']=6.;assert expected==policy
        sources.append(f'{PREFIX}{name}-{DATE}.json')
        for index,(job,c) in enumerate(zip(r['jobs'],r['checks'])):
            if not c['passed']:
                assert (name,c['id'],c['side'],c['variant']) in {
                    ('development-final','fresh-3',-1,'four-pairs'),
                    ('development-final','unseen-7',-1,'four-pairs'),
                    ('fine-final','unseen-7',-1,'finer-single-pair')}
                superseded.append(dict(source=sources[-1],index=index,id=c['id'],side=c['side'],variant=c['variant']));continue
            assert all(s['turn_goal_met'] for s in c['penguin']['steps'])
            assert c['penguin']['completed']
            j=copy.deepcopy(job);j['policy']['exhaustion_tolerance_deg']=6.
            c=copy.deepcopy(c);c.update(validation_method='identical_control_path_under_looser_final_retry_tolerance',
                source_report=sources[-1],source_index=index)
            checks.append(c);jobs.append(j)
    fixes=read('tolerance6-repair');assert fixes['completed'] and len(fixes['checks'])==4
    assert all(c['passed'] for c in fixes['checks'])
    assert {(x['id'],x['side']) for x in superseded}=={('fresh-3',-1),('unseen-7',-1)}
    assert len(superseded)==3
    for index,(job,c) in enumerate(zip(fixes['jobs'],fixes['checks'])):
        c=copy.deepcopy(c);c.update(validation_method='direct_rollout',source_report=f'{PREFIX}tolerance6-repair-{DATE}.json',source_index=index)
        c['variant']='four-pairs' if job['config']['timestep_s']==.00025 else 'finer-four-pairs'
        checks.append(c);jobs.append(job)
    single=read('finer-single-repair');assert single['completed'] and len(single['checks'])==1
    assert single['checks'][0]['passed']
    c=copy.deepcopy(single['checks'][0]);c.update(validation_method='direct_rollout',source_report=f'{PREFIX}finer-single-repair-{DATE}.json',source_index=0)
    checks.append(c);jobs.append(single['jobs'][0])
    fresh=read('fresh-final');assert fresh['completed'] and fresh['frozen_policy']==policy
    for index,(job,c) in enumerate(zip(fresh['jobs'],fresh['checks'])):
        c=copy.deepcopy(c);c.update(validation_method='direct_rollout',source_report=f'{PREFIX}fresh-final-{DATE}.json',source_index=index)
        checks.append(c);jobs.append(job)
    out=ROOT/f'{PREFIX}qualification-final-{DATE}.json'
    if out.exists():raise FileExistsError(out)
    out.write_text(json.dumps(dict(completed=True,evidence='uncalibrated_simulation',hardware_ready=False,
        frozen_policy=policy,jobs=jobs,checks=checks,superseded_checks=superseded,
        equivalence_scope='Only the final-retry steering tolerance changes. Old successful paths met the tighter 2-degree test at every half; no added acceptance branch is exercised.',
        heldout_seed=fresh['seed']),allow_nan=False)+'\n')
    print(len(checks),'checks,',sum(c['passed'] for c in checks),'passed;',len(superseded),'earlier failures replaced by direct repaired rollouts')
