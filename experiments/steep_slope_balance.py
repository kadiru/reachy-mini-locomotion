"""Find a gentle downhill tipping stroke on a 23 degree ramp with a free, initially flat foot."""
from slope_balance import WORK, run, spec, write


def train(pool):
    jobs=[]
    for side in (-1,1):
        for duration in (.16,.28,.44,.62):
            for strength in (.25,.45,.65,.85,1.,1.15):
                p=dict(actions=[[26.,-14.,0.]],frames=1,stop_s=.2+duration,
                       strength=strength,trigger=10000.,min_trigger=1000.,lead=0.)
                jobs.append(spec(side*23.,side,p,seconds=3.5))
    rows=[dict(settings=j[2],result=r) for j,r in zip(jobs,pool.map(run,jobs))]
    brackets=[]
    for side in (-1,1):
        for duration in (.16,.28,.44,.62):
            ordered=[v for v in rows if v['result']['side']==side and v['settings']['stop_s']==.2+duration]
            for lo,hi in zip(ordered,ordered[1:]):
                if lo['result']['outward_escape']!=hi['result']['outward_escape']:
                    brackets.append([side,lo,hi])
    for _ in range(8):
        jobs=[]
        for side,lo,hi in brackets:
            scale=(lo['settings']['strength']+hi['settings']['strength'])/2
            p=dict(lo['settings'],strength=scale)
            jobs.append(spec(side*23.,side,p,seconds=3.5))
        for bracket,job,r in zip(brackets,jobs,pool.map(run,jobs)):
            record=dict(settings=job[2],result=r)
            rows.append(record)
            same=r['outward_escape']==bracket[1]['result']['outward_escape']
            bracket[1 if same else 2]=record
    write(WORK/'steep-search.json',rows)
    jobs=[]
    for side in (-1,1):
        ranked=sorted((v for v in rows if v['result']['side']==side),key=lambda v:v['result']['coast_cost'])
        for v in ranked[:5]:
            for gap,rate,damp,mode in ((1.,15.,0.,'plane'),(2.,35.,0.,'plane'),
                                       (2.,35.,.65,'plane'),(.5,10.,0.,'plane'),(2.,25.,0.,'full')):
                p=dict(v['settings'],min_trigger=31.55-gap,trigger=31.55-gap,lead=.03,max_rate=rate,
                       unlock=v['settings']['stop_s']+.10,catch_model=mode,warm=True,pitch_damping=damp)
                jobs.append(spec(side*23.,side,p,seconds=6.))
    catches=[dict(settings=j[2],result=r) for j,r in zip(jobs,pool.map(run,jobs))]
    write(WORK/'steep-catch.json',catches)
    return catches
