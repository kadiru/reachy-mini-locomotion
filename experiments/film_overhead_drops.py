"""Film overhead drops from measured states, including motion after first floor contact."""
import argparse
from collections import defaultdict
from pathlib import Path

from film_hit_envelope import film
from train_balance_robustness import read

COLORS = {'floor contact':'#ffa58c','held edge':'#87e5b0','settled flat':'#ffd184'}


def panel(row,states,stop=None):
    floor=row['floor_contact']
    label={'held edge':'Held original edge','settled flat':'Settled flat','floor contact':'Body contacted floor'}.get(row['outcome'],row['outcome'].capitalize())
    if not row['prequalified']:
        label='Pre-drop hold unqualified'
    n=row['count']
    message=f"{row['direct_hits']}/{n} balls hit Reachy" if n else 'Unstruck control'
    if floor:
        message+=f"   ·   Floor at {floor['time_s']:.2f} s"
    else:
        message+=f"   ·   Final tilt {row['final_tilt_deg']:.1f}°"
    result=dict(label=f"{row['edge'].title()} edge   ·   {n} {'ball' if n==1 else 'balls'}",
                record=str(states/(row['id']+'.json')),start_s=row['release_s']-.20,
                outcome=label,detail=message,color=COLORS.get(row['outcome'],'#b6c9e1'),
                overhead=dict(count=n,release=row['release_s'],first_hit=row['first_hit_s'],
                              floor_s=floor['time_s'] if floor else None))
    if stop is not None:
        result['stop_s']=stop
    return result


def example_row(path):
    r=read(path)
    balls=[b for b in r['ball_disturbances'] if b['released']]
    hits=[b['first_robot_contact_s'] for b in balls if b['contacted_robot'] and
          (b['first_floor_contact_s'] is None or b['first_robot_contact_s']<b['first_floor_contact_s'])]
    qualified=all(e['balanced_before_impact'] for e in r['balance_curriculum']['impacts']['events'][:len(balls)])
    if not qualified or not r['completed'] or len(hits)!=len(balls):
        raise ValueError('Choose two qualified, completed direct-drop runs')
    floor=r['first_body_floor_contact']
    outcome='floor contact' if floor else 'held edge' if r['edge_balance']['metrics']['final_five_seconds_pass'] else 'lost edge'
    return dict(edge=r['balance_curriculum']['edge'],id=path.stem,count=len(balls),floor_contact=floor,
                prequalified=qualified,release_s=balls[0]['release_s'],outcome=outcome,first_hit_s=min(hits),
                direct_hits=len(hits),final_tilt_deg=r['trace'][-1]['tilt_deg'])


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    group=ap.add_mutually_exclusive_group(required=True)
    group.add_argument('--example',type=Path,nargs=2,metavar=('HOLD','FALL'))
    group.add_argument('--study',type=Path,nargs=2,action='append',metavar=('REPORT','STATES'))
    ap.add_argument('--work',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True)
    args=ap.parse_args()
    chapters=[]
    if args.example:
        hold,fall=map(example_row,args.example)
        if hold['outcome']!='held edge' or not fall['floor_contact'] or hold['release_s']!=fall['release_s']:
            raise ValueError('Expected a matched hold and confirmed fall')
        stop=fall['floor_contact']['time_s']+2.
        chapters.append(dict(title='Holding the edge and falling from it',
                             subtitle='Simultaneous overhead drops   ·   10 g per ball   ·   Replay continues through landing',
                             panels=[panel(r,p.parent,stop) for r,p in zip((hold,fall),args.example)]))
        film(chapters,args.output,args.work,.5,25)
    else:
        for report,states in args.study:
            pairs=defaultdict(dict)
            batch=read(report)
            for r in batch['rows']:
                pairs[(r['release_s'],r['seed'],r['count'])][r['edge']]=r
            for (release,seed,n),pair in sorted(pairs.items()):
                layout=('compact outer positions' if batch.get('compact_outer_slots') else 'wide outer positions') if n>=8 else 'centre cluster'
                chapters.append(dict(title=f"{n} overhead {'ball' if n==1 else 'balls'}" if n else 'Unstruck controls',
                    subtitle=f"Release at {release:g} s   ·   {layout}   ·   sensor seed {seed}   ·   5 V motor model",
                    panels=[panel(pair[edge],states) for edge in ('left','right')]))
        film(chapters,args.output,args.work,2.,12)


if __name__=='__main__':
    main()
