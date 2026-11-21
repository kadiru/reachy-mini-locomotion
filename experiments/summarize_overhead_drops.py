"""Index all overhead trials and identify the matched count sweep used for comparison."""
import argparse
from collections import defaultdict
import csv
import math
from pathlib import Path

from train_balance_robustness import read


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('reports',type=Path,nargs='+',help='Input order must match the all-trials film')
    ap.add_argument('--csv',type=Path,required=True)
    args=ap.parse_args()
    frames=0
    fps=12
    rows=[]
    for path in args.reports:
        b=read(path)
        groups=defaultdict(list)
        for r in b['rows']:
            groups[(r['release_s'],r['seed'],r['count'])].append(r)
        for (release,seed,n),pair in sorted(groups.items()):
            assert {r['edge'] for r in pair}=={'left','right'} and len(pair)==2
            compact=b.get('compact_outer_slots',False)
            for r in pair:
                row={k:v for k,v in r.items() if k not in ('balls','floor_contact','first_joint_warning')}
                floor=r['floor_contact']
                row.update(primary=(release in (6.,6.5) and (n<8 or compact)),
                           layout=('compact' if compact else 'wide') if n>=8 else 'shared',
                           floor_s=floor['time_s'] if floor else None,floor_body=floor['body'] if floor else None,
                           tilt_at_floor_deg=floor['tilt_deg'] if floor else None,
                           first_soft_joint_warning_s=(r['first_joint_warning'] or {}).get('time_s'),
                           video_s=frames/fps,report=path.name)
                rows.append(row)
            frames+=math.ceil(max(r['end_s']-(release-.2) for r in pair)*fps/2)+fps
    primary=[r for r in rows if r['primary']]
    keys={(r['count'],r['edge'],r['release_s']) for r in primary}
    assert len(primary)==len(keys)==44 and all(r['prequalified'] for r in primary)
    with args.csv.open('x',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(rows[0]),lineterminator='\n')
        writer.writeheader()
        writer.writerows(rows)
    print('count   left 6s   right 6s   left 6.5s   right 6.5s')
    for n in range(11):
        print(n,[(r['outcome'],r['direct_hits']) for r in sorted(primary,key=lambda r:(r['release_s'],r['edge'])) if r['count']==n])
    print('trials',len(rows),'film seconds',frames/fps)


if __name__=='__main__':
    main()
