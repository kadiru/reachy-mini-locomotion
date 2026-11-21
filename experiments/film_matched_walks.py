"""Film every frozen walking comparison with its floor, motor settings and outcome."""
import argparse
from pathlib import Path

from film_hit_envelope import film
from train_cadence import read


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    for key in ('report','states','work','output'):
        ap.add_argument('--'+key,type=Path,required=True)
    ap.add_argument('--alternating-report',type=Path,required=True)
    ap.add_argument('--alternating-states',type=Path,required=True)
    args = ap.parse_args()
    report = read(args.report)
    extra = read(args.alternating_report)
    if extra['conditions'] != report['conditions']:
        raise ValueError('Matched floor and motor conditions required')
    report['rows'].extend(extra['rows'])
    chapters = []
    labels = {'shallow':'Earlier shallow walk','timed':'Faster edge walk',
              'alternating':'Alternating edge brakes','encoder':'Encoder-braked edge walk'}
    for number, condition in enumerate(report['conditions']):
        motor = condition['motor_model']
        friction = condition['friction_slide_torsion_roll'][0]
        for side in (-1,1):
            panels = []
            for key,label in labels.items():
                name = f'{key}-{number}'
                row = next(r for r in report['rows'] if r['name']==name and r['side']==side)
                brakes = row['edge_brake_by_roll_sign']
                misses = []
                if not row['course_ok']:
                    misses.append('heading')
                if not row['cadence_ok']:
                    misses.append('cadence')
                if min(row['distance_mm'],row['late_distance_mm']) <= 0:
                    misses.append('forward travel')
                if not row['settled']:
                    misses.append('settling')
                outcome = ('Forward, settled' if row['passed'] else
                           'Failed '+', '.join(misses) if row['completed'] else row['termination'])
                directory = args.alternating_states if key=='alternating' else args.states
                panels.append(dict(record=directory/f'{name}-{side}.json',label=label,
                                   outcome=f"{row['distance_mm']:+.1f} mm   ·   {outcome}",
                                   detail=f"Edge-braking strokes   L {len(brakes['left'])}   R {len(brakes['right'])}     |     pair-end {row['max_pair_end_deg']:.1f}°",
                                   color='#ade1ba' if row['passed'] else '#ffad92'))
            chapters.append(dict(title=f"Floor {number+1}   ·   starting sign {side:+d}",
                                 subtitle=f"{motor['voltage_v']:.2f} V  ·  friction {friction:.2f}  ·  gain {motor['pid_scale']:.2f}  ·  delay {motor['delay_frames']} frames  ·  {motor['mass_distribution']} mass",
                                 panels=panels))
    film(chapters,args.output,args.work,speed=2,fps=15)


if __name__=='__main__':
    main()
