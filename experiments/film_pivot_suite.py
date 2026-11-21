"""Make a chaptered film of every frozen target-pivot qualification trial."""
import argparse
from collections import defaultdict
from pathlib import Path

from film_hit_envelope import film
from train_target_pivot import CONDITIONS, REPORT, read


def outcome(row):
    if row['passed']:
        return 'Pass'
    if row['termination'] != 'completed':
        return 'Joint travel guard' if row['termination'] == 'actual_joint_limit_exceeded' else row['termination'].replace('_', ' ')
    if row['max_tail_error_deg'] > 5:
        return 'missed heading'
    if row['max_excursion_mm'] > 56:
        return 'excess center travel'
    if row['mean_edge_slip_mm_per_stroke'] > 15:
        return 'excess slip'
    return 'insufficient turn on rim' if row['settled'] else 'did not settle'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('records', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    groups = defaultdict(list)
    for row in read(REPORT)['results']:
        groups[row['condition']].append(row)
    chapters = []
    for condition, rows in groups.items():
        friction, volt, _, cmd_delay, mass, dt, delay, _, _ = CONDITIONS[condition]
        for i in range(0,len(rows),4):
            panels = []
            for row in rows[i:i+4]:
                target = row['target_deg']
                path = args.records/f'{condition}-{target:g}.json'
                if read(path)['metrics'] != row:
                    raise ValueError(f'Qualification and replay differ in {path}')
                panels.append(dict(record=path, label=f"Goal {target:+g}°    ·    final torso {row['torso_deg']:+.1f}°",
                                   outcome=outcome(row), color='#85e4ae' if row['passed'] else '#ffac88',
                                   detail=f"Net on rim {row['signed_edge_deg']:+.1f}°    ·    max center shift {row['max_excursion_mm']:.0f} mm"))
            chapters.append(dict(title=f"{condition.replace('_',' ').capitalize()}   ·   {volt:g} V   ·   floor friction {friction:g}",
                                 subtitle=f"{mass.capitalize()} mass allocation   ·   sensing {delay*20} ms   ·   command delay {cmd_delay*20} ms   ·   physics step {dt*1000:g} ms",
                                 panels=panels))
    film(chapters,args.output,args.records/'chapters',speed=4,fps=15)


if __name__=='__main__':
    main()
