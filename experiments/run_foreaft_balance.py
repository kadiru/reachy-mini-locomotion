"""Replay front/back balance; artificial curriculum starts require an explicit flag."""
import argparse
import json
from pathlib import Path

from train_foreaft_balance import evaluate, read


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('policy')
    parser.add_argument('--edge', choices=['front', 'back'], required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--seconds', type=float, default=20)
    parser.add_argument('--timestep', type=float, default=.00025)
    parser.add_argument('--record', action='store_true')
    parser.add_argument('--config', default='configs/motors-speed-trained.json')
    parser.add_argument('--diagnostic-raised-start', action='store_true',
                        help='Artificial stationary 32.1 degree reset; does not demonstrate lifting from flat')
    args = parser.parse_args()
    out = Path(args.output)
    if out.exists():
        parser.error('Preserve previous evidence')
    source = read(args.policy)
    policy = source['edges'][args.edge] if 'edges' in source else source
    side = 1 if args.edge == 'front' else -1
    job = dict(id='replay-' + args.edge, policy=policy, side=side,
               seconds=args.seconds, dt=args.timestep, config=read(args.config))
    if args.diagnostic_raised_start:
        job['diagnostic_initial_pitch_deg'] = side * 32.1
    if args.record:
        job['record'] = str(out)
    result = evaluate(job)
    if not args.record:
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(result, allow_nan=False) + '\n')
    print(json.dumps(dict(natural_upright_start=result['result']['natural_upright_start'],
                         termination=result['result']['termination'], metrics=result['metrics']), indent=2))


if __name__ == '__main__':
    main()
