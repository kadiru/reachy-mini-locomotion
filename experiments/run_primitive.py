"""Replay a saved primitive in simulation; this script cannot connect to a robot."""
import argparse
import json
from pathlib import Path

from reachy_locomotion.primitives import PrimitiveSimulator
from finalize_primitives import record_terminal


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recipe')
    parser.add_argument('--output', required=True)
    parser.add_argument('--timestep', type=float, default=.00025)
    parser.add_argument('--seconds', type=float, default=8)
    parser.add_argument('--waist', choices=['hold_neutral', 'off'], default='hold_neutral')
    parser.add_argument('--record', action='store_true')
    args = parser.parse_args()
    output = Path(args.output)
    if output.exists():
        parser.error('Choose a new output file; previous evidence is preserved')
    config = json.loads(Path('configs/motors-speed-trained.json').read_text())
    config['timestep_s'] = args.timestep
    recipe = json.loads(Path(args.recipe).read_text())
    sim = PrimitiveSimulator('experiments/upstream-reachy-mini', config)
    result = sim.run_primitive(recipe, seconds=args.seconds, waist_mode=args.waist, record=args.record)
    if args.record:
        record_terminal(sim, result)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, allow_nan=False) + '\n')
    print(json.dumps({k: result[k] for k in ['termination', 'recovered', 'peak_roll_deg',
                                          'peak_side_height_difference_mm', 'final_foot_yaw_deg']}))


if __name__ == '__main__':
    main()
