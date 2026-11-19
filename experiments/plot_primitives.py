"""Plot recorded physics and all catch outcomes for the selected launch."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.patches import Patch
import numpy as np


def read(path):
    return json.loads(Path(path).read_text())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--record-dir', required=True)
    parser.add_argument('--final-report', required=True)
    parser.add_argument('--catch-search', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    output = Path(args.output)
    if output.exists():
        parser.error('Preserve earlier plots')
    final = read(args.final_report)
    recordings = {name: read(Path(args.record_dir)/(name+'.json')) for name in ['fall','catch','pivot']}
    search = read(args.catch_search)
    fig, axes = plt.subplots(2, 2, figsize=(14, 9), constrained_layout=True)
    fig.suptitle('Learn the edge before composing a walking step', fontsize=19, weight='bold')
    fig.supxlabel('Uncalibrated MuJoCo simulation | bounded motors | waist target fixed at 0° | no external pushes', fontsize=11)
    ax = axes[0, 0]
    for name, color in [('fall','#ce4c46'), ('catch','#19867c')]:
        r = recordings[name]
        t = [x['time_s'] for x in r['trajectory']]
        roll = [np.degrees(x['foot_rpy_rad'][0]) for x in r['trajectory']]
        ax.plot(t, roll, color=color, label='Continue launch → fall' if name=='fall' else 'Same launch → catch')
    trigger = final['prefix_matches_through_trigger_s']
    ax.axvline(trigger, color='#46556b', linestyle=':', label=f'Catch trigger: {trigger:.2f} s')
    ax.set(title='Identical motion until the feedback trigger', xlabel='Time (s)', ylabel='Signed base roll (°)', xlim=(0,8))
    ax.legend(fontsize=9)
    ax = axes[0, 1]
    thresholds = [6,10,14,18,22,26,30,34,38]
    actions = [(action, duration) for action in ['neutral','preload','continue'] for duration in [.06,.16,.32]]
    values = np.full((len(actions),len(thresholds)), np.nan)
    selected_seed = final['selected_ids']['catch'].split('-')[0]
    for c in search['candidates']:
        seed, threshold, action, duration = c['id'].split('-')
        if seed != selected_seed:
            continue
        r = c['result']
        value = 0 if r['recovered'] else 1 if r['fell'] else 2 if not r['completed'] else 3
        values[actions.index((action,float(duration))),thresholds.index(int(threshold))] = value
    colors = ['#19867c','#ce4c46','#d69a32','#9ca9b7']
    ax.imshow(values, aspect='auto', cmap=ListedColormap(colors), vmin=-.5, vmax=3.5, interpolation='nearest')
    ax.set_xticks(range(len(thresholds)),thresholds)
    ax.set_yticks(range(len(actions)),[f'{a} / {d:.2f} s' for a,d in actions], fontsize=8)
    ax.set(title='Catch outcomes for this launch (0.5 ms physics)', xlabel='Delayed |roll| trigger (°)', ylabel='Head response / duration')
    ax.legend(handles=[Patch(color=c,label=l) for c,l in zip(colors,['Recovered','Fell','Guard stop','Unsettled'])],
              loc='upper center',bbox_to_anchor=(.5,-.16),ncol=4,fontsize=8)
    ax = axes[1,0]
    pivot = recordings['pivot']; rows = pivot['trajectory']; trace = pivot['trace']
    t = np.array([x['time_s'] for x in rows])
    ax.plot(t,[np.degrees(x['heading_from_startup_rad']) for x in rows],color='#19867c',label='Foot rotation on floor')
    ax.plot(t,[np.degrees(x['head_rpy_rad'][2]) for x in rows],color='#8460ac',label='Measured head yaw relative to torso')
    ax.plot(t,[np.degrees(x['target_head_rpy_rad'][2]) for x in rows],color='#8460ac',linestyle=':',alpha=.7,label='Head yaw target')
    edge = np.array([x['edge_support'] for x in trace])
    ax.fill_between(t,0,1,where=edge,transform=ax.get_xaxis_transform(),color='#d69a32',alpha=.14,label='One-edge support')
    ax.set(title='Head turn and stop during edge support',xlabel='Time (s)',ylabel='Angle (°)',xlim=(0,4))
    ax.legend(fontsize=8)
    ax = axes[1,1]
    xy=np.array([x['foot_xyz_m'][:2] for x in rows])*1000
    xy-=xy[0]
    ax.plot(xy[:,0],xy[:,1],color='#46556b',linewidth=1.2)
    ax.scatter(xy[edge,0],xy[edge,1],s=10,color='#d69a32',label='One-edge support')
    ax.scatter(*xy[0],marker='o',s=70,color='#19867c',label='Start')
    ax.scatter(*xy[-1],marker='X',s=80,color='#8460ac',label='Settled finish')
    drift=np.linalg.norm(xy[-1]); slip=pivot['edge_contact_slip_distance_mm']
    ax.set(title=f'Pivot: {drift:.1f} mm center shift, {slip:.1f} mm estimated slip',
           xlabel='World X from start (mm)',ylabel='World Y from start (mm)',aspect='equal')
    ax.legend(fontsize=8,loc='lower left')
    for a in [axes[0,0],axes[1,0],axes[1,1]]:
        a.grid(alpha=.18)
    output.parent.mkdir(parents=True,exist_ok=True)
    fig.savefig(output,dpi=160)
    plt.close(fig)
    print(output)


if __name__ == '__main__':
    main()
