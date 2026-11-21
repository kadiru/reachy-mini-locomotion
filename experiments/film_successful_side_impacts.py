"""Show every contact-confirmed, sustained recovery in the side-impact recordings."""
import argparse
import json
import math
from pathlib import Path
import subprocess

import mujoco
import numpy as np
from PIL import Image, ImageDraw

from film_hit_envelope import Panel, face
from film_balance_qualification import ACCENT, BACKGROUND, INK, MUTED
from review_side_ball_study import recovered_edge

FPS = 25
WIDTH, VIEW, HEIGHT = 768, 350, 480


def frames(release, hit, hold, end):
    # Keep every phase in chronological order, including the entire final hold.
    pieces = [(release-.25, 3.), (min(hit+.18, hold+.5), .35), (min(end, hold+1.5), 1.), (end, 3.)]
    start = 0.
    for stop, rate in pieces:
        for i in range(math.ceil((stop-start)*FPS/rate)):
            yield start+i*rate/FPS, rate
        start = stop
    yield end, 1.


def description(row):
    if row['mode']=='direct':
        return 'Direct side hit'
    if row['mode']=='rolling':
        return 'Rolling ball'
    if not row['airborne_hit']:
        return 'Throw with a floor bounce'
    return 'Airborne throw'


def verified_direct_hit(row):
    event = row.get('event',{})
    hit, floor = event.get('first_robot_contact_s'), event.get('first_floor_contact_s')
    return (row['mode']=='direct' and event.get('contacted_robot') and hit is not None
            and event['parameters'].get('vertical_speed_m_s')==0.
            and hit>event['release_s']+2*row['dt'] and (floor is None or hit<floor))


def impact_excerpt(release, hit, hold, end):
    start = release-.10
    for stop, rate in ((hit+.12,.20), (min(end,hold+1.6),1.)):
        for i in range(math.ceil((stop-start)*FPS/rate)):
            yield start+i*rate/FPS, rate
        start = stop


def render_episode(pipe, row, path, index, total):
    panel = Panel(dict(record=str(path)))
    run = panel.run
    impact = run['balance_curriculum']['impacts']['events'][0]
    if not (impact['contacted_robot'] and impact['balanced_before_impact'] and
            run['completed'] and run['edge_balance']['metrics']['final_five_seconds_pass'] and
            impact['recovery_s']==row['recovery_s'] and run['ball_disturbances'][0]==row['event']):
        raise ValueError('Recording does not match the successful outcome for '+row['id'])
    # Use the common film camera and lights, with a larger single-episode viewport.
    panel.renderer.close()
    panel.model.vis.global_.offwidth = max(WIDTH,panel.model.vis.global_.offwidth)
    panel.model.vis.global_.offheight = max(VIEW,panel.model.vis.global_.offheight)
    panel.camera.distance = .64
    samples = sorted(run['trajectory']+run['collision_frames'],key=lambda s:s['time_s'])
    times = np.array([s['time_s'] for s in samples])
    event = row['event']
    release, hit = event['parameters']['launch_s'], event['first_robot_contact_s']
    hold = hit + impact['recovery_s']
    speed = event['incoming_3d_speed_m_s']
    count = 0
    base = Image.new('RGB',(WIDTH,HEIGHT),BACKGROUND)
    pen = ImageDraw.Draw(base)
    pen.text((16,7),f'REACHY MINI   /   MUJOCO SIMULATION   /   {index:02} OF {total:02}',font=face(14,True),fill=ACCENT)
    pen.text((16,27),f"{description(row)} from the {row['source']}   ·   {row['edge'].title()} edge",font=face(23,True),fill=INK)
    pen.text((16,HEIGHT-26),f"{speed:.2f} m/s at contact     ·     Edge hold began {row['recovery_s']:.2f} s after impact",font=face(17),fill=MUTED)
    direct = row['mode']=='direct'
    if direct:
        pen.text((WIDTH-165,9),f"{1000*event['parameters']['mass_kg']:g} g simulated ball",font=face(15),fill=MUTED)
    timeline = impact_excerpt if direct else frames
    with mujoco.Renderer(panel.model,height=VIEW,width=WIDTH) as renderer:
        for t, rate in timeline(release,hit,hold,run['elapsed_seconds']):
            sample = samples[np.searchsorted(times,t+1e-8,side='right')-1]
            panel.data.qpos[:] = sample['qpos']
            mujoco.mj_forward(panel.model,panel.data)
            renderer.update_scene(panel.data,camera=panel.camera)
            frame = base.copy()
            frame.paste(Image.fromarray(renderer.render()),(0,62))
            label = 'Lifting onto the edge' if t<release-1 else 'Balancing on the edge'
            if t>=release:
                label = 'Ball approaching'
            if t>=hit:
                label = 'Ball hits' if t<hit+.12 else 'Recovering balance'
                if direct and t<hit+.12:
                    label = f"Direct contact {100*event['first_contact_height_m']:.1f} cm above floor"
            if t>=hold+.5:
                label = 'Edge hold confirmed'
            if t>=run['elapsed_seconds']-1e-6:
                label = 'Held the edge for the final 5 seconds'
            pen = ImageDraw.Draw(frame)
            pen.text((16,419),label,font=face(20,True),fill='#9ae9c4' if t>=hold+.5 else INK)
            pen.text((536,422),f'{t:5.2f} s    |    {rate:g}× playback',font=face(16),fill=ACCENT)
            pipe.write(frame.tobytes())
            count+=1
    return count


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('report',type=Path)
    parser.add_argument('--states',type=Path,required=True)
    parser.add_argument('--work',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--direct-only',action='store_true',help='Include only verified robot-first horizontal throws')
    args = parser.parse_args()
    rows = json.loads(args.report.read_text())['rows']
    successes = [r for r in rows if r.get('event',{}).get('contacted_robot') and recovered_edge(r)]
    if args.direct_only:
        successes = [r for r in successes if verified_direct_hit(r)]
    if not successes:
        raise ValueError('No successful impacts')
    successes.sort(key=lambda r: -r['peak_roll_change_deg'])
    if args.direct_only:
        # Open with a body-side hit so that the airborne contact is easy to see.
        successes.sort(key=lambda r: r['event']['first_robot_body']=='body_foot_3dprint')
    args.work.mkdir(parents=True,exist_ok=False)
    raw, metadata = args.work/'episodes.mp4', args.work/'chapters.ffmeta'
    chapters=[';FFMETADATA1']
    cursor=0
    with subprocess.Popen(['ffmpeg','-v','error','-n','-f','rawvideo','-pix_fmt','rgb24','-s',f'{WIDTH}x{HEIGHT}',
                           '-r',str(FPS),'-i','-','-an','-c:v','libx264','-crf','20','-preset','veryfast',
                           '-threads','2','-pix_fmt','yuv420p',str(raw)],stdin=subprocess.PIPE) as enc:
        for i,row in enumerate(successes,1):
            n=render_episode(enc.stdin,row,args.states/(row['id']+'.json'),i,len(successes))
            title=f"{i:02}  {description(row)} from {row['source']}, {row['edge']} edge, {row['event']['incoming_3d_speed_m_s']:.2f} m/s"
            chapters.extend(['[CHAPTER]',f'TIMEBASE=1/{FPS}',f'START={cursor}',f'END={cursor+n}',f'title={title}'])
            cursor+=n
            print(row['id'],cursor/FPS,flush=True)
        enc.stdin.close()
        if enc.wait():
            raise RuntimeError('Frame encoder failed')
    metadata.write_text('\n'.join(chapters)+'\n')
    subprocess.run(['ffmpeg','-v','error','-n','-i',str(raw),'-f','ffmetadata','-i',str(metadata),
                    '-map','0:v','-map_chapters','1','-c','copy','-movflags','+faststart',str(args.output)],check=True)


if __name__=='__main__':
    main()
