"""Replay signed platform motion, free robot state and actual edge contact on a level camera."""
import argparse
import math
import json
import hashlib
from concurrent.futures import ProcessPoolExecutor
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw

from film_shell_release import face, INK, DIM, CYAN, GOLD
from moving_table import REPORT, UPSTREAM, WORK, read, simulate
from qualify_moving_table import edge_pass
from moving_table_feedback import FEEDBACK_REPORT, replay_level
from reachy_locomotion.contact_probe import scene_xml

MEDIA=Path('media/simulation')


def render_model(config):
    root=ET.fromstring(scene_xml(UPSTREAM,config)[0])
    world=root.find('worldbody')
    table=world.find("body[@name='tilt_table']")
    table.find("geom[@name='floor']").set('rgba','.21 .36 .38 1')
    for y in np.arange(-1.,1.01,.05):
        ET.SubElement(table,'geom',type='capsule',size='.00055',fromto=f'-.45 {y} .0003 .45 {y} .0003',
                      rgba='.63 .79 .76 1',contype='0',conaffinity='0')
    for x in np.arange(-.45,.46,.05):
        ET.SubElement(table,'geom',type='capsule',size='.00055',fromto=f'{x} -1 .0003 {x} 1 .0003',
                      rgba='.63 .79 .76 1',contype='0',conaffinity='0')
    ET.SubElement(world,'light',pos='1 -1 2',dir='-1 1 -2',directional='true',ambient='.35 .35 .35')
    m=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'))
    m.vis.global_.offwidth=672
    m.vis.global_.offheight=440
    return m


def choose(report, mode):
    summary=report['summary']
    rows=[r for r in report['trials'] if not r['job']['flat'] and r['job']['angle'] and
          r['job']['cycles']==(1 if mode=='angle' else 2)]
    selected=[]
    def control_key(job):
        return (job['side'],abs(job['angle']),job['rate'],job['cycles'],job['seed'])
    level_ok={control_key(r['job']):edge_pass(r) for r in report['trials'] if r['job']['flat']}

    def add(candidates, label, prefer_failure=False):
        if not candidates:
            return
        chosen = min(candidates, key=lambda r: (
            edge_pass(r) if prefer_failure else not edge_pass(r),
            not level_ok.get(control_key(r['job']),True),
            prefer_failure and r['shell'] is None, r['job']['seed']))
        if all(chosen['job'] != prev[0]['job'] for prev in selected):
            selected.append((chosen, label))

    if mode == 'angle':
        for side in (1,-1):
            add([r for r in rows if r['job']['angle']==10 and r['job']['side']==side],
                'Ten degrees and back', prefer_failure=side<0)
    for side in (1,-1):
        for orient in ('uphill','downhill'):
            group_summary=next(v for v in summary if v['side']==side and v['orientation']==orient)
            peak=group_summary['angle_pass' if mode=='angle' else 'rate_pass']
            miss=group_summary['angle_miss' if mode=='angle' else 'rate_miss']
            sign=side*(1 if orient=='downhill' else -1)
            group=[r for r in rows if r['job']['side']==side and np.sign(r['job']['angle'])==sign]
            if mode=='rate':
                group=[r for r in group if abs(r['job']['angle'])==group_summary['speed_amplitude']]
            def measure(r):
                return abs(r['job']['angle']) if mode=='angle' else r['job']['rate']
            if peak:
                add([r for r in group if measure(r)==peak and edge_pass(r)],'Upper passing test')
            if miss:
                add([r for r in group if measure(r)==miss and not edge_pass(r)],'First missed test',True)
    return selected


def plot(pen,box,ts,values,t,extent,color):
    x,y,w,h=box
    pen.rounded_rectangle((x,y,x+w,y+h),radius=4,fill='#1c343d')
    pen.line((x,y+h/2,x+w,y+h/2),fill='#668089')
    n=min(len(ts),int(np.searchsorted(ts,t,side='right')))
    points=[(x+w*ts[i]/ts[-1],y+h/2-(h/2-5)*max(-1.,min(1.,values[i]/extent))) for i in range(0,n,max(1,len(ts)//(2*w)))]
    if len(points)>1:pen.line(points,fill=color,width=2)
    if points:
        u,v=points[-1];pen.ellipse((u-3,v-3,u+3,v+3),fill=color)


def prepare(row):
    # A changed score, condition or policy gets a fresh deterministic-replay check.
    fingerprint = hashlib.sha256(json.dumps(row,sort_keys=True).encode()+Path('configs/cad-release-catch.json').read_bytes()).hexdigest()
    WORK.mkdir(parents=True,exist_ok=True)
    cache = WORK / ('film-' + fingerprint + '.npz')
    if not cache.exists():
        replay,track,config,frames = simulate(row['job'],recording=True)
        if replay != row:
            raise ValueError(f"Numerical replay mismatch for {row['job']}")
        for sample in track:
            sample.pop('qpos')
        with tempfile.NamedTemporaryFile(dir=WORK,suffix='.npz',delete=False) as out:
            partial=Path(out.name)
            try:
                np.savez_compressed(out,frames=frames,track=json.dumps(track),config=json.dumps(config))
            except BaseException:
                partial.unlink(missing_ok=True)
                raise
        partial.replace(cache)
    return cache


def film(mode, report_path=REPORT):
    report=read(report_path)
    examples=choose(report,mode)
    if not examples:
        return
    with ProcessPoolExecutor(4) as pool:
        files = dict(zip((json.dumps(r['job'],sort_keys=True) for r,_ in examples), pool.map(prepare,[r for r,_ in examples])))
    chapters=[]
    for r,label in examples:
        j=r['job']
        speed=max(2.,math.ceil(r['duration']/11)) if mode=='angle' else max(1.,math.ceil(r['duration']/14))
        chapters.append((r,label,1.6,r['duration'],speed))
        if mode=='rate' and j['rate']>=32:
            event=r['first_loss']
            at=event['time_s'] if event else 3.
            chapters.append((r,'Slow replay',max(2.75,at-.20),min(r['duration'],at+.45),.125))
    path=MEDIA/f'moving-table-{mode}.mp4'
    with tempfile.NamedTemporaryFile('w',suffix='.ffmeta') as meta:
        meta.write(';FFMETADATA1\n')
        counts=[];frame=0
        for r,label,start,stop,speed in chapters:
            length=math.ceil((stop-start)*25/speed)+15
            counts.append(length)
            j=r['job']
            title=f"{label} / {'right' if j['side']==1 else 'left'} edge / {j['angle']:+g} deg / {j['rate']:g} deg per s"
            meta.write(f'[CHAPTER]\nTIMEBASE=1/25\nSTART={frame}\nEND={frame+length}\ntitle={title}\n')
            frame+=length
        meta.flush()
        cmd=['ffmpeg','-v','error','-y','-f','rawvideo','-pix_fmt','rgb24','-s','1120x600','-r','25','-i','-',
             '-f','ffmetadata','-i',meta.name,'-map','0:v','-map_chapters','1','-c:v','libx264','-threads','3',
             '-preset','veryfast','-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',str(path)]
        model=renderer=previous=None
        with subprocess.Popen(cmd,stdin=subprocess.PIPE) as enc:
            try:
                for (r,label,start,stop,speed),length in zip(chapters,counts):
                    j=r['job']
                    if j!=previous:
                        with np.load(files[json.dumps(j,sort_keys=True)]) as saved:
                            track=json.loads(str(saved['track']))
                            cfg=json.loads(str(saved['config']))
                            frames=saved['frames']
                        previous=j
                        ts=np.array([x['t'] for x in track])
                        roll=np.array([x['roll'] for x in track])
                    if model is None:
                        model=render_model(cfg);data=mujoco.MjData(model)
                        renderer=mujoco.Renderer(model,height=440,width=672)
                        cam=mujoco.MjvCamera()
                        cam.azimuth,cam.elevation,cam.distance=180,-5,.60
                    name='Right' if j['side']>0 else 'Left'
                    direction='downhill' if j['side']*j['angle']>0 else 'uphill'
                    peers=[x for x in report['trials'] if all(x['job'][k]==j[k] for k in ('side','angle','rate','cycles','flat','dt'))]
                    data.qpos[:] = frames[0,3:]
                    mujoco.mj_forward(model,data)
                    initial_y=float(data.xpos[model.body('body_foot_3dprint').id,1])
                    for idx in range(length):
                        t=min(stop,start+idx*speed/25)
                        k=min(len(frames)-1,max(0,int(round(t/.005))-1))
                        _,bank,rate,*q=frames[k]
                        sample=track[min(len(track)-1,round(t/.02))]
                        data.qpos[:]=q;mujoco.mj_forward(model,data)
                        foot=data.xpos[model.body('body_foot_3dprint').id]
                        head=data.site_xpos[model.site('head').id]
                        cam.lookat[:]=.47*foot+.53*head
                        renderer.update_scene(data,camera=cam)
                        im=Image.new('RGB',(1120,600),'#10252c');pen=ImageDraw.Draw(im)
                        pen.text((18,10),f"MUJOCO   /   MOVING PLATFORM   /   {speed:g}× PLAYBACK   /   {t:.2f} s",font=face(16,True),fill=CYAN)
                        title='Slow angle excursion' if mode=='angle' else 'Repeated speed test'
                        pen.text((18,36),f"{title}   /   {name.lower()} edge {direction}",font=face(29,True),fill=INK)
                        im.paste(Image.fromarray(renderer.render()),(18,84))
                        x=711
                        pen.text((x,90),f"Platform   {bank:+.1f}°",font=face(27,True),fill=GOLD)
                        pen.text((x,128),f"Body roll   {sample['roll']:+.1f}°",font=face(24,True),fill=CYAN)
                        pen.text((x,169),f"Angular speed   {rate:+.1f}°/s",font=face(19),fill=INK)
                        plot(pen,(x,216,378,66),frames[:,0],frames[:,1],t,max(1.,abs(j['angle'])*1.15),GOLD)
                        pen.text((x,285),'Platform angle',font=face(16),fill=GOLD)
                        if mode=='angle':
                            plot(pen,(x,315,378,66),ts,roll,t,max(40.,float(np.max(abs(roll)))*1.05),CYAN)
                            caption='Body roll relative to gravity'
                        else:
                            plot(pen,(x,315,378,66),frames[:,0],frames[:,2],t,1.1*j['rate'],CYAN)
                            caption='Measured platform speed'
                        pen.text((x,385),caption,font=face(16),fill=CYAN)
                        loss=r['first_loss'];shell=r['shell']
                        state='Starting edge hold' if t<3. else 'On requested edge' if sample['edge'] else 'Requested edge unloaded'
                        if loss and t>=loss['time_s']:
                            state='Edge reloaded after loss' if sample['edge'] else 'Requested edge unloaded'
                        if shell and t>=shell['time_s']:state=f"Shell landed at {shell['time_s']:.2f} s"
                        pen.text((x,440),state,font=face(20,True),fill=GOLD if loss and t>=loss['time_s'] else INK)
                        pen.text((x,412),f"Head roll relative to foot   {sample['head']:+.1f}°",font=face(16),fill=DIM)
                        if loss and t>=loss['time_s']:
                            pen.text((x,466),f"First sustained loss  {loss['time_s']:.2f} s",font=face(15),fill=DIM)
                        pen.text((x,490),f"Rim height   {sample['lift']:.0f} mm   /   Lateral travel   {(foot[1]-initial_y)*100:+.1f} cm",font=face(16),fill=DIM)
                        pen.text((18,533),f"{label}   /   0° → {j['angle']:+g}° → 0°   /   {j['cycles']} cycle{'s' if j['cycles']>1 else ''}   /   {j['rate']:g}°/s {'cruise' if mode=='rate' else 'peak'}",font=face(21,True),fill=INK)
                        if edge_pass(r):
                            outcome='Held and settled' if r['passed'] else 'Held with drift during dwell'
                        else:
                            outcome='Missed the hold'
                        pen.text((18,569),f"{outcome}   /   {sum(edge_pass(x) for x in peers)} of {len(peers)} seeds passed this condition",font=face(17),fill=DIM)
                        enc.stdin.write(im.tobytes())
            finally:
                if renderer:renderer.close()
                enc.stdin.close()
            if enc.wait():raise RuntimeError('Video encoding failed')
    print(path,flush=True)


def feedback_film():
    pair = read(FEEDBACK_REPORT)[0]
    expected = pair['level_command_replay']
    job = expected['job']
    replay, track, config, blind_frames = replay_level(job, recording=True)
    if replay != expected:
        raise ValueError('Level-command replay diverged from the archived trial')
    with np.load(prepare(pair['feedback'])) as saved:
        live_frames = saved['frames']
        live_track = json.loads(str(saved['track']))
    model = render_model(config)
    data = mujoco.MjData(model)
    camera = mujoco.MjvCamera()
    camera.azimuth, camera.elevation, camera.distance = 180, -5, .58
    path = MEDIA/'moving-table-feedback.mp4'
    cmd = ['ffmpeg','-v','error','-y','-f','rawvideo','-pix_fmt','rgb24','-s','1120x564',
           '-r','25','-i','-','-c:v','libx264','-threads','3','-preset','veryfast',
           '-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',str(path)]
    with mujoco.Renderer(model,height=346,width=534) as renderer, subprocess.Popen(cmd,stdin=subprocess.PIPE) as enc:
        for frame in range(math.ceil((expected['duration']-2.)*25)+22):
            t = min(2.+frame/25, expected['duration'])
            k = min(len(live_frames)-1,round(t/.005)-1)
            picture = Image.new('RGB',(1120,564),'#10252c')
            pen = ImageDraw.Draw(picture)
            pen.text((18,10),'MUJOCO   /   10° TABLE EXCURSION   /   RIGHT EDGE DOWNHILL   /   REAL TIME',
                     font=face(16,True),fill=CYAN)
            pen.text((18,35),'Does Reachy use feedback to stay on the edge?',font=face(31,True),fill=INK)
            panels = [(18,live_frames,live_track,'Live balance feedback'),
                      (568,blind_frames,track,'Head commands recorded on level ground')]
            for x,frames,samples,label in panels:
                data.qpos[:] = frames[k,3:]
                mujoco.mj_forward(model,data)
                # Same fixed world camera for both runs; the table rotates within the frame.
                camera.lookat[:] = [0.,0.,.105]
                renderer.update_scene(data,camera=camera)
                pen.text((x,89),label,font=face(20,True),fill=INK)
                picture.paste(Image.fromarray(renderer.render()),(x,126))
                state=samples[min(len(samples)-1,round(t/.02))]
                pen.text((x,481),f"Body roll  {state['roll']:+.1f}°    /    Head roll  {state['head']:+.1f}°",
                         font=face(18),fill=CYAN)
                status = 'Right edge loaded' if state['edge'] else 'Right edge unloaded'
                pen.text((x,511),status,font=face(18,True),fill=INK if state['edge'] else GOLD)
            pen.text((18,540),f"Elapsed  {t:.2f} s     /     Measured deck angle  {live_frames[k,1]:+.2f}°",
                     font=face(15),fill=DIM)
            enc.stdin.write(picture.tobytes())
        enc.stdin.close()
        if enc.wait():
            raise RuntimeError('Comparison video encoding failed')
    print(path,flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('mode',choices=['angle','rate','feedback'])
    p.add_argument('--report',type=Path,default=REPORT)
    args=p.parse_args()
    mode=args.mode
    if mode=='feedback':
        feedback_film()
    else:
        film(mode,args.report)
