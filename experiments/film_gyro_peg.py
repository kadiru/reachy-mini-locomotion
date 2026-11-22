"""Film a saved gyro-peg run with synchronized foot-frame and floor-frame traces."""
import argparse
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from PIL import Image, ImageDraw

from film_balance_qualification import font
from reachy_locomotion.contact_probe import BASE, scene_xml
from train_target_pivot import read

INK = '#f1f4ef'
MUTED = '#b2c5cd'
COP = '#ffb562'
FOOT = '#69d9da'


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('record', type=Path)
    p.add_argument('--output', required=True, type=Path)
    p.add_argument('--report',type=Path,help='Published qualification report for the matching policy')
    a = p.parse_args()
    run = read(a.record)
    rows = run['trace']
    deep='captured_s' in run['metrics']
    course = run['metrics'].get('floor_course') if not deep else None
    loop = course.get('best_loop') if course else None
    lift_circle = run['metrics'].get('circle_pass',False)
    if lift_circle:
        loop = run['metrics']['loop']
    summary=None
    if lift_circle:
        summary=(f"Closed foot-center lap   /   {loop['diameter_mm']/10:.2f} cm",
                 f"Lean during lap   {loop['min_lean']:.1f}° to {loop['max_lean']:.1f}°")
    if a.report and course and 'params' in run['metrics']:
        report=read(a.report)
        if report['wide_policy']['orbit']!=run['metrics']['params']:
            raise ValueError('Power and qualification labels require the exact frozen wide-course policy')
        checks=[r for r in report['wide_tests'] if r['condition'] in report['held_out_conditions']]
        cost=[r for r in report['training_cost'] if r['stage'] in ('periodic orbit','tilt feedback','wide floor circle')]
        summary=(f"Wide course {sum(r['large_floor_pass'] for r in checks)}/{len(checks)} held-out trials",
                 f"CPU fit {sum(r['gross_wh'] for r in cost):.2f} Wh   /   {100*sum(r['usd'] for r in cost):.3f}¢ at $0.30/kWh")
    if a.report and deep:
        report=read(a.report)
        selected=report['deep_tests']['nominal']
        if run['metrics']!=selected:
            raise ValueError('Deep-roll labels require the replay of the selected natural capture')
        checks=report['deep_tests']['held_out']
        cost=[r for r in report['training_cost'] if r['stage'] in ('deep edge reference','deep edge endurance','four-sector blend')]
        summary=(f"Deep roll {sum(r['passed'] for r in checks)}/{len(checks)} held-out trials",
                 f"Added fit {sum(r['gross_wh'] for r in cost):.2f} Wh   /   {100*sum(r['usd'] for r in cost):.3f}¢ at $0.30/kWh")
    terminal=run.get('terminal_state')
    if deep and run['metrics']['fell'] and terminal:
        roll,pitch,yaw=terminal['foot_rpy_rad']
        last=rows[-1]
        actual_yaw=last['foot_yaw_deg'] + (np.degrees(yaw)-last['foot_yaw_deg']+180)%360-180
        rows.append(dict(last,time_s=terminal['time_s'],foot_xyz_m=terminal['qpos'][:3],
                         normal_n=0.,contact_foot_xy_m=None,contact_world_m=None,foot_yaw_deg=actual_yaw,
                         tilt_deg=float(np.degrees(np.arccos(np.cos(roll)*np.cos(pitch))))))
        run['trajectory'].append(terminal['qpos'])
    xml,_ = scene_xml('experiments/upstream-reachy-mini',run['config'])
    root = ET.fromstring(xml)
    world = root.find('worldbody')
    ET.SubElement(world,'light',pos='1 -1 2',dir='-1 1 -2',directional='true',
                  diffuse='.8 .8 .8',ambient='.3 .3 .3')
    for k in range(-12,13):
        z = k*.05
        for coords in ((z,-.6,z,.6),(-.6,z,.6,z)):
            x,y,u,v=coords
            ET.SubElement(world,'geom',type='capsule',size='.00035',fromto=f'{x} {y} .0003 {u} {v} .0003',
                          rgba='.46 .55 .6 1',contype='0',conaffinity='0')
    if loop:
        cx,cy=np.array(loop['center_xy_mm'])/1000
        radius=loop['diameter_mm']/2000
        for a0 in np.linspace(0,2*np.pi,48,endpoint=False)[::2]:
            a1=a0+2*np.pi/48
            coords=[cx+radius*np.cos(a0),cy+radius*np.sin(a0),.0007,
                    cx+radius*np.cos(a1),cy+radius*np.sin(a1),.0007]
            ET.SubElement(world,'geom',type='capsule',size='.0007',fromto=' '.join(map(str,coords)),
                          rgba='.25 .75 .8 .85',contype='0',conaffinity='0')
    foot = world.find(f".//body[@name='{BASE}']")
    ET.SubElement(foot,'site',type='capsule',fromto='.055 0 .005 .055 0 .029',size='.003',
                  rgba='.2 .9 .9 1',group='0')
    model=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'))
    model.vis.global_.offwidth=800
    model.vis.global_.offheight=516
    camera=mujoco.MjvCamera()
    camera.azimuth,camera.elevation,camera.distance=151,-19,.68
    data=mujoco.MjData(model)
    centers=np.array([r['foot_xyz_m'][:2] for r in rows])*1000
    contact=np.array([(r['contact_world_m'] or [np.nan,np.nan])[:2] for r in rows])*1000
    valid=np.array([r['normal_n']>.5 and r['tilt_deg']>=5. and r['contact_foot_xy_m'] is not None
                    and np.linalg.norm(r['contact_foot_xy_m'])>=.85*run['config']['foot_radius_m'] for r in rows])
    local=np.array([r['contact_foot_xy_m'] or [0,0] for r in rows])/run['config']['foot_radius_m']
    fonts={s:font(s,s>=24) for s in (16,19,23,25,32)}
    # Zoom to an equal-scale, fixed world window over the last second and a half.
    field=np.vstack((centers,contact[valid]))
    midpoint=(field.min(axis=0)+field.max(axis=0))/2
    side=max(110.,np.ptp(field,axis=0).max()*1.13)
    def floor_point(xy):
        x,y=(xy-midpoint)/side*228
        return (1041-y,441-x)
    def rim_point(xy):
        x,y=xy*86
        return (1041-y,192-x)
    def trail(pen,points,flags,project,color,width):
        prev=None
        for xy,ok in zip(points,flags):
            here=project(xy) if ok else None
            if prev and here:pen.line((prev,here),fill=color,width=width)
            prev=here
    def dot(pen,point,color):
        x,y=point
        pen.ellipse((x-5,y-5,x+5,y+5),fill=color)
    schedule=[(i,'real time') for i in range(0,len(rows) if not course else min(400,len(rows)),2)]
    if course:
        schedule.extend((i,'4× floor path') for i in range(400,len(rows),8))
        schedule.append((len(rows)-1,'4× floor path'))
    if lift_circle:
        begin=round(50*loop['start_s'])
        end=min(len(rows),1+round(50*loop['end_s']))
        launch_end=min(begin,round(50*(run['metrics']['capture_s']+1.5)))
        schedule=[(i,'launch in real time') for i in range(0,launch_end,2)]
        schedule.extend((i,'measured lap in real time') for i in range(begin,end,2))
        if len(rows)-end>100:
            schedule.extend((i,'end of the trial') for i in range(len(rows)-51,len(rows),2))
        schedule.extend((i,'2× overhead center lap') for i in range(begin,end,4))
    elif deep and run['metrics']['safe']:
        begin=round(50*(run['metrics']['captured_s']+1.))
        end=min(len(rows),begin+round(50*run['metrics']['period']))
        schedule.extend((i,'2× overhead rim lap') for i in range(begin,end,4))
    elif len(rows)>350:
        last_fall=deep and run['metrics']['fell']
        replay_start=max(0,len(rows)-100) if last_fall else 200
        if last_fall:schedule.extend([(len(rows)-1,'body touched floor')]*17)
        schedule.extend((i,'0.25× contact replay') for i in range(replay_start,replay_start+100) for _ in range(2))
    if deep:
        angles=np.arctan2(local[:,1],local[:,0])
        deltas=np.angle(np.exp(1j*np.diff(angles)))
        keep=valid[1:] & valid[:-1] & (np.abs(deltas)<np.pi/2)
        keep &= np.array([r['time_s']>=(run['metrics']['captured_s'] or 1.5) for r in rows[:-1]])
        turns=np.r_[0.,np.cumsum(np.where(keep,deltas,0.))]*run['metrics']['direction']/(2*np.pi)
    a.output.parent.mkdir(parents=True,exist_ok=True)
    with subprocess.Popen(['ffmpeg','-v','error','-n','-f','rawvideo','-pix_fmt','rgb24','-s','1280x720',
                           '-r','25','-i','-','-an','-c:v','libx264','-threads','3','-preset','veryfast',
                           '-crf','19','-pix_fmt','yuv420p','-movflags','+faststart',str(a.output)],
                          stdin=subprocess.PIPE) as enc, mujoco.Renderer(model,height=516,width=800) as renderer:
        for i,speed in schedule:
            r=rows[i]
            if speed in ('4× floor path','2× overhead rim lap','2× overhead center lap'):
                camera.elevation=-56
                camera.distance=max(.66,.4+1.6*side/1000)
                camera.lookat[:]=[*midpoint/1000,.11]
            else:
                camera.elevation=-19
                camera.distance=.68
                camera.lookat[:]=[*np.array(r['foot_xyz_m'][:2]),.145]
            data.qpos[:]=run['trajectory'][i]
            mujoco.mj_forward(model,data)
            renderer.update_scene(data,camera=camera)
            frame=Image.new('RGB',(1280,720),'#11232c')
            frame.paste(Image.fromarray(renderer.render()),(20,90))
            pen=ImageDraw.Draw(frame)
            def label(x,y,s,size,fill=INK):
                pen.text((x,y),s,font=fonts[size],fill=fill)
            title='Deep lean from an edge capture' if deep else 'Rolling around the foot rim'
            if lift_circle:title='A foot-center circle at deep lean'
            if loop and run['metrics'].get('large_floor_pass'):title='A wide circle on a rolling rim'
            label(24,8,title,32)
            label(24,51,'Reachy Mini   /   MuJoCo simulation   /   fixed waist   /   '+speed,19,MUTED)
            if summary:
                label(849,5,summary[0],19,INK)
                label(849,31,summary[1],16,MUTED)
            label(849,60,'Contact on the foot',25)
            pen.ellipse((955,106,1127,278),outline='#768d98',width=3)
            label(1020,84,'front',16,MUTED)
            label(1022,277,'back',16,MUTED)
            label(913,180,'left',16,MUTED)
            label(1141,180,'right',16,MUTED)
            j=max(0,i-(round(50*run['metrics']['period']) if deep else 17))
            if lift_circle:
                j=max(0,i-round(50*run['metrics']['orbit'].get('period',14.)))
            trail(pen,local[j:i+1],valid[j:i+1],rim_point,COP,4)
            if valid[i]:dot(pen,rim_point(local[i]),COP)
            label(849,310,'Ground tracks',25)
            if loop:
                cx,cy=floor_point(np.array(loop['center_xy_mm']))
                rad=loop['diameter_mm']/2/side*228
                pen.ellipse((cx-rad,cy-rad,cx+rad,cy+rad),outline='#647e89')
            if (course or lift_circle) and i>=14:
                pen.line([floor_point(v) for v in centers[:i+1:8]],fill='#438387',width=2)
            j=max(0,i-(round(50*run['metrics']['period']) if deep else 74))
            if lift_circle:
                j=max(0,i-round(50*loop['period_s']))
            trail(pen,contact[j:i+1],valid[j:i+1],floor_point,COP,2)
            trail(pen,centers[j:i+1],np.ones(i+1-j,bool),floor_point,FOOT,3)
            dot(pen,floor_point(centers[i]),FOOT)
            if valid[i]:dot(pen,floor_point(contact[i]),COP)
            label(854,563,'Foot center',19,FOOT)
            label(1000,563,'Loaded contact',19,COP)
            label(854,595,f'World axes fixed   /   plot spans {side/10:.0f} cm',16,MUTED)
            pen.line((24,626,1252,626),fill='#3b5260',width=2)
            label(24,644,f"{r['time_s']:.2f} s",25)
            label(175,644,f"Base lean  {r['tilt_deg']:.1f}°",25)
            label(483,644,f"Floor-relative base yaw  {r['foot_yaw_deg']:+.1f}°",25)
            if deep:label(913,646,f'Rim laps  {turns[i]:.2f}',23,FOOT)
            if loop:
                label(913,646,f"Floor loop  {loop['diameter_mm']/10:.1f} cm",23,FOOT)
            note=('Fixed overhead camera. The dashed ring is fitted to a measured lap of the foot center.'
                  if speed in ('4× floor path','2× overhead center lap') and loop else
                  'Orange follows the measured center of pressure. The cyan stripe marks the front of the foot.')
            label(24,684,note,19,MUTED)
            enc.stdin.write(frame.tobytes())
        enc.stdin.close()
        if enc.wait():raise RuntimeError('Video encoder failed')


if __name__=='__main__':
    main()
