"""Film synchronized cable and robot views for each simulated release."""
import argparse
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from PIL import Image, ImageDraw

from film_balance_qualification import font
from reachy_locomotion.contact_probe import scene_xml
from train_deep_lift import WORK, read
from test_deep_pendulum import ANGLES

WHITE='#f0f3f3'
MUTED='#b6c4cc'
ORANGE='#ffa858'
CYAN='#75ded3'


def frames(path):
    clip=read(path)
    r=clip['metrics']
    setting=r['settings']
    raw,_=scene_xml('experiments/upstream-reachy-mini',clip['config'])
    root=ET.fromstring(raw)
    world=root.find('worldbody')
    ET.SubElement(world,'light',pos='.5 -.5 1.4',dir='-.3 .2 -1',directional='true',
                  diffuse='.8 .8 .8',ambient='.32 .32 .32')
    anchor=setting['anchor_xyz_m']
    # Fine, fixed dashed plumb line gives the release angle a vertical reference.
    for z in np.arange(anchor[2]-.61,anchor[2],.028):
        ET.SubElement(world,'geom',type='capsule',fromto=f'-.007 {anchor[1]} {z} -.007 {anchor[1]} {z+.012}',
                      size='.001',rgba='.44 .57 .61 1',contype='0',conaffinity='0')
    model=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'))
    model.vis.global_.offwidth=750
    model.vis.global_.offheight=492
    data=mujoco.MjData(model)
    camera=mujoco.MjvCamera()
    ball=model.body('pendulum_bob').id
    q=clip['trajectory']
    trace=clip['trace']
    release=setting['release_s']
    first=r['hits'][0] if r['hits'] else None
    side='left' if setting['side']==1 else 'right'
    schedule=[(i,'Real time') for i in range(round((release-.48)*50),min(len(q),round((release+2.30)*50)),2)]
    if first:
        begin=max(0,int((first['time_s']-.19)*50))
        end=min(len(q),int((first['time_s']+.38)*50))
        schedule.extend((i,'Half-speed contact replay') for i in range(begin,end))
    if r['body_floor']:
        landing=r['body_floor']['time_s']
        begin=max(0,int((landing-.7)*50))
        end=min(len(q),int((landing+.38)*50))
        schedule.extend((i,'Landing replay') for i in range(begin,end,2))
    end_index=len(q)-1
    schedule.extend((i,'Final second') for i in range(max(0,end_index-50),end_index+1,2))
    spring=setting['length_m']
    fonts={size:font(size,size>=25) for size in (17,20,23,25,28,35)}
    with mujoco.Renderer(model,height=492,width=750) as renderer:
        for i,speed in schedule:
            data.qpos[:]=q[i]
            mujoco.mj_forward(model,data)
            y,z=data.xpos[ball][1:]
            angle=np.degrees(np.arctan2(setting['side']*(y-anchor[1]),anchor[2]-z))
            # Both views keep a fixed world camera throughout this release.
            camera.azimuth,camera.elevation,camera.distance=2,-6,1.10
            camera.lookat[:]=[0,setting['side']*.27,.39]
            renderer.update_scene(data,camera=camera)
            frame=Image.new('RGB',(1280,720),'#11232c')
            frame.paste(Image.fromarray(renderer.render()),(20,95))
            camera.azimuth,camera.elevation,camera.distance=25,-16,.55
            camera.lookat[:]=[0,0,.145]
            renderer.update_scene(data,camera=camera)
            detail=Image.fromarray(renderer.render()).resize((450,295),Image.Resampling.LANCZOS)
            frame.paste(detail,(805,97))
            pen=ImageDraw.Draw(frame)
            def label(x,y,value,size,color=WHITE):
                pen.text((x,y),value,font=fonts[size],fill=color)
            t=trace[i]['time_s']
            label(20,8,f"Pendulum from the {side}   /   {setting['release_deg']}° release",35)
            label(22,54,'Reachy Mini   /   MuJoCo   /   fixed waist   /   '+speed,20,MUTED)
            label(808,53,'Robot close-up',23,MUTED)
            if any(h['time_s']<=t+.01 and h.get('last_s',h['time_s'])>=t-.01 for h in r['hits']):
                pen.rectangle((804,96,1256,393),outline=ORANGE,width=4)
                label(815,104,'Contact',23,ORANGE)
            label(806,410,'Time since release',20,MUTED)
            label(1090,406,f'{t-release:+.2f} s',25)
            label(806,452,'Bob angle from vertical',20,MUTED)
            label(1090,449,f'{angle:+.1f}°',25,ORANGE)
            label(806,495,'Base lean',20,MUTED)
            label(1090,491,f"{trace[i]['tilt_deg']:.1f}°",25,CYAN)
            if first:
                label(806,537,'First contact speed',20,MUTED)
                label(1090,534,f"{first['speed_m_s']:.2f} m/s",25)
            else:
                label(806,537,'No robot contact recorded',20,MUTED)
            pen.line((20,607,1258,607),fill='#4c616c',width=2)
            if first:
                outcome='Held the edge after contact' if r['recovered'] else (
                    'Body contacted the floor' if r['body_floor'] else 'Lost the qualified edge hold')
            else:
                outcome='Ball stayed clear of Reachy'
            if not r['captured_before_release']:
                outcome+='   /   initial edge capture missed'
            label(24,618,'Outcome   /   '+outcome,28,CYAN if r['recovered'] else WHITE)
            label(24,667,f"25 g ball   /   5 cm diameter   /   {100*spring:.0f} cm cable   /   8 in horizontal clearance at rest",20,MUTED)
            yield frame


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    a=parser.parse_args()
    with subprocess.Popen(['ffmpeg','-v','error','-n','-f','rawvideo','-pix_fmt','rgb24',
                           '-s','1280x720','-r','25','-i','-','-an','-c:v','libx264',
                           '-threads','3','-preset','veryfast','-crf','19','-pix_fmt','yuv420p',
                           '-movflags','+faststart',str(a.output)],stdin=subprocess.PIPE) as enc:
        rows=read(WORK/'pendulum-checks.json')
        extra_path=WORK/'pendulum-grazing.json'
        extras=read(extra_path) if extra_path.exists() else []
        impacts=[r for r in rows+extras if r['hits'] and r['captured_before_release']]
        best=max(impacts,key=lambda r:(r['recovered'],r['hits'][0]['impulse_ns']))
        lead=(best['settings']['side'],best['settings']['release_deg'])
        schedule=[lead]+[(side,angle) for angle in ANGLES for side in (1,-1) if (side,angle)!=lead]
        for side,angle in schedule:
            for frame in frames(WORK/f'pendulum-{side}-{angle}.json'):
                enc.stdin.write(frame.tobytes())
        enc.stdin.close()
        if enc.wait():
            raise RuntimeError('Video encoder failed')


if __name__=='__main__':
    main()
