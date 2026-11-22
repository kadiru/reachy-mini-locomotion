"""Replay measured flat-to-edge launches on a visibly inclined CAD ramp."""
import argparse
import math
import subprocess
import tempfile
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from PIL import Image, ImageDraw

from film_shell_release import face, INK, DIM, CYAN, GOLD
from reachy_locomotion.contact_probe import scene_xml
from slope_balance import WORK, UPSTREAM, read


class View:
    def __init__(self,name):
        d=read(WORK/(name+'.json'))
        self.rows=d['track'];self.r=d['result']
        bank=math.radians(self.r['bank'])
        root=ET.fromstring(scene_xml(UPSTREAM,d['config'])[0])
        world=root.find('worldbody')
        assets=root.find('asset')
        ET.SubElement(assets,'texture',name='incline_checks',type='2d',builtin='checker',
                      rgb1='.21 .36 .38',rgb2='.28 .44 .45',width='128',height='128')
        ET.SubElement(assets,'material',name='incline_surface',texture='incline_checks',
                      texrepeat='10 10',texuniform='true',reflectance='0',specular='.05')
        floor=world.find("geom[@name='floor']")
        floor.set('size','0 0 .05')
        floor.set('material','incline_surface')
        floor.set('rgba','1 1 1 1')
        ramp=ET.SubElement(world,'body',name='display_ramp',quat=f'{math.cos(bank/2)} {math.sin(bank/2)} 0 0')
        for y in np.arange(-.35,.36,.05):
            ET.SubElement(ramp,'geom',type='capsule',size='.00048',fromto=f'-.32 {y} .0002 .32 {y} .0002',
                          rgba='.53 .66 .65 1',contype='0',conaffinity='0')
        for x in np.arange(-.30,.31,.05):
            ET.SubElement(ramp,'geom',type='capsule',size='.00048',fromto=f'{x} -.39 .0002 {x} .39 .0002',
                          rgba='.53 .66 .65 1',contype='0',conaffinity='0')
        ET.SubElement(world,'light',pos='1 -1 2',dir='-1 1 -2',directional='true',diffuse='.72 .72 .72',ambient='.30 .30 .30')
        self.m=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'))
        self.m.vis.global_.offwidth=608
        self.d=mujoco.MjData(self.m)
        self.foot_id=self.m.body('body_foot_3dprint').id
        self.head_id=self.m.site('head').id
        self.renderer=mujoco.Renderer(self.m,height=320,width=608)
        self.camera=mujoco.MjvCamera()
        self.camera.azimuth,self.camera.elevation,self.camera.distance=177,-6,.70
        self.camera.lookat[:]=[0,0,.10]

    def draw(self,image,x,t,label,angle_limit):
        pen=ImageDraw.Draw(image)
        n=min(round(t*50),len(self.rows)-1);row=self.rows[n]
        self.d.qpos[:]=row['qpos'];mujoco.mj_forward(self.m,self.d)
        self.camera.lookat[:]=.47*self.d.xpos[self.foot_id]+.53*self.d.site_xpos[self.head_id]
        self.renderer.update_scene(self.d,camera=self.camera)
        image.paste(Image.fromarray(self.renderer.render()),(x,120))
        s=self.r['side'];b=self.r['bank']
        edge='left' if s==-1 else 'right'
        pen.text((x,71),label,font=face(23,True),fill=INK)
        pen.text((x,99),f"{abs(b):g}° ramp   /   {edge} edge   /   {self.r['orientation']}",font=face(16),fill=DIM)
        pen.text((x,453),f"Body roll {row['roll']:+.1f}°    Head roll {row['head']:+.1f}°",font=face(22,True),fill=CYAN)
        pen.text((x,489),f"Rim height difference {row['lift']:.1f} mm",font=face(18),fill=INK)
        if t>=1.4 and 'head_positive_work_first_1_4s_j' in self.r:
            work=self.r['head_positive_work_first_1_4s_j']
            pen.text((x+306,489),f"First 1.4 s   /   {work:.2f} J head work",font=face(17),fill=DIM)
        # A plan view of the foot locates actual measured pressure without occlusion.
        cx,cy=x+548,368
        pen.rounded_rectangle((cx-43,cy-47,cx+43,cy+51),radius=8,fill='#10262f')
        pen.text((cx-27,cy-43),'FOOT',font=face(15,True),fill=DIM)
        pen.ellipse((cx-31,cy-23,cx+31,cy+39),outline=CYAN if row['edge'] else DIM,width=3)
        if row['cop']:
            u,v=row['cop'];px,py=cx-31*v/.056,cy+8-31*u/.056
            pen.ellipse((px-5,py-5,px+5,py+5),fill=GOLD)
        shell=self.r['shell']
        if shell and t>=shell['time_s']:
            status=f"Shell hit at {shell['time_s']:.2f} s"
            color='#ffaaa0'
        elif n==len(self.rows)-1:
            if self.r['hold']:status='Held the requested edge'
            elif self.r['flat']:status='Finished flat on the ramp'
            else:status='Edge hold missed'
            if self.r['joint']<0:status+='   /   joint warning'
            color=CYAN if self.r['hold'] else GOLD
        else:
            caught=self.r['trigger']
            status='Flat start' if t<.2 else 'Catch feedback' if caught is not None and t>=caught else 'Launch controller'
            color=DIM
        pen.text((x,531),status,font=face(20,True),fill=color)
        top,base=573,617
        pen.rectangle((x,top,x+606,base+35),fill='#1b3640')
        pen.line((x,base,x+606,base),fill='#687e85')
        def points(k):
            return [(x+605*i/(len(self.rows)-1),base-34*self.rows[i][k]/angle_limit) for i in range(n+1)]
        if n:
            pen.line(points('head'),fill=GOLD,width=2)
            pen.line(points('roll'),fill=CYAN,width=3)
        pen.text((x,663),'Body roll',font=face(17,True),fill=CYAN)
        pen.text((x+110,663),'Head roll',font=face(17,True),fill=GOLD)
        pen.text((x+222,663),f'±{angle_limit}°   /   entire episode',font=face(16),fill=DIM)


def film(chapters,output):
    lengths=[math.ceil(25*ch['seconds']/ch.get('speed',1.))+20 for ch in chapters]
    with tempfile.NamedTemporaryFile('w',suffix='.ffmeta') as meta:
        meta.write(';FFMETADATA1\n');frame=0
        for ch,length in zip(chapters,lengths):
            meta.write(f"[CHAPTER]\nTIMEBASE=1/25\nSTART={frame}\nEND={frame+length}\ntitle={ch['title']}\n")
            frame+=length
        meta.flush()
        cmd=['ffmpeg','-v','error','-y','-f','rawvideo','-pix_fmt','rgb24','-s','1280x710','-r','25','-i','-',
             '-f','ffmetadata','-i',meta.name,'-map','0:v:0','-map_chapters','1','-c:v','libx264','-threads','3',
             '-preset','veryfast','-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',str(output)]
        with subprocess.Popen(cmd,stdin=subprocess.PIPE) as enc:
            for ch,length in zip(chapters,lengths):
                views=[View(name) for name,label in ch['panels']]
                try:
                    peak=max(abs(row[k]) for v in views for row in v.rows for k in ('roll','head'))
                    angle_limit=max(60,30*math.ceil(peak/30))
                    for i in range(length):
                        t=min(ch['seconds'],i*ch.get('speed',1.)/25)
                        img=Image.new('RGB',(1280,710),'#10262f');pen=ImageDraw.Draw(img)
                        pen.text((22,8),f"REACHY MINI   /   CAD FOOT   /   MUJOCO   /   {ch.get('speed',1):g}× SPEED   /   {t:.2f} s",font=face(15,True),fill=CYAN)
                        pen.text((22,31),ch['title'],font=face(30,True),fill=INK)
                        for j,view in enumerate(views):
                            view.draw(img,22+j*628,t,ch['panels'][j][1],angle_limit)
                        enc.stdin.write(img.tobytes())
                finally:
                    for v in views:v.renderer.close()
            enc.stdin.close()
            if enc.wait():raise RuntimeError('Film encoding failed')


if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('manifest');ap.add_argument('output')
    a=ap.parse_args();film(read(a.manifest),a.output)
