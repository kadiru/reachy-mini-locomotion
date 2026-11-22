"""Render two synchronized, contact-instrumented releases per chapter at real speed."""
import argparse
import subprocess
from functools import lru_cache
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from PIL import Image, ImageDraw

from film_balance_qualification import font
from reachy_locomotion.contact_probe import scene_xml
from tilted_release import WORK, UPSTREAM, read

INK, DIM, CYAN, GOLD = '#edf4f1','#b9cad0','#7fe4dc','#ffc178'


@lru_cache(None)
def face(size,bold=False):
    return font(size,bold)


class View:
    def __init__(self, panel, side_rest):
        self.run = read(WORK/(panel['clip']+'.json'))
        self.panel = panel
        rows = self.run['trajectory']
        self.angles = np.degrees(np.arccos(np.clip([np.cos(r['foot_rpy_rad'][0])*np.cos(r['foot_rpy_rad'][1]) for r in rows],-1,1)))
        root = ET.fromstring(scene_xml(UPSTREAM,self.run['config'])[0])
        world = root.find('worldbody')
        ET.SubElement(world,'light',pos='1 -1 2',dir='-1 1 -2',directional='true',diffuse='.78 .78 .78',ambient='.32 .32 .32')
        for s in np.arange(-.55,.551,.05):
            for pts in [(s,-.6,s,.6),(-.6,s,.6,s)]:
                x,y,u,v = pts
                ET.SubElement(world,'geom',type='capsule',size='.0004',fromto=f'{x} {y} .0002 {u} {v} .0002',
                              rgba='.47 .51 .53 1',contype='0',conaffinity='0')
        self.model = mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'))
        self.model.vis.global_.offwidth = 608
        self.data = mujoco.MjData(self.model)
        self.renderer = mujoco.Renderer(self.model,height=340,width=608)
        self.camera = mujoco.MjvCamera()
        self.camera.azimuth,self.camera.elevation,self.camera.distance = (160,-18,.66) if side_rest else (158,-14,.68)
        self.side_rest=side_rest
        self.camera.lookat[:] = [0, self.run['metrics']['job'].get('side',1)*(-.08 if side_rest else -.03), .10 if side_rest else .12]

    def draw(self,img,x,t):
        rows = self.run['trajectory']
        n = min(round(t*50),len(rows)-1)
        self.data.qpos[:] = rows[n]['qpos']
        mujoco.mj_forward(self.model,self.data)
        if self.side_rest:
            mid = .5*(self.data.xpos[self.model.body('body_foot_3dprint').id] +
                      self.data.site_xpos[self.model.site('head').id])
            self.camera.lookat[:2] = mid[:2]
        self.renderer.update_scene(self.data,camera=self.camera)
        img.paste(Image.fromarray(self.renderer.render()),(x,128))
        pen = ImageDraw.Draw(img)
        pen.text((x,79),self.panel['label'],font=face(25,True),fill=INK)
        pen.text((x,108),self.panel['setup'],font=face(16),fill=DIM)
        contacts = self.run['contacts']
        snap = contacts[n-1] if n and n<=len(contacts) else None
        support = ' + '.join(snap['support']) if snap else 'release'
        title = f'{t:.2f} s    Lean {self.angles[n]:.1f}°    Floor: {support or "clear"}'
        pen.text((x,480),title,font=face(19,True),fill=GOLD if 'head' in support else CYAN)
        if self.side_rest:
            spin = snap['spin'] if snap else 0.
            pen.text((x,514),f'Torso rotation about its long axis   {spin:+.1f}°',font=face(18),fill=INK)
            values = np.r_[0.,[s['spin'] for s in contacts]]
            span = self.panel.get('span',70)
            low,high = -span,span
        else:
            pen.text((x,514),'Lean from vertical',font=face(18),fill=INK)
            values=self.angles
            low,high=0.,120.
        top,bot=546,602
        xy=lambda i,v:(x+round(605*i/max(1,len(values)-1)),bot-round((np.clip(v,low,high)-low)/(high-low)*(bot-top)))
        pen.rectangle((x,top,x+607,bot),fill='#192f3b')
        guide=0 if self.side_rest else 32.1
        yy=xy(0,guide)[1]
        pen.line((x,yy,x+606,yy),fill='#637783')
        if n:
            pen.line([xy(i,values[i]) for i in range(min(n+1,len(values)))],fill=CYAN,width=2)
        pen.text((x,619),self.panel['note'],font=face(18,True),fill=INK)
        pen.text((x,649),self.panel['detail'],font=face(16),fill=DIM)


def film(manifest,out):
    chapters=read(manifest)
    with subprocess.Popen(['ffmpeg','-v','error','-y','-f','rawvideo','-pix_fmt','rgb24','-s','1280x704',
            '-r','25','-i','-','-an','-c:v','libx264','-threads','3','-preset','veryfast','-crf','20',
            '-pix_fmt','yuv420p','-movflags','+faststart',str(out)],stdin=subprocess.PIPE) as enc:
        for k,ch in enumerate(chapters):
            views = [View(p,ch.get('side_rest',False)) for p in ch['panels']]
            try:
                frames = round((ch['end']-ch['begin'])*25)+1
                for i in range(frames+20):
                    t = ch['begin']+min(i,frames-1)/25
                    img = Image.new('RGB',(1280,704),'#10242c')
                    pen=ImageDraw.Draw(img)
                    pen.text((22,10),f'REACHY MINI   /   MUJOCO   /   {k+1} OF {len(chapters)}   /   REAL TIME',font=face(15,True),fill=CYAN)
                    pen.text((22,31),ch['title'],font=face(33,True),fill=INK)
                    for j,view in enumerate(views):
                        view.draw(img,22+628*j,t)
                    enc.stdin.write(img.tobytes())
            finally:
                for view in views:
                    view.renderer.close()
        enc.stdin.close()
        if enc.wait():
            raise RuntimeError('Video encoding failed')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('manifest',type=Path)
    p.add_argument('output',type=Path)
    args=p.parse_args()
    film(args.manifest,args.output)
