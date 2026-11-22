"""Replay matched MuJoCo states with solver contact points and measured body roll rates."""
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
from train_release_catch import WORK, RESERVED, read
from tilted_release import UPSTREAM

RED='#ff9b95'
W,H=1440,690
P=450


class Panel:
    def __init__(self,spec):
        self.label,self.file=spec
        self.run=read(WORK/(self.file+'.json'))
        self.result=self.run['result']
        root=ET.fromstring(scene_xml(UPSTREAM,self.run['config'])[0])
        world=root.find('worldbody')
        ET.SubElement(world,'light',pos='1 -1 2',dir='-1 1 -2',directional='true',
                      diffuse='.82 .82 .82',ambient='.32 .32 .32')
        for k in range(-7,8):
            a=.05*k
            for coords in [(a,-.35,a,.35),(-.35,a,.35,a)]:
                x,y,u,v=coords
                ET.SubElement(world,'geom',type='capsule',size='.0004',fromto=f'{x} {y} .0004 {u} {v} .0004',
                              rgba='.45 .55 .58 1',contype='0',conaffinity='0')
        self.model=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'))
        self.data=mujoco.MjData(self.model)
        self.cam=mujoco.MjvCamera()
        self.cam.azimuth,self.cam.elevation,self.cam.distance=162,-12,.64
        self.cam.lookat[:]=[0,-self.result['side']*.027,.116]
        self.renderer=mujoco.Renderer(self.model,height=292,width=P)

    def draw(self,img,x,t,begin,end):
        pen=ImageDraw.Draw(img)
        trace=self.run['trace']
        n=min(round(t*50),len(trace)-1)
        row=trace[n]
        actual=self.run['trajectory'][n]
        self.data.qpos[:]=actual['qpos']
        mujoco.mj_forward(self.model,self.data)
        self.renderer.update_scene(self.data,camera=self.cam)
        contact=row['contact_world_m']
        if contact:
            sc=self.renderer.scene
            mujoco.mjv_initGeom(sc.geoms[sc.ngeom],mujoco.mjtGeom.mjGEOM_SPHERE,
                np.array([.004]*3),np.array(contact)+[0,0,.002],np.eye(3).ravel(),
                np.array([1.,.59,.16,1.],dtype=np.float32))
            sc.ngeom+=1
        img.paste(Image.fromarray(self.renderer.render()),(x,121))
        # Top view stays visible even when the shell occludes the 3D pressure marker.
        cx,cy=x+P-55,354
        pen.rounded_rectangle((cx-45,cy-50,cx+45,cy+54),radius=8,fill='#10262f')
        pen.text((cx-26,cy-47),'FOOT',font=face(14,True),fill=DIM)
        pen.ellipse((cx-32,cy-22,cx+32,cy+42),outline='#8ca5ad',width=2)
        pen.line((cx,cy-22,cx,cy-13),fill=INK,width=2)
        local=row['contact_foot_xy_m']
        if local:
            dot_x,dot_y=cx-32*local[1]/.056,cy+10-32*local[0]/.056
            pen.ellipse((dot_x-5,dot_y-5,dot_x+5,dot_y+5),fill=GOLD)
        side=self.result['side']
        pen.text((x,86),self.label,font=face(24,True),fill=INK)
        angle=side*row['roll_deg']
        rate=side*row['roll_rate_deg_s'] if n else self.result['outward_rate']
        pen.text((x,425),f'Roll {angle:5.1f}°     Rate {rate:+6.1f}°/s',font=face(21,True),fill=CYAN)
        info=(f'Foot contact    x {1000*local[0]:+.1f}     y {1000*local[1]:+.1f} mm'
              if local else 'Foot contact    no measured load')
        pen.text((x,461),info,font=face(18),fill=GOLD)
        a,b=round(begin*50),min(len(trace)-1,round(end*50))
        plot=(x,498,x+P-2,575)
        pen.rectangle(plot,fill='#193642')
        def xy(i,y):
            return x+(P-2)*(i-a)/max(1,b-a),537-min(40.,max(-40.,y))*.9
        pen.line((x,537,x+P-2,537),fill='#799097')
        points=[xy(i,side*trace[i]['roll_rate_deg_s'] if i else self.result['outward_rate']) for i in range(a,min(n,b)+1)]
        if len(points)>1:
            pen.line(points,fill=CYAN,width=2)
        pen.text((x,583),'Outward roll rate    −40 to +40°/s',font=face(16),fill=DIM)
        shell=self.result['shell']
        if shell and t>=shell['time_s']:
            note=f"Shell hit the floor at {shell['time_s']:.2f} s"
            color=RED
        elif n==len(trace)-1:
            note=('Passed the five-second hold' if self.result['strict'] else
                  {'flat':'Settled onto its foot','edge':'Edge hold outside strict limits'}.get(self.result['outcome'],'Hold incomplete'))
            if self.result['joint']<0:
                note+=' • joint warning'
            color=CYAN if self.result['strict'] else DIM
        else:
            note=f'Measured head roll    {np.degrees(actual["head_rpy_rad"][0]):+.1f}°'
            color=DIM
        pen.text((x,619),note,font=face(19,True),fill=color)


def scorecard():
    img=Image.new('RGB',(W,H),'#10262f')
    pen=ImageDraw.Draw(img)
    pen.text((38,25),'REACHY MINI    /    FROZEN CONTROLLER    /    SIMULATION',font=face(17,True),fill=CYAN)
    pen.text((38,63),'What happened on the ten fresh releases per side?',font=face(40,True),fill=INK)
    pen.text((38,126),'Each column is one eight-second release. Positive initial rates are outward.',font=face(24),fill=DIM)
    xs=[341+109*i for i in range(10)]
    for i,((a,v),x) in enumerate(zip(RESERVED,xs)):
        pen.text((x,209),f'{a:.2f}°',font=face(19,True),fill=INK)
        pen.text((x,240),f'{v:+.1f}°/s',font=face(16),fill=DIM)
    colors={'HOLD':CYAN,'EDGE':'#a0c6e7','FLAT':'#b2c2ad','SHELL':RED,'OTHER':GOLD}
    def outcome(row):
        if row['strict']:return 'HOLD'
        if row['edge']:return 'EDGE'
        if row['safe_flat']:return 'FLAT'
        if row['shell']:return 'SHELL'
        return 'OTHER'
    report=read(WORK/'qualification.json')['trials']
    for j,(side,controller) in enumerate(((1,'original'),(1,'trained'),(-1,'original'),(-1,'trained'))):
        rows=sorted((r for r in report if r['group']=='reserved' and r['side']==side and r['controller']==controller),
                    key=lambda r:r['index'])
        y=303+58*j
        label=('Right' if side==1 else 'Left')+'   '+controller
        pen.text((40,y),label,font=face(24,True),fill=INK)
        for x,row in zip(xs,rows):
            k=outcome(row)
            pen.rounded_rectangle((x-5,y-3,x+83,y+29),radius=5,fill=colors[k])
            pen.text((x+1,y+1),k,font=face(18,True),fill='#10262f')
    pen.text((38,550),'HOLD   Strict final five seconds          EDGE   On the edge at the end          FLAT   Settled safely upright',font=face(21),fill=INK)
    pen.text((38,593),'SHELL   Confirmed shell contact         OTHER   Incomplete hold or joint warning',font=face(21),fill=DIM)
    pen.text((38,649),'Fixed waist target    /    5 V motor envelope    /    CAD shell floor contacts',font=face(17),fill=CYAN)
    return img


def film(chapters,output):
    has_scorecard=any(ch['panels'][0][1].startswith('prior-right-') for ch in chapters)
    lengths=[math.ceil((ch['stop']-ch['start'])*25/ch['speed'])+20 for ch in chapters]
    titles=[ch['title'] for ch in chapters]
    if has_scorecard:
        lengths.append(125)
        titles.append('Fresh release results')
    with tempfile.NamedTemporaryFile('w',suffix='.ffmeta') as meta:
        meta.write(';FFMETADATA1\n')
        frame=0
        for length,title in zip(lengths,titles):
            meta.write(f'[CHAPTER]\nTIMEBASE=1/25\nSTART={frame}\nEND={frame+length}\ntitle={title}\n')
            frame+=length
        meta.flush()
        encode(chapters,output,meta.name,has_scorecard)


def encode(chapters,output,metadata,has_scorecard):
    with subprocess.Popen(['ffmpeg','-v','error','-y','-f','rawvideo','-pix_fmt','rgb24','-s',f'{W}x{H}',
        '-r','25','-i','-','-f','ffmetadata','-i',metadata,'-map','0:v:0','-map_chapters','1',
        '-an','-c:v','libx264','-threads','3','-preset','veryfast','-crf','20',
        '-pix_fmt','yuv420p','-movflags','+faststart',str(output)],stdin=subprocess.PIPE) as enc:
        for ch in chapters:
            views=[Panel(v) for v in ch['panels']]
            try:
                start,stop,speed=ch['start'],ch['stop'],ch['speed']
                frames=math.ceil((stop-start)*25/speed)
                for i in range(frames+20):
                    t=min(stop,start+i*speed/25)
                    img=Image.new('RGB',(W,H),'#10262f')
                    pen=ImageDraw.Draw(img)
                    pen.text((23,10),f'REACHY MINI    /    CAD FOOT    /    MUJOCO    /    {speed:g}× SPEED    /    {t:.2f} s',font=face(16,True),fill=CYAN)
                    pen.text((23,35),ch['title'],font=face(34,True),fill=INK)
                    for j,view in enumerate(views):view.draw(img,23+474*j,t,start,stop)
                    pen.text((23,660),'Orange dot shows the foot’s center of pressure from above.   Positive roll rate means moving away from upright.',font=face(15),fill=DIM)
                    enc.stdin.write(img.tobytes())
            finally:
                for view in views:view.renderer.close()
        if has_scorecard:
            card=scorecard().tobytes()
            for _ in range(125):
                enc.stdin.write(card)
        enc.stdin.close()
        if enc.wait(): raise RuntimeError('Encoding failed')


if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('manifest');ap.add_argument('output')
    a=ap.parse_args()
    film(read(a.manifest),a.output)
