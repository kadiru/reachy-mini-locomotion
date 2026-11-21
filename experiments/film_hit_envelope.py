"""Build a chaptered comparison film from recorded states listed in a manifest.

A chapter contains one to four trials, played on a shared simulated clock.
Every panel freezes on its true terminal substep. Empty slots are untested.
"""
import argparse
import json
import math
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from PIL import Image, ImageDraw
from film_balance_qualification import font, INK, MUTED, ACCENT, BACKGROUND
from reachy_locomotion.contact_probe import scene_xml

FPS = 25
FONTS = {}
def face(size, bold=False):
    if (size,bold) not in FONTS:
        FONTS[size,bold] = font(size,bold)
    return FONTS[size,bold]


class Panel:
    def __init__(self, item):
        self.item = item
        self.run = json.loads(Path(item['record']).read_text())
        self.start = item.get('start_s',0.)
        self.stop = min(item.get('stop_s',self.run['elapsed_seconds']),self.run['elapsed_seconds'])
        self.states = self.run['collision_frames'] if item.get('dense') else self.run['trajectory']
        self.times = np.array([row['time_s'] for row in self.states])
        self.end = self.stop-self.start
        if self.end<=0 or not len(self.states):raise ValueError('Empty trial excerpt')
        xml,_ = scene_xml('experiments/upstream-reachy-mini', self.run['config'])
        root = ET.fromstring(xml)
        ET.SubElement(root.find('worldbody'), 'light', pos='1 -1 2', dir='-1 1 -2',
                      directional='true', diffuse='.75 .75 .75', ambient='.35 .35 .35')
        self.model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
        self.data = mujoco.MjData(self.model)
        self.camera = mujoco.MjvCamera()
        self.camera.lookat[:] = [0,0,.14]
        self.camera.azimuth, self.camera.elevation, self.camera.distance = 185,-12,.73
        self.renderer = mujoco.Renderer(self.model,height=206,width=488)
        self.last = None

    def image(self,t):
        if self.last is not None:
            return self.last
        final = t >= self.end-1e-8
        t = min(self.start+t,self.stop)
        terminal = final and self.stop == self.run['elapsed_seconds']
        index = max(0,np.searchsorted(self.times,t+1e-8,side='right')-1)
        row = self.run['terminal_state'] if terminal else self.states[index]
        self.data.qpos[:] = row['qpos']
        mujoco.mj_forward(self.model,self.data)
        self.renderer.update_scene(self.data,camera=self.camera)
        panel = Image.new('RGB',(488,292),BACKGROUND)
        panel.paste(Image.fromarray(self.renderer.render()),(0,33))
        pen = ImageDraw.Draw(panel)
        pen.text((8,3),self.item['label'],font=face(20,True),fill=INK)
        roll,pitch = row['foot_rpy_rad'][:2]
        tilt = math.degrees(math.acos(np.clip(math.cos(roll)*math.cos(pitch),-1,1)))
        stamp = f'{t:.3f} s  |  {tilt:.1f}°'
        ball = self.run.get('ball_disturbances',[{}])[0]
        hit = ball.get('first_robot_contact_s')
        if final:
            stamp += '  |  final pose' if terminal else '  |  excerpt ends'
        elif self.item.get('trigger') is not None and 0 <= t-self.item['trigger'] < .3:
            stamp += '  |  CATCH'
        elif hit is not None and 0 <= t-hit < .36:
            stamp += '  |  HIT'
        pen.rectangle((4,35,260,62),fill=BACKGROUND)
        pen.text((8,38),stamp,font=face(17),fill=ACCENT)
        pen.text((8,242),self.item['outcome'],font=face(18,True),fill=self.item['color'])
        pen.text((8,266),self.item['detail'],font=face(16),fill=MUTED)
        if final:
            self.last = panel
        return panel


def film(chapters, output, work, speed, fps=FPS):
    work.mkdir(parents=True,exist_ok=True)
    output.parent.mkdir(exist_ok=True,parents=True)
    if output.exists():
        raise FileExistsError(output)
    playlist, metadata, total = [], [';FFMETADATA1'], 0
    height = 404 if all(len(c['panels'])<=2 for c in chapters) else 680
    for i,chapter in enumerate(chapters,1):
        panels = [Panel(item) if item else None for item in chapter['panels']]
        duration = max(panel.end for panel in panels if panel)
        playback = chapter.get('playback',speed)
        frames = math.ceil(duration*fps/playback)+fps
        clip = work/f'chapter-{i:03d}.mp4'
        locations = [(8,85),(504,85),(8,382),(504,382)] if len(panels)>2 else [(8,(height-292+68)//2),(504,(height-292+68)//2)]
        try:
            with subprocess.Popen(['ffmpeg','-v','error','-n','-f','rawvideo','-pix_fmt','rgb24',
                                   '-s',f'1000x{height}','-r',str(fps),'-i','-','-an','-c:v','libx264',
                                   '-preset','veryfast','-crf','21','-pix_fmt','yuv420p','-threads','2',
                                   str(clip)],stdin=subprocess.PIPE) as enc:
                base = Image.new('RGB',(1000,height),'#0c1720')
                draw = ImageDraw.Draw(base)
                draw.text((16,6),f'REACHY MINI SIMULATION    /    {i:02d} OF {len(chapters):02d}    /    {playback:g}×',font=face(16,True),fill=ACCENT)
                draw.text((16,29),chapter['title'],font=face(26,True),fill=INK)
                draw.text((16,60),chapter['subtitle'],font=face(16),fill=MUTED)
                for frame in range(frames):
                    canvas=base.copy()
                    for panel,location in zip(panels,locations):
                        if panel:
                            canvas.paste(panel.image(frame*playback/fps),location)
                        else:
                            ImageDraw.Draw(canvas).text((location[0]+16,location[1]+112),
                                'No trial at this setting',font=face(18),fill=MUTED)
                    enc.stdin.write(canvas.tobytes())
                enc.stdin.close()
                if enc.wait():
                    raise RuntimeError(str(clip))
        finally:
            for panel in panels:
                if panel:
                    panel.renderer.close()
        playlist.append(f"file '{clip.name}'")
        metadata.extend(['[CHAPTER]',f'TIMEBASE=1/{fps}',f'START={total}',f'END={total+frames}',f'title={chapter["title"]}'])
        total += frames
        print(i,chapter['title'],flush=True)
    listing,meta=work/'clips.txt',work/'chapters.ffmeta'
    listing.write_text('\n'.join(playlist)+'\n')
    meta.write_text('\n'.join(metadata)+'\n')
    subprocess.run(['ffmpeg','-v','error','-n','-f','concat','-safe','0','-i',str(listing),
                    '-f','ffmetadata','-i',str(meta),'-map','0:v:0','-map_chapters','1',
                    '-c:v','copy','-movflags','+faststart',str(output)],check=True)
    return total


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('manifest',type=Path)
    parser.add_argument('output',type=Path)
    parser.add_argument('--work',type=Path,required=True)
    parser.add_argument('--speed',type=float,choices=[.5,1.,2.],default=2.)
    parser.add_argument('--fps',type=int,choices=[12,25],default=FPS)
    args=parser.parse_args()
    print(film(json.loads(args.manifest.read_text()), args.output, args.work, args.speed, args.fps))
