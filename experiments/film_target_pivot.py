"""Film saved left and right target turns using measured torso and rim motion."""
import argparse
import math
import subprocess
import xml.etree.ElementTree as ET

import mujoco
from PIL import Image, ImageDraw

from film_balance_qualification import font
from reachy_locomotion.contact_probe import scene_xml
from train_target_pivot import read


def ray(pen, center, length, deg, color, width):
    radians = math.radians(deg)
    dx, dy = -math.sin(radians), -math.cos(radians)
    x, y = center
    tip = x + length * dx, y + length * dy
    pen.line([center, tip], fill=color, width=width)
    pen.polygon([tip, (tip[0]-11*dx+5*dy, tip[1]-11*dy-5*dx),
                 (tip[0]-11*dx-5*dy, tip[1]-11*dy+5*dx)], fill=color)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('record', nargs='+')
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    style = {s:font(s, s >= 25) for s in (16, 19, 26, 32)}
    stream = subprocess.Popen(['ffmpeg','-v','error','-n','-f','rawvideo','-pix_fmt','rgb24',
                               '-s','960x540','-r','25','-i','-','-an','-c:v','libx264',
                               '-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',args.output],
                              stdin=subprocess.PIPE)
    for file in args.record:
        run = read(file)
        xml, _ = scene_xml('experiments/upstream-reachy-mini', run['config'])
        root = ET.fromstring(xml)
        ET.SubElement(root.find('worldbody'),'light',pos='1 -1 2',dir='-1 1 -2',directional='true',
                      diffuse='.75 .75 .75',ambient='.3 .3 .3')
        for n in range(-5, 6):
            pos = n * .05
            for ends in ((pos,-.25,pos,.25),(-.25,pos,.25,pos)):
                a,b,c,d = ends
                ET.SubElement(root.find('worldbody'),'geom',type='capsule',size='.00025',
                              fromto=f'{a} {b} .0003 {c} {d} .0003',rgba='.55 .65 .7 .8',
                              contype='0',conaffinity='0')
        model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
        data = mujoco.MjData(model)
        camera = mujoco.MjvCamera()
        camera.azimuth, camera.elevation, camera.distance = 195, -18, .79
        camera.lookat[:] = [0,0,.145]
        metrics = run['metrics']
        goal = metrics['target_deg']
        start = run['trace'][0]['foot_xyz_m']
        with mujoco.Renderer(model,height=408,width=600) as renderer:
            for row, state in zip(run['trace'][::2],run['trajectory'][::2]):
                data.qpos[:] = state['qpos']
                mujoco.mj_forward(model,data)
                renderer.update_scene(data,camera=camera)
                frame = Image.new('RGB',(960,540),'#13232d')
                frame.paste(Image.fromarray(renderer.render()),(0,78))
                pen = ImageDraw.Draw(frame)
                def label(x,y,s,size,color='#eff6f8'):
                    pen.text((x,y),s,font=style[size],fill=color)
                label(20,8,'Head-driven edge pivot',32)
                label(22,49,'Reachy Mini    /    MuJoCo simulation    /    real-time replay',16,'#c3d2d9')
                label(625,14,f"Turn {abs(goal):g}° {'left' if goal>0 else 'right'}",26)
                label(625,51,f"{metrics['condition']} condition   ·   fixed waist target",16,'#c3d2d9')
                center = (775,228)
                pen.ellipse((680,133,870,323),outline='#648394',width=2)
                pen.line([(775,125),(775,331)],fill='#617580',width=1)
                label(724,98,'Start heading',16)
                ray(pen,center,94,goal,'#ffbd69',3)
                ray(pen,center,74,row['torso_yaw_deg'],'#65e3de',6)
                label(630,345,f"Target   {goal:+.0f}°",19,'#ffbd69')
                label(630,375,f"Torso   {row['torso_yaw_deg']:+.1f}°",26,'#65e3de')
                label(630,417,f"Base   {row['foot_yaw_deg']:+.1f}°",19)
                xy = row['foot_xyz_m']
                drift = 1000*math.hypot(xy[0]-start[0],xy[1]-start[1])
                label(630,449,f'Center travel from start   {drift:.1f} mm',16)
                t = row['time_s']
                rim = ('Right rim loaded' if row['roll_deg']>0 else 'Left rim loaded') if row['edge_support'] else 'Between edge contacts'
                label(20,501,f'{t:5.2f} s     {rim}',19)
                label(510,501,f"Rim height difference   {row['side_height_difference_mm']:.1f} mm",19)
                stream.stdin.write(frame.tobytes())
            for _ in range(18):
                stream.stdin.write(frame.tobytes())
    stream.stdin.close()
    if stream.wait():
        raise RuntimeError('Video encoder failed')


if __name__ == '__main__':
    main()
