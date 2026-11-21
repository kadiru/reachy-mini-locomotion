"""Film saved left and right target turns using measured torso and rim motion."""
import argparse
import math
import subprocess
import xml.etree.ElementTree as ET

import mujoco
from PIL import Image, ImageDraw

from film_balance_qualification import font
from reachy_locomotion.contact_probe import BASE, scene_xml
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
    parser.add_argument('--slow-edge',action='store_true',help='Replay the strongest continuous rim contact at half speed')
    parser.add_argument('--first-cycle',action='store_true',help='Show launch and the first full active-waist sweep')
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
        spin = run['metrics'].get('mode') == 'continuous_spin'
        waist_sweep = run['metrics'].get('mode') == 'waist_balance'
        if spin:
            # Render-only base index mark; the trajectory was integrated without it.
            body=root.find(f".//worldbody/body[@name='{BASE}']")
            ET.SubElement(body,'site',type='capsule',fromto='.057 0 .005 .057 0 .029',
                          size='.003',rgba='.21 .94 .87 1',group='0')
        model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
        data = mujoco.MjData(model)
        camera = mujoco.MjvCamera()
        camera.azimuth, camera.elevation, camera.distance = 195, -18, .79
        camera.lookat[:] = [0,0,.145]
        metrics = run['metrics']
        goal = metrics.get('target_deg',0.)
        start = run['trace'][0]['foot_xyz_m']
        with mujoco.Renderer(model,height=408,width=600) as renderer:
            rows = run['trace']
            frames = [(j,False) for j in range(0,len(rows),2)]
            if args.first_cycle:
                if not waist_sweep or metrics['start_s'] is None:
                    raise ValueError('A full-cycle replay requires an active-waist capture')
                stop = metrics['start_s'] + metrics['period_s']
                frames = [(j,slow) for j,slow in frames if rows[j]['time_s'] <= stop]
            if args.slow_edge:
                best_gain, best_window, first = 0., None, 0
                for j in range(1,len(rows)):
                    a,b = rows[j-1:j+1]
                    if a['edge_support'] and b['edge_support'] and a['roll_deg']*b['roll_deg']>0:
                        gain = math.copysign(1.,goal)*(b['torso_yaw_deg']-rows[first]['torso_yaw_deg'])
                        if gain>best_gain:
                            best_gain,best_window = gain,(first,j)
                    else:
                        first=j
                if best_window:
                    frames += [(len(rows)-1,False)]*15
                    frames += [(j,True) for j in range(max(0,best_window[0]-15),min(len(rows),best_window[1]+16))]
                    frames += [(len(rows)-1,False)]*12
            for j,slow in frames:
                row, state = rows[j],run['trajectory'][j]
                camera.distance = .64 if spin or waist_sweep else (.69 if slow else .79)
                data.qpos[:] = state['qpos']
                mujoco.mj_forward(model,data)
                renderer.update_scene(data,camera=camera)
                frame = Image.new('RGB',(960,540),'#13232d')
                frame.paste(Image.fromarray(renderer.render()),(0,78))
                pen = ImageDraw.Draw(frame)
                def label(x,y,s,size,color='#eff6f8'):
                    pen.text((x,y),s,font=style[size],fill=color)
                label(20,8,run.get('title','Head-driven edge pivot'),32)
                playback = 'strongest rim contact    /    0.5×' if slow else 'real-time replay'
                if args.first_cycle:
                    playback = 'first sweep cycle   /   real time'
                label(22,49,f'Reachy Mini    /    MuJoCo simulation    /    {playback}',16,'#c3d2d9')
                if waist_sweep:
                    # Foot-fixed inset. The dot comes from the measured center of pressure.
                    pen.rounded_rectangle((15,90,147,209),radius=8,fill='#13232d')
                    label(29,96,'Foot contact',16)
                    cx,cy,radius=81,161,33
                    box=(cx-radius,cy-radius,cx+radius,cy+radius)
                    pen.ellipse(box,outline='#829ba6',width=2)
                    arc=180 if metrics['side']<0 else 0
                    pen.arc(box,arc-15,arc+15,fill='#ffbd69',width=5)
                    point=row['contact_foot_xy_m']
                    if point is not None:
                        scale=radius/run['config']['foot_radius_m']
                        x,y=cx-point[1]*scale,cy-point[0]*scale
                        pen.ellipse((x-4,y-4,x+4,y+4),fill='#65e3de')
                    label(625,14,('Left' if metrics['side'] < 0 else 'Right') + ' foot edge',26)
                elif spin:
                    label(625,14,'Spin '+('left' if metrics['direction']==1 else 'right'),26)
                else:
                    label(625,14,f"Turn {abs(goal):g}° {'left' if goal>0 else 'right'}",26)
                condition_label="Active waist   ·   head balance feedback" if waist_sweep else f"{metrics['condition']} condition   ·   fixed waist target"
                if spin or 'head_swings' in row:
                    condition_label=f"{run['config']['motor_model']['voltage_v']:.2f} V   ·   friction {run['config']['friction_slide_torsion_roll'][0]:.3g}   ·   fixed waist"
                label(625,51,condition_label,16,'#c3d2d9')
                center = (775,228)
                pen.ellipse((680,133,870,323),outline='#648394',width=2)
                pen.line([(775,125),(775,331)],fill='#617580',width=1)
                label(724,98,'Start heading',16)
                ray(pen,center,94,goal,'#ffbd69',3)
                ray(pen,center,74,row['torso_yaw_deg'],'#65e3de',6)
                if waist_sweep:
                    label(630,345,f"Waist encoder   {row['waist_angle_change_deg']:+.1f}°",26,'#65e3de')
                    label(630,388,f"Torso heading   {row['torso_yaw_deg']:+.1f}°",19)
                    label(630,417,f"Foot heading   {row['foot_yaw_deg']:+.1f}°",19)
                elif spin:
                    prev=rows[max(0,j-25)]
                    dt=row['time_s']-prev['time_s']
                    rpm=(row['foot_yaw_deg']-prev['foot_yaw_deg'])/(6*dt) if dt else 0.
                    label(630,339,f"Base turns   {abs(row['foot_yaw_deg'])/360:.2f}",26,'#65e3de')
                    label(630,375,f"Base speed   {rpm*metrics['direction']:.1f} rpm",19)
                    label(630,412,f"Torso   {row['torso_yaw_deg']:+.0f}°",19)
                else:
                    label(630,345,f"Target   {goal:+.0f}°",19,'#ffbd69')
                    label(630,375,f"Torso   {row['torso_yaw_deg']:+.1f}°",26,'#65e3de')
                    label(630,417,f"Base   {row['foot_yaw_deg']:+.1f}°",19)
                xy = row['foot_xyz_m']
                drift = 1000*math.hypot(xy[0]-start[0],xy[1]-start[1])
                label(630,449,f'Center travel from start   {drift:.1f} mm',16)
                if 'head_swings' in row:
                    label(630,473,f"Torso turn on rim   {row['torso_edge_deg']:+.1f}°",16,'#65e3de')
                    label(20,86,f"Stroke {row['stroke_number']} of {metrics['strokes']}   ·   measured swings {row['head_swings']}",19)

                t = row['time_s']
                rim = ('Right rim loaded' if row['roll_deg']>0 else 'Left rim loaded') if row['edge_support'] else 'Between edge contacts'
                if spin or waist_sweep:
                    if row['foreaft_edge_support'] and not row['edge_support']:
                        back,front=row['foot_front_back_normal_forces_n_back_front']
                        rim='Front rim loaded' if front>back else 'Back rim loaded'
                    elif not row['edge_support']:
                        rim='Between confirmed rim loads'
                    label(510,501,f"Body tilt   {row['tilt_deg']:.1f}°",19)
                else:
                    label(510,501,f"Rim height difference   {row['side_height_difference_mm']:.1f} mm",19)
                label(20,501,f'{t:5.2f} s     {rim}',19)
                stream.stdin.write(frame.tobytes())
            for _ in range(18):
                stream.stdin.write(frame.tobytes())
    stream.stdin.close()
    if stream.wait():
        raise RuntimeError('Video encoder failed')


if __name__ == '__main__':
    main()
