"""Render saved MuJoCo states; never rerun or replace recorded physics."""
import argparse
import json
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from reachy_locomotion.contact_probe import scene_xml, BASE


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("report")
    parser.add_argument("--output", required=True)
    parser.add_argument("--upstream", default="experiments/upstream-reachy-mini")
    parser.add_argument("--slowdown", type=float, default=2)
    parser.add_argument("--title", default="HEAD-DRIVEN SWAY")
    parser.add_argument("--diagnostics", action="store_true", help="Show recorded support and yaw deceleration")
    parser.add_argument("--frame-path", action="store_true", help="Use fixed cameras framing the entire saved path")
    parser.add_argument("--follow-front", action="store_true", help="Follow the base in the front close-up; keep the side camera fixed")
    parser.add_argument("--until",type=float,help="End the video at this recorded time; physics report remains complete")
    args = parser.parse_args()
    output = Path(args.output)
    if output.exists():
        parser.error("Choose a new output file")
    result = json.loads(Path(args.report).read_text())
    primitive_trial = result.get('primitive_trial',False)
    xml, _ = scene_xml(args.upstream, result["config"])
    root = ET.fromstring(xml)
    world = root.find("worldbody")
    ET.SubElement(world, "light", pos="1 -1 2", dir="-1 1 -2", directional="true",
                  diffuse="0.7 0.7 0.7", ambient="0.3 0.3 0.3")
    ET.SubElement(world, "light", pos="-1 1 1", dir="1 -1 -1", directional="true",
                  diffuse="0.5 0.5 0.5")
    positions = np.array([row['foot_xyz_m'] for row in result['trajectory']])
    lo = np.floor(np.minimum([-.3, -.3], positions[:, :2].min(axis=0)-.15)/.05)*.05
    hi = np.ceil(np.maximum([.5, .5], positions[:, :2].max(axis=0)+.15)/.05)*.05
    lines = [([x, lo[1], .0005], [x, hi[1], .0005]) for x in np.arange(lo[0], hi[0]+.001, .05)]
    lines += [([lo[0], y, .0005], [hi[0], y, .0005]) for y in np.arange(lo[1], hi[1]+.001, .05)]
    for a, b in lines:
        ET.SubElement(world, "geom", type="capsule", size=".00025", fromto=" ".join(map(str,a+b)),
                      rgba="0.4 0.5 0.6 1", contype="0", conaffinity="0")
    ET.SubElement(world, "geom", type="capsule", size=".001", fromto="0 0 .001 .15 0 .001",
                  rgba="0.25 0.85 0.45 1", contype="0", conaffinity="0")
    for y in [-.012, .012]:
        ET.SubElement(world, "geom", type="capsule", size=".001", fromto=f".15 0 .001 .13 {y} .001",
                      rgba="0.25 0.85 0.45 1", contype="0", conaffinity="0")
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    data = mujoco.MjData(model)
    base = model.body(BASE).id
    midpoint = (positions.min(axis=0)+positions.max(axis=0))/2
    path_span = float(np.max(np.ptp(positions[:,:2],axis=0)))
    cameras = []
    for azimuth in (190, 90):
        cam = mujoco.MjvCamera()
        cam.lookat[:] = [.06, 0, .13]
        cam.distance = .62
        if args.frame_path:
            cam.lookat[:] = [midpoint[0],midpoint[1],.13]
            cam.distance = .65+.6*path_span
        if primitive_trial:
            cam.lookat[2] = .16
            cam.distance = .78+.6*path_span
        cam.azimuth = azimuth
        cam.elevation = -15
        cameras.append(cam)
    font_path = "/System/Library/Fonts/Supplemental/Arial.ttf"
    try:
        font = ImageFont.truetype(font_path, 19)
        big = ImageFont.truetype(font_path, 24)
    except OSError:
        font = big = ImageFont.load_default()
    renderer = mujoco.Renderer(model, height=480, width=640)
    output.parent.mkdir(parents=True, exist_ok=True)
    fps = 25 / args.slowdown
    process = subprocess.Popen(["ffmpeg", "-hide_banner", "-loglevel", "error", "-n", "-f", "rawvideo",
                                "-pix_fmt", "rgb24", "-s", "1280x580", "-r", str(fps), "-i", "-",
                                "-an", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20",
                                "-movflags", "+faststart", str(output)], stdin=subprocess.PIPE)
    start = None
    start_rotation = None
    images = []
    previous_head_yaw = previous_head_rate = None
    initial_heading = None
    last_cycle_heading = 0.
    completed_cycle_rule = result.get('evaluation_heading_rule') == 'completed_cycle'
    cycle_frames = round(50/result['actual_frequency_hz']) if completed_cycle_rule else 1
    edge_holds=[];hold=0.;previous_ok=False
    if 'edge_balance' in result:
        side=result['edge_balance']['side'];target=result['edge_balance']['policy']['feedback']['target_roll_deg']
        for sample in result['trace']:
            ok=sample['edge_support'] and abs(side*sample['roll_deg']-target)<=2 and abs(sample['roll_rate_deg_s'])<=5 and abs(sample['pitch_deg'])<=5
            hold=hold+.02 if ok and previous_ok else 0.;edge_holds.append(hold);previous_ok=ok
    if 'foreaft_balance' in result:
        side=result['foreaft_balance']['side'];target=result['foreaft_balance']['policy']['feedback']['target_roll_deg']
        for sample in result['trace']:
            forces=sample['foot_front_back_normal_forces_n_back_front'];loaded=1 if side==1 else 0
            ok=sample['foreaft_edge_support'] and forces[loaded]>.5 and forces[1-loaded]<.1 and abs(side*sample['pitch_deg']-target)<=2 and abs(sample['pitch_rate_deg_s'])<=5 and abs(sample['roll_deg'])<=5
            hold=hold+.02 if ok and previous_ok else 0.;edge_holds.append(hold);previous_ok=ok
    try:
        for frame_index, row in enumerate(result["trajectory"]):
            if args.until is not None and row['time_s']>args.until+1e-8:break
            if completed_cycle_rule and frame_index and frame_index % cycle_frames == 0:
                last_cycle_heading = np.degrees(row['heading_from_startup_rad'])
            data.qpos[:] = row["qpos"]
            mujoco.mj_forward(model, data)
            if row["measuring"] and start is None:
                start = data.xpos[base].copy()
                yaw = result['trajectory'][0]['foot_rpy_rad'][2] if result.get('render_forward_frame') == 'startup' else row["foot_rpy_rad"][2]
                start_rotation = np.array([np.cos(yaw), np.sin(yaw), 0])
                initial_heading = yaw
            head_yaw = row["head_rpy_rad"][2]
            head_rate = 0 if previous_head_yaw is None else ((head_yaw-previous_head_yaw+np.pi)%(2*np.pi)-np.pi)/.02
            slowing = previous_head_rate is not None and abs(head_rate)>.3 and abs(head_rate)<abs(previous_head_rate)-.02
            previous_head_yaw, previous_head_rate = head_yaw, head_rate
            if frame_index % 2 and not (primitive_trial and frame_index==len(result['trajectory'])-1):
                continue
            canvas = Image.new("RGB", (1280, 580), (15, 22, 32))
            for index, camera in enumerate(cameras):
                if index == 0 and args.follow_front:
                    camera.lookat[:] = [data.xpos[base, 0], data.xpos[base, 1], .16 if primitive_trial else .13]
                    camera.distance = .75 if primitive_trial else .62
                renderer.update_scene(data, camera=camera)
                canvas.paste(Image.fromarray(renderer.render()), (index*640, 52))
            draw = ImageDraw.Draw(canvas)
            draw.text((20, 12), args.title + "  |  UNCALIBRATED SIMULATION", font=big, fill="white")
            draw.text((1030, 16), f"{1/args.slowdown:g}x playback", font=font, fill=(110, 210, 235))
            draw.text((20, 65), "Head close-up (following)" if args.follow_front else "Front view", font=font, fill="white")
            draw.text((660, 65), "Side view  |  green arrow = forward (+X)", font=font, fill="white")
            distance = float((data.xpos[base]-start) @ start_rotation)*1000 if start is not None else 0
            tilt = np.degrees(np.arccos(np.clip(data.xmat[base].reshape(3,3)[2,2],-1,1)))
            label = f"t = {row['time_s']:.2f} s   |   forward = {distance:+.1f} mm   |   base tilt = {tilt:.1f} deg"
            if args.diagnostics and start is not None:
                turn = np.degrees((row['foot_rpy_rad'][2]-initial_heading+np.pi)%(2*np.pi)-np.pi)
                label = f"t = {row['time_s']:.2f} s   |   forward = {distance:+.1f} mm   |   turn = {turn:+.1f} deg"
                if 'heading_feedback' in result:
                    turn = np.degrees(row['heading_from_startup_rad'])
                    label = f"t = {row['time_s']:.2f} s   |   forward = {distance:+.1f} mm   |   heading error = {turn:+.1f} deg"
                if completed_cycle_rule:
                    turn = np.degrees(row['heading_from_startup_rad'])
                    label = f"t = {row['time_s']:.2f} s   |   forward = {distance:+.1f} mm   |   torso yaw = {turn:+.1f} deg"
                if primitive_trial:
                    drift=1000*float(np.linalg.norm(data.xpos[base,:2]-start[:2]))
                    turn=np.degrees(row['heading_from_startup_rad'])
                    label=f"t = {row['time_s']:.2f} s   |   center shift = {drift:.1f} mm   |   base yaw = {turn:+.1f} deg"
                    if 'penguin' in result:
                        label=f"t = {row['time_s']:.2f} s   |   forward = {distance:+.1f} mm   |   base yaw = {turn:+.1f} deg"
            if not row["measuring"]:
                label = f"t = {row['time_s']:.2f} s   |   startup / warm-up (excluded from displacement)"
            draw.text((20, 544), label, font=font, fill="white")
            draw.text((870, 544), "5 cm grid  |  no external pushes", font=font, fill=(110,210,235))
            if args.diagnostics:
                forces = row.get('foot_side_normal_forces_n_right_left') or [0,0]
                support = 'BOTH SIDES'
                if forces[0] < .1 and forces[1] > .1:
                    support = 'LEFT EDGE'
                elif forces[1] < .1 and forces[0] > .1:
                    support = 'RIGHT EDGE'
                elif sum(forces) < .1:
                    support = 'UNLOADED'
                if 'foreaft_balance' in result:
                    forces=row['foot_front_back_normal_forces_n_back_front'];support='BOTH ENDS'
                    if forces[0]<.1 and forces[1]>.5:support='FRONT EDGE'
                    elif forces[1]<.1 and forces[0]>.5:support='BACK EDGE'
                    elif sum(forces)<.1:support='UNLOADED'
                if row.get('terminal_event'):
                    support='BODY CONTACT' if row['terminal_event']=='nonfoot_floor_contact' else 'TERMINATED'
                draw.rectangle((12,92,335,148), fill=(15,22,32))
                draw.text((20,96), 'Support: '+support, font=font, fill=(110,210,235))
                draw.text((20,122), 'Head yaw: '+('SLOWING' if slowing else 'SWING / HOLD'), font=font,
                          fill=(255,195,100) if slowing else 'white')
                if primitive_trial:
                    draw.rectangle((12,118,335,148),fill=(15,22,32))
                    draw.text((20,122),'Stage: '+row['primitive_stage'].upper(),font=font,fill=(255,195,100))
                    draw.rectangle((900,92,1268,177),fill=(15,22,32))
                    waist='OFF' if result['waist_mode']=='off' else 'FIXED TARGET (0 deg)'
                    draw.text((908,96),'Waist: '+waist,font=font,fill=(110,210,235))
                    draw.text((908,122),f"Roll: {np.degrees(row['foot_rpy_rad'][0]):+.1f} deg",font=font,fill='white')
                    draw.text((908,148),f"Edge height difference: {row['side_height_difference_mm']:.1f} mm",font=font,fill='white')
                    if 'foreaft_balance' in result:
                        draw.rectangle((900,118,1268,177),fill=(15,22,32))
                        draw.text((908,122),f"Pitch: {np.degrees(row['foot_rpy_rad'][1]):+.1f} deg",font=font,fill='white')
                        draw.text((908,148),f"Front/back height gap: {row['front_back_height_difference_mm']:.1f} mm",font=font,fill='white')
                    if 'edge_balance' in result or 'foreaft_balance' in result:
                        draw.rectangle((12,470,625,530),fill=(15,22,32))
                        label='ARTIFICIAL TILTED START' if not result.get('natural_upright_start',True) else 'Natural lift from flat'
                        draw.text((20,474),label,font=font,fill='white')
                        draw.text((20,501),f"Continuous stable edge hold: {edge_holds[frame_index]:.2f} s",font=font,fill=(110,210,235))
                    if 'penguin' in result:
                        done=[s for s in result['penguin']['steps'] if s['end_s']<=row['time_s']+1e-8]
                        pairs=[s for i,s in enumerate(done) if i%2]
                        heading=f"{pairs[-1]['heading_deg']:+.1f} deg" if pairs else 'pending'
                        draw.rectangle((12,470,625,530),fill=(15,22,32))
                        draw.text((20,474),f"Completed left-right pairs: {len(pairs)}",font=font,fill='white')
                        draw.text((20,501),f"Last pair heading: {heading}",font=font,fill=(110,210,235))
                elif completed_cycle_rule:
                    draw.rectangle((925,92,1268,149), fill=(15,22,32))
                    draw.text((933,96), f"Last cycle: {last_cycle_heading:+.1f} / +/-20 deg", font=font, fill=(110,210,235))
                    draw.text((933,122), f"Body roll: {np.degrees(row['foot_rpy_rad'][0]):+.1f} deg", font=font, fill='white')
                elif 'heading_feedback' in result:
                    draw.rectangle((925,92,1268,149), fill=(15,22,32))
                    draw.text((933,96), f"Steering trim: {row['steering_correction_deg']:+.2f} deg", font=font, fill=(110,210,235))
                    sensor = result['heading_feedback']['sensor']
                    draw.text((933,122), f"Sensor: {sensor['delay_frames']*20} ms / {sensor['noise_std_deg']:g} deg noise", font=font, fill='white')
            process.stdin.write(canvas.tobytes())
            if len(images) < 160:
                images.append(canvas.resize((768,348)))
        process.stdin.close()
        if process.wait() != 0:
            raise RuntimeError("ffmpeg failed")
        # Short inline preview; full MP4 contains the whole recorded trajectory.
        gif = output.with_suffix(".gif")
        if gif.exists():
            raise FileExistsError(gif)
        images[0].save(gif, save_all=True, append_images=images[1:], duration=round(1000/fps), loop=0)
        images[len(images)//2].save(output.with_suffix(".png"))
    finally:
        renderer.close()
        if process.poll() is None:
            process.terminate()
    print(output)


if __name__ == "__main__":
    main()
