"""Render every case in the frozen ball qualification as one chaptered film.

Run from the project root with the project Python, MuJoCo assets, and ffmpeg.
Existing full-state recordings can be supplied through --work.
"""
import argparse
import json
import math
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from reachy_locomotion.contact_probe import scene_xml
from train_balance_robustness import evaluate

REPORT = Path("experiments/reports/motor-training-ball-qualification-2026-11-20.json")
FPS = 25
REVIEW_FRAMES = 50
INK = "#eef5f8"
MUTED = "#adc2ce"
ACCENT = "#7ad5e4"
BACKGROUND = "#14232e"


def font(size, bold=False):
    name = "Arial Bold.ttf" if bold else "Arial.ttf"
    for path in ("/System/Library/Fonts/Supplemental/" + name,
                 "DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf"):
        try:
            return ImageFont.truetype(path, size)
        except OSError:
            continue
    return ImageFont.load_default(size=size)


def describe(job, run):
    edge = job["edge"]
    dt = job["dt"] * 1000
    conditions = {
        "dt100": f"Physics step {dt:g} ms", "dt250": f"Physics step {dt:g} ms",
        "fresh": "Fresh sensor noise seed", "noise010": "Attitude noise raised to 0.1°",
        "gyro030": "Gyro noise raised to 0.3°/s", "delay40": "Observation delay doubled to 40 ms",
        "stronger": "Heavier, faster ball", "smaller": "Ball diameter reduced to 6 cm",
        "nominal": "Nominal conditions",
    }
    if "balls" in job:
        variant = job["id"].split("-")[2]
        if variant == "mass":
            title = "Robot mass " + ("−5%" if job["id"].endswith("low") else "+5%")
        else:
            title = conditions[variant]
    elif job.get("outward_rate_deg_s"):
        title = f"Moving front catch at {dt:g} ms"
    else:
        title = f"Hold and return at {dt:g} ms"
    start = "Flat start"
    if "initial_angle_deg" in job:
        start = f"Raised start at {job['initial_angle_deg']:g}°"
        if job.get("outward_rate_deg_s"):
            start += f" · {job['outward_rate_deg_s']:g}°/s outward"
    curriculum = run["balance_curriculum"]
    metrics = run.get("edge_balance", run.get("foreaft_balance"))["metrics"]
    impacts = curriculum["impacts"]["events"]
    passed = curriculum["impact_trial_pass" if impacts else "hold_return_pass"]
    event = impacts[0] if impacts else None
    end = run["elapsed_seconds"]
    if run["termination"] != "completed":
        if run["termination"] == "actual_joint_limit_exceeded":
            stop = f"Joint limit exceeded by {math.degrees(-run['min_joint_margin_rad']):.2f}°"
        elif run["termination"] == "tilt_reached_35_deg":
            stop = "Body tilt reached the 35° stop"
        else:
            raise ValueError(run["termination"])
        timing = ""
        if impacts:
            ball = run["ball_disturbances"][0]
            if ball["contacted_robot"]:
                timing = f", {end - ball['first_robot_contact_s']:.2f} s after ball contact"
            elif not ball["released"]:
                timing = ", before ball release"
        outcome = f"{stop} at {end:.2f} s{timing}."
    elif passed and impacts:
        outcome = f"Ball rebounded. Balance recovered in {event['recovery_s']:.2f} s and held through the final 5 s."
    elif impacts and not event["balanced_before_impact"]:
        outcome = "Ball rebounded against the upright base. Qualified edge hold lasted 0.00 s."
        assert event["rebounded"] and metrics["longest_strict_hold_s"] == 0
    else:
        held = metrics["longest_strict_hold_s"]
        outcome = f"Returned flat. Longest qualifying hold was {held:.2f} s (required 4.90 s)."
        assert run["settled"] and curriculum["return_started_s"] is not None
    return title, start, passed, outcome


def render(number, total, job, run, clip):
    title, start, passed, outcome = describe(job, run)
    edge = job["edge"]
    side_view = edge in ("front", "back")
    axis = "pitch" if side_view else "roll"
    angle_index = 1 if side_view else 0
    sign = -1 if edge in ("left", "back") else 1
    xml, _ = scene_xml("experiments/upstream-reachy-mini", run["config"])
    root = ET.fromstring(xml)
    ET.SubElement(root.find("worldbody"), "light", pos="1 -1 2", dir="-1 1 -2",
                  directional="true", diffuse=".75 .75 .75", ambient=".35 .35 .35")
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    data = mujoco.MjData(model)
    camera = mujoco.MjvCamera()
    camera.lookat[:] = [0, 0, .15]
    camera.distance, camera.azimuth, camera.elevation = .80, 90 if side_view else 185, -11
    small, body, heading, medium = font(18), font(23), font(33, True), font(20, True)
    base = Image.new("RGB", (1080, 604), BACKGROUND)
    pen = ImageDraw.Draw(base)
    def label(x, y, s, f=small, color=INK):
        pen.text((x, y), s, font=f, fill=color)
    color = "#85e4ae" if passed else "#ffad92"
    label(22, 12, f"REACHY MINI    /    BALANCE TEST {number:02d} OF {total}", medium, ACCENT)
    label(22, 42, title, heading)
    label(740, 17, "MUJOCO SIMULATION   ·   1×", medium, MUTED)
    label(740, 48, f"{edge.capitalize()} edge   ·   {'Side' if side_view else 'Front'} view", body)
    x = 660
    label(x, 102, start, medium)
    ball = run.get("ball_disturbances", [None])[0]
    if ball:
        p = ball["parameters"]
        direction = "left" if p["from_side"] == 1 else "right"
        label(x, 141, f"Ball from {direction} at {p['launch_s']:g} s", body)
        label(x, 174, f"{200*p['radius_m']:g} cm diameter  ·  {1000*p['mass_kg']:g} g  ·  {p['speed_m_s']:g} m/s")
    else:
        label(x, 141, "Hold the edge, then return flat", body)
        label(x, 174, "Required continuous hold ≥ 4.90 s")
    policy = job["policy"]
    label(x, 213, f"Physics {job['dt']*1000:g} ms  ·  mass {run['config']['motor_model']['total_mass_kg']:.3f} kg", small, MUTED)
    label(x, 238, f"Sensor delay {policy['feedback']['sensor_delay_frames']*20} ms  ·  angle noise {policy['feedback']['sensor_noise_deg']:g}°", small, MUTED)
    if "sensors" in policy:
        label(x, 263, f"Gyro noise {policy['sensors']['gyro_std_deg_s']:g}°/s  ·  seed {policy['feedback']['sensor_seed']}", small, MUTED)
    label(x, 313, "Tilt toward the requested edge", medium)
    x0, x1, y0, y1 = x, 1012, 350, 451
    duration = job["seconds"]
    def point(t, a):
        return (round(x0 + t/duration*(x1-x0)), round(y1-(a+40)/80*(y1-y0)))
    pen.rectangle((x0,y0,x1,y1), fill="#0d1b25")
    for a in (0, 32):
        y = point(0,a)[1]
        pen.line((x0,y,x1,y), fill="#476774")
        label(x1+4,y-10,f"{a}°",small,MUTED)
    label(22, 555, "PASS" if passed else "FAIL", body, color)
    label(106, 556, outcome, medium)
    states = run["trajectory"]
    terminal = run["terminal_state"]
    # Include the guard-triggering physics substep even when it falls between
    # the saved 50 Hz controller frames. The graph and freeze use that pose.
    end = run["elapsed_seconds"]
    frame_count = math.ceil(round(end * FPS, 7))
    times = np.array([r["time_s"] for r in states])
    points = [point(row["time_s"], sign*math.degrees(row["foot_rpy_rad"][angle_index])) for row in states]
    end_point = point(end, sign*math.degrees(terminal["foot_rpy_rad"][angle_index]))
    args = ["ffmpeg","-v","error","-n","-f","rawvideo","-pix_fmt","rgb24","-s","1080x604",
            "-r",str(FPS),"-i","-","-an","-c:v","libx264","-preset","veryfast","-crf","20",
            "-pix_fmt","yuv420p","-threads","2",str(clip)]
    with mujoco.Renderer(model, height=432, width=640) as renderer, subprocess.Popen(args, stdin=subprocess.PIPE) as encoder:
        for frame in range(frame_count + 1):
            final = frame == frame_count
            t = end if final else frame / FPS
            idx = int(np.searchsorted(times,t+1e-8,side="right")-1)
            row = terminal if final else states[idx]
            data.qpos[:] = row["qpos"]
            mujoco.mj_forward(model,data)
            renderer.update_scene(data,camera=camera)
            canvas = base.copy()
            canvas.paste(Image.fromarray(renderer.render()), (0,94))
            draw = ImageDraw.Draw(canvas)
            line = points[:idx+1]
            if final:
                line.append(end_point)
            if len(line)>1:
                draw.line(line, fill=ACCENT, width=2)
            if ball and ball["first_robot_contact_s"] is not None and t>=ball["first_robot_contact_s"]:
                hit_x = point(ball["first_robot_contact_s"],0)[0]
                draw.line((hit_x,y0,hit_x,y1), fill="#ffbd74", width=2)
            angle = sign*math.degrees(row["foot_rpy_rad"][angle_index])
            draw.text((x,460), f"{t:5.2f} s   ·   {axis} toward edge {angle:+.1f}°", font=medium, fill=INK)
            stage = states[idx]["primitive_stage"].replace("_"," ").capitalize()
            status = "Final pose held for review" if final else stage
            if ball and not final and ball["first_robot_contact_s"] is not None and 0<=t-ball["first_robot_contact_s"]<=1:
                status = "Ball contact"
            draw.text((x,491),status,font=body,fill=ACCENT)
            pixels = canvas.tobytes()
            encoder.stdin.write(pixels)
        for _ in range(REVIEW_FRAMES-1):
            encoder.stdin.write(pixels)
        encoder.stdin.close()
        if encoder.wait():
            raise RuntimeError(f"Encoding failed for {clip}")
    return frame_count + REVIEW_FRAMES


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--work",type=Path,default=Path("experiments/generated/qualification-film"))
    parser.add_argument("--output",type=Path,default=Path("media/simulation/balance-qualification-all-38.mp4"))
    args = parser.parse_args()
    index = args.output.with_suffix(".md")
    if args.output.exists() or index.exists():
        parser.error("Choose a new output filename")
    args.work.mkdir(exist_ok=True,parents=True)
    args.output.parent.mkdir(exist_ok=True,parents=True)
    report = json.loads(REPORT.read_text())
    assert report["completed"] and len(report["jobs"]) == len(report["candidates"]) == 38
    manifest = [";FFMETADATA1"]
    table = ["# Reachy Mini balance tests", "", "Each chapter shows one full simulated trial at real speed, followed by a two-second freeze of its terminal pose. The orange line marks first ball contact. The tilt plot is positive toward the requested edge. The 38 chapter outcomes were checked against the frozen qualification report.", "", "| Start (m:ss) | Test | Edge and condition | Result | Observation |", "| --- | ---: | --- | --- | --- |"]
    files, frames = [], 0
    for i,(job,expected) in enumerate(zip(report["jobs"],report["candidates"]),1):
        assert job["id"] == expected["id"]
        path = args.work / (job["id"]+".json")
        if not path.exists():
            evaluate(dict(job,record=str(path)))
        run = json.loads(path.read_text())
        if any(run.get(k)!=v for k,v in expected["result"].items()):
            raise ValueError(f"Recording differs from the frozen report for {job['id']}")
        title,start,passed,outcome = describe(job,run)
        clip = args.work / f"test-{i:02d}.mp4"
        count = render(i,38,job,run,clip)
        files.append(f"file '{clip.name}'")
        end = frames+count
        mark = "PASS" if passed else "FAIL"
        direction = ""
        if job.get("balls"):
            direction = " from " + ("left" if job["balls"][0]["from_side"]==1 else "right")
        entry = f"{job['edge'].capitalize()} edge, {title.lower()}" + (f", ball{direction}" if direction else f", {start.lower()}")
        chapter = f"{i:02d} {mark} | {entry}"
        manifest.extend(["[CHAPTER]",f"TIMEBASE=1/{FPS}",f"START={frames}",f"END={end}",f"title={chapter}"])
        sec = frames // FPS
        table.append(f"| {sec//60}:{sec%60:02d} | {i:02d} | {entry} | {mark} | {outcome} |")
        print(chapter,flush=True)
        frames = end
    playlist, metadata = args.work/"clips.txt", args.work/"chapters.ffmeta"
    playlist.write_text("\n".join(files)+"\n")
    metadata.write_text("\n".join(manifest)+"\n")
    subprocess.run(["ffmpeg","-v","error","-n","-f","concat","-safe","0","-i",str(playlist),
                    "-f","ffmetadata","-i",str(metadata),"-map","0:v:0","-map_chapters","1",
                    "-c:v","copy","-movflags","+faststart",str(args.output)],check=True)
    index.write_text("\n".join(table)+"\n")
    print(f"{frames} frames / {FPS} fps",flush=True)


if __name__ == "__main__":
    main()
