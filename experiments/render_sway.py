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
    args = parser.parse_args()
    output = Path(args.output)
    if output.exists():
        parser.error("Choose a new output file")
    result = json.loads(Path(args.report).read_text())
    xml, _ = scene_xml(args.upstream, result["config"])
    root = ET.fromstring(xml)
    world = root.find("worldbody")
    ET.SubElement(world, "light", pos="1 -1 2", dir="-1 1 -2", directional="true",
                  diffuse="0.7 0.7 0.7", ambient="0.3 0.3 0.3")
    ET.SubElement(world, "light", pos="-1 1 1", dir="1 -1 -1", directional="true",
                  diffuse="0.5 0.5 0.5")
    for value in np.arange(-.3, .501, .05):
        for a, b in (([value, -.3, .0005], [value, .5, .0005]), ([-.3, value, .0005], [.5, value, .0005])):
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
    cameras = []
    for azimuth in (190, 90):
        cam = mujoco.MjvCamera()
        cam.lookat[:] = [.06, 0, .13]
        cam.distance = .62
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
    try:
        for frame_index, row in enumerate(result["trajectory"]):
            data.qpos[:] = row["qpos"]
            mujoco.mj_forward(model, data)
            if row["measuring"] and start is None:
                start = data.xpos[base].copy()
                yaw = row["foot_rpy_rad"][2]
                start_rotation = np.array([np.cos(yaw), np.sin(yaw), 0])
            if frame_index % 2:
                continue
            canvas = Image.new("RGB", (1280, 580), (15, 22, 32))
            for index, camera in enumerate(cameras):
                renderer.update_scene(data, camera=camera)
                canvas.paste(Image.fromarray(renderer.render()), (index*640, 52))
            draw = ImageDraw.Draw(canvas)
            draw.text((20, 12), "HEAD-DRIVEN SWAY  |  UNCALIBRATED SIMULATION", font=big, fill="white")
            draw.text((1030, 16), f"{1/args.slowdown:g}x playback", font=font, fill=(110, 210, 235))
            draw.text((20, 65), "Front view", font=font, fill="white")
            draw.text((660, 65), "Side view  |  green arrow = forward (+X)", font=font, fill="white")
            distance = float((data.xpos[base]-start) @ start_rotation)*1000 if start is not None else 0
            tilt = np.degrees(np.arccos(np.clip(data.xmat[base].reshape(3,3)[2,2],-1,1)))
            label = f"t = {row['time_s']:.2f} s   |   forward = {distance:+.1f} mm   |   base tilt = {tilt:.1f} deg"
            if not row["measuring"]:
                label = f"t = {row['time_s']:.2f} s   |   startup / warm-up (excluded from displacement)"
            draw.text((20, 544), label, font=font, fill="white")
            draw.text((870, 544), "5 cm grid  |  no external pushes", font=font, fill=(110,210,235))
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
