"""Film matched fresh releases from the saved IMU qualification trajectories."""
import json
import math
import subprocess
import tempfile

import mujoco
import numpy as np
from PIL import Image, ImageDraw

from film_moving_table import MEDIA, render_model
from film_shell_release import CYAN, DIM, GOLD, INK, face
from moving_table import plant, read
from train_imu_catch import REPORT, replay_path


def outcome(row):
    if row['held']:
        return 'Held the edge'
    if row['shell']:
        return 'Shell landing'
    return 'Settled flat' if row['flat'] else 'Missed the steady hold'


def load_clip(group, row):
    with np.load(replay_path(group['name'], row['condition'])) as frames:
        if json.loads(str(frames['result'])) != row or json.loads(str(frames['params'])) != group['params']:
            raise ValueError('Replay and qualification records differ')
        return {key: frames[key] for key in ('qpos', 'roll', 'estimated_roll', 'loaded')}


def film():
    groups = {g['name']: g for g in read(REPORT)['qualification']}
    fresh = groups['refit']['trials']
    original = {tuple(r['condition']): r for r in groups['original']['trials']}
    replay = {tuple(r['condition']): r for r in groups.get('replay', {}).get('trials', [])}
    def gain(row):
        before = original[tuple(row['condition'])]
        blind = replay.get(tuple(row['condition']), before)
        return (row['held'] and not blind['held'], row['held'] and not before['held'],
                row['score'] - before['score'])
    best = max(fresh, key=gain)
    challenge = min((r for r in fresh if r is not best), key=lambda r: (r['held'], not bool(r['shell']), r['score']))
    chapters = [('Largest improvement', best, 1), ('Hardest remaining release', challenge, 1)]
    chapters.extend(('Fresh release', r, 3) for r in fresh if r is not best and r is not challenge)
    config = plant(.000125, True).config
    model = render_model(config)
    data = mujoco.MjData(model)
    camera = mujoco.MjvCamera()
    camera.azimuth, camera.elevation, camera.distance = 180, -7, .62
    camera.lookat[:] = [0., 0., .105]
    output = MEDIA / 'imu-right-catch.mp4'
    panel_width = 384 if replay else 518
    width = 1200 if replay else 1080
    with tempfile.NamedTemporaryFile('w', suffix='.ffmeta') as meta:
        meta.write(';FFMETADATA1\n')
        start = 0
        for title, row, speed in chapters:
            stop = start + math.ceil(25 * row['seconds'] / speed) + 25
            angle, rate, seed, age = row['condition']
            meta.write(f'[CHAPTER]\nTIMEBASE=1/25\nSTART={start}\nEND={stop}\n'
                       f'title={title} / {angle:g} deg / {rate:+g} deg per sec / {age:g} ms / {seed}\n')
            start = stop
        meta.flush()
        cmd = ['ffmpeg', '-v', 'error', '-y', '-f', 'rawvideo', '-pix_fmt', 'rgb24',
               '-s', f'{width}x490', '-r', '25', '-i', '-', '-f', 'ffmetadata', '-i', meta.name,
               '-map', '0:v', '-map_chapters', '1', '-c:v', 'libx264', '-threads', '3',
               '-preset', 'veryfast', '-crf', '19', '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(output)]
        with mujoco.Renderer(model, height=285, width=panel_width) as renderer, subprocess.Popen(cmd, stdin=subprocess.PIPE) as enc:
            render_chapters(enc, renderer, model, data, camera, groups, original, replay, chapters, width, panel_width)
            enc.stdin.close()
            if enc.wait():
                raise RuntimeError('Video encoding failed')
    print(output)


def render_chapters(enc, renderer, model, data, camera, groups, original, replay, chapters, width, panel_width):
    for chapter, (title, selected, speed) in enumerate(chapters, 1):
        angle, rate, seed, age = selected['condition']
        views = []
        key = tuple(selected['condition'])
        rows = [('original', original[key]), ('refit', selected)]
        if replay:
            rows.append(('replay', replay[key]))
        for name, row in rows:
            group = groups[name]
            tally = sum(r['held'] for r in group['trials'])
            views.append((name, row, load_clip(group, row), f"{tally}/{len(group['trials'])} held"))
        for frame in range(math.ceil(25 * selected['seconds'] / speed) + 25):
            t = min(frame * speed / 25, selected['seconds'])
            i = round(t * 50)
            image = Image.new('RGB', (width, 490), '#10252c')
            pen = ImageDraw.Draw(image)
            pen.text((14, 7), f'{title}    ·    {chapter}/{len(chapters)}', font=face(26, True), fill=INK)
            pen.text((14, 45), f"Right edge   /   {angle:g}° release at {rate:+g}°/s   /   {age:g} ms reading age   /   seed {seed}",
                     font=face(17), fill=CYAN)
            for j, (name, row, clip, tally) in enumerate(views):
                x = 12 + (panel_width + 16) * j
                label = {'original':'Original', 'refit':'Refit with feedback', 'replay':'Same head plan, replayed'}[name]
                pen.text((x, 76), f'{label}    ·    {tally}', font=face(18, True), fill=INK)
                data.qpos[:] = clip['qpos'][i]
                mujoco.mj_forward(model, data)
                renderer.update_scene(data, camera=camera)
                image.paste(Image.fromarray(renderer.render()), (x, 106))
                pen.text((x, 400), f"Body {clip['roll'][i]:+.1f}°     IMU {clip['estimated_roll'][i]:+.1f}°",
                         font=face(18), fill=CYAN)
                status = 'Right edge loaded' if clip['loaded'][i] else 'Right edge unloaded'
                pen.text((x, 430), outcome(row) if t == selected['seconds'] else status, font=face(18), fill=GOLD)
            pen.text((14, 466), f'MuJoCo    /    Level deck    /    Waist held fixed    /    {speed}× playback    /    {t:.2f} s',
                     font=face(15), fill=DIM)
            enc.stdin.write(image.tobytes())


if __name__ == '__main__':
    film()
