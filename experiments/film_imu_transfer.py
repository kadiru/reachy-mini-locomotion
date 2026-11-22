"""Compare three matched downhill-right releases with a fixed camera and identical clock."""
import subprocess

import mujoco
import numpy as np
from PIL import Image, ImageDraw

from film_moving_table import MEDIA, render_model
from film_shell_release import CYAN, DIM, GOLD, INK, face
from imu_transfer import FilteredPackets, IMUPackets, OUTPUT
from moving_table import SEEDS, read, simulate


def replay(archive, mode, observer):
    row = next(r for r in archive if r['mode'] == mode and not r['result']['job']['flat']
               and r['result']['job']['seed'] == SEEDS[0])
    result, track, config, frames = simulate(row['result']['job'], recording=True, imu_observer=observer)
    if result != row['result']:
        raise ValueError(f'{mode} replay differs from the recorded trial')
    return result, track, config, frames


def film():
    trials = read(OUTPUT)['trials'] + read(OUTPUT.with_name('moving-table-imu-bandwidth.json'))['trials']
    panels = [
        ('Original feedback', '3/3 held', replay(trials, 'reference', IMUPackets('reference'))),
        ('IMU read immediately', '3/3 settled flat', replay(trials, 'filtered_0', FilteredPackets(0))),
        ('IMU read 5 ms later', '1/3 hit shell, 2/3 flat', replay(trials, 'filtered_5', FilteredPackets(5))),
    ]
    model = render_model(panels[0][2][2])
    data = mujoco.MjData(model)
    camera = mujoco.MjvCamera()
    camera.azimuth, camera.elevation, camera.distance = 180, -7, .65
    camera.lookat[:] = [0., 0., .105]
    fps = 25
    duration = panels[0][2][0]['duration']
    # Half speed through the initial catch, then show the complete excursion and return.
    playback = [(t, '0.5×') for t in np.arange(0., 3., .5/fps)]
    playback.extend((t, '3×') for t in np.arange(3., duration, 3./fps))
    playback.extend([(duration, 'End')] * fps)
    output = MEDIA / 'moving-table-imu.mp4'
    command = ['ffmpeg', '-v', 'error', '-y', '-f', 'rawvideo', '-pix_fmt', 'rgb24',
               '-s', '1200x450', '-r', str(fps), '-i', '-', '-c:v', 'libx264', '-threads', '3',
               '-preset', 'veryfast', '-crf', '19', '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(output)]
    with mujoco.Renderer(model, height=250, width=384) as renderer, subprocess.Popen(command, stdin=subprocess.PIPE) as enc:
        for t, speed in playback:
            image = Image.new('RGB', (1200, 450), '#10252c')
            pen = ImageDraw.Draw(image)
            pen.text((12, 8), 'Can Reachy catch the same 32° release?', font=face(27, True), fill=INK)
            pen.text((12, 44), 'MuJoCo   /   Fixed waist   /   Same controller and noise seed in all three views',
                     font=face(17), fill=DIM)
            for index, (title, tally, (result, track, _, frames)) in enumerate(panels):
                x = 12 + index * 396
                k = np.clip(np.searchsorted(frames[:, 0], t, side='right') - 1, 0, len(frames) - 1)
                data.qpos[:] = frames[k, 3:]
                mujoco.mj_forward(model, data)
                renderer.update_scene(data, camera=camera)
                pen.text((x, 83), title, font=face(20, True), fill=INK)
                image.paste(Image.fromarray(renderer.render()), (x, 112))
                sample = track[min(len(track) - 1, int(t/.02))]
                pen.text((x, 368), f"Body roll {sample['roll']:+.1f}°", font=face(17), fill=CYAN)
                pen.text((x, 393), tally, font=face(17), fill=GOLD)
            pen.text((12, 425), f"Elapsed {t:.2f} s    /    Deck {sample['bank']:+.1f}°    /    {speed} playback",
                     font=face(15), fill=DIM)
            enc.stdin.write(image.tobytes())
        enc.stdin.close()
        if enc.wait():
            raise RuntimeError('IMU comparison video encoding failed')
    print(output)


if __name__ == '__main__':
    film()
