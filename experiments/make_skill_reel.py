"""Assemble the chaptered skill reel from archived simulation films and the energy ledger."""
import argparse
import csv
import json
from pathlib import Path
import shutil
import subprocess
import tempfile

from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
MEDIA = ROOT / 'media/simulation'
FPS = 25
W, H = 1920, 1080
# The complete source frame sits inside this viewport, including its telemetry.
VIEW = (40, 162, 1424, 802)
BG = '#101a26'
INK = '#f4f4ef'
DIM = '#b7c6d1'
SUBTLE = '#748a9b'
ACCENTS = {'Walking': '#ffca74', 'Balance': '#7de1c1', 'Turning': '#8ccdfb'}


def font(size, bold=False):
    filenames = ('Arial Bold.ttf', 'DejaVuSans-Bold.ttf') if bold else ('Arial.ttf', 'DejaVuSans.ttf')
    for parent in ('/System/Library/Fonts/Supplemental', '/usr/share/fonts/truetype/dejavu'):
        for name in filenames:
            path = Path(parent) / name
            if path.exists():
                return ImageFont.truetype(str(path), size)
    raise RuntimeError('Arial or DejaVu Sans is required to render the labels.')


def run(*args):
    subprocess.run([str(x) for x in args], check=True)


def probe(path):
    info = subprocess.check_output(['ffprobe', '-v', 'error', '-select_streams', 'v:0',
                                    '-show_entries', 'stream=width,height,duration', '-of', 'json', str(path)])
    return json.loads(info)['streams'][0]


def plate(chapters, index, example, row, output):
    c = chapters[index]
    accent = ACCENTS[c['group']]
    img = Image.new('RGB', (W, H), BG)
    draw = ImageDraw.Draw(img)

    def label(x, y, msg, size=26, color=INK, bold=False):
        draw.text((x, y), msg, font=font(size, bold), fill=color, anchor='lt')

    def paragraph(x, y, msg, width, bottom, size=26):
        face = font(size)
        line = ''
        for word in msg.split():
            proposed = (line+' '+word).strip()
            if line and draw.textlength(proposed, font=face) > width:
                label(x, y, line, size, DIM)
                y += size + 9
                line = word
            else:
                line = proposed
        if y + size > bottom:
            raise ValueError(f'Text exceeds panel: {msg}')
        label(x, y, line, size, DIM)

    label(48, 32, 'REACHY MINI   /   MUJOCO SIMULATION', 24, DIM, True)
    label(48, 84, c['title'], 52, INK, True)
    top = f"{c['group'].upper()}   /   SKILL {index+1:02} OF {len(chapters):02}"
    draw.text((1872, 46), top, font=font(23, True), fill=accent, anchor='rt')
    # The position strip groups the eleven skills by walking, balance and turning.
    for i, chapter in enumerate(chapters):
        x = 1505 + 34*i
        draw.rectangle((x, 94, x+25, 100), fill=ACCENTS[chapter['group']] if i==index else '#354959')

    vx, vy, vw, vh = VIEW
    draw.rounded_rectangle((vx-1, vy-1, vx+vw+1, vy+vh+1), 6, fill='#0b131d', outline='#324451', width=2)
    x = 1508
    label(x, 179, example['variant'].upper(), 21, accent, True)
    label(x, 230, example['metric'], 61, INK, True)
    paragraph(x, 313, example['detail'], 361, 462)
    draw.line((x, 474, 1872, 474), fill='#3d5061', width=2)

    passed, tested = ((int(row['passed']), int(row['tested'])) if row else example['passed'])
    label(x, 504, 'TESTS PASSED', 21, accent, True)
    label(x, 546, f'{passed} of {tested}', 45, INK, True)
    pct = 100*passed/tested
    draw.text((1872, 558), f'{pct:.1f}'.rstrip('0').rstrip('.')+'%', font=font(27), fill=DIM, anchor='rt')
    draw.rounded_rectangle((x, 611, 1872, 621), 5, fill='#344759')
    if passed:
        draw.rounded_rectangle((x, 611, round(x+(1872-x)*passed/tested), 621), 5, fill=accent)
    paragraph(x, 651, example['suite'], 364, 799, size=24)
    draw.line((x, 808, 1872, 808), fill='#3d5061', width=2)
    label(x, 837, 'TRAINING ELECTRICITY', 21, accent, True)
    if row and row['meter_receipt']:
        cents = 100*float(row['gross_system_cost_usd'])
        wh = 1000*float(row['gross_system_kwh'])
        label(x, 882, f'{wh:.2f} Wh   /   {cents:.3f}¢', 33, INK, True)
        label(x, 936, f"M4 refit at ${float(row['rate_usd_kwh']):.2f}/kWh (example)", 21, DIM)
    else:
        label(x, 882, 'Unmetered', 33, INK, True)
    draw.line((48, 991, 1872, 991), fill='#3d5061', width=2)
    paragraph(48, 1014, example['watch'], 1824, 1076, size=27)
    img.save(output)


def segment(source, start, duration, background, dest):
    stream = probe(source)
    if start < 0 or start+duration > float(stream['duration'])+.001:
        raise ValueError(f'Excerpt exceeds the source: {source} at {start}+{duration}')
    vx, vy, vw, vh = VIEW
    scale = min(vw/stream['width'], vh/stream['height'])
    sw, sh = [2*int(d*scale/2) for d in (stream['width'], stream['height'])]
    x, y = vx+(vw-sw)//2, vy+(vh-sh)//2
    filt = f'[1:v]fps={FPS},scale={sw}:{sh}:flags=lanczos,setsar=1[v];[0:v][v]overlay={x}:{y},format=yuv420p'
    count = round(duration*FPS)
    run('ffmpeg','-v','error','-nostdin','-loop','1','-framerate',FPS,'-i',background,
        '-ss',start,'-i',source,'-filter_complex_threads',2,'-filter_complex',filt,'-an',
        '-frames:v',count,'-c:v','libx264','-preset','veryfast','-crf','20','-threads',4,
        '-g',FPS*2,'-sc_threshold','0',dest)
    return count


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    out = args.output.resolve()
    if out.exists():
        p.error('Choose a new output filename.')
    out.parent.mkdir(parents=True, exist_ok=True)
    chapters = json.loads((ROOT/'experiments/skill_reel.json').read_text())
    with (ROOT/'experiments/reports/training-compute.csv').open() as f:
        ledger = {r['model']: r for r in csv.DictReader(f)}
    with tempfile.TemporaryDirectory(dir=out.parent) as directory:
        tmp = Path(directory)
        meta = [';FFMETADATA1', 'title=Reachy Mini | Trained skills in simulation']
        listing = []
        frames = 0
        for i, ch in enumerate(chapters):
            begin = frames
            print(f"{begin/FPS:6.2f}s  {ch['title']}", flush=True)
            for j, ex in enumerate(ch['examples']):
                image = tmp/f'{i}-{j}.png'
                row = ledger[ex['ledger']] if 'ledger' in ex else None
                plate(chapters, i, ex, row, image)
                for start, duration in ex['cuts']:
                    part = tmp/f'part-{len(listing):03}.mp4'
                    frames += segment(MEDIA/(ex['source']+'.mp4'),start,duration,image,part)
                    listing.append(f"file '{part.name}'")
            meta.extend(('[CHAPTER]', f'TIMEBASE=1/{FPS}', f'START={begin}', f'END={frames}', f"title={ch['title']}"))
        (tmp/'chapters.txt').write_text('\n'.join(meta)+'\n')
        (tmp/'concat.txt').write_text('\n'.join(listing)+'\n')
        joined = tmp/'joined.mp4'
        run('ffmpeg','-v','error','-nostdin','-f','concat','-safe','1','-i',tmp/'concat.txt',
            '-f','ffmetadata','-i',tmp/'chapters.txt','-map','0:v:0','-map_metadata','1','-map_chapters','1',
            '-c:v','copy','-movflags','+faststart',joined)
        if abs(float(probe(joined)['duration'])-frames/FPS) > .01:
            raise ValueError('Unexpected output duration')
        shutil.move(str(joined), str(out))


if __name__ == '__main__':
    main()
