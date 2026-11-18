"""Preserve a supplied GIF and extract timestamped analysis contact sheets.

No motion, distance, source-camera rate or robot telemetry is inferred here.
Requires Pillow; outputs are analysis views of original pixels.
"""
import argparse
import hashlib
import json
import shutil
from pathlib import Path
from PIL import Image, ImageDraw


def inspect(source, out):
    source, out = Path(source).resolve(), Path(out).resolve()
    out.mkdir(parents=True, exist_ok=True)
    dest = out / "original.gif"
    checksum = hashlib.sha256(source.read_bytes()).hexdigest()
    if dest.exists():
        if hashlib.sha256(dest.read_bytes()).hexdigest() != checksum:
            raise ValueError("Destination contains a different original; use a new directory")
    elif source != dest:
        shutil.copy2(source, dest)
    frames, times, durations = [], [], []
    elapsed_ms = 0
    with Image.open(dest) as gif:
        loop = gif.info.get("loop")
        dimensions = list(gif.size)
        for i in range(gif.n_frames):
            gif.seek(i)
            duration = gif.info.get("duration")
            if duration is None or duration <= 0:
                raise ValueError("GIF timing missing/nonpositive; inspect before using timestamps")
            times.append(elapsed_ms)
            durations.append(duration)
            frames.append(gif.convert("RGB").copy())
            elapsed_ms += duration
    count = min(24, len(frames))
    selected = sorted(set(round(i*(len(frames)-1)/max(count-1, 1)) for i in range(count)))

    def contact_sheet(indices, filename, width=280):
        height = width + 24
        sheet = Image.new("RGB", (width*4, height*((len(indices)+3)//4)), (245,245,245))
        draw = ImageDraw.Draw(sheet)
        for j, i in enumerate(indices):
            thumbnail = frames[i].copy()
            thumbnail.thumbnail((width, width))
            x, y = j%4*width, j//4*height
            sheet.paste(thumbnail, (x, y+24))
            draw.text((x+6, y+6), f"Frame {i} | GIF {times[i]/1000:.2f} s", fill="black")
        sheet.save(out/filename)

    contact_sheet(selected, "contact-sheet.png", 240)
    for i in selected:
        frames[i].save(out/f"frame-{i:04d}.png")
    for batch, start in enumerate(range(0, len(frames), 21)):
        contact_sheet(list(range(start, min(start+21, len(frames)))), f"all-frames-{batch+1}.png")
    metadata = {
        "source_path": str(source), "preserved_copy": str(dest),
        "sha256": checksum, "bytes": dest.stat().st_size,
        "dimensions": dimensions, "frames": len(frames),
        "gif_duration_s": elapsed_ms/1000,
        "gif_frame_duration_ms_counts": {str(t): durations.count(t) for t in sorted(set(durations))},
        "gif_loop": loop, "selected_frame_indices": selected, "frame_start_ms": times,
        "provenance": "User-supplied depiction of physical teleoperation; not a new physical experiment. GIF playback timing is not verified camera acquisition timing.",
        "video_calibration": None, "command_log": None, "state_log": None,
    }
    (out/"metadata.json").write_text(json.dumps(metadata, indent=2)+"\n")
    print(json.dumps({k:v for k,v in metadata.items() if k!="frame_start_ms"}, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source")
    parser.add_argument("output_directory")
    args = parser.parse_args()
    inspect(args.source, args.output_directory)
