"""Join the three study films losslessly and index every trial by playback time."""
import argparse
from collections import defaultdict
import csv
from fractions import Fraction
import json
from pathlib import Path
import subprocess

from replay_raised_study import panels as raised_panels
from summarize_hit_envelope import write_manifest
from summarize_impact_returns import add_panel,make_chapters

REPORTS = Path('experiments/reports')
MEDIA = Path('media/simulation')


def sections(work):
    study = json.loads((REPORTS/'motor-training-hit-speeds-2026-11-20.json').read_text())
    returns = json.loads((REPORTS/'motor-training-impact-returns-2026-11-20.json').read_text())
    ball_chapters = write_manifest(study['hits'],work)
    groups = defaultdict(list)
    for row in returns['results']:
        add_panel(groups,row,work/(row['id']+'.json'))
    _, raised = raised_panels(study,work)
    return [('Original edge controller','ball-hit-speed-sweep.mp4',ball_chapters),
            ('Emergency return','ball-emergency-returns.mp4',make_chapters(groups)),
            ('Raised-start tests','raised-tilt-all.mp4',raised)]


def probe(path):
    data=subprocess.check_output(['ffprobe','-v','error','-select_streams','v:0','-show_chapters',
                                  '-show_entries','stream=duration_ts,time_base','-of','json',str(path)])
    return json.loads(data)


def presentation_times(path):
    ticks=subprocess.check_output(['ffprobe','-v','error','-select_streams','v:0','-show_packets',
                                   '-show_entries','packet=pts','-of','csv=p=0',str(path)])
    return [int(t) for t in ticks.splitlines()]


def join(work,output,index):
    if output.exists() or index.exists():
        raise FileExistsError('Choose fresh output and index paths')
    work.mkdir(exist_ok=True,parents=True)
    output.parent.mkdir(exist_ok=True,parents=True)
    index.parent.mkdir(exist_ok=True,parents=True)
    playlist, metadata, rows = [], [';FFMETADATA1'], []
    offset = Fraction(0)
    expected_times = []
    for i,(section,source,manifest) in enumerate(sections(work)):
        path = MEDIA/source
        video = probe(path)
        stream = video['streams'][0]
        duration = stream['duration_ts']*Fraction(stream['time_base'])
        if len(manifest) != len(video['chapters']):
            raise ValueError(source+' chapter count differs from report')
        part = work/f'section-{i+1}.mp4'
        subprocess.run(['ffmpeg','-v','error','-n','-i',str(path),'-map','0:v:0','-map_chapters','-1',
                        '-c:v','copy','-video_track_timescale','60000',str(part)],check=True)
        expected_times.extend(offset+t*Fraction(stream['time_base']) for t in presentation_times(path))
        playlist.extend([f"file '{part.name}'",f"duration {float(duration):.9f}"])
        for chapter, recorded in zip(manifest,video['chapters']):
            if recorded['tags']['title'] != chapter['title']:
                raise ValueError(source+' chapter title differs from report')
            scale=Fraction(recorded['time_base'])
            start=round(1000*(offset+recorded['start']*scale))
            end=round(1000*(offset+recorded['end']*scale))
            metadata.extend(['[CHAPTER]','TIMEBASE=1/1000',f'START={start}',f'END={end}',
                             f'title={section} | {chapter["title"]}'])
            for number,item in enumerate(chapter['panels'],1):
                if item:
                    rows.append([f'{start//60000}:{(start%60000)/1000:06.3f}',start/1000,section,number,
                                 Path(item['record']).stem,chapter['title'],chapter['subtitle'],item['label'],
                                 item['outcome'],item['detail']])
        offset += duration
    listing,meta=work/'sections.ffconcat',work/'all.ffmeta'
    listing.write_text('ffconcat version 1.0\n'+'\n'.join(playlist)+'\n')
    meta.write_text('\n'.join(metadata)+'\n')
    subprocess.run(['ffmpeg','-v','error','-n','-f','concat','-safe','0','-i',str(listing),'-f','ffmetadata',
                    '-i',str(meta),'-map','0:v:0','-map_chapters','1','-c:v','copy','-video_track_timescale','60000',
                    '-movflags','+faststart+disable_chpl',str(output)],check=True)
    result=probe(output)
    stream=result['streams'][0]
    # MP4 decode duration can differ by the change in B-frame delay when
    # joining 25 and 12 fps streams. Verify every presented frame and its timestamp.
    times=presentation_times(output)
    tick=Fraction(stream['time_base'])
    if len(times)!=len(expected_times) or any(abs(t*tick-e)>2*tick for t,e in zip(times,expected_times)):
        raise ValueError('Joined video frame count or presentation times differ')
    if len(result['chapters'])!=metadata.count('[CHAPTER]'):
        raise ValueError('Joined video is missing chapters')
    with index.open('x',newline='') as f:
        writer=csv.writer(f,lineterminator='\n')
        writer.writerow(['start_m_ss','start_seconds','study','panel','trial_id','condition','settings',
                         'panel_label','outcome','observation'])
        writer.writerows(rows)
    print(f'{len(rows)} trials, {len(result["chapters"])} chapters, {float(offset):.3f} seconds')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--work',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--index',type=Path,required=True)
    args=parser.parse_args()
    join(args.work,args.output,args.index)
