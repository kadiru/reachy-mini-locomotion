"""Measure an entire training command with time-stamped host or GPU board watts."""
import argparse
import bisect
import csv
import json
import math
import os
from pathlib import Path
import platform
import resource
import subprocess
import sys
import threading
import time
from datetime import datetime, timezone


def at(samples, t):
    times = [row[0] for row in samples]
    i = min(max(1, bisect.bisect_left(times, t)), len(times)-1)
    a, b = samples[i-1:i+1]
    return a[1] + (b[1]-a[1]) * (t-a[0]) / (b[0]-a[0])


def meter_window(samples, start, end):
    if len(samples) < 2 or not samples[0][0] <= start < end <= samples[-1][0]:
        raise ValueError('Power readings do not cover both boundaries')
    for (t0, _), (t1, _) in zip(samples, samples[1:]):
        if t1 <= t0 or (t0 < end and t1 > start and t1-t0 > 5):
            raise ValueError('Non-monotonic or missing power samples')
    points = [(start, at(samples, start)), *[(t,w) for t,w in samples if start<t<end], (end, at(samples,end))]
    gaps = [b[0]-a[0] for a,b in zip(points, points[1:])]
    joules = sum((a[1]+b[1])*.5*dt for a,b,dt in zip(points, points[1:], gaps))
    return dict(mean_w=joules/(end-start), peak_w=max(w for _,w in points), kwh=joules/3_600_000,
                max_sampling_gap_s=max(gaps))


def cpu_seconds():
    usage = resource.getrusage(resource.RUSAGE_CHILDREN)
    return usage.ru_utime + usage.ru_stime


class PowerStream:
    def __init__(self, args):
        self.nvidia = bool(args.gpu_uuid)
        self.gpu_uuid = args.gpu_uuid
        self.rows = []
        self.error = None
        self.epoch = time.monotonic()
        if self.nvidia:
            query = ['nvidia-smi', '-i', args.gpu_uuid, '--query-gpu=name,uuid,power.draw',
                     '--format=csv,noheader,nounits','--loop-ms=1000']
        else:
            query = [args.macmon, 'pipe','-i','1000','--soc-info']
        self.process = subprocess.Popen(query, stdout=subprocess.PIPE, text=True)
        self.thread = threading.Thread(target=self.collect, daemon=True)
        self.thread.start()

    def collect(self):
        try:
            for line in self.process.stdout:
                t = time.monotonic()-self.epoch
                if self.nvidia:
                    name, uuid, power = next(csv.reader([line], skipinitialspace=True))
                    if uuid.strip() != self.gpu_uuid:
                        raise ValueError('Power sensor UUID differs from pinned CUDA device')
                    self.hardware = dict(gpu=name.strip(), uuid=uuid.strip())
                    row = dict(t_s=t, watts=float(power))
                else:
                    sample = json.loads(line)
                    soc = sample['soc']
                    self.hardware = dict(cpu=soc['chip_name'], model=soc['mac_model'],
                                         cpu_cores=sum(len(k['cores']) if isinstance(k['cores'],list) else k['cores']
                                                       for k in soc['cpu_tiers']),
                                         gpu_cores=soc['gpu_cores'], memory_gb=soc['memory_gb'])
                    row = dict(t_s=t, watts=sample['sys_power'], cpu_w=sample['cpu_power'],
                               gpu_w=sample['gpu_power'])
                if not math.isfinite(row['watts']) or row['watts'] <= 0:
                    raise ValueError('Missing or nonpositive power reading')
                self.rows.append(row)
        except Exception as exc:
            self.error = str(exc)

    def wait_for(self, t):
        limit = time.monotonic()+8
        while not self.rows or self.rows[-1]['t_s'] < t:
            if self.error or self.process.poll() is not None or time.monotonic() >= limit:
                raise RuntimeError(self.error or 'Power sampler stopped or timed out')
            time.sleep(.05)

    def close(self):
        self.process.terminate()
        try:
            self.process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait()
        self.thread.join(timeout=3)


def compute_process_count(uuid):
    output = subprocess.check_output(
        ['nvidia-smi','--query-compute-apps=gpu_uuid','--format=csv,noheader'],text=True)
    return sum(line.strip()==uuid for line in output.splitlines())


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--model',required=True)
    p.add_argument('--phase',choices=('training','evaluation','rendering'),required=True)
    sensor = p.add_mutually_exclusive_group(required=True)
    sensor.add_argument('--macmon',help='macmon executable for a CPU MuJoCo run')
    sensor.add_argument('--gpu-uuid',help='Physical NVIDIA GPU UUID for a CUDA workload')
    p.add_argument('--device', choices=('cpu','metal','cuda'),
                   help='Compute backend selected by the workload (defaults to CPU on Mac and CUDA for a GPU UUID)')
    p.add_argument('--rate-usd-kwh',type=float)
    p.add_argument('--baseline-seconds',type=float,default=15.)
    p.add_argument('command',nargs=argparse.REMAINDER)
    a = p.parse_args()
    cmd = a.command[1:] if a.command[:1]==['--'] else a.command
    if (not cmd or not math.isfinite(a.baseline_seconds) or a.baseline_seconds < 3
            or (a.rate_usd_kwh is not None and (not math.isfinite(a.rate_usd_kwh) or a.rate_usd_kwh < 0))):
        p.error('Provide a command, at least three background seconds, and a nonnegative optional rate')
    backend = a.device or ('cuda' if a.gpu_uuid else 'cpu')
    if (backend == 'cuda') != bool(a.gpu_uuid):
        p.error('Specify a physical NVIDIA UUID for CUDA, or macmon for CPU/Metal')
    a.output.parent.mkdir(parents=True,exist_ok=True)
    out = a.output.open('x')
    report = dict(model=a.model, phase=a.phase, command=cmd, status='incomplete', os=platform.platform(),
                  clock_origin_utc=datetime.now(timezone.utc).isoformat(), training_device=backend.upper(),
                  scope='NVIDIA board only' if a.gpu_uuid else 'Mac system sensor (macmon sys_power)',
                  rate_usd_kwh=a.rate_usd_kwh)
    env = os.environ.copy()
    stream = None
    try:
        if a.gpu_uuid:
            env['CUDA_VISIBLE_DEVICES'] = a.gpu_uuid
            report['attribution'] = 'Selected CUDA device; board power includes other processes using the card.'
            report['gpu_processes_at_start'] = compute_process_count(a.gpu_uuid)
        else:
            report['meter_version'] = subprocess.check_output([a.macmon,'--version'],text=True).strip()
        stream = PowerStream(a)
        stream.wait_for(0)
        time.sleep(a.baseline_seconds+.5)
        if stream.error or stream.process.poll() is not None:
            raise RuntimeError(stream.error or 'Power sampler stopped before training')
        before = cpu_seconds()
        start = time.monotonic()-stream.epoch
        print(f"Starting {a.model} {a.phase}",flush=True)
        result = subprocess.run(cmd,env=env)
        end = time.monotonic()-stream.epoch
        report.update(returncode=result.returncode, wall_s=end-start, cpu_s=cpu_seconds()-before,
                      hardware=stream.hardware, start_s=start, end_s=end)
        if a.gpu_uuid:
            report['gpu_processes_at_end'] = compute_process_count(a.gpu_uuid)
        stream.wait_for(end)
        if stream.error:
            raise RuntimeError(stream.error)
        series = [(r['t_s'], r['watts']) for r in stream.rows]
        baseline = meter_window(series,start-a.baseline_seconds,start)
        gross = meter_window(series,start,end)
        incremental = gross['kwh'] - baseline['mean_w']*(end-start)/3_600_000
        report.update(baseline=baseline, gross=gross, incremental_kwh=incremental,
                      gross_usd=None if a.rate_usd_kwh is None else gross['kwh']*a.rate_usd_kwh,
                      status='complete' if result.returncode==0 else 'workload_failed')
    except BaseException as exc:
        report['error'] = str(exc)
        raise
    finally:
        if stream is not None:
            stream.close()
            report['samples'] = stream.rows
        with out:
            json.dump(report,out,allow_nan=False,indent=1)
            out.write('\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ('command','samples')},indent=1),flush=True)
    return result.returncode


if __name__=='__main__':
    sys.exit(main())
