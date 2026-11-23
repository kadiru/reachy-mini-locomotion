"""Measure fresh-looking IMU values and HTTP latency without issuing motion commands."""
import argparse
import csv
import http.client
import json
import math
from pathlib import Path
import time
from urllib.parse import urlsplit


def percentile(values, p):
    if not values:
        return None
    values = sorted(values)
    return round(values[math.ceil(len(values)*p)-1], 3)


def capture(url, output, hz, seconds, with_pose=False):
    endpoint = urlsplit(url)
    if endpoint.scheme not in ('http','https') or not endpoint.hostname:
        raise ValueError('Provide the daemon origin, including http:// or https://')
    conn = (http.client.HTTPSConnection if endpoint.scheme == 'https' else http.client.HTTPConnection)(
        endpoint.hostname,endpoint.port,timeout=3)
    resource = '/api/state/imu'
    columns = ['send_s','receive_s','status','changed','ax','ay','az','gx','gy','gz']
    if with_pose:
        resource = ('/api/state/full?with_imu=true&with_head_pose=true&use_pose_matrix=true'
                    '&with_body_yaw=true&with_head_joints=true&with_antenna_positions=false&with_control_mode=false')
        columns += ['head_pose_json','head_joints_rad_json','body_yaw_rad']
    rows, changed_times, rtts = 0, [], []
    previous = None
    origin = time.monotonic()
    deadline = origin
    try:
        with output.open('x',newline='') as file:
            writer = csv.writer(file)
            writer.writerow(columns)
            while deadline < origin + seconds:
                time.sleep(max(0.,deadline-time.monotonic()))
                send = time.monotonic()
                conn.request('GET',resource)
                response = conn.getresponse()
                body = response.read()
                rx = time.monotonic()
                if response.status != 200:
                    raise RuntimeError(f'IMU endpoint returned HTTP {response.status}')
                state = json.loads(body)
                payload = state.get('imu') if with_pose else state
                channels = tuple(payload['accelerometer'] + payload['gyroscope']) if payload else ()
                if channels and (len(channels)!=6 or not all(math.isfinite(v) for v in channels)):
                    raise ValueError('Expected six finite inertial channels')
                changed = bool(channels and channels != previous)
                if changed:
                    changed_times.append(rx)
                if channels:
                    previous = channels
                rtts.append((rx-send)*1000)
                fields = [send-origin,rx-origin,'ok' if channels else 'unavailable',int(changed)]
                fields.extend(channels or ('',)*6)
                if with_pose:
                    fields.extend([json.dumps(state.get('head_pose')),json.dumps(state.get('head_joints')),state.get('body_yaw')])
                writer.writerow(fields)
                rows += 1
                # Skip missed requests. Bursts after an overrun inflate the polling figure.
                deadline = origin + (math.floor((rx-origin)*hz)+1)/hz
    finally:
        conn.close()
    elapsed = time.monotonic()-origin
    gaps = [1000*(b-a) for a,b in zip(changed_times,changed_times[1:])]
    return dict(requests=rows,received_hz=rows/elapsed,elapsed_s=elapsed,
                observed_change_hz=(len(gaps)/(changed_times[-1]-changed_times[0])) if gaps else None,
                gap_p95_ms=percentile(gaps,.95),gap_p99_ms=percentile(gaps,.99),gap_max_ms=max(gaps,default=None),
                http_rtt_p95_ms=percentile(rtts,.95),http_rtt_p99_ms=percentile(rtts,.99))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('url')
    parser.add_argument('output',type=Path)
    parser.add_argument('--with-pose',action='store_true',help='Include measured head pose, joints and waist in each response')
    parser.add_argument('--hz',type=int,default=150)
    parser.add_argument('--seconds',type=float,default=20.)
    args = parser.parse_args()
    if not 1 <= args.hz <= 300 or not 0 < args.seconds <= 120:
        parser.error('Use 1–300 HTTP requests/s for at most 120 s')
    print(json.dumps(capture(args.url,args.output,args.hz,args.seconds,args.with_pose),indent=2))
