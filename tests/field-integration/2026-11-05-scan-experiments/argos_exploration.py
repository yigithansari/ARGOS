"""Supervised scan experiments. No transport, motor commands or automatic driving."""
import argparse
from datetime import datetime, timezone
import itertools
import json
import math
from pathlib import Path
import time


def scan_points(sample, pose):
    """Nominal ToF midpoint at robot origin; telemetry yaw is positive RIGHT."""
    baseline = float(sample['baseline'])
    tilt = math.radians(float(sample['up_deg']))
    if not 0 <= baseline <= 500 or not abs(tilt) < math.pi/4:
        raise ValueError('Invalid nominal sensor geometry')
    x, y, heading = (float(pose[key]) for key in ('x_mm', 'y_mm', 'heading_rad'))
    if not all(math.isfinite(v) for v in (x, y, heading)):
        raise ValueError('Invalid pose')
    result = []
    for side, offset in (('left', baseline/2), ('right', -baseline/2)):
        distance = float(sample[f'{side}_mm'])
        yaw = math.radians(float(sample[f'{side}_yaw']))
        if not sample[f'{side}_ok'] or not 0 < distance <= 4000 or not abs(yaw) <= math.pi/2:
            continue
        local_x = distance*math.cos(tilt)*math.cos(yaw)
        local_y = offset-distance*math.cos(tilt)*math.sin(yaw)
        result.append(dict(side=side, x_mm=x+local_x*math.cos(heading)-local_y*math.sin(heading),
                           y_mm=y+local_x*math.sin(heading)+local_y*math.cos(heading)))
    return result


def wall_candidate(points, tolerance_mm=15):
    """Robust line candidate only; never corrects pose or authorizes movement."""
    unique = []
    for point in points[-64:]:
        p = (float(point['x_mm']), float(point['y_mm']))
        if all(math.isfinite(v) for v in p) and all(math.dist(p, q) >= 15 for q in unique):
            unique.append(p)
    if len(unique) < 6:
        return None
    best = []
    candidates = unique[::max(1, len(unique)//16)]
    for a, b in itertools.combinations(candidates, 2):
        span = math.dist(a, b)
        if span < 100:
            continue
        nx, ny = (b[1]-a[1])/span, (a[0]-b[0])/span
        d = nx*a[0]+ny*a[1]
        inliers = [p for p in unique if abs(nx*p[0]+ny*p[1]-d) <= tolerance_mm]
        if len(inliers) > len(best):
            best = inliers
    if len(best) < 6 or len(best)/len(unique) < .7:
        return None
    cx, cy = (sum(p[i] for p in best)/len(best) for i in (0, 1))
    xx = sum((p[0]-cx)**2 for p in best)/len(best)
    yy = sum((p[1]-cy)**2 for p in best)/len(best)
    xy = sum((p[0]-cx)*(p[1]-cy) for p in best)/len(best)
    angle = .5*math.atan2(2*xy, xx-yy)
    nx, ny = -math.sin(angle), math.cos(angle)
    residual = math.sqrt(sum((nx*(p[0]-cx)+ny*(p[1]-cy))**2 for p in best)/len(best))
    spread = math.sqrt((xx+yy+math.hypot(xx-yy, 2*xy))/2)
    if residual > tolerance_mm or spread < 80:
        return None
    return dict(centre_x_mm=cx, centre_y_mm=cy, tangent_deg=math.degrees(angle),
                residual_mm=residual, inliers=len(best), total=len(unique))


def next_advice(sample, age_seconds, stopped, left_side_mm=None):
    """Conservative proposed step. Caller must never execute it automatically."""
    if not stopped:
        return dict(state='WAIT_STOP', action=None, reason='Wait for confirmed STOP')
    if sample is None or not math.isfinite(age_seconds) or not 0 <= age_seconds <= 1.5:
        return dict(state='HOLD', action=None, reason='Fresh sensor data required')
    if not all(sample.get(f'{s}_ok') and 0 < float(sample[f'{s}_mm']) <= 4000 for s in ('left', 'right')):
        return dict(state='HOLD', action=None, reason='Both forward ranges must be valid')
    rays = scan_points(sample, dict(x_mm=0, y_mm=0, heading_rad=0))
    if len(rays) != 2:
        return dict(state='HOLD', action=None, reason='Forward sensor geometry invalid')
    front = min(p['x_mm'] for p in rays)
    if front < 200:
        return dict(state='BLOCKED', action=None, reason='Too close; inspect clearance before any turn')
    if front < 350:
        return dict(state='SCAN_WALL', action=['R', 15], reason='Wall ahead; propose a short right scan, inspect swept area')
    if left_side_mm is not None:
        if not math.isfinite(left_side_mm) or left_side_mm <= 0:
            return dict(state='HOLD', action=None, reason='Left side range invalid')
        if left_side_mm > 600:
            return dict(state='WALL_LOST', action=None, reason='Possible corner/gap; stop and rescan')
        if left_side_mm < 150:
            return dict(state='BLOCKED', action=None, reason='Side clearance too small')
        if left_side_mm < 200:
            return dict(state='FOLLOW_LEFT', action=['R', 15], reason='Propose turning away from left wall')
        if left_side_mm > 300:
            return dict(state='FOLLOW_LEFT', action=['L', 15], reason='Propose turning toward left wall')
        return dict(state='FOLLOW_LEFT', action=['F', 30], reason='Side distance within target band')
    return dict(state='APPROACH', action=['F', 30], reason='Propose a short forward step; side-wall tracking needs side range/scan')


class ExperimentLog:
    def __init__(self, directory, metadata):
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
        self.path = directory/f'experiment_{stamp}.jsonl'
        self.file = self.path.open('x', encoding='utf-8', buffering=1)
        self.sequence = 0
        try:
            self.record('session', schema=1, metadata=metadata)
        except Exception:
            self.file.close()
            raise

    def record(self, event, **data):
        row = dict(sequence=self.sequence, utc=datetime.now(timezone.utc).isoformat(),
                   monotonic_ns=time.monotonic_ns(), event=event, data=data)
        encoded = json.dumps(row, ensure_ascii=False, allow_nan=False)
        self.file.write(encoded+'\n')
        self.sequence += 1

    def close(self):
        if not self.file.closed:
            try:
                self.record('end')
            finally:
                self.file.close()


def replay(path):
    points, poses, events, errors = [], [], {}, 0
    segments = []
    outcomes, metadata = [], {}
    with Path(path).open(encoding='utf-8', errors='replace') as source:
        for line in source:
            try:
                row = json.loads(line)
                events[row['event']] = events.get(row['event'], 0)+1
                if row['event'] == 'session':
                    metadata = row['data']['metadata']
                elif row['event'] == 'rx':
                    outcomes.extend(line.strip() for line in row['data']['text'].splitlines()
                                    if line.startswith(('MOVE DONE', 'MOVE ABORT', 'MOVE CANCELLED', 'MOVE ERROR')))
                elif row['event'] == 'origin_reset':
                    segments.append(dict(points=points, poses=poses))
                    points, poses = [], []
                elif row['event'] == 'tof':
                    points.extend(scan_points(row['data']['sample'], row['data']['pose']))
                elif row['event'] == 'motor':
                    poses.append(row['data']['pose'])
            except (ValueError, TypeError, KeyError):
                errors += 1
    segments.append(dict(points=points, poses=poses))
    return dict(events=events, malformed_records=errors, points=points, poses=poses,
                segments=segments, metadata=metadata, motion_outcomes=outcomes,
                wall_candidate=wall_candidate(points),
                note='Encoder/nominal-extrinsic estimate, not a corrected room map')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Replay an experiment without connecting to the robot')
    parser.add_argument('--replay', required=True)
    args = parser.parse_args()
    result = replay(args.replay)
    target = Path(args.replay).with_suffix('.summary.json')
    target.write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding='utf-8')
    print(target)
