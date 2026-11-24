"""Supervised scan experiments. No transport, motor commands or automatic driving."""
import argparse
from collections import deque
from datetime import datetime, timezone
import itertools
from html import escape
import json
import math
from pathlib import Path
import time


CORNER_ENTRY_MM = 600


class ScanPoseBuffer:
    """Associate moving scans with poses bracketed by encoder telemetry."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.anchor = None
        self.pending = deque(maxlen=64)

    def scan(self, stamp, sample, pose):
        if sample.get('map_eligible', True):
            return scan_points(sample, pose)
        valid = sample['ok'] if sample.get('sensor') == 'ultrasonic_left' else (
            sample['left_ok'] or sample['right_ok'])
        if self.anchor and stamp >= self.anchor[0] and valid:
            self.pending.append((stamp, sample))
        return []

    def motor(self, stamp, pose):
        previous = self.anchor
        self.anchor = (stamp, pose)
        points = []
        while self.pending and self.pending[0][0] <= stamp:
            scan_time, sample = self.pending.popleft()
            if not previous or not 0 < stamp - previous[0] <= 1.5 or scan_time < previous[0]:
                continue
            start = previous[1]
            weight = (scan_time - previous[0]) / (stamp - previous[0])
            estimated = {key: start[key] + weight * (pose[key] - start[key])
                         for key in ('x_mm', 'y_mm')}
            delta = (pose['heading_rad'] - start['heading_rad'] + math.pi) % math.tau - math.pi
            estimated['heading_rad'] = start['heading_rad'] + weight * delta
            points.extend(scan_points(sample, estimated))
        return points


def search_clear(front, side_mm, *, open_space=False):
    statuses = (4, 6, 8, 9) if open_space else (4, 6, 9)
    return (front and (side_mm is None and open_space or side_mm is not None and side_mm >= 110)
            and all(front.get(f'{side}_mm', 0) > 200 if front.get(f'{side}_ok')
                    else front.get(f'{side}_raw') in statuses for side in ('left', 'right')))


def exploration_step(front, side_mm, *, side_closing=False, side_receding=False,
                     allow_curve=True, open_space=False, previous_front=None):
    """Return direction, distance/angle, and the inner wheel for a forward arc."""
    distances = {side: front[f'{side}_mm'] for side in ('left', 'right')
                 if front and front.get(f'{side}_ok') and 0 < front[f'{side}_mm'] <= 4000}
    valid = distances.values()
    if not valid:
        if search_clear(front, side_mm, open_space=open_space):
            return 'S', 60 if open_space else 100, None
        return None
    if len(valid) == 2 and max(valid) <= 200:
        return 'R', 90, None
    left = distances.get('left')
    right = distances.get('right')
    if left is not None and left <= 150 and side_mm is not None and side_mm <= 230:
        return 'R', 90, None
    if (allow_curve and len(valid) == 2 and 200 < min(valid) and max(valid) <= CORNER_ENTRY_MM and
            max(valid) - min(valid) <= 160):
        return 'Q', 90, None
    if (allow_curve and previous_front and (side_mm is None or side_mm > 500) and
            0 < front['received_at'] - previous_front['received_at'] <= 0.6):
        for key, other in (('left', 'right'), ('right', 'left')):
            sensor = distances.get(key)
            if (sensor is not None and 350 <= sensor <= CORNER_ENTRY_MM and
                    (distances.get(other, 0) > CORNER_ENTRY_MM or
                     (other not in distances and front.get(f'{other}_raw') in (4, 6, 8, 9))) and
                    previous_front.get(f'{key}_ok') and
                    abs(previous_front[f'{key}_mm'] - sensor) <= 80):
                return 'Q', 90, None
    left_wall_ahead = (left is not None and left <= 420 and
                       (right is None or right - left >= 100))
    if ((side_mm is not None and (side_mm < 250 or (side_mm <= 450 and
                                                   (side_closing or left_wall_ahead)))) or
            (side_mm is None and left_wall_ahead and left <= 300)):
        return 'F', 50, 'R'
    if (side_receding and side_mm is not None and 320 <= side_mm <= 500 and
            len(valid) == 2 and min(valid) > CORNER_ENTRY_MM):
        return 'F', 50, 'L'
    return 'F', 50, None


def side_approaching(samples, now, *, receding=False):
    recent = [mm for stamp, mm in samples if 0 <= now - stamp <= .4]
    earlier = [mm for stamp, mm in samples if .45 <= now - stamp <= 1.2]
    if len(recent) < 2 or len(earlier) < 2:
        return False
    if receding:
        earlier, recent = recent, earlier
    # Upper/lower edges reject a single short or long echo as a trend.
    return min(earlier) - max(recent) >= 25


def traction_clear(front, side_mm):
    return (side_mm is not None and side_mm >= 250 and
            all(front.get(f'{side}_ok') and front[f'{side}_mm'] >= 400 for side in ('left', 'right')))


def corner_step(front, remaining, retreating, must_back=False, recovery_turning=False, side_mm=None):
    distances = [front[f'{side}_mm'] for side in ('left', 'right')
                 if front.get(f'{side}_ok') and front[f'{side}_mm'] > 0]
    if not distances:
        return None
    back_mm = min(90, max(30, 200 - front['left_mm'])) if front.get('left_ok') else 30
    if must_back or (side_mm is not None and side_mm < 80):
        return 'B', back_mm, None
    if retreating:
        if not front.get('left_ok'):
            return None
        if front['left_mm'] < (150 if recovery_turning else 190):
            return 'B', back_mm, None
        return 'H', min(30, max(15, round(remaining))), None
    if min(distances) < 150:
        return 'B', back_mm, None
    return 'R', min(90, max(15, round(remaining))), None


def corner_can_exit(front, remaining, side_mm):
    """Resume forward after a nearly completed turn with usable clearance."""
    right_open = (front['right_mm'] > 200 if front.get('right_ok')
                  else front.get('right_raw') in (4, 6, 9))
    return (abs(remaining) <= 8 and side_mm is not None and side_mm >= 40 and
            front.get('left_ok') and front['left_mm'] > 200 and right_open)


def cruise_command(inner, *, update=False):
    return f"G {'U' if update else 'C'} {22 if inner == 'L' else 25} {22 if inner == 'R' else 25}"


def save_map_svg(summary, path):
    """Save origin segments as separate panels in a metric, top-down map."""
    segments = [s for s in summary['segments'] if s['points'] or s['poses']] or [dict(points=[], poses=[])]
    width, height = 800, 520
    svg = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height*len(segments)}">',
           f'<rect width="100%" height="100%" fill="#101b29"/>']
    for index, segment in enumerate(segments):
        points, poses = segment['points'], segment['poses']
        angle = map_wall_rotation(points)
        if angle is not None:
            radians = math.radians(angle)
            cosine, sine = math.cos(radians), math.sin(radians)

            def rotate(p):
                return dict(p, x_mm=p['x_mm']*cosine - p['y_mm']*sine,
                            y_mm=p['x_mm']*sine + p['y_mm']*cosine)

            points, poses = [rotate(p) for p in points], [rotate(p) for p in poses]
        positions = points + poses
        xs = [p['x_mm'] for p in positions] + [0]
        ys = [p['y_mm'] for p in positions] + [0]
        cx, cy = (min(xs)+max(xs))/2, (min(ys)+max(ys))/2
        scale = min(690/max(100, max(ys)-min(ys)), 400/max(100, max(xs)-min(xs)))
        top = index * height
        def project(x, y):
            return 400-(y-cy)*scale, top+270-(x-cx)*scale
        if poses:
            trace = ' '.join(f'{x:.2f},{y:.2f}' for x, y in
                             (project(p['x_mm'], p['y_mm']) for p in poses))
            svg.append(f'<polyline points="{trace}" fill="none" stroke="#79ecb4" stroke-width="2"/>')
        for point in points:
            x, y = project(point['x_mm'], point['y_mm'])
            color = '#d9a4ff' if point.get('sensor') == 'ultrasonic_left' else (
                '#52c1ff' if point['side'] == 'left' else '#ffd074')
            svg.append(f'<circle cx="{x:.2f}" cy="{y:.2f}" r="2" fill="{color}"/>')
        alignment = f' | view rotation {angle:+.1f} deg' if angle is not None else ''
        label = escape(f'ARGOS | segment {index+1} | {len(points)} returns | {1/scale:.1f} mm/pixel{alignment}')
        svg.append(f'<text x="24" y="{top+28}" fill="white" font-family="sans-serif">{label}</text>')
        if any(point.get('sensor') == 'ultrasonic_left' for point in points):
            svg.append(f'<text x="24" y="{top+49}" fill="#d9a4ff" font-family="sans-serif">Purple: left ultrasound</text>')
    Path(path).write_text('\n'.join(svg + ['</svg>']), encoding='utf-8')


def side_scan(sample, calibration):
    """Prepare a logged side reading using the measured mount or a track-width estimate."""
    if 'sensor' in sample:
        return sample
    try:
        width = float(calibration['track_width_mm'])
        local_x = float(calibration.get('side_sensor_x_mm', 0))
        local_y = float(calibration.get('side_sensor_y_mm', width/2 - sample['inset']))
    except (KeyError, TypeError, ValueError):
        return None
    if width <= 0 or not all(math.isfinite(v) for v in (width, local_x, local_y)):
        return None
    return dict(sample, sensor='ultrasonic_left', sensor_x_mm=local_x, sensor_y_mm=local_y,
                map_eligible=sample.get('map_eligible', False))


def scan_points(sample, pose):
    """Nominal ToF midpoint at robot origin; telemetry yaw is positive RIGHT."""
    x, y, heading = (float(pose[key]) for key in ('x_mm', 'y_mm', 'heading_rad'))
    if not all(math.isfinite(v) for v in (x, y, heading)):
        raise ValueError('Invalid pose')
    if sample.get('sensor') == 'ultrasonic_left':
        if not sample['ok'] or not 20 <= sample['mm'] <= 4000:
            return []
        lx = sample['sensor_x_mm']
        ly = sample['sensor_y_mm'] + sample['mm']
        if not all(math.isfinite(v) for v in (lx, ly)):
            raise ValueError('Invalid side sensor geometry')
        return [dict(side='left', sensor='ultrasonic_left',
                     x_mm=x+lx*math.cos(heading)-ly*math.sin(heading),
                     y_mm=y+lx*math.sin(heading)+ly*math.cos(heading))]
    baseline = float(sample['baseline'])
    tilt = math.radians(float(sample['up_deg']))
    if not 0 <= baseline <= 500 or not abs(tilt) < math.pi/4:
        raise ValueError('Invalid nominal sensor geometry')
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


def map_wall_rotation(points):
    """Use a consistent early wall as a display axis, leaving raw poses intact."""
    points = [point for point in points if point.get('sensor') != 'ultrasonic_left']
    fits = [wall_candidate(points[start:start+40])
            for start in range(0, min(200, len(points)-39), 40)]
    corrections = [(-fit['tangent_deg'] + 45) % 90 - 45 for fit in fits if fit]
    if len(corrections) >= 3 and max(corrections) - min(corrections) <= 4:
        return sum(corrections) / len(corrections)
    return None


def replay(path):
    points, poses, events, errors = [], [], {}, 0
    segments = []
    outcomes, metadata = [], {}
    scan_buffer = ScanPoseBuffer()
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
                    scan_buffer.reset()
                elif row['event'] == 'tof':
                    points.extend(scan_buffer.scan(row['monotonic_ns'] / 1e9,
                                                   row['data']['sample'], row['data']['pose']))
                elif row['event'] == 'side':
                    sample = side_scan(row['data']['sample'], metadata.get('calibration', {}))
                    if sample is not None:
                        points.extend(scan_buffer.scan(row['monotonic_ns'] / 1e9, sample, row['data']['pose']))
                elif row['event'] == 'motor':
                    pose = row['data']['pose']
                    poses.append(pose)
                    points.extend(scan_buffer.motor(row['monotonic_ns'] / 1e9, pose))
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
