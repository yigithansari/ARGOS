"""Encoder-only differential-drive estimate. No hardware access or drive commands."""
import math


class WheelOdometry:
    def __init__(self):
        self.reset()

    def reset(self):
        self.x_mm = self.y_mm = self.heading_rad = self.path_mm = 0.0
        self.previous = None
        self.geometry = None

    def update(self, left, right, calibration, tick=None):
        geometry = tuple(float(calibration[key]) for key in (
            'wheel_circumference_mm', 'track_width_mm',
            'left_counts_per_wheel_rev', 'right_counts_per_wheel_rev',
            'left_forward_sign', 'right_forward_sign'))
        if (not all(math.isfinite(v) and v > 0 for v in geometry[:4]) or
                any(v not in (-1, 1) for v in geometry[4:])):
            raise ValueError('Valid wheel geometry and encoder calibration required')
        current = (int(left), int(right), tick)
        if self.geometry != geometry:
            self.reset()
            self.geometry = geometry
        if self.previous is not None:
            old_left, old_right, old_tick = self.previous
            # A normal 32-bit tick rollover advances; a backward jump is a reboot.
            if (tick is not None and old_tick is not None and
                    ((tick - old_tick) & 0xffffffff) > 0x7fffffff):
                self.reset()
                self.geometry = geometry
            else:
                circumference, track, left_cpr, right_cpr, left_sign, right_sign = geometry
                dl = ((int(left)-old_left+0x80000000) & 0xffffffff)-0x80000000
                dr = ((int(right)-old_right+0x80000000) & 0xffffffff)-0x80000000
                dl *= left_sign * circumference / left_cpr
                dr *= right_sign * circumference / right_cpr
                travel = (dl + dr) / 2
                turn = (dr - dl) / track
                half = turn / 2
                chord = travel * (math.sin(half)/half if abs(half) > 1e-9 else 1.0)
                self.x_mm += chord * math.cos(self.heading_rad + half)
                self.y_mm += chord * math.sin(self.heading_rad + half)
                self.heading_rad = (self.heading_rad + turn + math.pi) % (2*math.pi) - math.pi
                self.path_mm += abs(travel)
        self.previous = current
        return self.x_mm, self.y_mm, math.degrees(self.heading_rad), self.path_mm
