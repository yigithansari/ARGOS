"""Offline geometry migration/loading checks; never connects to hardware."""
import ast
import json
import math
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest

SOURCE = Path(__file__).with_name('argos_control_panel.py')
tree = ast.parse(SOURCE.read_text(encoding='utf-8-sig'))
names = {'normalize_motion_geometry', '_load_motion_calibration'}
env = dict(math=math, json=json, DEFAULT_WHEEL_CIRCUMFERENCE_MM=0.0,
           DEFAULT_TRACK_WIDTH_MM=0.0)
exec(compile(ast.Module(body=[n for n in ast.walk(tree)
                             if isinstance(n, ast.FunctionDef) and n.name in names],
                        type_ignores=[]), '<panel geometry>', 'exec'), env)
normalize = env['normalize_motion_geometry']


class GeometryTests(unittest.TestCase):
    def test_current_file(self):
        stored = json.loads(SOURCE.with_name('argos_motion_calibration.json').read_text())
        result = normalize(stored)
        self.assertAlmostEqual(result['wheel_circumference_mm'], stored['wheel_diameter_mm'] * math.pi)
        self.assertEqual(result['track_width_mm'], stored['track_width_mm'])
        self.assertEqual(result['left_counts_per_wheel_rev'], stored['left_counts_per_wheel_rev'])
        self.assertEqual(result['right_forward_sign'], stored['right_forward_sign'])

    def test_diameter_is_authoritative_and_input_not_mutated(self):
        stored = dict(wheel_diameter_mm=100, wheel_circumference_mm=260,
                      track_width_mm=300)
        self.assertAlmostEqual(normalize(stored)['wheel_circumference_mm'], 100*math.pi)
        self.assertEqual(stored['wheel_circumference_mm'], 260)

    def test_legacy_circumference(self):
        result = normalize(dict(wheel_circumference_mm=260, track_width_mm=198))
        self.assertAlmostEqual(result['wheel_diameter_mm'], 260/math.pi)
        self.assertAlmostEqual(result['wheel_circumference_mm'], 260)

    def test_invalid_geometry_rejected(self):
        for value in (0, -1, float('nan'), float('inf'), 'invalid'):
            for key in ('wheel_diameter_mm', 'track_width_mm'):
                stored = dict(wheel_diameter_mm=87, track_width_mm=255)
                stored[key] = value
                with self.subTest(key=key, value=value), self.assertRaises((ValueError, TypeError)):
                    normalize(stored)

    def test_loader_and_missing_invalid_fallback(self):
        with TemporaryDirectory() as directory:
            path = Path(directory)/'calibration.json'
            panel = SimpleNamespace(motion_calibration_path=path)
            load = env['_load_motion_calibration']
            self.assertEqual(load(panel)['wheel_circumference_mm'], 0)
            path.write_text(json.dumps(dict(wheel_diameter_mm=87,track_width_mm=255)))
            self.assertAlmostEqual(load(panel)['wheel_circumference_mm'], 87*math.pi)
            path.write_text('{invalid json')
            self.assertEqual(load(panel)['track_width_mm'], 0)
            path.write_text(json.dumps(dict(wheel_diameter_mm=-1,track_width_mm=255)))
            self.assertEqual(load(panel)['wheel_circumference_mm'], 0)


if __name__ == '__main__':
    unittest.main(verbosity=2)
