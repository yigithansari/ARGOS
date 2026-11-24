"""Offline scan geometry, advice, wall fitting and durable experiment replay."""
import json
import math
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import Mock, patch

from argos_exploration import scan_points, wall_candidate, next_advice, ExperimentLog, replay
import argos_control_panel as panel


def sample(distance=1000, **updates):
    return dict(left_ok=True, right_ok=True, left_mm=distance, right_mm=distance,
                left_yaw=-15, right_yaw=15, up_deg=5, baseline=70, **updates)


POSE = dict(x_mm=0, y_mm=0, heading_rad=0, path_mm=0)


class ExplorationTests(unittest.TestCase):
    def test_yaw_sign_sensor_offsets_and_pose_transform(self):
        points = scan_points(sample(), POSE)
        self.assertGreater(points[0]['y_mm'], 35)
        self.assertLess(points[1]['y_mm'], -35)
        self.assertAlmostEqual(points[0]['x_mm'], points[1]['x_mm'])
        rotated = scan_points(sample(), dict(POSE, x_mm=100, heading_rad=math.pi/2))
        self.assertAlmostEqual(rotated[0]['x_mm'], 100-points[0]['y_mm'])
        self.assertAlmostEqual(rotated[0]['y_mm'], points[0]['x_mm'])

    def test_invalid_and_stale_inputs_hold(self):
        for data, age, stopped in ((None, 0, True), (sample(), 2, True),
                                   (sample(), 0, False), (sample(), -1, True),
                                   (dict(sample(), left_ok=False), 0, True),
                                   (dict(sample(), right_mm=0), 0, True)):
            self.assertIsNone(next_advice(data, age, stopped)['action'])

    def test_approach_blocked_scan_and_optional_side_tracking(self):
        self.assertEqual(next_advice(sample(), 0, True)['action'], ['F', 30])
        self.assertEqual(next_advice(sample(300), 0, True)['state'], 'SCAN_WALL')
        self.assertIsNone(next_advice(sample(100), 0, True)['action'])
        for side, action in ((250, ['F',30]), (175, ['R',15]), (400, ['L',15]),
                             (700, None), (100, None), (float('nan'), None)):
            self.assertEqual(next_advice(sample(), 0, True, side)['action'], action)

    def test_wall_fit_rejects_duplicate_points_and_handles_outlier(self):
        self.assertIsNone(wall_candidate([dict(x_mm=1000, y_mm=0)]*10))
        points = [dict(x_mm=1000+(i%2), y_mm=i*40) for i in range(10)]
        points += [dict(x_mm=300, y_mm=300)]
        wall = wall_candidate(points)
        self.assertIsNotNone(wall)
        self.assertEqual(wall['inliers'], 10)
        self.assertLess(wall['residual_mm'], 1)
        self.assertAlmostEqual(abs(wall['tangent_deg']), 90, delta=1)

    def test_log_flush_replay_segments_and_truncated_tail(self):
        with TemporaryDirectory(dir=Path(__file__).parent) as directory:
            log = ExperimentLog(directory, dict(calibration='test', advisory_only=True))
            log.record('tx', command='G F 6000 6000')
            log.record('tof', sample=sample(), pose=POSE)
            self.assertEqual(replay(log.path)['events']['tof'], 1)  # flushed before close
            log.record('origin_reset', reason='reconnect')
            log.record('motor', motor={}, pose=POSE)
            log.record('tof', sample=sample(500), pose=POSE)
            log.close()
            log.close()
            with log.path.open('a') as out:
                out.write('{truncated')
            result = replay(log.path)
            self.assertEqual(result['events']['tx'], 1)
            self.assertEqual(result['malformed_records'], 1)
            self.assertEqual(len(result['segments']), 2)
            self.assertEqual(len(result['points']), 2)
            self.assertLess(result['points'][0]['x_mm'], 500)
            rows = [json.loads(line) for line in log.path.read_text().splitlines()[:-1]]
            self.assertEqual([r['sequence'] for r in rows], list(range(len(rows))))

    def test_gui_records_and_suggests_without_transmitting(self):
        with TemporaryDirectory(dir=Path(__file__).parent) as directory, \
                patch.object(panel.ArgosControlPanel, 'after', return_value='suppressed'), \
                patch.object(panel.ArgosControlPanel, 'refresh_ports'):
            app = panel.ArgosControlPanel()
            try:
                app.withdraw()
                app.send_command = Mock()
                app.open_scan_experiment()
                app.update_idletasks()
                app.experiment_log = ExperimentLog(directory, dict(advisory_only=True))
                path = app.experiment_log.path
                app.serial_port = Mock(is_open=True)
                self.assertTrue(panel.ArgosControlPanel.send_command(app, 'E'))
                frame = ('MOTOR | LEFT:0 RPWM:0 LPWM:0 ENC:0 DENC:0 RPMx100:0 AB:0 | '
                         'RIGHT:0 RPWM:0 LPWM:0 ENC:0 DENC:0 RPMx100:0 AB:0 | SAFE:STOP T_MS:')
                app._parse_motor_sample(frame+'10000')
                app._parse_tof_sample('TOF RANGE | LEFT:OK 1000 mm | RIGHT:OK 1000 mm | YAW L:-15 R:15 | UP:5 | BASE:70')
                app.motion_stop_confirmed = True
                app._scan_advice()
                self.assertIn('APPROACH', app.experiment_advice.get())
                self.assertEqual(len(app.experiment_points), 2)
                app._parse_motor_sample(frame+'100')  # reboot must not glue map origins
                self.assertEqual(len(app.experiment_points), 0)
                app._stop_experiment()
                self.assertEqual(replay(path)['events']['advice'], 1)
                self.assertEqual(replay(path)['events']['tx'], 1)
                self.assertEqual(replay(path)['points'], [])
                app.send_command.assert_not_called()
                self.assertLessEqual(app.experiment_window.winfo_reqheight(), 720)
                app.experiment_window.destroy()
            finally:
                app._stop_experiment()
                app.destroy()

    def test_log_failure_is_visible_and_does_not_transmit(self):
        with TemporaryDirectory(dir=Path(__file__).parent) as directory, \
                patch.object(panel.ArgosControlPanel, 'after', return_value='suppressed'), \
                patch.object(panel.ArgosControlPanel, 'refresh_ports'):
            app = panel.ArgosControlPanel()
            try:
                app.withdraw()
                app.send_command = Mock()
                app.experiment_log = ExperimentLog(directory, {})
                file = app.experiment_log.file
                app.experiment_log.record = Mock(side_effect=OSError('disk failure'))
                app._experiment_record('rx', text='test')
                self.assertIsNone(app.experiment_log)
                self.assertTrue(file.closed)
                self.assertIn('Recording failed', app.experiment_status.get())
                app.send_command.assert_not_called()
            finally:
                app.destroy()


if __name__ == '__main__':
    unittest.main(verbosity=2)
