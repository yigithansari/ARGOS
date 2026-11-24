"""Offline scan geometry, advice, wall fitting and durable experiment replay."""
import json
import time
import math
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import Mock, patch

from argos_exploration import (scan_points, wall_candidate, next_advice, ExperimentLog,
                               replay, exploration_step, ScanPoseBuffer, corner_step,
                               corner_can_exit, side_scan, side_approaching)
import argos_control_panel as panel


def sample(distance=1000, **updates):
    return dict(left_ok=True, right_ok=True, left_mm=distance, right_mm=distance,
                left_yaw=-15, right_yaw=15, up_deg=5, baseline=70, **updates)


POSE = dict(x_mm=0, y_mm=0, heading_rad=0, path_mm=0)


class ExplorationTests(unittest.TestCase):
    def test_side_scan_projects_with_heading_and_interpolated_pose(self):
        scan = side_scan(dict(ok=True, mm=200, inset=105), {'track_width_mm': 255})
        buffer = ScanPoseBuffer()
        pose = dict(POSE, heading_rad=math.pi/2)
        buffer.motor(1, pose)
        self.assertEqual(buffer.scan(1.25, scan, pose), [])
        point, = buffer.motor(1.5, dict(pose, x_mm=100))
        self.assertEqual(point['sensor'], 'ultrasonic_left')
        self.assertAlmostEqual(point['x_mm'], -172.5)
        self.assertAlmostEqual(point['y_mm'], 0)

    def test_moving_scans_use_bracketed_pose_and_discard_long_gaps(self):
        buffer = ScanPoseBuffer()
        buffer.motor(10, POSE)
        observation = dict(sample(), map_eligible=False)
        self.assertEqual(buffer.scan(10.25, observation, POSE), [])
        points = buffer.motor(10.5, dict(POSE, x_mm=100))
        stationary = scan_points(observation, POSE)
        self.assertEqual(len(points), 2)
        self.assertAlmostEqual(points[0]['x_mm'], stationary[0]['x_mm'] + 50)
        buffer.scan(11, observation, POSE)
        self.assertEqual(buffer.motor(13, POSE), [])
        buffer.scan(13.25, observation, POSE)
        buffer.reset()
        self.assertEqual(buffer.motor(13.5, POSE), [])

    def test_guarded_exploration_recovers_and_pivots_after_obstacle(self):
        with TemporaryDirectory(dir=Path(__file__).parent) as directory, \
                patch.object(panel.ArgosControlPanel, 'after', return_value='suppressed'), \
                patch.object(panel.ArgosControlPanel, 'after_cancel'), \
                patch.object(panel.ArgosControlPanel, 'refresh_ports'), \
                patch.object(panel.messagebox, 'askokcancel', side_effect=AssertionError('Unexpected prompt')), \
                patch.object(panel, 'ExperimentLog', side_effect=lambda _, meta: ExperimentLog(directory, meta)):
            app = panel.ArgosControlPanel()
            try:
                app.withdraw()
                app.update_idletasks()
                app.robot_ready = app.motion_stop_confirmed = True
                app.drive_active = True
                app.serial_port = Mock(is_open=True)
                app._start_exploration('offline test')
                self.assertTrue(app.explore_active)
                self.assertFalse(app.drive_active)
                self.assertEqual([c.args[0].strip() for c in app.serial_port.write.call_args_list],
                                 [b'x', b'S TOF 1', b'E'])
                app.serial_port.write.reset_mock()
                app.explore_after -= 0.1
                app.motion_stop_confirmed = True
                app.tof_samples.append(dict(sample(), left_ok=False, received_at=time.monotonic()))
                app._update_metrics('SIDE RANGE | LEFT:OK 280 mm | INSET:105 | GUARD:10')
                app._explore_tick()
                self.assertTrue(app.serial_port.write.called, app.experiment_advice.get())
                packet = app.serial_port.write.call_args.args[0].decode().split()
                self.assertEqual(packet, ['G', 'C', '25', '22'])
                app._explore_tick()
                self.assertEqual(app.serial_port.write.call_count, 1)
                app.motion_started -= 7
                app._local_move_tick()
                self.assertEqual(app.serial_port.write.call_args.args[0].strip(), b'G U 25 22')
                app.tof_samples.append(dict(sample(400), received_at=time.monotonic()))
                app.motion_feedback -= 1.1
                app._handle_local_move_line(
                    'MOTOR | LEFT:52 RPWM:520 LPWM:0 | RIGHT:49 RPWM:490 LPWM:0 | SAFE:RUN | CTRL:PI')
                app._local_move_tick()
                self.assertTrue(app.serial_port.write.call_args.args[0].startswith(b'G A Q '))
                app.odometry.heading_rad -= math.radians(72)
                app.motion_stop_confirmed = True
                app._finish_local_move('MOVE ABORT CURVE_FRONT_INVALID | COUNTS:0/0')
                app.explore_after -= 0.1
                app.tof_samples.append(dict(sample(900), received_at=time.monotonic()))
                app._update_metrics('SIDE RANGE | LEFT:OK 350 mm | INSET:105 | GUARD:10')
                app._explore_tick()
                self.assertTrue(app.serial_port.write.call_args.args[0].startswith(b'G C '))
                app.motion_stop_confirmed = True
                app._finish_local_move('MOVE ABORT FRONT_OBSTACLE | COUNTS:0/0')
                self.assertTrue(app.explore_active)
                app.explore_after -= 0.1
                app.serial_port.write.reset_mock()
                app.tof_samples.append(dict(sample(140), right_mm=240, received_at=time.monotonic()))
                app._update_metrics('SIDE RANGE | LEFT:YOK 0 mm | INSET:105 | GUARD:10')
                app._explore_tick()
                self.assertTrue(app.serial_port.write.call_args.args[0].startswith(b'G A B '))
                app.motion_stop_confirmed = True
                app._finish_local_move('MOVE DONE | COUNTS:0/0')
                app.explore_after -= 0.1
                app.tof_samples.append(dict(sample(195), right_mm=277, received_at=time.monotonic()))
                app._update_metrics('SIDE RANGE | LEFT:YOK 0 mm | INSET:105 | GUARD:10')
                app._explore_tick()
                command = panel.plan_local_move(app.motion_calibration, 'R', 30)
                self.assertEqual(app.serial_port.write.call_args.args[0].decode().strip(), 'G A H ' + command[4:])
                app.odometry.heading_rad -= math.radians(25)
                app.motion_stop_confirmed = True
                app._finish_local_move('MOVE ABORT STALL | COUNTS:0/0')
                app.explore_after -= 0.1
                app.tof_samples.append(dict(sample(230), received_at=time.monotonic()))
                app._update_metrics('SIDE RANGE | LEFT:YOK 0 mm | INSET:105 | GUARD:10')
                app._explore_tick()
                self.assertTrue(app.serial_port.write.call_args.args[0].startswith(b'G A B '))
                app.motion_stop_confirmed = True
                app._finish_local_move('MOVE DONE | COUNTS:0/0')
                app.explore_after -= 0.1
                app.tof_samples.append(dict(sample(230), received_at=time.monotonic()))
                app._update_metrics('SIDE RANGE | LEFT:YOK 0 mm | INSET:105 | GUARD:10')
                app._explore_tick()
                self.assertEqual(app.serial_port.write.call_args.args[0].decode().strip(), 'G A H ' + command[4:])
                app._finish_local_move('MOVE DONE | COUNTS:0/0')
                self.assertFalse(app.explore_active)
                self.assertTrue(app.experiment_last_path.with_suffix('.map.svg').exists())
            finally:
                app._stop_experiment()
                app.destroy()

    def test_traction_retry_needs_fresh_clearance_and_runs_once(self):
        with TemporaryDirectory(dir=Path(__file__).parent) as directory, \
                patch.object(panel.ArgosControlPanel, 'after', return_value='suppressed'), \
                patch.object(panel.ArgosControlPanel, 'after_cancel'), \
                patch.object(panel.ArgosControlPanel, 'refresh_ports'), \
                patch.object(panel, 'ExperimentLog', side_effect=lambda _, meta: ExperimentLog(directory, meta)):
            app = panel.ArgosControlPanel()
            try:
                app.withdraw()
                app.robot_ready = True
                app.serial_port = Mock(is_open=True)
                for right_open in (False, True):
                    app._start_exploration('traction test')
                    app.motion_stop_confirmed = True
                    app._update_metrics('SIDE RANGE | LEFT:OK 450 mm | INSET:105 | GUARD:11')
                    self.assertTrue(app.start_local_move('F', 50, automatic=True))
                    app.motion_stop_confirmed = True
                    app._finish_local_move('MOVE ABORT STALL | COUNTS:0/0')
                    app.serial_port.write.reset_mock()
                    app._explore_tick()
                    app.serial_port.write.assert_not_called()
                    app.explore_after -= 0.1
                    app.tof_samples.append(dict(sample(), right_ok=right_open, received_at=time.monotonic()))
                    app._update_metrics('SIDE RANGE | LEFT:OK 450 mm | INSET:105 | GUARD:11')
                    app._explore_tick()
                    if not right_open:
                        app.serial_port.write.assert_not_called()
                        self.assertFalse(app.explore_active)
                        continue
                    self.assertTrue(app.serial_port.write.call_args.args[0].startswith(b'G A T '))
                    app.motion_stop_confirmed = True
                    app._finish_local_move('MOVE DONE | COUNTS:7000/7000')
                    self.assertTrue(app.start_local_move('F', 50, automatic=True))
                    app.motion_stop_confirmed = True
                    app._finish_local_move('MOVE ABORT STALL | COUNTS:0/0')
                    self.assertFalse(app.explore_active)
            finally:
                app._stop_experiment()
                app.destroy()

    def test_one_close_or_missing_front_sensor_does_not_stop_exploration(self):
        self.assertEqual(exploration_step(dict(sample(), left_mm=90), None), ('F', 50, 'R'))
        self.assertEqual(exploration_step(dict(sample(), left_ok=False), None), ('F', 50, None))
        self.assertEqual(exploration_step(sample(200), None), ('R', 90, None))
        self.assertEqual(exploration_step(sample(400), None), ('Q', 90, None))
        self.assertEqual(exploration_step(dict(sample(540), right_mm=600), None), ('Q', 90, None))
        self.assertEqual(exploration_step(dict(sample(540), right_mm=601), None), ('F', 50, None))
        self.assertEqual(exploration_step(dict(sample(400), right_mm=750), None), ('F', 50, None))
        self.assertIsNone(exploration_step(dict(sample(), left_ok=False, right_ok=False), 250))
        weak = dict(sample(), left_ok=False, right_ok=False, left_raw=4, right_raw=4)
        self.assertEqual(exploration_step(weak, 250), ('S', 100, None))
        self.assertIsNone(exploration_step(weak, 109))
        self.assertIsNone(exploration_step(dict(weak, right_raw=254), 250))
        self.assertEqual(exploration_step(dict(weak, left_raw=9, right_raw=6), 250), ('S', 100, None))
        self.assertEqual(exploration_step(sample(), 500), ('F', 50, None))
        self.assertEqual(exploration_step(sample(), 330), ('F', 50, None))
        self.assertEqual(exploration_step(sample(), 330, side_closing=True), ('F', 50, 'R'))
        self.assertEqual(exploration_step(sample(), 420, side_closing=True), ('F', 50, 'R'))
        self.assertEqual(exploration_step(sample(550), 240, allow_curve=False), ('F', 50, 'R'))
        self.assertEqual(exploration_step(dict(sample(407), right_mm=807), 320), ('F', 50, 'R'))
        self.assertEqual(exploration_step(dict(sample(140), right_mm=700), 225), ('R', 90, None))

    def test_open_space_keeps_moving_without_wall(self):
        weak = dict(sample(), left_ok=False, right_ok=False, left_raw=8, right_raw=8)
        self.assertEqual(exploration_step(weak, None, open_space=True), ('S', 60, None))
        self.assertIsNone(exploration_step(dict(weak, right_raw=255), None, open_space=True))
        self.assertIsNone(exploration_step(weak, 80, open_space=True))

    def test_wall_follow_tracks_receding_left_and_requires_consistent_single_front(self):
        self.assertEqual(exploration_step(sample(), 360, side_receding=True), ('F', 50, 'L'))
        self.assertEqual(exploration_step(sample(), 290, side_receding=True), ('F', 50, None))
        right_wall = dict(sample(), right_mm=500, received_at=10.0)
        self.assertEqual(exploration_step(right_wall, None), ('F', 50, None))
        earlier = dict(right_wall, right_mm=530, received_at=9.8)
        self.assertEqual(exploration_step(right_wall, None, previous_front=earlier), ('Q', 90, None))
        self.assertEqual(exploration_step(right_wall, 330, previous_front=earlier), ('F', 50, None))
        self.assertEqual(exploration_step(right_wall, None, previous_front=dict(earlier, received_at=9.0)),
                         ('F', 50, None))

    def test_side_approaching_requires_consistent_recent_echoes(self):
        earlier = [(9.1, 440), (9.4, 435)]
        self.assertTrue(side_approaching(earlier + [(9.7, 400), (9.9, 405)], 10))
        self.assertFalse(side_approaching(earlier + [(9.7, 440), (9.9, 365)], 10))
        self.assertFalse(side_approaching(earlier + [(9.7, 400), (9.9, 405)], 11))
        self.assertEqual(corner_step(sample(170), 90, True), ('B', 30, None))
        self.assertIsNone(corner_step(dict(sample(210), left_ok=False), 90, True))
        self.assertEqual(corner_step(sample(220), 90, False, side_mm=79), ('B', 30, None))
        self.assertEqual(corner_step(dict(sample(244), right_mm=42), 90, True, side_mm=107),
                         ('H', 30, None))
        self.assertEqual(corner_step(dict(sample(244), right_ok=False), 90, True, side_mm=107),
                         ('H', 30, None))
        self.assertEqual(corner_step(dict(sample(144), right_mm=242), 90, True, side_mm=107),
                         ('B', 56, None))
        self.assertEqual(corner_step(sample(110), 90, False), ('B', 90, None))
        near_parallel = dict(sample(280), right_ok=False, right_raw=9)
        self.assertTrue(corner_can_exit(near_parallel, 5, 55))
        self.assertFalse(corner_can_exit(near_parallel, 20, 55))
        self.assertFalse(corner_can_exit(dict(near_parallel, right_ok=True, right_mm=100), 5, 55))

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

    def test_gui_records_and_exports_without_transmitting(self):
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
                app.motion_stop_confirmed = True
                app._parse_tof_sample('TOF RANGE | LEFT:OK 1000 mm | RIGHT:OK 1000 mm | YAW L:-15 R:15 | UP:5 | BASE:70')
                self.assertEqual(len(app.experiment_points), 2)
                app._parse_motor_sample(frame+'100')  # reboot must not glue map origins
                self.assertEqual(len(app.experiment_points), 0)
                app._stop_experiment()
                self.assertEqual(replay(path)['events']['tof'], 1)
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
