"""No hardware or network: kinematics, restart handling and panel integration."""
import math
import json
from pathlib import Path
import unittest
from unittest.mock import Mock, patch

from argos_odometry import WheelOdometry
import argos_control_panel as panel


GEOMETRY = dict(wheel_circumference_mm=100, track_width_mm=200,
                left_counts_per_wheel_rev=1000, right_counts_per_wheel_rev=1000,
                left_forward_sign=-1, right_forward_sign=1)


class OdometryTests(unittest.TestCase):
    def test_nonzero_initial_counts_are_origin_and_reverse_returns(self):
        odo = WheelOdometry()
        self.assertEqual(odo.update(-5000, 5000, GEOMETRY, 100), (0, 0, 0, 0))
        self.assertEqual(odo.update(-6000, 6000, GEOMETRY, 200), (100, 0, 0, 100))
        x, y, heading, path = odo.update(-5000, 5000, GEOMETRY, 300)
        self.assertEqual((x, y, heading, path), (0, 0, 0, 200))

    def test_in_place_turn_and_then_forward(self):
        odo = WheelOdometry()
        odo.update(0, 0, GEOMETRY)
        x, y, angle, path = odo.update(1000, 1000, GEOMETRY)
        self.assertAlmostEqual(angle, math.degrees(1))
        self.assertEqual((x, y, path), (0, 0, 0))
        x, y, _, path = odo.update(0, 2000, GEOMETRY)
        self.assertAlmostEqual(x, 100*math.cos(1))
        self.assertAlmostEqual(y, 100*math.sin(1))
        self.assertEqual(path, 100)

    def test_arc_uses_exact_chord_and_duplicate_has_no_motion(self):
        odo = WheelOdometry()
        odo.update(0, 0, GEOMETRY)
        result = odo.update(0, 1000, GEOMETRY)
        self.assertAlmostEqual(result[0], 100*math.sin(.5))
        self.assertAlmostEqual(result[1], 100*(1-math.cos(.5)))
        self.assertEqual(odo.update(0, 1000, GEOMETRY), result)

    def test_signed_count_and_tick_rollover(self):
        odo = WheelOdometry()
        odo.update(-2147483640, 2147483640, GEOMETRY, 0xfffffff0)
        x, y, angle, path = odo.update(2147483640, -2147483640, GEOMETRY, 20)
        self.assertAlmostEqual(x, 1.6)
        self.assertEqual((y, angle), (0, 0))
        self.assertAlmostEqual(path, 1.6)

    def test_reboot_and_geometry_change_rebase_without_false_travel(self):
        odo = WheelOdometry()
        odo.update(-1000, 1000, GEOMETRY, 10000)
        odo.update(-2000, 2000, GEOMETRY, 11000)
        self.assertEqual(odo.update(0, 0, GEOMETRY, 100), (0, 0, 0, 0))
        changed = dict(GEOMETRY, track_width_mm=250)
        self.assertEqual(odo.update(-50000, 50000, changed, 200), (0, 0, 0, 0))

    def test_invalid_calibration_rejected(self):
        for key, value in (('track_width_mm', 0), ('wheel_circumference_mm', float('nan')),
                           ('left_counts_per_wheel_rev', -1), ('left_forward_sign', 0)):
            with self.assertRaises(ValueError):
                WheelOdometry().update(0, 0, dict(GEOMETRY, **{key: value}))

    def test_heading_wrap_remains_in_signed_degrees(self):
        odo = WheelOdometry()
        odo.update(0, 0, GEOMETRY)
        _, _, angle, _ = odo.update(10000, 10000, GEOMETRY)
        self.assertAlmostEqual(angle, math.degrees(10-4*math.pi))

    def test_panel_current_rpm_is_not_scaled_twice_and_reset_is_local(self):
        with patch.object(panel.ArgosControlPanel, 'after', return_value='suppressed'), \
                patch.object(panel.ArgosControlPanel, 'refresh_ports'):
            app = panel.ArgosControlPanel()
            try:
                app.withdraw()
                app._draw_motor_field = Mock()
                app.send_command = Mock()
                template = ('MOTOR | LEFT:54 RPWM:54 LPWM:0 ENC:{left} DENC:0 RPMx100:3500 AB:0 | '
                            'RIGHT:54 RPWM:54 LPWM:0 ENC:{right} DENC:0 RPMx100:3500 AB:0 | SAFE:STOP T_MS:{tick}')
                app._parse_motor_sample(template.format(left=0, right=0, tick=100))
                app._parse_motor_sample(template.format(left=-48097, right=48037, tick=200))
                self.assertEqual(app.motor_state['left_rpm'], 35)
                self.assertEqual(app.motor_state['right_rpm'], 35)
                self.assertGreater(app.odometry.x_mm, 270)
                app.motion_stop_confirmed = True
                app.reset_odometry_origin()
                self.assertEqual(app.odometry.x_mm, 0)
                app.send_command.assert_not_called()
                app.odometry.x_mm = 12
                app._update_metrics('Encoder sayaclari sifirlandi')
                self.assertEqual(app.odometry.x_mm, 0)
                app.motion_active = True
                app.odometry.x_mm = 12
                with patch.object(panel.messagebox, 'showwarning'):
                    app.reset_odometry_origin()
                self.assertEqual(app.odometry.x_mm, 12)
            finally:
                app.destroy()

    def test_recorded_floor_forward_replay(self):
        recorded = Path(__file__).resolve().parents[2] / 'tests/fixtures/odometry/floor-forward'
        expected = json.loads((recorded/'result.json').read_text())['final_encoder_mm']
        with patch.object(panel.ArgosControlPanel, 'after', return_value='suppressed'), \
                patch.object(panel.ArgosControlPanel, 'refresh_ports'):
            app = panel.ArgosControlPanel()
            try:
                app.withdraw()
                app._draw_motor_field = Mock()
                for line in (recorded/'telemetry.log').read_text().splitlines():
                    if 'MOTOR |' in line:
                        app._parse_motor_sample(line)
                self.assertAlmostEqual(app.odometry.x_mm, sum(expected)/2, delta=.2)
                self.assertLess(abs(app.odometry.y_mm), 2)
                self.assertLess(abs(math.degrees(app.odometry.heading_rad)), .2)
            finally:
                app.destroy()


if __name__ == '__main__':
    unittest.main(verbosity=2)
