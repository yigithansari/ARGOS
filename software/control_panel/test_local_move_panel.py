"""Offline planner and panel safety lifecycle tests; no robot connection."""
import json
from pathlib import Path
from types import MethodType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import argos_control_panel as panel


CONFIG = json.loads(Path(__file__).with_name('argos_motion_calibration.json').read_text())
STOP = 'MOTOR | LEFT:0 RPWM:0 LPWM:0 ENC:0 | RIGHT:0 RPWM:0 LPWM:0 ENC:0 | SAFE:STOP'


def controller():
    obj = SimpleNamespace(
        motion_active=False, motion_timer=None, motion_terminal=None,
        motion_stop_confirmed=True, motion_stopping_at=None,
        motion_status_var=Mock(), drive_command_var=Mock(),
        firmware_update_active=False, system_test_active=False,
        lowband_test_active=False, drive_active=False, robot_ready=True,
        motion_calibration=CONFIG.copy(), pressed_drive_keys=set(),
        last_keyboard_drive=(0, 0), tx_count=0, serial_port=Mock(is_open=True),
        after=Mock(return_value='timer'), after_cancel=Mock(), _log=Mock(),
        disconnect_serial=Mock(), _reset_odometry=Mock(), _experiment_record=Mock(),
    )
    for name in ('_acceptance_busy', 'start_local_move', '_local_move_tick', '_finish_local_move',
                 '_handle_local_move_line', 'send_command'):
        setattr(obj, name, MethodType(getattr(panel.ArgosControlPanel, name), obj))
    return obj


class LocalMoveTests(unittest.TestCase):
    def test_geometry_and_direction_commands(self):
        for direction in ('F', 'B'):
            self.assertEqual(panel.plan_local_move(CONFIG, direction, 100), f'G {direction} 17597 17575')
        for direction in ('L', 'R'):
            command = panel.plan_local_move(CONFIG, direction, 90)
            self.assertEqual(command, f'G {direction} 35244 35199')
        changed = dict(CONFIG, wheel_diameter_mm=100, track_width_mm=300)
        self.assertNotEqual(panel.plan_local_move(CONFIG, 'L', 30), panel.plan_local_move(changed, 'L', 30))

    def test_invalid_requests_and_geometry_never_produce_command(self):
        for direction, value in (('F', 29), ('B', 151), ('L', 14), ('R', 91),
                                 ('X', 100), ('F', 'bad'), ('F', float('nan')), ('R', float('inf'))):
            with self.subTest(direction=direction, value=value), self.assertRaises(ValueError):
                panel.plan_local_move(CONFIG, direction, value)
        for config in (dict(CONFIG, left_forward_sign=1), dict(CONFIG, wheel_diameter_mm=0),
                       dict(CONFIG, left_counts_per_wheel_rev=float('nan')),
                       dict(CONFIG, track_width_mm=2000)):
            with self.assertRaises(ValueError):
                panel.plan_local_move(config, 'L', 90)

    @patch.object(panel.messagebox, 'showwarning')
    @patch.object(panel.messagebox, 'askokcancel', return_value=True)
    def test_start_keepalive_done_waits_for_zero_outputs(self, _confirm, _warning):
        obj = controller()
        self.assertTrue(obj.start_local_move('F', '100'))
        obj.serial_port.write.assert_called_with(b'G F 17597 17575\r\n')
        obj._handle_local_move_line('MOVE START | GOALS:17597/17575 RPM:15 DIR:F')
        obj._local_move_tick()
        obj.serial_port.write.assert_called_with(b'G KEEP\r\n')
        obj._handle_local_move_line('MOVE DONE | COUNTS:16718/16903')
        self.assertTrue(obj.motion_active)
        obj._handle_local_move_line('MOTOR | LEFT:1 RPWM:1 LPWM:0 RIGHT:0 RPWM:0 LPWM:0 SAFE:RUN')
        self.assertTrue(obj.motion_active)
        obj._handle_local_move_line(STOP)
        self.assertFalse(obj.motion_active)
        self.assertTrue(obj.motion_stop_confirmed)
        obj.after_cancel.assert_called()

    @patch.object(panel.messagebox, 'showwarning')
    @patch.object(panel.messagebox, 'askokcancel', return_value=True)
    def test_busy_unconfirmed_and_cancelled_start_do_not_move(self, confirm, _warning):
        for flag in ('motion_active', 'firmware_update_active', 'system_test_active',
                     'lowband_test_active', 'drive_active'):
            obj = controller()
            setattr(obj, flag, True)
            self.assertFalse(obj.start_local_move('F', 100))
            obj.serial_port.write.assert_not_called()
        obj = controller()
        obj.motion_stop_confirmed = False
        self.assertFalse(obj.start_local_move('F', 100))
        obj.serial_port.write.assert_called_once_with(b'E\r\n')
        obj = controller()
        confirm.return_value = False
        self.assertFalse(obj.start_local_move('F', 100))
        obj.serial_port.write.assert_not_called()

    def test_active_job_blocks_unrelated_commands_but_allows_stop(self):
        obj = controller()
        obj.motion_active = True
        for command in ('V 15 15', 'P 20 20', 'G B 17597 17575', 'S MPU 1', 'U ARGOS_OTA'):
            self.assertFalse(obj.send_command(command))
        obj.serial_port.write.assert_not_called()
        self.assertTrue(obj.send_command('x'))
        obj.serial_port.write.assert_called_once_with(b'x\r\n')

    def test_feedback_timeout_stops_and_cancels_timer(self):
        obj = controller()
        obj.motion_active = True
        obj.motion_started = obj.motion_feedback = 1
        with patch.object(panel.time, 'monotonic', return_value=3):
            obj._local_move_tick()
        self.assertFalse(obj.motion_active)
        self.assertFalse(obj.motion_stop_confirmed)
        obj.serial_port.write.assert_called_once_with(b'x\r\n')
        obj.after.assert_not_called()

    def test_abort_and_cancel_only_poll_stop(self):
        for terminal in ('MOVE ABORT STALL', 'MOVE CANCELLED', 'MOVE ERROR VALUES', 'MOVE IDLE'):
            obj = controller()
            obj.motion_active = True
            obj.motion_started = obj.motion_feedback = panel.time.monotonic()
            obj._handle_local_move_line(terminal)
            obj._local_move_tick()
            obj.serial_port.write.assert_called_with(b'E\r\n')
            self.assertNotIn(b'G KEEP\r\n', [call.args[0] for call in obj.serial_port.write.call_args_list])
            obj._handle_local_move_line(STOP)
            self.assertFalse(obj.motion_active)

    def test_disconnect_write_failure_does_not_recurse(self):
        obj = controller()
        obj.motion_active = True
        obj.motion_timer = 'timer'
        obj.connection_label = 'test'
        obj.serial_port.write.side_effect = OSError('link lost')
        transport = obj.serial_port
        for name in ('reader_alive', 'left_drive_var', 'right_drive_var', 'status_var',
                     'status_badge', 'bridge_health_var', 'wifi_rssi_var', '_append_connection_event'):
            setattr(obj, name, Mock())
        panel.ArgosControlPanel.disconnect_serial(obj)
        self.assertFalse(obj.motion_active)
        self.assertFalse(obj.motion_stop_confirmed)
        self.assertIsNone(obj.serial_port)
        transport.write.assert_called_once_with(b'x\r\n')
        transport.close.assert_called_once()
        obj.after_cancel.assert_called_once_with('timer')


if __name__ == '__main__':
    unittest.main(verbosity=2)
