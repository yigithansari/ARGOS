"""Acceptance regressions without transport or motor access."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

import argos_acceptance as acceptance
import argos_control_panel as panel

FRAME = ("MOTOR | LEFT:0 RPWM:0 LPWM:0 ENC:-100 DENC:0 RPMx100:0 AB:0 | "
         "RIGHT:0 RPWM:0 LPWM:0 ENC:200 DENC:0 RPMx100:0 AB:0 | SAFE:STOP | "
         "CTRL:PI TARGET_RPMx100:+0/+0 RAMP_RPMx100:+0/+0 MEAS_RPMx100:+0/+0 T_MS:100")

class AcceptanceTests(unittest.TestCase):
    def test_real_frame_and_truncated_line(self):
        result = acceptance.motor_frame(FRAME)
        self.assertEqual(result["counts"], [-100, 200])
        self.assertEqual(result["pwm"], [0, 0, 0, 0])
        self.assertIsNone(acceptance.motor_frame(FRAME[8:]))
        self.assertIsNone(acceptance.motor_frame(FRAME.split("T_MS")[0]))

    def test_floor_speed_command_has_balance_without_range_guard(self):
        cfg = dict(wheel_diameter_mm=87, left_counts_per_wheel_rev=48097.2)
        self.assertEqual(acceptance.straight_command([0,0],cfg), "V 25 25")
        self.assertEqual(acceptance.straight_command([40,20],cfg), "V 23 27")
        self.assertEqual(acceptance.straight_command([20,40],cfg), "V 27 23")

    def test_external_boundary_and_nonfinite_rejected(self):
        self.assertTrue(acceptance.external_result("Straight 1000 mm", 950, 950)["external_pass"])
        self.assertFalse(acceptance.external_result("Straight 1000 mm", 925, 950)["external_pass"])
        self.assertTrue(acceptance.external_result("Left 90 deg", 85, 85)["external_pass"])
        with self.assertRaises(ValueError):
            acceptance.external_result("Straight 1000 mm", float("nan"), 1000)

    def test_telemetry_watchdog_and_distance_cutoff(self):
        run = acceptance.Acceptance.__new__(acceptance.Acceptance)
        run.test, run.trigger_tick = "Watchdog", 1000
        run.samples = []
        run.halt = Mock()
        run.observe(dict(tick=2180, safe="RUN", pwm=[500,0,500,0], rpm=[25,25]))
        self.assertEqual(run.watchdog_last, 1180)
        run.observe(dict(tick=2260, safe="TIMEOUT_STOP", pwm=[0]*4, rpm=[0,0]))
        self.assertEqual(run.watchdog_first, 1260)
        run.halt.assert_called_with("watchdog completed")
        run.test, run.origin = "Straight 1000 mm", [0,0]
        run.calibration = dict(wheel_diameter_mm=100, left_counts_per_wheel_rev=10000,
                               right_counts_per_wheel_rev=10000, left_forward_sign=-1,
                               right_forward_sign=1)
        run.observe(dict(tick=2000, safe="RUN", pwm=[500,0,500,0],
                         rpm=[25,25], counts=[-31300,31300]))
        run.halt.assert_called_with("distance cutoff")

    def test_watchdog_does_not_refresh_drive(self):
        run = acceptance.Acceptance.__new__(acceptance.Acceptance)
        run.app = Mock()
        run.app.serial_port.is_open = run.app.robot_ready = True
        run.running, run.phase, run.test = True, "run", "Watchdog"
        run.started = run.feedback = acceptance.time.monotonic()
        run.sent_at = run.started-1
        run.send = Mock()
        run.tick()
        run.send.assert_called_once_with("E")

    def test_stop_is_sent_even_when_log_write_fails(self):
        run = acceptance.Acceptance.__new__(acceptance.Acceptance)
        run.app = Mock()
        run.app.send_command.return_value = True
        run.write = Mock(side_effect=OSError("disk full"))
        run.sending = False
        run.send("x")
        run.app.send_command.assert_called_once_with("x")
        self.assertFalse(run.sending)

    def test_encoder_counter_wrap(self):
        cfg = dict(wheel_diameter_mm=100, left_counts_per_wheel_rev=10000,
                   right_counts_per_wheel_rev=10000, left_forward_sign=-1, right_forward_sign=1)
        distances = acceptance.travel([-2147483640,2147483640], [2147483640,-2147483640], cfg)
        self.assertGreater(min(distances), 0)
        self.assertLess(max(distances), 1)

    def test_panel_command_exclusion_and_stop(self):
        app = Mock()
        app.acceptance_run.running = True
        app.acceptance_run.sending = False
        self.assertFalse(panel.ArgosControlPanel.send_command(app, "V 45 45"))
        self.assertTrue(panel.ArgosControlPanel.send_command(app, "x"))
        app.acceptance_run.cancel.assert_called_once()

    def test_full_window_stop_and_external_result(self):
        with tempfile.TemporaryDirectory() as directory, \
                patch.object(panel.ArgosControlPanel, "after", return_value="suppressed"), \
                patch.object(panel.ArgosControlPanel, "refresh_ports"):
            app = panel.ArgosControlPanel()
            app.withdraw()
            app.update_idletasks()
            try:
                app.open_acceptance()
                run = app.acceptance_run
                run.directory = Path(directory)
                run.case.set("Straight 1000 mm")
                run.ready.set(True)
                app.serial_port = Mock(is_open=True)
                app.robot_ready = True
                app.motion_stop_confirmed = False
                app.link_mode_var.set("Wi-Fi")
                run.start()
                self.assertTrue(run.running)
                packets = [c.args[0] for c in app.serial_port.write.call_args_list]
                self.assertEqual(packets[-2:], [b"x\r\n", b"E\r\n"])
                self.assertFalse(any(b"V 25" in packet or b"G C " in packet for packet in packets))
                run.handle(FRAME)
                self.assertEqual(run.phase, "run")
                self.assertEqual(app.serial_port.write.call_args.args[0], b"V 25 25\r\n")
                self.assertFalse(run.result["range_guard"])
                run.handle("MOVE ERROR FRONT_INVALID")
                self.assertEqual(run.phase, "stopping")
                run.handle(FRAME)
                self.assertFalse(run.running)
                self.assertFalse(run.result["execution_pass"])
                self.assertTrue(json.loads(run.path.with_suffix(".json").read_text())["stop_confirmed"])
                run.case.set("Right 90 deg")
                run.ready.set(True)
                run.start()
                run.handle(FRAME)
                self.assertTrue(app.serial_port.write.call_args.args[0].startswith(b"G R "))
                run.handle("MOVE DONE | COUNTS:35000/35000")
                run.handle(FRAME)
                self.assertIsNotNone(run.pending)
                run.left.set("89")
                run.measure()
                self.assertTrue(run.result["external_pass"])
                self.assertIsNone(run.pending)
            finally:
                app.on_close()

if __name__ == "__main__":
    unittest.main()
