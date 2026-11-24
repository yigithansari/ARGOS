import json
import math
from pathlib import Path
import unittest
from argos_straight_probe import target_counts, snapshot, stopped


class StraightProbeTests(unittest.TestCase):
    def setUp(self):
        self.config=json.loads(Path(__file__).with_name('argos_motion_calibration.json').read_text())

    def test_counts_from_single_geometry_source(self):
        goals=target_counts(100,self.config)
        for i,side in enumerate(('left','right')):
            actual=goals[i]*math.pi*self.config['wheel_diameter_mm']/self.config[f'{side}_counts_per_wheel_rev']
            self.assertAlmostEqual(actual,100,delta=.01)

    def test_invalid_distance_rejected(self):
        for value in (0,29,151,float('nan'),float('inf')):
            with self.subTest(value=value),self.assertRaises(ValueError):
                target_counts(value,self.config)

    def test_changed_direction_requires_validation(self):
        self.config['left_forward_sign']=1
        with self.assertRaises(ValueError):
            target_counts(100,self.config)

    def test_stop_requires_both_channels_zero(self):
        line='MOTOR | LEFT:0 RPWM:0 LPWM:0 ENC:-100 | RIGHT:0 RPWM:0 LPWM:0 ENC:101 | SAFE:STOP'
        self.assertTrue(stopped([line]))
        self.assertFalse(stopped([line.replace('RIGHT:0 RPWM:0','RIGHT:0 RPWM:500')]))
        self.assertEqual(snapshot([line]),(-100,101))


if __name__=='__main__':
    unittest.main(verbosity=2)
