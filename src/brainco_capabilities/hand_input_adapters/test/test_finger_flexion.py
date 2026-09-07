"""Regression checks using recorded glove endpoints and upstream conventions."""

import math
from pathlib import Path
import unittest

import yaml

from hand_input_adapters.finger_flexion import CalibratedFingerFlexion, JOINTS


ROOT = Path(__file__).resolve().parents[1]
OPEN = [208.19, 164.83, 192, 176.94, 198.84, 178.7,
        195.53, 180.98, 172.4, 199.41, 163.64, 190.06]
CLOSED = [279.58, 242.78, 260.55, 260.2, 272.95, 259.33,
          240.5, 294.03, 248.23, 247.62, 242.79, 265.13]
SIGNS = [1, 1, 1, 1, 1, 1, -1, 1, 1, -1, 1, 1]


def upstream(raw):
    return dict(zip(JOINTS, [math.radians((v - 180) * s) for v, s in zip(raw, SIGNS)]))


def load_calibration():
    params = yaml.safe_load((ROOT / 'config/humandex_right_calibrated.yaml').read_text())
    p = params['/humandex_hand_adapter']['ros__parameters']
    return CalibratedFingerFlexion(
        p['right_four_finger_open_rad'], p['right_four_finger_closed_rad'],
        p['four_finger_weights'], p['four_finger_flexion_range_rad'])


class FingerFlexionTest(unittest.TestCase):
    def setUp(self):
        self.calibration = load_calibration()

    def test_recorded_endpoints_and_monotonic_closing(self):
        for fraction in (0, 0.25, 0.5, 0.75, 1):
            raw = [o + fraction * (c - o) for o, c in zip(OPEN, CLOSED)]
            for value in self.calibration.compute(upstream(raw)).values():
                self.assertAlmostEqual(value, fraction * 1.4661, places=9)

    def test_each_bending_encoder_contributes_independently(self):
        for i in range(12):
            raw = OPEN.copy()
            raw[i] = CLOSED[i]
            result = list(self.calibration.compute(upstream(raw)).values())
            for finger, value in enumerate(result):
                self.assertAlmostEqual(value, 1.4661 / 3 if finger == i // 3 else 0, places=9)

    def test_outside_endpoints_clips_and_mpr_has_no_effect(self):
        for fraction, expected in ((-0.5, 0), (1.5, 1.4661)):
            measured = upstream([o + fraction * (c - o) for o, c in zip(OPEN, CLOSED)])
            measured.update(index_mpr=99, thumb_dip=-99)
            for value in self.calibration.compute(measured).values():
                self.assertAlmostEqual(value, expected)

    def test_missing_or_invalid_joint_rejected(self):
        measured = upstream(OPEN)
        del measured['ring_dip']
        with self.assertRaises(KeyError):
            self.calibration.compute(measured)
        measured['ring_dip'] = math.nan
        with self.assertRaises(ValueError):
            self.calibration.compute(measured)

    def test_invalid_calibration_rejected(self):
        for opened, closed, weights, span in (
            ([0.] * 11, [1.] * 12, [1.] * 3, 1.),
            ([0.] * 12, [0.] * 12, [1.] * 3, 1.),
            ([0.] * 12, [1.] * 12, [0.] * 3, 1.),
            ([0.] * 12, [1.] * 12, [1., -1., 1.], 1.),
            ([0.] * 12, [1.] * 12, [1.] * 3, math.nan),
        ):
            with self.assertRaises(ValueError):
                CalibratedFingerFlexion(opened, closed, weights, span)


if __name__ == '__main__':
    unittest.main()
