import unittest
from pathlib import Path

import numpy as np

from manus_revo2_retarget.revo2_joints import REVO2_JOINT_UPPER_LIMITS_RAD
from manus_revo2_retarget.retargeters import RetargeterRegistry


class TestRevo3ThumbRetargeter(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = (
            Path(__file__).parent.parent
            / "manus_revo2_retarget"
            / "brainco_hand"
            / "brainco.yml"
        )
        cls.rt = RetargeterRegistry.create("revo3_thumb", cls.config)

    def _new_retargeter(self):
        return RetargeterRegistry.create("revo3_thumb", self.config)

    def _make_tips(self, thumb_tip, spread=0.08):
        """Build a plausible 5-finger tip array."""
        return [
            list(thumb_tip),
            [-0.02, spread, 0.10],
            [0.0, spread, 0.10],
            [0.02, spread, 0.10],
            [0.04, spread, 0.09],
        ]

    def _make_pose(self, thumb_tip, thumb_pip, spread=0.08, noise=1e-6):
        return {
            "finger_tips": self._make_tips(thumb_tip, spread=spread),
            "thumb_tip": list(thumb_tip),
            "thumb_pip": list(thumb_pip),
            "thumb_tip_noise": noise,
            "thumb_pip_noise": noise,
        }

    def _make_standard4_poses(self):
        return {
            "open": self._make_pose([0.01, 0.06, 0.03], [0.007, 0.048, 0.022], spread=0.09),
            "rotate": self._make_pose([0.03, 0.08, 0.05], [0.020, 0.060, 0.030], spread=0.09),
            "pinch": self._make_pose([-0.01, 0.03, 0.06], [-0.005, 0.022, 0.042], spread=0.07),
            "flex": self._make_pose([-0.02, 0.01, 0.03], [-0.015, 0.008, 0.018], spread=0.06),
        }

    def test_output_range(self):
        """All 6 joint targets must lie in the configured rad range."""
        tips = self._make_tips([0.01, 0.05, 0.03])
        l, r = self.rt.retarget_process(tips, tips)
        upper = np.asarray(REVO2_JOINT_UPPER_LIMITS_RAD)
        self.assertTrue(np.all(l >= 0) and np.all(l <= upper))
        self.assertTrue(np.all(r >= 0) and np.all(r <= upper))

    def test_thumb_changes_with_tip(self):
        """Thumb joint targets should change when fingertip moves."""
        tips1 = self._make_tips([0.01, 0.05, 0.03])
        tips2 = self._make_tips([0.03, 0.07, 0.05])
        l1, _ = self.rt.retarget_process(tips1, tips1)
        l2, _ = self.rt.retarget_process(tips2, tips2)
        # thumb proximal (index 0) or metacarpal (index 1) should differ
        self.assertFalse(np.allclose(l1[:2], l2[:2]))

    def test_four_fingers_use_manus_ergonomics_directly(self):
        """Four-finger targets should follow Manus stretch ergonomics without Dex."""
        tips = self._make_tips([0.01, 0.05, 0.03])
        ergonomics = {
            "IndexMCPStretch": 0.20,
            "IndexPIPStretch": 0.40,
            "IndexDIPStretch": 0.10,
            "MiddleMCPStretch": 0.35,
            "MiddlePIPStretch": 0.50,
            "MiddleDIPStretch": 0.20,
            "RingMCPStretch": 0.50,
            "RingPIPStretch": 0.60,
            "RingDIPStretch": 0.30,
            "PinkyMCPStretch": 0.65,
            "PinkyPIPStretch": 0.70,
            "PinkyDIPStretch": 0.40,
        }
        _, right = self.rt.retarget_process(
            tips,
            tips,
            right_ergonomics=ergonomics,
        )

        self.assertTrue(np.all(right[2:] > 0))
        self.assertGreater(right[5], right[2])

    def test_four_finger_negative_stretch_stays_open(self):
        """Negative Manus stretch values should not close four-finger targets."""
        rt = self._new_retargeter()
        tips = self._make_tips([0.01, 0.05, 0.03])
        ergonomics = {
            f"{finger}{joint}Stretch": -30.0
            for finger in ("Index", "Middle", "Ring", "Pinky")
            for joint in ("MCP", "PIP", "DIP")
        }

        _, right = rt.retarget_process(
            tips,
            tips,
            right_ergonomics=ergonomics,
        )

        self.assertTrue(np.allclose(right[2:], 0.0))

    def test_ema_smoothing(self):
        """EMA should prevent a single outlier from jumping instantly."""
        tips1 = self._make_tips([0.01, 0.05, 0.03])
        tips2 = self._make_tips([0.10, 0.15, 0.10])
        l1, _ = self.rt.retarget_process(tips1, tips1)
        l2, _ = self.rt.retarget_process(tips2, tips2)
        delta = np.abs(l2[:2].astype(float) - l1[:2].astype(float))
        # with default EMA 0.1/0.9 the jump should still be damped
        self.assertTrue(np.all(delta < 1.4))

    def test_pip_constraint_affects_result(self):
        """Supplying a PIP position should change the thumb solution."""
        tips = self._make_tips([0.02, 0.06, 0.04])
        l_no_pip, _ = self.rt.retarget_process(tips, tips)
        l_with_pip, _ = self.rt.retarget_process(
            tips,
            tips,
            left_thumb_pip_pos=[0.01, 0.03, 0.02],
            right_thumb_pip_pos=[0.01, 0.03, 0.02],
        )
        self.assertFalse(np.allclose(l_no_pip[:2], l_with_pip[:2]))

    def test_thumb_proximal_uses_chain_bend_when_dip_is_available(self):
        """Thumb splay should not force proximal flexion when chain bend is unchanged."""
        rt_a = self._new_retargeter()
        rt_b = self._new_retargeter()

        pip_a = [0.005, 0.045, 0.025]
        dip_a = [0.008, 0.053, 0.028]
        tip_a = [0.011, 0.061, 0.031]
        shift = [-0.080, 0.000, 0.000]
        pip_b = (np.asarray(pip_a) + shift).tolist()
        dip_b = (np.asarray(dip_a) + shift).tolist()
        tip_b = (np.asarray(tip_a) + shift).tolist()

        tips_a = self._make_tips(tip_a)
        tips_b = self._make_tips(tip_b)
        left_a, _ = rt_a.retarget_process(
            tips_a,
            tips_a,
            left_thumb_dip_pos=dip_a,
            left_thumb_pip_pos=pip_a,
        )
        left_b, _ = rt_b.retarget_process(
            tips_b,
            tips_b,
            left_thumb_dip_pos=dip_b,
            left_thumb_pip_pos=pip_b,
        )

        self.assertGreater(abs(float(left_b[1]) - float(left_a[1])), 0.02)
        self.assertLess(abs(float(left_b[0]) - float(left_a[0])), 0.01)

    def test_solve_calibration_bounds_and_quality(self):
        """Calibrated params should satisfy configured bounds."""
        rt = self._new_retargeter()
        poses = self._make_standard4_poses()
        params, quality = rt.solve_calibration_for_side("left", poses)

        self.assertGreaterEqual(params["thumb_ik_position_scale"], 0.85)
        self.assertLessEqual(params["thumb_ik_position_scale"], 1.30)
        self.assertGreaterEqual(params["thumb_cmp_scale"], 0.5)
        self.assertLessEqual(params["thumb_cmp_scale"], 1.6)
        self.assertGreaterEqual(params["thumb_mcp_scale"], 0.5)
        self.assertLessEqual(params["thumb_mcp_scale"], 1.6)
        self.assertGreaterEqual(params["pip_constraint_weight"], 0.05)
        self.assertLessEqual(params["pip_constraint_weight"], 0.25)
        self.assertAlmostEqual(params["ema_prev"] + params["ema_cur"], 1.0, places=5)
        self.assertGreaterEqual(params["ema_cur"], 0.75)
        self.assertLessEqual(params["ema_cur"], 0.95)
        self.assertLess(quality["fit_rmse"], 1.0)

    def test_apply_calibration_changes_thumb_output(self):
        """Applying calibration should noticeably change thumb behavior."""
        rt = self._new_retargeter()
        tips = self._make_tips([0.02, 0.06, 0.04])
        l_before, _ = rt.retarget_process(tips, tips)

        poses = self._make_standard4_poses()
        params, _ = rt.solve_calibration_for_side("left", poses)
        rt.apply_calibration({"left": params})

        l_after, _ = rt.retarget_process(tips, tips)
        self.assertFalse(np.allclose(l_before[:2], l_after[:2]))
        upper = np.asarray(REVO2_JOINT_UPPER_LIMITS_RAD)
        self.assertTrue(np.all(l_after >= 0) and np.all(l_after <= upper))

    def test_output_calibration_does_not_feed_back_into_ik_state(self):
        """Output affine calibration must not overwrite the raw IK state."""
        tips = self._make_tips([0.02, 0.06, 0.04])

        baseline = self._new_retargeter()
        baseline_left, _ = baseline.retarget_process(tips, tips)
        baseline_q = baseline.left_q.copy()

        calibrated = self._new_retargeter()
        calibrated.apply_calibration(
            {
                "left": {
                    "thumb_joint_offset_deg": 20.0,
                    "thumb_cmp_scale": 1.2,
                    "thumb_mcp_scale": 0.7,
                    "thumb_cmp_offset_deg": 5.0,
                    "thumb_mcp_offset_deg": -3.0,
                }
            }
        )
        calibrated_left, _ = calibrated.retarget_process(tips, tips)

        self.assertFalse(np.allclose(baseline_left[:2], calibrated_left[:2]))
        np.testing.assert_allclose(calibrated.left_q, baseline_q, atol=1e-9)


if __name__ == "__main__":
    unittest.main()
