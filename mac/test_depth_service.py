"""Run: cd mac && .vision-venv/bin/python -m unittest test_depth_service   (numpy + cv2; no model)"""
import unittest

import numpy as np

import depth_service as ds


def floor_scene(h=240, w=320, horizon=100):
    rows = np.arange(h)[:, None].astype(float)
    disp = np.clip(rows - horizon, 0, None) * np.ones((1, w)) + 2.0   # flat floor + far walls
    disp[:horizon] = 2.0
    return disp


class AnalyseTest(unittest.TestCase):
    def test_clear_floor(self):
        reading, mask, free = ds.analyse(floor_scene())
        self.assertTrue(reading['ok'])
        self.assertAlmostEqual(reading['horizon_row'], 100, delta=3)
        self.assertEqual(reading['sectors']['centre']['cm'], ds.MAX_CM)
        self.assertEqual({s['state'] for s in reading['sectors'].values()}, {'clear'})
        self.assertFalse(mask.any())

    def test_box_in_the_centre_blocks_centre_only(self):
        disp = floor_scene()
        disp[120:200, 130:190] = 102.0          # standing up, base on the floor at row 200: ~28 cm
        reading, mask, _ = ds.analyse(disp)
        self.assertEqual(reading['sectors']['centre']['state'], 'blocked')
        self.assertEqual(reading['sectors']['left']['state'], 'clear')
        self.assertEqual(reading['sectors']['right']['state'], 'clear')
        self.assertTrue(mask[150, 160])
        self.assertAlmostEqual(reading['sectors']['centre']['cm'], 20 * 139 / 100, delta=4)

    def test_far_box_is_near_not_blocked(self):
        disp = floor_scene()
        disp[105:150, 220:300] = 52.0            # base at row 150: ~56 cm
        reading, _, _ = ds.analyse(disp)
        self.assertEqual(reading['sectors']['right']['state'], 'near')
        self.assertEqual(reading['sectors']['left']['state'], 'clear')

    def test_scale_and_shift_do_not_matter(self):
        disp = floor_scene()
        disp[120:200, 130:190] = 102.0
        a, _, _ = ds.analyse(disp)
        b, _, _ = ds.analyse(disp * 0.01 + 7.0)   # the model's units are arbitrary
        self.assertEqual(a['sectors'], b['sectors'])

    def test_no_floor_at_the_bottom(self):
        disp = np.ones((240, 320)) * 5.0          # a wall right in front: flat, no floor gradient
        reading, _, _ = ds.analyse(disp)
        self.assertFalse(reading['ok'])

    def test_smoothing_ignores_a_single_glitch(self):
        history = [{'left': 145, 'centre': 300, 'right': 95}] * 4 + [{'left': 24, 'centre': 300, 'right': 95}]
        out = ds.smooth(history)
        self.assertEqual(out['left'], {'cm': 145, 'state': 'clear'})
        out = ds.smooth([{'left': 20, 'centre': 300, 'right': 95}] * 3 + [{'left': 145, 'centre': 300, 'right': 95}] * 2)
        self.assertEqual(out['left']['state'], 'blocked')   # a real obstacle shows within 3 frames

    def test_overlay_shape(self):
        disp = floor_scene()
        reading, mask, free = ds.analyse(disp)
        frame = np.zeros((240, 320, 3), np.uint8)
        self.assertEqual(ds.overlay(frame, disp, mask, free, reading['horizon_row']).shape, (240, 640, 3))


if __name__ == '__main__':
    unittest.main()
