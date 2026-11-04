"""Run: cd mac && .vision-venv/bin/python -m unittest test_depth_service   (numpy + cv2; no model)"""
import unittest

import numpy as np

import depth_service as ds


def floor_scene(h=240, w=320, horizon=None):
    horizon = int(ds.HORIZON_FRAC * h) if horizon is None else horizon          # matches the camera geometry
    rows = np.arange(h)[:, None].astype(float)
    disp = np.clip(rows - horizon, 0, None) * np.ones((1, w)) + 2.0   # flat floor + far walls
    disp[:horizon] = 2.0
    return disp


def foot_row(cm, h=240):
    """The image row where something standing `cm` ahead meets the floor."""
    hz = int(ds.HORIZON_FRAC * h)
    return int(round(hz + ds.K_PX_CM * h / 480 / cm))


def standing(disp, cm, cols, top=None):
    """An upright face `cm` away: one depth from its foot up (the floor's depth at its foot)."""
    r = foot_row(cm, disp.shape[0])
    hz = int(ds.HORIZON_FRAC * disp.shape[0])
    disp[(top or hz + 2):r, cols[0]:cols[1]] = r - hz + 2.0
    return r


class AnalyseTest(unittest.TestCase):
    def test_clear_floor(self):
        reading, mask, free = ds.analyse(floor_scene())
        self.assertTrue(reading['ok'])
        self.assertEqual(reading['horizon_row'], int(ds.HORIZON_FRAC * 240))
        self.assertEqual(reading['sectors']['centre']['cm'], ds.MAX_CM)
        self.assertEqual({s['state'] for s in reading['sectors'].values()}, {'clear'})
        self.assertFalse(mask.any())

    def test_box_in_the_centre_blocks_centre_only(self):
        disp = floor_scene()
        foot = standing(disp, 28, (130, 190))   # standing up, 28 cm ahead
        reading, mask, _ = ds.analyse(disp)
        self.assertEqual(reading['sectors']['centre']['state'], 'blocked')
        self.assertEqual(reading['sectors']['left']['state'], 'clear')
        self.assertEqual(reading['sectors']['right']['state'], 'clear')
        self.assertTrue(mask[(int(ds.HORIZON_FRAC * 240) + foot) // 2, 160])   # its upper part is flagged
        self.assertLess(abs(reading['sectors']['centre']['cm'] - 28), 8)

    def test_far_box_is_near_not_blocked(self):
        disp = floor_scene()
        standing(disp, 56, (220, 300))           # 56 cm ahead
        reading, _, _ = ds.analyse(disp)
        self.assertEqual(reading['sectors']['right']['state'], 'near')
        self.assertEqual(reading['sectors']['left']['state'], 'clear')

    def test_scale_and_shift_do_not_matter(self):
        disp = floor_scene()
        standing(disp, 28, (130, 190))
        a, _, _ = ds.analyse(disp)
        b, _, _ = ds.analyse(disp * 0.01 + 7.0)   # the model's units are arbitrary
        self.assertEqual(a['sectors'], b['sectors'])

    def test_blinds_that_look_far_still_block(self):
        disp = floor_scene()
        hz = int(ds.HORIZON_FRAC * 240)
        disp[hz + 8:foot_row(30), 100:220] = 2.0  # reads as far as the walls, but below the horizon
        reading, mask, _ = ds.analyse(disp)
        self.assertEqual(reading['sectors']['centre']['state'], 'blocked')
        self.assertEqual(reading['sectors']['left']['state'], 'clear')

    def test_geometry_matches_the_door_check(self):
        # after the rebuild, facing a door: this detector's floor end at rows 183 / 156 of 480 = 1.0 / 2.0 m
        hz = ds.HORIZON_FRAC * 480
        self.assertAlmostEqual(ds.floor_cm(183, 480, hz), 100, delta=6)
        self.assertAlmostEqual(ds.floor_cm(156, 480, hz), 200, delta=12)

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

    def test_one_frame_detection_does_not_stop(self):
        svc = ds.DepthService('', 336, 'cpu')
        import time as t
        svc.hits = {'person': [t.monotonic()]}
        self.assertEqual(svc.stop_for(), [])
        svc.hits['person'].append(t.monotonic())
        self.assertEqual(svc.stop_for(), ['person'])
        svc.hits = {'dog': [t.monotonic() - 5, t.monotonic() - 4]}
        self.assertEqual(svc.stop_for(), [])          # old sightings expire

    def test_overlay_shape(self):
        disp = floor_scene()
        reading, mask, free = ds.analyse(disp)
        frame = np.zeros((240, 320, 3), np.uint8)
        self.assertEqual(ds.overlay(frame, disp, mask, free, reading['horizon_row']).shape, (240, 640, 3))


if __name__ == '__main__':
    unittest.main()
