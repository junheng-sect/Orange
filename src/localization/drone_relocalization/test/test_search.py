"""Check the accelerated search against fine-grid scoring, including map borders."""
import unittest
import numpy as np

from drone_relocalization.matching import MapModel
from drone_relocalization.search import coarse_to_fine, spatial_peaks
from drone_relocalization.settings import load


class SearchTests(unittest.TestCase):
    def test_bound_retains_every_eligible_peak_and_original_ties(self):
        rng = np.random.RandomState(13)
        model = MapModel.__new__(MapModel)
        model.cfg = load()
        model.H, model.W = 47, 63
        model.ox, model.oy = -2.8, -1.3
        likelihood = np.float32(rng.uniform(size=(model.H, model.W)))
        res = model.cfg['res']
        # Subpixel ties, rays crossing all four edges, and nonuniform coverage.
        px, py = (rng.randint(-50, 51, (2, 70)) + .5/32) * res
        angles = np.radians(np.arange(-180, 180, 2))
        for stride in (1, 3):
            r, c = np.nonzero(rng.uniform(size=(model.H, model.W))[::stride, ::stride] > .25)
            r, c = r*stride, c*stride
            x = model.ox + (c+.5)*res
            y = model.oy + (r+.5)*res
            dense = []
            for angle in angles:
                dense.extend(spatial_peaks(model.score(x, y, angle, px, py, likelihood), x, y, angle))
            fast = coarse_to_fine(model, x, y, r, c, stride, likelihood, px, py, angles, map)
            cutoff = max(v[0] for v in dense) * model.cfg['candidate_ratio']
            self.assertEqual([p for p in dense if p[0] >= cutoff], [p for p in fast if p[0] >= cutoff])

    def test_identical_rooms_fall_back_to_dense_search(self):
        model = MapModel.__new__(MapModel)
        model.cfg = load()
        model.H, model.W = 26, 140
        model.ox, model.oy = -1., -.6
        model.cand = np.ones((model.H, model.W), dtype=bool)
        model.lik = np.ones((model.H, model.W), dtype='f4')
        model.configure_search(2)
        points = np.linspace(-.3,.3,121)
        fast = model.global2d(points, points)
        dense = model.global2d(points, points, dense=True)
        self.assertTrue(fast['fallback'])
        self.assertEqual(fast['peaks'], dense['peaks'])
