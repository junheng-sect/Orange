"""Bounded coarse-to-fine search on the same grid and yaw lattice as dense search."""
import numpy as np


def coarse_to_fine(model, x, y, rows, cols, stride, likelihood, px, py, angles, ordered_map):
    # One parent covers four fine cells. Interpolating their pixelwise maximum
    # bounds every child's bilinear score because interpolation weights are positive.
    cell = model.cfg['res']
    parent_width = (model.W + 2*stride - 1) // (2*stride)
    parent_ids = (rows // (2*stride)) * parent_width + cols // (2*stride)
    keys, parent = np.unique(parent_ids, return_inverse=True)
    by, bx = np.divmod(keys, parent_width)
    bx = bx * (2*stride)
    by = by * (2*stride)
    parent_x = model.ox + (bx + .5) * cell
    parent_y = model.oy + (by + .5) * cell
    # Include an outside-image halo: a parent's sample can be outside even
    # when the same wall point from a translated child lands on the boundary.
    padded = np.pad(likelihood, stride)
    upper = padded.copy()
    upper[:-stride, :] = np.maximum(upper[:-stride, :], padded[stride:, :])
    upper[:, :-stride] = np.maximum(upper[:, :-stride], upper[:, stride:])
    parent_x += stride * cell
    parent_y += stride * cell

    coarse = (rows % (2*stride) == 0) & (cols % (2*stride) == 0)
    if not coarse.any() or len(angles) < 3 or max(model.H, model.W) >= 16000:
        return None
    # Start inside 2 m, but obtain an actual lower bound from the whole map.
    origin = np.hypot(x, y) <= 2.
    coarse_idx = np.r_[np.flatnonzero(coarse & origin), np.flatnonzero(coarse & ~origin)]
    def coarse_peak(angle):
        return model.score(x[coarse_idx], y[coarse_idx], angle, px, py, likelihood).max()
    seed = max(ordered_map(coarse_peak, angles[::3]))
    cutoff = seed * model.cfg['candidate_ratio']

    def fine_peaks(angle):
        bound = model.score(parent_x, parent_y, angle, px, py, upper)
        # remap rounds to 1/32 pixel. Translating by an integer may change
        # float32 tie rounding by one bin per axis. Scores lie in [0,1].
        possible = bound[parent] + (2./32 + 1e-5) >= cutoff
        scores = np.full(len(x), -1., dtype='f4')
        # Keep the full-grid ordering for ties regardless of work order.
        for region in (possible & origin, possible & ~origin):
            if region.any():
                scores[region] = model.score(x[region], y[region], angle, px, py, likelihood)
        return spatial_peaks(scores, x, y, angle, cutoff)
    return [peak for group in ordered_map(fine_peaks, angles) for peak in group]


def spatial_peaks(scores, x, y, angle, cutoff=0.):
    peaks = []
    for _ in range(3):
        i = int(np.argmax(scores))
        if scores[i] < cutoff:
            break
        peaks.append((float(scores[i]), x[i], y[i], angle))
        scores[(x-x[i])**2 + (y-y[i])**2 < .5**2] = -1
    return peaks
