"""Shared, testable PCT terrain-grid coordinate and layer-selection semantics."""

import numpy as np


def pos2idx(pos, center, resolution, map_dim):
    """Return PCT [column, row] index, exactly as the legacy wrapper did."""
    relative = np.asarray(pos, dtype=np.float64) - np.asarray(center, dtype=np.float64)
    offset = np.array([int(map_dim[0] / 2), int(map_dim[1] / 2)], dtype=np.int32)
    index = np.round(relative / resolution).astype(np.int32) + offset
    return np.array([index[1], index[0]], dtype=np.float32)


def select_layer(trav, elev_g, pos, center, resolution, desired_ground_height,
                 max_height_error, threshold):
    """Return closest valid traversable [layer, ground-z], or None."""
    index = pos2idx(pos, center, resolution, elev_g.shape[1:])
    row, col = int(index[1]), int(index[0])
    if row < 0 or col < 0 or row >= elev_g.shape[1] or col >= elev_g.shape[2]:
        raise ValueError('Pose is outside tomogram bounds')
    # Keep the loop explicit: it mirrors the O(layers) C++ query hot path.
    candidates = []
    for layer in range(elev_g.shape[0]):
        height = elev_g[layer, row, col]
        if np.isfinite(height) and trav[layer, row, col] <= threshold:
            candidates.append((abs(float(height) - desired_ground_height), layer, float(height)))
    if not candidates:
        return None
    error, layer, height = min(candidates)
    return (layer, height) if error <= max_height_error else None
