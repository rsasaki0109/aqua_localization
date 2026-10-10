"""Rigid metric-scale position scoring with integer-nanosecond correspondence."""
from bisect import bisect_left
import numpy as np


def correspond(stamps, positions, reference_stamps, reference_positions, max_gap_ns=100000000):
    if any(b <= a for a, b in zip(stamps, stamps[1:])) or any(b <= a for a, b in zip(reference_stamps, reference_stamps[1:])):
        raise ValueError('timestamps must be strictly increasing')
    matched, source, target, excluded = [], [], [], []
    for ns, point in zip(stamps, positions):
        i = bisect_left(reference_stamps, ns)
        if i < len(reference_stamps) and reference_stamps[i] == ns:
            expected = reference_positions[i]
        elif i == 0 or i == len(reference_stamps):
            excluded.append({'timestamp_ns': ns, 'reason': 'outside_reference_support'})
            continue
        elif reference_stamps[i] - reference_stamps[i-1] > max_gap_ns:
            excluded.append({'timestamp_ns': ns, 'reason': 'reference_interpolation_interval_exceeds_0.1_s'})
            continue
        else:
            alpha = (ns-reference_stamps[i-1]) / (reference_stamps[i]-reference_stamps[i-1])
            expected = (1-alpha)*reference_positions[i-1] + alpha*reference_positions[i]
        matched.append(ns); source.append(point); target.append(expected)
    return matched, np.array(source), np.array(target), excluded


def rigid_score(source, target):
    source, target = np.asarray(source, dtype=float), np.asarray(target, dtype=float)
    if source.shape != target.shape or source.ndim != 2 or source.shape[1] != 3 or len(source) < 3:
        raise ValueError('at least three paired positions required')
    if not np.isfinite(source).all() or not np.isfinite(target).all():
        raise ValueError('nonfinite metric input')
    center_s, center_t = source.mean(axis=0), target.mean(axis=0)
    a, b = source-center_s, target-center_t
    if np.linalg.matrix_rank(a) < 2 or np.linalg.matrix_rank(b) < 2:
        raise ValueError('degenerate rigid alignment; no unique rotation')
    u, singular, vt = np.linalg.svd(a.T @ b)
    sign = np.eye(3); sign[2, 2] = np.linalg.det(vt.T @ u.T)
    rotation = vt.T @ sign @ u.T
    translation = center_t - rotation @ center_s
    error = source @ rotation.T + translation - target
    norms = np.linalg.norm(error, axis=1)
    return {'count': len(source), 'rmse_m': float(np.sqrt(np.mean(norms**2))),
            'mean_m': float(norms.mean()), 'median_m': float(np.median(norms)),
            'maximum_m': float(norms.max()), 'rotation': rotation.tolist(),
            'translation_m': translation.tolist(), 'scale': 1.0}
