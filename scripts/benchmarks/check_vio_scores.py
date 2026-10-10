"""Independent linear time matcher and Horn alignment for new v3 metrics."""
import numpy as np


def independent_correspond(stamps, positions, reference_stamps, reference_positions, max_gap_ns=100000000):
    pairs, source, target = [], [], []
    j = 0
    for ns, point in zip(stamps, positions):
        while j < len(reference_stamps) and reference_stamps[j] < ns:
            j += 1
        if j < len(reference_stamps) and reference_stamps[j] == ns:
            value = np.array(reference_positions[j])
        elif j == 0 or j == len(reference_stamps):
            continue
        elif reference_stamps[j] - reference_stamps[j-1] > max_gap_ns:
            continue
        else:
            weight_right = (ns-reference_stamps[j-1]) / (reference_stamps[j]-reference_stamps[j-1])
            value = np.array(reference_positions[j-1]) + weight_right * (np.array(reference_positions[j])-reference_positions[j-1])
        pairs.append(ns); source.append(point); target.append(value)
    return pairs, np.array(source), np.array(target)


def horn_score(source, target):
    x, y = np.asarray(source), np.asarray(target)
    mx, my = np.mean(x, axis=0), np.mean(y, axis=0)
    s = sum((np.outer(a-mx, b-my) for a, b in zip(x, y)), np.zeros((3, 3)))
    trace = np.trace(s)
    delta = np.array([s[1,2]-s[2,1], s[2,0]-s[0,2], s[0,1]-s[1,0]])
    n = np.empty((4, 4)); n[0,0] = trace; n[0,1:] = delta; n[1:,0] = delta
    n[1:,1:] = s + s.T - np.eye(3)*trace
    values, vectors = np.linalg.eigh(n)
    w, a, b, c = vectors[:, np.argmax(values)]
    r = np.array([[1-2*(b*b+c*c), 2*(a*b-w*c), 2*(a*c+w*b)],
                  [2*(a*b+w*c), 1-2*(a*a+c*c), 2*(b*c-w*a)],
                  [2*(a*c-w*b), 2*(b*c+w*a), 1-2*(a*a+b*b)]])
    t = my - r @ mx
    residual = [np.linalg.norm(r @ point + t - truth) for point, truth in zip(x, y)]
    return {'rmse_m': float(np.sqrt(sum(v*v for v in residual)/len(residual))),
            'rotation': r.tolist(), 'translation_m': t.tolist(), 'count': len(residual), 'scale': 1.0}


def verify(primary, stamps, positions, reference_stamps, reference_positions):
    matched, source, target = independent_correspond(stamps, positions, reference_stamps, reference_positions)
    if primary['matched_timestamps_ns'] != matched:
        raise ValueError('independent correspondence disagreement')
    independent = horn_score(source, target)
    for key in ('rotation', 'translation_m', 'rmse_m'):
        if not np.allclose(primary['score'][key], independent[key], rtol=1e-8, atol=1e-8):
            raise ValueError(f'independent Horn disagreement: {key}')
    return independent
