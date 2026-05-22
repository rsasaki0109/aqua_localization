# MBES Loop Candidate Visual Audit

- Source CSV: `/tmp/aqua_mbes_loop_benchmark_strict_source_120/mbes_beach_pond_loop_status.csv`
- Priority filter: high
- Gate assumptions: fitness <= 2, translation <= 5 m, rotation <= 0.4 rad
- Keyframe gap warning: <= 80

## Summary

- Samples: 405
- Accepted loops: 44
- Rejected candidates: 233
- No-candidate statuses: 128
- Converged registrations: 155

## Accepted Loop Audit Priority

| Rank | Priority | Candidate -> Current | Gap | Fitness | Correction m | Rotation rad | Descriptor c/e/r | Flags | Audit note |
|-----:|----------|----------------------|----:|--------:|-------------:|-------------:|------------------|-------|------------|
| 1 | high | 530 -> 936 | 406 | 1.417 | 4.1036 | 0.3185 | 1.3027/1.2052/0.9455 | translation near gate, rotation near gate | TODO: inspect accepted marker geometry |
| 2 | high | 127 -> 227 | 100 | 1.5539 | 3.5653 | 0.2006 | 0.0518/1.1641/0.9459 | fitness near gate | TODO: inspect accepted marker geometry |
| 3 | high | 515 -> 934 | 419 | 1.0487 | 2.8421 | 0.3034 | 3.1848/1.0065/0.6773 | rotation near gate | TODO: inspect accepted marker geometry |
| 4 | high | 448 -> 890 | 442 | 0.0584 | 4.1854 | 0.337 | 0.3169/1.0169/0.7641 | translation near gate, rotation near gate | TODO: inspect accepted marker geometry |
| 5 | high | 479 -> 908 | 429 | 0.3905 | 4.0983 | 0.2607 | 0.3936/1.0733/0.9862 | translation near gate | TODO: inspect accepted marker geometry |
| 6 | high | 16 -> 75 | 59 | 1.3906 | 2.6337 | 0.1695 | 0.3245/1.0372/0.982 | short keyframe gap | TODO: inspect accepted marker geometry |
| 7 | high | 26 -> 97 | 71 | 0.267 | 2.9506 | 0.3307 | 0.0062/1.0033/0.9769 | rotation near gate, short keyframe gap | TODO: inspect accepted marker geometry |
| 8 | high | 10 -> 109 | 99 | 0.0356 | 2.5048 | 0.3824 | 0.2663/1.0213/0.96 | rotation near gate | TODO: inspect accepted marker geometry |
| 9 | high | 61 -> 182 | 121 | 1.3053 | 2.7276 | 0.0979 | 0.0848/1.1012/0.9732 | none | TODO: inspect accepted marker geometry |
| 10 | high | 79 -> 448 | 369 | 0.1731 | 2.1358 | 0.3669 | 0.1841/1.1515/0.7641 | rotation near gate | TODO: inspect accepted marker geometry |
| 11 | high | 35 -> 332 | 297 | 0.4131 | 2.3186 | 0.2736 | 0.449/1.0262/0.9353 | none | TODO: inspect accepted marker geometry |
| 12 | high | 35 -> 93 | 58 | 0.1056 | 2.1518 | 0.314 | 0.7526/1.1825/0.9181 | rotation near gate, short keyframe gap | TODO: inspect accepted marker geometry |
| 13 | high | 43 -> 92 | 49 | 0.0067 | 1.2919 | 0.322 | 0.045/1.0422/0.9738 | rotation near gate, short keyframe gap | TODO: inspect accepted marker geometry |
| 14 | high | 0 -> 43 | 43 | 0.256 | 1.1675 | 0.2156 | 0.7918/1.1707/0.4535 | short keyframe gap, low point-count ratio | TODO: inspect accepted marker geometry |

## Status Counts

| Status | Count |
|--------|------:|
| no candidate submaps | 128 |
| registration did not converge | 113 |
| duplicate loop suppressed | 66 |
| accepted | 44 |
| rotation correction exceeds gate | 30 |
| fitness score exceeds gate | 15 |
| descriptor gate rejected | 9 |

## Audit Rule

Mark an accepted loop as usable evidence only after its accepted RViz/rerun edge connects a plausible revisit, not an adjacent duplicate or an obvious registration jump. Keep the benchmark row labelled unaudited until every accepted loop above has a note.

