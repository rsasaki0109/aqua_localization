# MBES Accepted Loop Geometry Review

- Source bag: `/tmp/aqua_mbes_beach_pond_with_loop_status`
- Source CSV: `/tmp/aqua_mbes_loop_benchmark_gap40_gate_120/mbes_beach_pond_loop_status.csv`
- Gate assumptions: fitness <= 2, translation <= 5 m, rotation <= 0.4 rad

## Summary

- Accepted loops in CSV: 17
- Accepted loops with keyframe geometry: 16
- Accepted loops missing keyframe geometry: 1
- Keyframes loaded: 1172
- Loaded keyframe ID range: 0 -> 1171
- High / medium / low review rows: 5 / 6 / 5

## Review Table

| Rank | Priority | Candidate -> Current | Gap | Plan XY m | Depth delta m | Correction m | Rotation rad | Candidate xyz | Current xyz | Review focus |
|-----:|----------|----------------------|----:|----------:|--------------:|-------------:|-------------:|---------------|-------------|--------------|
| 1 | high | 396 -> 1108 | 712 | 161.189 | 0.09 | 4.866 | 0.178 | -136.733, 84.092, -0.114 | -52.856, -53.554, -0.024 | high-risk gate margin; translation near gate |
| 2 | high | 48 -> 709 | 661 | 24.816 | 0.121 | 3.938 | 0.109 | -1.785, -8.898, -0.11 | -19.901, 8.062, 0.011 | high-risk gate margin; translation near gate |
| 3 | high | 669 -> 985 | 316 | 151.697 | 0.071 | 2.555 | 0.38 | -19.852, -159.778, -0.108 | 35.622, -18.588, -0.038 | high-risk gate margin; rotation near gate |
| 4 | high | 357 -> 799 | 442 | 44.068 | 0.069 | 3.863 | 0.231 | -37.61, 28.993, -0.166 | -80.153, 17.501, -0.097 | high-risk gate margin; translation near gate |
| 5 | high | 0 -> 357 | 357 | 47.488 | 0.166 | 1.555 | 0.351 | 0, 0, 0 | -37.61, 28.993, -0.166 | high-risk gate margin; rotation near gate, low point-count ratio |
| 6 | medium | 376 -> 468 | 92 | 88.566 | 0.181 | 3.071 | 0.251 | 11.282, 49.965, -0.179 | -0.372, -37.831, 0.001 | geometry-only review |
| 7 | medium | 2 -> 219 | 217 | 27.837 | 0.106 | 1.534 | 0.281 | 0.131, -0.976, 0.039 | -25.454, -11.942, -0.067 | geometry-only review |
| 8 | medium | 499 -> 669 | 170 | 105.092 | 0.078 | 1.632 | 0.223 | -0.763, -56.434, -0.031 | -19.852, -159.778, -0.108 | geometry-only review |
| 9 | medium | 29 -> 501 | 472 | 48.047 | 0.064 | 1.622 | 0.19 | -1.935, -9.641, -0.093 | -0.788, -57.675, -0.03 | geometry-only review |
| 10 | medium | 14 -> 216 | 202 | 26.738 | 0.067 | 1.424 | 0.19 | 0.078, -1.07, 0.012 | -24.073, -12.543, -0.055 | geometry-only review |
| 11 | medium | 6 -> 359 | 353 | 43.402 | 0.2 | 1.02 | 0.183 | 0.201, -1.032, 0.02 | -32.295, 27.738, -0.18 | geometry-only review |
| 12 | low | 26 -> 667 | 641 | 152.977 | 0.038 | 0.678 | 0.199 | -1.214, -6.575, -0.083 | -20.585, -158.32, -0.121 | geometry-only review |
| 13 | low | 669 -> 788 | 119 | 187.289 | 0.008 | 1.433 | 0.133 | -19.852, -159.778, -0.108 | -62.181, 22.665, -0.101 | geometry-only review |
| 14 | low | 21 -> 499 | 478 | 51.878 | 0.015 | 1.036 | 0.161 | -0.797, -4.556, -0.015 | -0.763, -56.434, -0.031 | geometry-only review |
| 15 | low | 23 -> 218 | 195 | 25.394 | 0.022 | 1.083 | 0.15 | -0.873, -4.569, -0.054 | -25.07, -12.276, -0.076 | geometry-only review |
| 16 | low | 34 -> 671 | 637 | 160.584 | 0.051 | 0.46 | 0.11 | -0.152, -1.822, -0.146 | -16.798, -161.54, -0.095 | geometry-only review |

## Missing Keyframe Geometry

| Rank | Priority | Candidate -> Current | Missing side | Gap | Flags |
|-----:|----------|----------------------|--------------|----:|-------|
| 1 | high | 1105 -> 1231 | current | 126 | rotation near gate |

## Interpretation

Use this worksheet with the RViz audit markers or plan-view PNG. A row is not a validated loop-closure claim until its marker edge has been checked against the replayed map geometry and marked as a plausible revisit.

