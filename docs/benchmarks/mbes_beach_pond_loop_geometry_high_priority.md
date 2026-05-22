# MBES Accepted Loop Geometry Review

- Source bag: `/tmp/aqua_mbes_beach_pond_strict_source_120`
- Source CSV: `/tmp/aqua_mbes_loop_benchmark_strict_source_120/mbes_beach_pond_loop_status.csv`
- Gate assumptions: fitness <= 2, translation <= 5 m, rotation <= 0.4 rad

## Summary

- Priority filter: high
- Accepted loops in CSV: 44
- Accepted loops with keyframe geometry: 14
- Accepted loops missing keyframe geometry: 0
- Keyframes loaded: 1058
- Loaded keyframe ID range: 0 -> 1057
- High / medium / low review rows: 14 / 0 / 0

## Review Table

| Rank | Priority | Candidate -> Current | Gap | Plan XY m | Depth delta m | Correction m | Rotation rad | Candidate xyz | Current xyz | Review focus |
|-----:|----------|----------------------|----:|----------:|--------------:|-------------:|-------------:|---------------|-------------|--------------|
| 1 | high | 530 -> 936 | 406 | 8.112 | 2.337 | 4.104 | 0.319 | 50.025, 17.44, 7.806 | 48.009, 25.298, 10.142 | high-risk gate margin; translation near gate, rotation near gate |
| 2 | high | 127 -> 227 | 100 | 7.964 | 1.948 | 3.565 | 0.201 | 2.643, -6.754, -0.81 | 2.924, -14.713, 1.138 | high-risk gate margin; fitness near gate |
| 3 | high | 515 -> 934 | 419 | 8.228 | 2.378 | 2.842 | 0.303 | 41.87, 14.714, 6.514 | 42.148, 22.937, 8.891 | high-risk gate margin; rotation near gate |
| 4 | high | 448 -> 890 | 442 | 5.305 | 0.765 | 4.185 | 0.337 | 2.672, 6.121, -0.186 | 7.339, 8.644, 0.578 | high-risk gate margin; translation near gate, rotation near gate |
| 5 | high | 479 -> 908 | 429 | 6.348 | 0.212 | 4.098 | 0.261 | 22.256, 11.649, 3.352 | 27.433, 15.322, 3.139 | high-risk gate margin; translation near gate |
| 6 | high | 16 -> 75 | 59 | 7.967 | 0.822 | 2.634 | 0.169 | -0.962, 3.422, 0.053 | -1.891, 11.335, 0.875 | high-risk gate margin; short keyframe gap |
| 7 | high | 26 -> 97 | 71 | 2.832 | 0.173 | 2.951 | 0.331 | -2.926, 9.257, -0.024 | -0.697, 7.51, 0.149 | high-risk gate margin; rotation near gate, short keyframe gap |
| 8 | high | 10 -> 109 | 99 | 2.927 | 0.633 | 2.505 | 0.382 | 0.018, 0.337, 0.07 | -0.247, 3.252, -0.563 | high-risk gate margin; rotation near gate |
| 9 | high | 61 -> 182 | 121 | 6.857 | 2.456 | 2.728 | 0.098 | -1.785, 11.602, 1.248 | 0.128, 18.187, -1.208 | high-risk gate margin |
| 10 | high | 79 -> 448 | 369 | 2.809 | 0.239 | 2.136 | 0.367 | 0.362, 4.523, -0.425 | 2.672, 6.121, -0.186 | high-risk gate margin; rotation near gate |
| 11 | high | 35 -> 332 | 297 | 2.943 | 0.094 | 2.319 | 0.274 | -1.516, 5.504, 0.039 | -2.972, 2.946, -0.056 | high-risk gate margin |
| 12 | high | 35 -> 93 | 58 | 1.477 | 0.279 | 2.152 | 0.314 | -1.516, 5.504, 0.039 | -0.041, 5.563, -0.24 | high-risk gate margin; rotation near gate, short keyframe gap; short plan-view edge |
| 13 | high | 43 -> 92 | 49 | 1.391 | 0.361 | 1.292 | 0.322 | -0.905, 4.139, 0.034 | 0.117, 5.083, -0.327 | high-risk gate margin; rotation near gate, short keyframe gap; short plan-view edge |
| 14 | high | 0 -> 43 | 43 | 4.237 | 0.034 | 1.167 | 0.216 | 0, 0, 0 | -0.905, 4.139, 0.034 | high-risk gate margin; short keyframe gap, low point-count ratio |

## Missing Keyframe Geometry

- None

## Interpretation

Use this worksheet with the RViz audit markers or plan-view PNG. A row is not a validated loop-closure claim until its marker edge has been checked against the replayed map geometry and marked as a plausible revisit.

