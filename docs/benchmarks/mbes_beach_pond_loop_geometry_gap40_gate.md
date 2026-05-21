# MBES Accepted Loop Geometry Review

- Source bag: `/tmp/aqua_mbes_beach_pond_strict_source_120`
- Source CSV: `/tmp/aqua_mbes_loop_benchmark_strict_source_120/mbes_beach_pond_loop_status.csv`
- Gate assumptions: fitness <= 2, translation <= 5 m, rotation <= 0.4 rad

## Summary

- Accepted loops in CSV: 44
- Accepted loops with keyframe geometry: 44
- Accepted loops missing keyframe geometry: 0
- Keyframes loaded: 1058
- Loaded keyframe ID range: 0 -> 1057
- High / medium / low review rows: 14 / 25 / 5

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
| 15 | medium | 456 -> 889 | 433 | 1.954 | 1.092 | 1.531 | 0.372 | 7.096, 6.526, -0.539 | 6.325, 8.321, 0.552 | rotation near gate; short plan-view edge |
| 16 | medium | 16 -> 59 | 43 | 7.215 | 0.999 | 1.909 | 0.114 | -0.962, 3.422, 0.053 | -1.526, 10.615, 1.052 | short keyframe gap |
| 17 | medium | 7 -> 125 | 118 | 5.04 | 0.669 | 2.34 | 0.213 | 0.367, -1.148, -0.057 | 2.424, -5.749, -0.725 | geometry-only review |
| 18 | medium | 26 -> 91 | 65 | 1.837 | 0.462 | 2.481 | 0.278 | -2.926, 9.257, -0.024 | -1.095, 9.109, 0.438 | short keyframe gap; short plan-view edge |
| 19 | medium | 16 -> 646 | 630 | 0.704 | 0.391 | 1.514 | 0.349 | -0.962, 3.422, 0.053 | -0.715, 4.082, -0.338 | rotation near gate; short plan-view edge |
| 20 | medium | 2 -> 334 | 332 | 3.91 | 0.051 | 3 | 0.176 | -0.709, -0.418, -0.138 | -3.653, 2.156, -0.087 | geometry-only review |
| 21 | medium | 43 -> 645 | 602 | 0.566 | 0.368 | 0.885 | 0.382 | -0.905, 4.139, 0.034 | -0.521, 4.554, -0.334 | rotation near gate; short plan-view edge |
| 22 | medium | 16 -> 95 | 79 | 3.165 | 0.104 | 2.028 | 0.286 | -0.962, 3.422, 0.053 | -0.363, 6.53, -0.051 | short keyframe gap |
| 23 | medium | 43 -> 329 | 286 | 0.969 | 0.054 | 1.997 | 0.27 | -0.905, 4.139, 0.034 | -1.874, 4.15, -0.02 | short plan-view edge |
| 24 | medium | 7 -> 118 | 111 | 1.655 | 0.415 | 2.548 | 0.212 | 0.367, -1.148, -0.057 | 1.611, -2.239, -0.472 | short plan-view edge |
| 25 | medium | 95 -> 773 | 678 | 0.758 | 0.387 | 1.429 | 0.298 | -0.363, 6.53, -0.051 | 0.287, 6.92, 0.336 | short plan-view edge |
| 26 | medium | 16 -> 331 | 315 | 1.656 | 0.094 | 2.089 | 0.247 | -0.962, 3.422, 0.053 | -2.616, 3.347, -0.041 | short plan-view edge |
| 27 | medium | 75 -> 174 | 99 | 3.316 | 1.86 | 2.823 | 0.151 | -1.891, 11.335, 0.875 | 0.104, 13.984, -0.985 | geometry-only review |
| 28 | medium | 91 -> 167 | 76 | 1.753 | 1.228 | 2.081 | 0.236 | -1.095, 9.109, 0.438 | 0.231, 10.257, -0.79 | short keyframe gap; short plan-view edge |
| 29 | medium | 36 -> 196 | 160 | 2.088 | 0.012 | 2.112 | 0.237 | -1.676, 5.993, 0.029 | -0.098, 7.361, 0.041 | geometry-only review |
| 30 | medium | 74 -> 168 | 94 | 1.944 | 1.593 | 2.68 | 0.173 | -1.732, 10.828, 0.776 | 0.212, 10.784, -0.816 | short plan-view edge |
| 31 | medium | 38 -> 79 | 41 | 1.58 | 0.449 | 1.638 | 0.229 | -0.592, 3.263, 0.024 | 0.362, 4.523, -0.425 | short keyframe gap; short plan-view edge |
| 32 | medium | 773 -> 887 | 114 | 0.579 | 0.07 | 1.428 | 0.232 | 0.287, 6.92, 0.336 | 0.632, 7.385, 0.266 | low point-count ratio; short plan-view edge |
| 33 | medium | 27 -> 74 | 47 | 1.739 | 0.805 | 1.948 | 0.199 | -3.098, 9.751, -0.029 | -1.732, 10.828, 0.776 | short keyframe gap; short plan-view edge |
| 34 | medium | 61 -> 172 | 111 | 2.311 | 2.172 | 2.601 | 0.138 | -1.785, 11.602, 1.248 | 0.143, 12.876, -0.924 | geometry-only review |
| 35 | medium | 95 -> 885 | 790 | 0.299 | 0.266 | 0.548 | 0.299 | -0.363, 6.53, -0.051 | -0.636, 6.652, 0.215 | short plan-view edge |
| 36 | medium | 230 -> 289 | 59 | 1.146 | 1.767 | 1.963 | 0.183 | 4.442, -17.915, 1.369 | 4.558, -19.056, -0.398 | short keyframe gap; short plan-view edge |
| 37 | medium | 329 -> 648 | 319 | 1.04 | 0.367 | 0.649 | 0.28 | -1.874, 4.15, -0.02 | -1.299, 3.284, -0.388 | short plan-view edge |
| 38 | medium | 194 -> 772 | 578 | 0.87 | 0.249 | 1.383 | 0.202 | -0.196, 6.207, 0.077 | 0.612, 6.53, 0.326 | short plan-view edge |
| 39 | medium | 194 -> 242 | 48 | 0.604 | 0.067 | 1.737 | 0.172 | -0.196, 6.207, 0.077 | -0.773, 6.031, 0.144 | short keyframe gap; short plan-view edge |
| 40 | low | 187 -> 770 | 583 | 1.108 | 0.703 | 0.955 | 0.124 | 0.702, 4.66, -0.517 | 1.33, 5.572, 0.186 | short plan-view edge |
| 41 | low | 2 -> 221 | 219 | 2.326 | 0.641 | 1.222 | 0.108 | -0.709, -0.418, -0.138 | -1.744, -2.501, 0.503 | geometry-only review |
| 42 | low | 7 -> 142 | 135 | 3.565 | 0.083 | 0.837 | 0.097 | 0.367, -1.148, -0.057 | 1.241, -4.604, 0.026 | geometry-only review |
| 43 | low | 79 -> 187 | 108 | 0.366 | 0.092 | 0.43 | 0.122 | 0.362, 4.523, -0.425 | 0.702, 4.66, -0.517 | short plan-view edge |
| 44 | low | 95 -> 194 | 99 | 0.363 | 0.128 | 0.452 | 0.087 | -0.363, 6.53, -0.051 | -0.196, 6.207, 0.077 | short plan-view edge |

## Missing Keyframe Geometry

- None

## Interpretation

Use this worksheet with the RViz audit markers or plan-view PNG. A row is not a validated loop-closure claim until its marker edge has been checked against the replayed map geometry and marked as a plausible revisit.

