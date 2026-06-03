# SOTA Gap Analysis

Checked on 2026-06-04. This page records the paper-backed gap between
`aqua_localization` and current underwater SLAM / sonar SLAM systems. Treat it
as a development gate, not as a marketing page.

## Bottom Line

`aqua_localization` is not SOTA yet.

The repo is strong on ROS 2 packaging, public-bag replay, MBES visualization,
status exports, and auditable loop-closure diagnostics. The missing SOTA pieces
are tighter: competitive held-out accuracy, tightly-coupled multi-sensor
optimization, online calibration, learned or consensus-based loop proposal, and
robust loop selection/back-end behavior that survives false positives without
manual audit.

## Current SOTA Bar

| Thread | Paper / system | What it implies for this repo |
|--------|----------------|-------------------------------|
| Full underwater SLAM | AQUA-SLAM fuses DVL, stereo camera, and IMU in a graph optimization framework and includes online sensor calibration. Its paper reports SOTA-level underwater accuracy and robustness, and the upstream repo exposes per-sequence Tank Dataset launches. | A fair "beats AQUA-SLAM" claim requires the same Tank window, same alignment statement, a held-out sequence, and a sensor-equivalent visual-DVL-IMU path. Current `aqua_*` Tank rows are progress, not a claimable win. |
| Emerging visual-inertial-sonar SLAM | VISO fuses stereo, IMU, and 3D sonar, estimates sonar-camera extrinsics online, and targets dense reconstruction as well as localization. | A SOTA story cannot stop at sparse odometry and status CSVs. The repo needs either dense sonar/visual map value or a deliberately narrower MBES-specific claim. |
| Visual-inertial-acoustic-depth fusion | Recent DVL/preintegration work frames underwater SLAM as graph optimization over stereo, IMU, DVL, and pressure measurements under degraded visuals. | The loose `aqua_fusion` path is useful engineering, but a paper-grade trajectory result likely needs DVL/pressure/visual residuals in one optimizer. |
| Older underwater multi-sensor baseline | SVIn2 uses a tightly-coupled keyframe SLAM formulation with loop closing, relocalization, sonar, visual, inertial, and pressure inputs. | Loop closure and relocalization are expected features, not stretch goals, for an underwater SLAM claim. |
| Sonar place recognition | Sonar Context proposes training-free imaging-sonar place recognition plus adaptive matching and ICP loop factors. | The MBES front end should not rely only on distance-ranked historical submaps. It needs sonar/bathymetry descriptors that propose plausible revisits before registration. |
| Bathymetric / MBES loop detection | Data-driven bathymetric point-cloud loop closure work calls out sparse landmarks, large dead-reckoning drift, and low MBES resolution as the hard data-association problem. | The current descriptor gate is a start, but a SOTA MBES path needs learned or data-driven bathymetric loop proposals and coarse alignment. |
| Probabilistic MBES registration | Multibeam 3D underwater SLAM with probabilistic registration models swath uncertainty and uses point-to-point then point-to-plane registration. | The current PCL ICP/GICP/NDT gates do not yet produce calibrated information matrices or degeneracy-aware partial constraints. |
| MBES dataset evidence | The bathymetric mapping dataset used by MBES-SLAM provides multibeam sonar, underwater navigation sensors, and high-precision RTK GPS ground truth for survey-style evaluation. | `beach_pond` should become a trajectory benchmark, not only a loop-status benchmark. The loop graph effect must be measured against the provided reference. |
| Robust loop closure selection | PCM turns largest internally consistent loop-measurement selection into a maximum clique problem and outperforms DCS, switchable constraints, and RANSAC in its evaluation. | The accepted-loop support guard is only an online heuristic. The next robust front-end step is batch pairwise consistency selection over candidate loop transforms. |
| Group-k consistency | GkCM extends pairwise consistency to group-k checks over generalized graphs and reports hardware-data evaluation in an underwater range-only SLAM scenario. | The repo's `min_support_count` is not the same as group-k consistency. If pairwise checks fail on ambiguous bathymetry, group-k or batch consistency should replace the scalar guard. |
| Robust back ends | Switchable constraints, GNC, and certifiable robust perception papers show that least-squares pose graphs are brittle under outliers and need explicit robust estimation. | DCS is a reasonable first g2o guard, but a stronger claim needs switchable factors, GNC/riSAM-style behavior, or an explicit maximum-consensus front end. |

## What Is Missing

| Gap | Current state | SOTA-grade requirement | Next repo work |
|-----|---------------|------------------------|----------------|
| Held-out AQUA-SLAM comparison | `short_test` rows exist, but Medium held-out inputs are blocked. | Claim gate must pass on a held-out Tank sequence with AQUA-SLAM and `aqua_*` rows generated from scripts. | Finish Medium input discovery, ingest AQUA-SLAM trajectory, run `aqua_slam_head_to_head_report.py --fail-without-claimable-win`. |
| Sensor-equivalent Tank stack | Visual frontend is diagnostic; fusion can regress relative to standalone visual. | Stereo + IMU + DVL + pressure should be optimized with explicit residuals and calibrated frames. | Add graph/UKF path that uses DVL velocity, visual pose/feature residuals, pressure, and IMU in one estimator; keep scale/extrinsic tuning out of the held-out segment. |
| MBES trajectory proof | `beach_pond` loop-status counts, sweeps, audit worksheets, `mbes_loop_trajectory_metrics.py` APE reports, and `run_mbes_selected_loop_replay_comparison.sh` paired replay automation exist. The paired replay now includes an allowlist integrity audit. A 2026-06-04 120 s selected-loop probe passed strict allowlist integrity with 0 off-list accepted loops, but accepted 0 selected loops and carried a baseline-drift warning. The replay gate now has an opt-in timestamp/registration-signature fallback for keyframe-ID drift, but it still needs a successful paired metric run. | APE/RPE, pose-graph effect, selected-loop integrity, and false-positive audit must be tied to the same run and commit. | Run the paired comparison with signature fallback, then replace it with a stable loop identity or descriptor-level selector before claiming improvement. |
| Loop proposal quality | Candidate selection is temporal/spatial plus simple descriptor gates. | Sonar/bathymetry place recognition should propose loop candidates robustly under drift and low-resolution terrain. | Add bathymetric descriptor export and offline retrieval evaluation: recall@k against trusted loop/audit pairs, then use it before registration. |
| Loop consistency | Online accepted-loop support count is observable. `export_mbes_loop_status.py --batch-consistency-out` exports a PCM-like selected/rejected loop-id report, `--batch-consistency-selected-csv-out` exports replayable IDs, `loop.selection.allowlist_csv` can feed those IDs back into the MBES loop node, and `check_mbes_loop_allowlist_replay.py` verifies the replay accepted only selected IDs. Exact-ID matching remains the default; opt-in signature matching can bridge replay ID drift for experiments. | Batch PCM/GkCM-style selection or robust maximum-consensus should select a globally consistent set of candidate loops and improve measured trajectory metrics without off-allowlist accepted loops. | Add stable loop identifiers or descriptor retrieval so the selected set can survive replay timing/keyframe drift without broad timestamp tolerances, then compare trajectory metrics against the dataset reference. |
| Registration uncertainty | Fitness/correction gates are exported, but loop information is mostly configured. | Registration should return covariance/information that reflects geometry, overlap, and degeneracy. | Add overlap/condition-number diagnostics and map them into loop information; support partially constrained factors when yaw/translation are degenerate. |
| Robust backend | DCS/huber/none are available backend guards. | False loop closures should be downweighted or switched off automatically and reported. | Add switchable loop factors or an offline GNC/PCM-selected loop set; compare against DCS on the same `beach_pond` replay. |
| Reproducibility claim | Public docs and Pages are good. | Paper evidence needs exact source bag, duration, config, commit, metrics, and failure gates. | Add a generated SOTA-readiness report that fails when a claim lacks held-out data, baseline row, trajectory metric, or loop audit. |

## Recommended Development Order

1. **Close the Tank held-out gate.**
   This is the fastest way to know whether a broad underwater SLAM claim is
   plausible. Without it, any "SOTA" wording should be blocked.

2. **Run MBES trajectory evidence.**
   The repo can now export input-odometry vs. pose-graph APE from each replay
   and run the normal/selected comparison as one wrapper. The missing evidence
   is a full replay whose selected-loop comparison improves the pose graph,
   whose allowlist audit passes, and whose accepted loops have false-positive
   decisions.

3. **Measure the selected-loop replay.**
   The repo now has a PCM-like offline selector and a selected-loop replay
   allowlist. The 2026-06-04 strict replay probe proved the integrity gate can
   reject all off-allowlist loops, but exact replay keyframe IDs were too brittle
   to accept the exported selected set. The next step is a stable loop identity
   or descriptor-level selector, then a paired replay that shows whether the pose
   graph improves against the dataset reference.

4. **Add uncertainty and degeneracy diagnostics.**
   For MBES, calibrated loop information and partial constraints matter more
   than accepting more loops. The benchmark should expose when registration only
   constrains a subset of SE(3).

5. **Decide the paper claim scope.**
   If Tank held-out does not beat AQUA-SLAM, scope the claim to "ROS 2 public
   underwater localization and MBES loop-diagnostic tooling" rather than "best
   underwater SLAM accuracy." That narrower claim can still be strong.

## Claim Gate

Do not use SOTA wording unless all selected claim rows satisfy:

- baseline and target use the same bag segment, duration, and alignment mode,
- held-out validation exists or the row is explicitly labeled diagnostic,
- reference trajectory source is recorded,
- runtime factor and sample coverage are recorded,
- loop-closure rows include false-positive audit decisions,
- config files and commit hash are recorded,
- regenerated docs and scripts reproduce the table row.

## Primary Sources

- AQUA-SLAM arXiv: <https://arxiv.org/abs/2503.11420>
- AQUA-SLAM repository: <https://github.com/SenseRoboticsLab/AQUA-SLAM>
- Tank Dataset page: <https://senseroboticslab.github.io/underwater-tank-dataset/>
- VISO: <https://arxiv.org/abs/2601.01144>
- Underwater Visual-Inertial-Acoustic-Depth SLAM with DVL Preintegration:
  <https://arxiv.org/abs/2510.21215>
- SVIn2: <https://arxiv.org/abs/1810.03200>
- Sonar Context / imaging sonar place recognition:
  <https://arxiv.org/abs/2305.14773>
- Data-driven bathymetric point-cloud loop closure:
  <https://arxiv.org/abs/2209.08578>
- Multibeam 3D underwater SLAM with probabilistic registration:
  <https://recerca.udg.edu/en/publications/multibeam-3d-underwater-slam-with-probabilistic-registration/>
- Bathymetric mapping and SLAM dataset:
  <https://doi.org/10.1177/02783649211044749>
- Robust bathymetric SLAM with invalid loop closures:
  <https://doi.org/10.1016/j.apor.2020.102298>
- Pairwise Consistent Measurement Set Maximization:
  <https://doi.org/10.1109/ICRA.2018.8460217>
- Group-k Consistent Measurement Set Maximization:
  <https://doi.org/10.1177/02783649241256970>
- Switchable Constraints for robust pose graph SLAM:
  <https://nikosuenderhauf.github.io/assets/papers/IROS12-switchableConstraints.pdf>
- Graduated Non-Convexity for robust spatial perception:
  <https://arxiv.org/abs/1909.08605>
- Certifiably optimal outlier-robust geometric perception:
  <https://arxiv.org/abs/2109.03349>
