# SOTA Gap Analysis

## Active priority, 2026-10-10

The harbor_05 boundary and real-data reconstruction comparison are complete.
The native dataset was fully reacquired and verified. Old local commit `51cea95`
and its fixed method/plans remain unavailable; the newly registered v4 is a
separate implementation. All four v4 trials succeeded and were frozen before
reference access, with independent score verification: common-support RMSE
2.384777 m baseline versus 2.132948 m pressure (10.559860% improvement, 674
positions). Both preceding v3 pressure failures remain reported, and the v4
negative-pressure quality exclusion was registered before reference access.
See [input/method details](aqualoc_vio_pressure.md), [scoring](aqualoc_vio_scoring.md)
and [artifact-backed results](harbor05_reconstruction_v4_results.json).

This does not establish SoTA. Shared-image COLMAP and depth/pressure scale make
AQUALOC especially limited for independent assessment of pressure fusion.
Independent hardware truth, multiple held-out sequences and competitive
implementations remain unexecuted gates; do not tune v4 against harbor_05 scores.

The Tank application was skipped by the user. The older Tank-first recommendation
below is historical and does not authorize any application/contact. After the
harbor_05 comparison, independent hardware ground truth and competitive methods
remain necessary for a SoTA claim. The user-reported harbor_06 ~4.3% improvement
is development evidence only; AQUALOC COLMAP/depth reference is not independent
hardware truth.

During native-mirror discovery, the public
[LBL-AQUALOC README](https://github.com/SYSU-CPNTLab/LBL-AQUALOC-Dataset)
was checked. It describes semi-physical simulated LBL measurements generated from
the COLMAP trajectory, rather than independently measured hardware truth. It is
neither a mirror of harbor_05 nor a way to satisfy the independent-truth gate.
Only metadata were read; no sequences or reference trajectories were acquired.

### Next hardware-reference gate

Subsequent work resolved access through the verified-TLS `www` distribution
host and sufficient fresh storage. Beach Pond native acquisition and an exclusive
GNSS-free raw-sensor input now completed, with 1072947 packets independently
byte-verified. See [current preparation](mbes_hardware_reference.md). The earlier
403/storage observations below describe the preceding environment. No hardware
reference values, RTK quality metrics, competitor result or held-out score have
yet been read/computed in this stage.

The existing MBES `beach_pond` candidate is a separate sensor/task benchmark,
not a substitute visual-pressure test. On 2026-10-10, neither its native nor
converted dataset existed in this workspace. TLS-preserving HEAD requests to
`https://seaward.science/files/pos-datasets/bag/beach_pond.tar.gz` and the public
landing page both failed at the network proxy with `403 Forbidden`; no bytes
were acquired. Current free space is about 1.9 GB, below even the historically
reported 2584672367-byte archive and roughly 6.8 GB extraction. Do not reuse the
older dated size as a current verified object identity or claim a replay here.

Before this hardware benchmark can run, provision sufficient storage and verify
current source size/ETag, distribution terms, archive integrity and raw sensor
coverage. Inspect RTK fix quality, covariance, clock and antenna-to-sonar/body
lever arm. `/nav/processed/odometry` fuses IMU/GPS/DVL; it is a dependent navigation
estimate and must not silently serve as independent hardware truth for a method
using those same sensors. Reserve raw, withheld hardware RTK positions as the
candidate independent reference only after quality/frame/time checks establish
that contract; report surface-vessel/depth limitations explicitly.

Register held-out sequence selection and source/config/input hashes for the
repository method, odometry-only ablation and a published MBES competitor before
reference access. Use identical windows/sensors and fixed SE(3)/scale/time rules,
freeze all outcomes, then evaluate accuracy, coverage, runtime and loop false
positives. A competitive MBES result would apply to bathymetric localization,
not establish visual-inertial-pressure SoTA across underwater tasks. For that
claim, an additional accessible dataset with independent hardware truth and
matching visual/IMU/pressure modalities remains necessary. Tank remains skipped.

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
| MBES trajectory proof | `beach_pond` loop-status counts, sweeps, audit worksheets, `mbes_loop_trajectory_metrics.py` APE reports, and `run_mbes_selected_loop_replay_comparison.sh` paired replay automation exist. The paired replay now includes an allowlist integrity audit. A 2026-06-04 100 s signature-replay probe accepted 7 selected loops with 0 outside the signature-aware allowlist, but selected pose-graph RMSE worsened by 44.09 m vs normal and carried a baseline-drift warning. A DCS delta 0.1 selected-only probe reduced the loop harm but still worsened pose-graph RMSE by 12.11 m vs its input odometry. A short-plan-view rotation gate rejected 15 suspect loops but still worsened RMSE by 11.81 m. A stricter rotation cap of 0.2 rad accepted 9 low-rotation selected loops and improved pose-graph RMSE by 4.38 m vs input in a diagnostic selected-only replay. Re-running 0.2 rad as a paired normal/selected comparison accepted 3 selected loops with 0 outside the timestamp/metric signature-aware allowlist and improved the selected replay pose graph by 4.82 m vs its input, but still had replay baseline drift and exact-ID drift. Re-exporting the same bags with endpoint keyframe timestamps made the stricter audit fail: the selected replay's 3 accepted loops did not match the normal-selected candidate endpoints. A later Humble sqlite replay with `min_points=120`, `voxel=0.25`, 0.2 rad rotation cap, DCS delta 0.1, endpoint signatures, and allowlist-aware candidate prioritization accepted 1 selected loop, passed the endpoint allowlist audit with 0 outside loops, improved selected pose-graph RMSE by 7.25 m vs normal, but still worsened RMSE by 4.83 m vs its own input odometry. | APE/RPE, pose-graph effect, selected-loop integrity, and false-positive audit must be tied to the same run and commit. | Stabilize loop identity or descriptor-level selection so more selected endpoints survive replay timing/keyframe drift, then re-run the 0.2 rad cap with accepted-marker audit notes and aligned input-odometry baselines. |
| Loop proposal quality | Candidate selection is temporal/spatial plus simple descriptor gates; selected-loop replay can now refine timestamp signatures with exported descriptor metrics, and the wrapper now emits `mbes_descriptor_retrieval.md` recall@k reports. The 5 s descriptor-signature replay ranked 5 signatures against replay candidates, but endpoint-timestamp recall was 0/5 through top 5 and only 1/5 at top 10. | Sonar/bathymetry place recognition should propose loop candidates robustly under drift and low-resolution terrain. | Replace scalar signature matching with a bathymetric descriptor/candidate generator, then track recall@k against trusted loop/audit pairs before registration. |
| Loop consistency | Online accepted-loop support count is observable. `export_mbes_loop_status.py --batch-consistency-out` exports a PCM-like selected/rejected loop-id report, `--batch-consistency-selected-csv-out` exports replayable IDs, `loop.selection.allowlist_csv` can feed those IDs back into the MBES loop node, and `check_mbes_loop_allowlist_replay.py` verifies the replay accepted only selected IDs. Exact-ID matching remains the default; opt-in signature matching can bridge replay ID drift for experiments, endpoint keyframe timestamps now make that bridge stricter when the bag contains `/aqua_pose_graph/keyframe`, descriptor-refined signatures can narrow broad timestamp windows, and allowlist-aware candidate prioritization spends `max_per_keyframe` on selected endpoints first. A 5 s descriptor-signature replay accepted 0 selected loops, showing the offline selector also needs quality filtering before clique selection. | Batch PCM/GkCM-style selection or robust maximum-consensus should select a globally consistent set of candidate loops and improve measured trajectory metrics without off-allowlist accepted loops. | Add stable loop identifiers or descriptor retrieval so the selected set can survive replay timing/keyframe drift without broad timestamp tolerances, then compare trajectory metrics against the dataset reference. |
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
   to accept the exported selected set. Endpoint keyframe timestamp signatures
   made the audit more honest: the 0.2 rad diagnostic replay no longer passes
   when candidate endpoints are required to match. Candidate prioritization
   moved the next replay to an endpoint-audited PASS, but only 1 selected loop
   survived and the pose graph was still worse than input odometry. The new
   descriptor retrieval report shows the scalar descriptor signature is not
   enough: the 5 s descriptor-signature replay had endpoint recall 0/5 through
   top 5 and 1/5 at top 10. The next step is a real bathymetric loop-retrieval
   front end so more selected endpoints survive before registration, then a
   paired replay that shows whether the pose graph improves against the dataset
   reference.

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

## North River and independent underwater VIO follow-up

GICP pose composition was corrected with a physical rotation/translation
regression (25 C++ tests pass). Four first-1600-packet point-only trials
(GICP/KISS-ICP twice) were frozen before RTK deserialization. V1 hardware scoring
failed because it used receive-time quality headers. Separately registered v2
uses explicit native GPS timestamps, without residual-driven time adjustment;
it was registered after v1 reference access and is a scoring repair, not a
new blind protocol. Independent correspondence and Horn checks passed. On 443
common positions RMSE was GICP 14.140753 m and KISS-ICP 18.048894 m; both repeats
agreed. These are large short-window surface-vessel errors, not underwater
VIO/pressure or general SoTA evidence. Complete exclusions, runtimes, failure
and hashes are in north_river_comparison_results.json and retained artifacts.

The official NTNU underwater dataset subset 3 has cameras, IMU, pressure and
external MoCap truth, matching the intended evaluation family. The candidate
11.6 GB sequence and calibration downloads remain blocked by proxy CONNECT
403 to ntnu.box.com; Huggingface metadata access also returned 403. Required
hosts were saved additively to the environment draft; saving has not applied
or published access. No data/reference values were read, Tank remains skipped.

## Post-publication NTNU access and competitor preparation

The published snapshot restored HEAD 4df25dd and frozen harbor_05/North River
source/runtime/native/output bindings were validated. Hugging Face metadata
now returns 200. Its current d79489fc tree supplies fjord/mclab bags and
ReAqROVIO-generated baseline trajectories, **not MoCap ground truth**. Five
publisher stereo calibration YAML files were acquired with pinned revision
and per-file hashes; rigid transform/stereo-chain consistency passed. Their
applicability to the old Box subset 3 has not been established.

Actual GET to the official subset-3 bag share redirects from ntnu.box.com to
ntnu.app.box.com, which still fails proxy CONNECT 403. The original share's
HEAD returned 404 and is not a reliable GET/download check. The exact observed
redirect hostname has been added to the draft without changing other domains.
No native NTNU sensor bag or MoCap coordinates have been acquired/read.

Official ReAqROVIO 7fc95c1 and its pinned lightweight_filtering/kindr sources
were cloned. Its unmodified numeric core and 25-feature stereo filter were
built in the retained C++ runtime. The probe uses upstream-style heap
allocation and finite initialization passes repeatedly; initial helper
compiler/stack failures and three upstream property-handler warnings remain
in logs. This is SDK readiness only; the ROS1 node, sensor updates and real
trials have not run. The default launch disables the pressure remap and its
bag loader selects only IMU and two images, so a pressure-enabled comparison
requires an explicitly registered adapter. Raw barometer values use the
publisher's -(value-2660)/241 depth mapping; do not assume SI Pa or silently
reuse AQUALOC's units/calibration/time correction.

See ntnu_mocap_postpublish.json for hashes, exact readiness and limitations.

## Pressure adapter development after environment recovery

The environment finally started with published HEAD b00d3a5. Box's redirect
host is now reachable: all six official subset-3 sensor shares return HTTP404,
so the previously proposed independent MoCap comparison is still unavailable.
The current pinned Hugging Face native fjord_1 object redirects to
`us.aws.cdn.hf.co` and that request fails proxy CONNECT403. This exact observed
CDN host was added to the draft. No NTNU native bag was acquired and no MoCap
coordinates were read. Publisher Git LFS metadata supplies an expected SHA256,
which must be verified against all native bytes before conversion.

New, separate sources implement stereo/IMU/pressure input isolation and a
pressure event adapter without editing frozen harbor_05/MBES sources. Six
isolation tests pass: all selected raw bytes/time/order/counts survive and
reference/navigation topics are excluded; missing, empty or wrongly typed
pressure blocks conversion. Native connections/counts and exclusion reasons
are reported; native data remains unchanged. This is fixture validation only,
not completed native conversion or estimator registration.

The pressure event adapter enforces every ordinal, exact native epochs and
strictly increasing pressure timestamps. It reports support/initialization/
finite-value exclusions and fixes its anchor at the first eligible sample.
The publisher's raw profile2660/241 is explicit, not assumed SI Pa. Contract
checks pass. A separate integration fixture actually executed two upstream
SDK BaroUpdates after one anchor, checked their processed count, finite
covariance and negative-z response to increasing pressure. Initial failed
checks remain in logs. The fixture sets positive update noise/zero offsets
and feeds beyond the upstream0.1s missing-image wait; these are synthetic
fixture settings, not a reference-tuned dataset method. Upstream source remains
unchanged, including its three constructor property-handler warnings and
six-entry unregister loop despite three-dimensional pressure noise. No full
visual/pressure replay or real-data accuracy result is claimed.

Current DeepVL metadata labels ReAqROVIO estimates as supervision. ROVTIO uses
LOAM near-ground-truth and ballast tank data is aerial/handheld inspection;
these metadata screens do not supply independent underwater hardware truth.
No Tank application/contact was made. Existing harbor_05 and North River
frozen source/native/runtime/output bindings were revalidated. See
ntnu_pressure_adapter_readiness.json for exact tested/unexecuted status.

## NTNU native development replay after CDN access

The fixed HF fjord_1 native object now matches the publisher LFS SHA256; all
292,712 native ROS records and selected sensor bytes/counts were verified.
A separately registered cam0 mono/depth ablation preserves all native data
and records each camera-profile/prefix exclusion. The prefix is a preselected
1,800 native frames (about 90s), not the complete sequence or a fresh holdout.

An SDK regression exposed a reversed merged-IMU mean increment; an isolated
overlay fixes it without modifying pinned dependency worktrees. Independent
numeric tests fail before and pass after the correction. Four earlier method
versions failed all 16 trials; v5 produced finite trajectories but left its
last camera queued. Those outcomes remain frozen. v6 uses feature reprojection
EKF, publisher underwater calibration, and a strict-boundary SDK processing
horizon that leaves native times unchanged. Its four trials process every
camera update and every eligible pressure update; source/output hashes,
initialization and common supports are frozen before reference access.

This verifies an executable real-data comparison workflow, not underwater
accuracy or SoTA. No reference coordinates were read, and no RMSE was computed.
The available HF `.tum` is a ReAqROVIO estimate; independent hardware truth and
competitive evaluation remain blocked. See ntnu_fjord_development_results.json
and ntnu_fjord_development.md for the full provenance and all failed/successful
outcomes. Tank applications remain skipped.

## Full native sequence development extension

The separately registered full-sequence adapter keeps the v6 model and fixed
calibration while extending replay to all 6,368 native cam0 images, 63,698 IMU
packets and 15,926 pressure packets. The final pressure is 6.132534 ms beyond
native IMU support and is retained with an explicit no-update reason. On complete
input only, SDK queue waiting is disabled to process the supported tail without
extrapolation or changing native timestamps. An actual-SDK regression and an
independent ROS-to-wire check validate this boundary handling. Previous prefix
registrations and failures are immutable. See `ntnu_fjord_full.md` and
`ntnu_fjord_full_results.json` for full outcomes and support accounting.

This extends execution coverage, not accuracy evidence. The full sequence is
development data with no independent hardware reference acquired or read; the
modified mono adapter has not been evaluated against competitive methods with
independent underwater truth. No SoTA result follows from these runs.
