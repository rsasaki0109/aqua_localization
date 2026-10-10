# NTNU full-sequence OpenVINS baseline development

Twelve native-data trials across six separately registered profiles failed to
initialize. They have no retained pose trajectory and cannot support an RMSE or
SoTA claim. All 6,368 cam0 and 15,926 pressure event ordinals are accounted for in
each trial, including the unsupported final pressure. The original complete
native bag, all cam1 packets, previous ReAqROVIO freezes and failed registrations
remain retained. See [ntnu_openvins_development_results.json](ntnu_openvins_development_results.json)
for every runtime, source/input/output binding, trial artifact hash and exclusion.

This standalone mono adapter uses the pinned OpenVINS numerical SDK
`69488123ed9362dd44b6f28e7f4680abbff1442b` and the same independently verified
AQMONO01 full native stream as the existing four-trial ReAqROVIO pressure ablation.
Publisher underwater equidistant intrinsics, air extrinsics and IMU noise are
checked independently against executable values. The verified wire already
applies the publisher's -1,770,000 ns IMU correction; the additional camera
correction is zero. Images, native epochs, scale and calibration are unchanged.
There is no pressure update in standard OpenVINS; all pressure ticks are retained
with an explicit no-update reason. Ordinal15925 is 6,132,534 ns beyond IMU support.

| Version | Window (s) | Iteration limit | Initial feature budget | Two trials | Observed diagnostic |
|---|---:|---:|---:|---|---|
| v1 | 1 | 20 | 50 | Both failed | Optimizer iteration limit reached |
| v2 | 1 | 100 | 50 | Both failed | Optimizer iteration limit reached |
| v3 | 1 | 1000 | 50 | Both failed | Optimization converged; covariance recovery failed |
| v4 | 4 | 1000 | 50 | Both failed | No initialized trajectory; final MLE cause not established |
| v5 | 2 | 1000 | 50 | Both failed | Only 2 triangulated features, below SDK minimum8 |
| v6 | 2 | 1000 | 200 | Both failed | Only 6 triangulated features, below SDK minimum8 |

Each profile was registered before execution and before reference access; it
binds earlier failures and the unchanged four successful ReAqROVIO full trials.
Changing an input or method requires a new version. These versions are informed
by the same development sequence; they are not fresh holdout evaluations. The
5-second MLE cap, one optimizer thread, cheirality, minimum feature count, strict
convergence and covariance rank checks remain enforced. No reference trajectory
was acquired or read. The preregistered scoring contract is SE(3), scale1,
interpolation gap at most0.1 seconds, without clock/scale/calibration optimization.
It has not been executed. With no initialized OpenVINS poses, there is no common
support for the proposed six-trial comparison.

An actual installed-Ceres regression independently verifies that one camera view
cannot recover depth covariance, while two translated views can. This passes
without relaxing covariance checks. It demonstrates the rank issue in a small
geometric fixture, not the cause of every native-data failure and not accuracy.
The pinned initializer couples its minimum feature observation count to the
integer initialization window; a one-second window admits single-view features.
The two-second window matches its upstream Euroc example. The native v3 failure
and later feature shortages remain explicit negative results.

The SDK's DEBUG formatter prints integer `init_max_features` with `%.2f`; that
label is unreliable. The registered sources preserve actual integer settings;
v6 also exposes typed initialization fields in `CALIBRATION_JSON` and checks
them before registration. The SDK itself was not edited or rebuilt in place.
v5/v6 bind the actual transitive Ceres/OpenCV/Boost runtime libraries as well as
the SDK library, source, calibration, executable profile and observability fixture.

Build failures are preserved outside the repository: the first native build
missed explicit Boost linkage, and the initial covariance fixture had an
ambiguous Ceres overload. Isolated corrected builds and the fixture passed.
The read-only `check_ntnu_openvins.bash` verifies every freeze and artifact and
reruns the actual Ceres fixture. Integrity success is distinct from estimator
success or readiness for independent accuracy scoring.
