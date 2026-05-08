# Foxglove Studio layouts

Web-based replay views for `aqua_localization` public-data demos. Use the
free [Foxglove Studio](https://studio.foxglove.dev/) web app — no local
install required, no account needed for local-file playback.

## Why Foxglove (vs RViz)

- Runs entirely in the browser; nothing to install.
- Reads ROS 2 `.mcap` bags directly via drag-and-drop.
- Layout is captured as a single JSON file you can commit and share — anyone
  who drags the same bag onto Foxglove with this layout imported sees the
  same panels, camera angle, and plots.
- Decent screenshot / export workflow for README assets.

## Available layouts

| Layout | Demo | Bag |
|--------|------|-----|
| [`aqua_tank_demo.json`](aqua_tank_demo.json) | Tank Dataset `short_test` — IMU + pressure + DVL fusion vs AprilTag GT | recorded via `datasets/tank_dataset_demo.md` |

## Recording a results-included bag

Foxglove and rerun.io both play whatever is in the bag — by themselves the
source public bags do not contain `aqua_localization` output topics. Record
a "demo" bag that captures both the inputs and our pipeline outputs, so the
resulting `.mcap` is self-contained and reproducible.

One-shot recorder per dataset (each builds on the same pattern: estimator in
the background, `ros2 bag record` writing to mcap, `ros2 bag play --clock`
driving the source bag in the foreground):

```bash
ros2 run aqua_localization record_tank_demo.sh
ros2 run aqua_localization record_mbes_demo.sh
ros2 run aqua_localization record_ntnu_demo.sh
ros2 run aqua_localization record_aqualoc_demo.sh
```

Each script honors environment overrides for source/output paths and replay
duration — see the header comment of each `aqua_localization/scripts/record_*_demo.sh`
file.

The Tank demo bag contains ~27 k messages over ~15 s including 5 321
`/aqua_imu_loc/odometry` samples; the MBES demo bag is ~50 MB across 60 s
of multibeam fans plus our pipeline outputs; the NTNU bag is ~21 MB over
90 s; AQUALOC clocks in around 400 MB because of the camera stream.

If you prefer the manual three-terminal flow:

```bash
# Source bag (input only).
SRC=aqua_localization/datasets/public/tank_dataset/short_test_ros2
# Output bag (input + estimate + TF).
OUT=aqua_localization/datasets/public/tank_dataset/demo_with_estimate

source install/setup.bash

# Terminal A: estimator.
ros2 run aqua_imu_loc imu_loc_node --ros-args \
  --params-file install/aqua_imu_loc/share/aqua_imu_loc/config/tank_dataset.yaml \
  -p use_sim_time:=true

# Terminal B: recorder.
ros2 bag record -s mcap -o "$OUT" \
  --topics /imu/data /pressure /dvl/twist /apriltag_slam/GT \
           /aqua_imu_loc/odometry /aqua_imu_loc/status \
           /tf /tf_static

# Terminal C: replay.
ros2 bag play "$SRC" --clock
```

After the source bag finishes, ctrl-C the recorder and estimator.

## Loading the layout in Foxglove

1. Open <https://studio.foxglove.dev/> in any modern browser.
2. **Open local file…** → select the `.mcap` from the `demo_with_estimate`
   directory above.
3. **Layout menu (top-right)** → **Import from file…** → choose the layout
   JSON in this directory.
4. Press play. The 3D panel follows `base_link`; the green trail is the
   AprilTag GT, the blue trail is the `aqua_imu_loc` estimate, the orange
   arrows are the DVL body-frame twist samples. The right column shows
   depth (estimate vs GT) and DVL velocity components.

## Capturing screenshots / video

Foxglove has no built-in image export. The standard workflow:

- **Single frame**: pause at the moment of interest, take an OS screenshot
  of the browser viewport.
- **Animated GIF / MP4**: use OBS or `wf-recorder` (Wayland) /
  `simplescreenrecorder` (X11) on the browser tab; Foxglove's own framerate
  matches whatever your screen captures.

Save README-bound assets under `docs/media/`.
