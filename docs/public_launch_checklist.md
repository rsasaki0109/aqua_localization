# Public Launch Checklist

Use this after a README or release refresh to make the repository easier
to discover. These are repository-administration and outreach steps, not
runtime requirements.

## GitHub Repository Settings

Set a concise repository description:

```text
ROS 2 underwater localization: IMU, pressure, DVL, sonar scan matching, and pose graph demos on public AUV/ROV bags.
```

Recommended topics:

```text
ros2
underwater-robotics
localization
slam
auv
rov
sonar
dvl
ukf
g2o
pcl
rerun
bluerov2
robotics
```

Pin these links in the repository sidebar if useful:

- Latest release: `https://github.com/rsasaki0109/aqua_localization/releases`
- Demo media: `docs/media/`
- Dataset notes: `datasets/`

## Release Post Template

```markdown
aqua_localization v0.2 is a ROS 2 underwater localization stack built around
real public AUV/ROV data:

- 15-state additive UKF for IMU + pressure + DVL + sonar updates
- PCL ICP/GICP/NDT sonar point-cloud registration
- g2o SE(3) pose graph backend
- rerun.io exports for Tank Dataset, MBES-SLAM, NTNU, and AQUALOC
- Tank Dataset short_test: 0.43 m APE RMSE vs AprilTag GT

Repo: https://github.com/rsasaki0109/aqua_localization
Demo notes: https://github.com/rsasaki0109/aqua_localization/tree/main/datasets
```

## Good Places To Share

- ROS Discourse, with the `projects` or `release` category depending on
  the post format.
- Rerun community channels, because the demo path uses `.rrd` exports and
  curated 3D + plot views.
- Robotics and ROS communities where public datasets and reproducibility
  are the main angle.
- Underwater robotics researchers or labs working with AUV, ROV, DVL, and
  sonar localization.

## Before Posting

- Confirm the default branch README renders the GIF and screenshots.
- Confirm the latest release link points at the release being announced.
- Run the Tank Dataset demo command or at least verify that the documented
  artifact paths still exist.
- Include one concrete metric in the post. The current strongest number is
  Tank Dataset `short_test`: 0.43 m APE RMSE vs AprilTag ground truth.
- Mention limitations clearly: the pose graph backend and experimental
  MBES loop-closure front end ship, but real-bag loop-closure tuning and
  false-positive analysis are still in progress.
