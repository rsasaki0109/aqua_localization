#!/bin/bash

# Experiment automation (UKF blind navigation) for ROS 2 Jazzy
# Outputs:
#   outputs/estimated_trajectory.tum
#   outputs/ground_truth_trajectory.tum
#   outputs/results_error.txt

BAG_ID="${BAG_ID:-1U2APRrDJYpTHktil1evhvAsF__L42BYL}"
OUT_DIR="${OUT_DIR:-outputs}"

source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash

echo "=== Starting Experiment: UKF Evaluation (Dead Reckoning) ==="

# 1. Download and apply the bag conversion script
gdown "$BAG_ID" -O short_test.bag
ros2 run aqua_localization convert_tank_dataset_bag.py --src short_test.bag --dst short_test_ros2

# Jazzy compatibility: remove ROS 1 style hash suffixes (RIHS01_*) from stored topic types.
# Some converted bags include these suffixes and playback/type resolution can fail on Jazzy.
echo "-> Applying incompatibility fixes..."
sqlite3 short_test_ros2/short_test_ros2.db3 "UPDATE topics SET type = substr(type, 1, instr(type, ' ') - 1) WHERE type LIKE '% RIHS01_%';"
sed -i 's/ RIHS01_[a-f0-9]*//g' short_test_ros2/metadata.yaml
# ----------------------------------

# 2. Start fusion node (UKF) in BACKGROUND
echo "-> Starting UKF fusion node..."
ros2 run aqua_imu_loc imu_loc_node --ros-args \
  --params-file $(ros2 pkg prefix aqua_imu_loc)/share/aqua_imu_loc/config/tank_dataset.yaml \
  -p use_sim_time:=true & 
UKF_PID=$!

# 3. Start trajectory recorders (TUM format) in BACKGROUND
echo "-> Starting trajectory recorders (TUM)..."
mkdir -p $OUT_DIR
ros2 run aqua_localization record_odometry.py --topic /aqua_imu_loc/odometry --out $OUT_DIR/estimated_trajectory.tum --format tum &
REC_EST_PID=$!

ros2 run aqua_localization record_odometry.py --topic /apriltag_slam/GT --out $OUT_DIR/ground_truth_trajectory.tum --format tum &
REC_GT_PID=$!

# Wait a few seconds to ensure all nodes are up and running
sleep 2

# 4. Replay the bag recording
echo "-> Replaying bag recording..."
ros2 bag play short_test_ros2 --clock

# 5. Clean up: Stop the fusion node and trajectory recorders
echo "-> Cleaning up..."
kill -2 $UKF_PID $REC_EST_PID $REC_GT_PID
sleep 3

echo "-> Calculating Absolute Pose Error (APE)..."
ros2 run aqua_localization compare_trajectories.py \
  $OUT_DIR/ground_truth_trajectory.tum \
  $OUT_DIR/estimated_trajectory.tum > $OUT_DIR/results_error.txt

echo "=== Experiment Finished. Check results_error.txt ==="