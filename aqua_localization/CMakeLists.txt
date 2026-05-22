cmake_minimum_required(VERSION 3.8)
project(aqua_localization)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/inspect_bag_topics.py
  scripts/record_odometry.py
  scripts/record_status.py
  scripts/compare_trajectories.py
  scripts/trajectory_benchmark_row.py
  scripts/benchmark_manifest_report.py
  scripts/ingest_external_tum_result.py
  scripts/check_3dgs_bag_ready.py
  scripts/check_3dgs_training_ready.py
  scripts/export_3dgs_manifest.py
  scripts/export_3dgs_dataset_pack.py
  scripts/export_3dgs_frames.py
  scripts/export_3dgs_transforms.py
  scripts/export_3dgs_pack_pipeline.py
  scripts/benchmark_gap_report.py
  scripts/ros1_odometry_csv_to_tum.py
  scripts/convert_scalar_pressure_bag.py
  scripts/bench_fjord_1.sh
  scripts/diagnose_yaw_frame.py
  scripts/plot_trajectories.py
  scripts/make_synthetic_sonar_bag.py
  scripts/make_demo_gif.py
  scripts/convert_tank_dataset_bag.py
  scripts/stereo_visual_odometry.py
  scripts/rerun_export.py
  scripts/rerun_export_mbes.py
  scripts/rerun_export_ntnu.py
  scripts/rerun_export_aqualoc.py
  scripts/lichtblick_screenshot.py
  scripts/record_tank_demo.sh
  scripts/record_mbes_demo.sh
  scripts/record_ntnu_demo.sh
  scripts/record_aqualoc_demo.sh
  scripts/calibrate_sonar_covariance.py
  scripts/calibrate_visual_scale.py
  scripts/validate_visual_scale.py
  scripts/visual_calibration_profile.py
  scripts/run_tank_visual_benchmark.py
  scripts/run_tank_visual_matching_sweep.py
  scripts/run_tank_visual_fusion_benchmark.py
  scripts/sweep_visual_extrinsics.py
  scripts/summarize_visual_frontend_status.py
  scripts/analyze_visual_drift.py
  scripts/analyze_visual_motion_segments.py
  scripts/check_mbes_benchmark_ready.py
  scripts/export_mbes_loop_status.py
  scripts/mbes_loop_benchmark_row.py
  scripts/audit_mbes_loop_candidates.py
  scripts/publish_mbes_loop_audit_markers.py
  scripts/plot_mbes_loop_audit.py
  scripts/run_mbes_loop_benchmark.sh
  scripts/pose_graph_loop_demo.py
  scripts/pose_graph_loop_smoke.sh
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(test_inspect_bag_topics test/test_inspect_bag_topics.py)
  ament_add_pytest_test(test_record_odometry_format test/test_record_odometry_format.py)
  ament_add_pytest_test(test_compare_trajectories test/test_compare_trajectories.py)
  ament_add_pytest_test(test_trajectory_benchmark_row test/test_trajectory_benchmark_row.py)
  ament_add_pytest_test(test_benchmark_manifest_report test/test_benchmark_manifest_report.py)
  ament_add_pytest_test(test_ingest_external_tum_result test/test_ingest_external_tum_result.py)
  ament_add_pytest_test(test_check_3dgs_bag_ready test/test_check_3dgs_bag_ready.py)
  ament_add_pytest_test(test_check_3dgs_training_ready test/test_check_3dgs_training_ready.py)
  ament_add_pytest_test(test_export_3dgs_manifest test/test_export_3dgs_manifest.py)
  ament_add_pytest_test(test_export_3dgs_dataset_pack test/test_export_3dgs_dataset_pack.py)
  ament_add_pytest_test(test_export_3dgs_frames test/test_export_3dgs_frames.py)
  ament_add_pytest_test(test_export_3dgs_transforms test/test_export_3dgs_transforms.py)
  ament_add_pytest_test(test_export_3dgs_pack_pipeline test/test_export_3dgs_pack_pipeline.py)
  ament_add_pytest_test(test_benchmark_gap_report test/test_benchmark_gap_report.py)
  ament_add_pytest_test(test_ros1_odometry_csv_to_tum test/test_ros1_odometry_csv_to_tum.py)
  ament_add_pytest_test(test_convert_tank_dataset_bag test/test_convert_tank_dataset_bag.py)
  ament_add_pytest_test(test_stereo_visual_odometry test/test_stereo_visual_odometry.py)
  ament_add_pytest_test(test_calibrate_visual_scale test/test_calibrate_visual_scale.py)
  ament_add_pytest_test(test_validate_visual_scale test/test_validate_visual_scale.py)
  ament_add_pytest_test(test_visual_calibration_profile test/test_visual_calibration_profile.py)
  ament_add_pytest_test(test_run_tank_visual_benchmark test/test_run_tank_visual_benchmark.py)
  ament_add_pytest_test(test_run_tank_visual_matching_sweep test/test_run_tank_visual_matching_sweep.py)
  ament_add_pytest_test(test_run_tank_visual_fusion_benchmark test/test_run_tank_visual_fusion_benchmark.py)
  ament_add_pytest_test(test_sweep_visual_extrinsics test/test_sweep_visual_extrinsics.py)
  ament_add_pytest_test(test_summarize_visual_frontend_status test/test_summarize_visual_frontend_status.py)
  ament_add_pytest_test(test_analyze_visual_drift test/test_analyze_visual_drift.py)
  ament_add_pytest_test(test_analyze_visual_motion_segments test/test_analyze_visual_motion_segments.py)
  ament_add_pytest_test(test_check_mbes_benchmark_ready test/test_check_mbes_benchmark_ready.py)
  ament_add_pytest_test(test_convert_scalar_pressure_bag test/test_convert_scalar_pressure_bag.py)
  ament_add_pytest_test(test_diagnose_yaw_frame test/test_diagnose_yaw_frame.py)
  ament_add_pytest_test(test_plot_trajectories test/test_plot_trajectories.py)
  ament_add_pytest_test(test_record_status_format test/test_record_status_format.py)
  ament_add_pytest_test(test_make_synthetic_sonar_bag test/test_make_synthetic_sonar_bag.py)
  ament_add_pytest_test(test_rerun_export_mbes_helpers test/test_rerun_export_mbes_helpers.py)
  ament_add_pytest_test(test_export_mbes_loop_status test/test_export_mbes_loop_status.py)
  ament_add_pytest_test(test_mbes_loop_benchmark_row test/test_mbes_loop_benchmark_row.py)
  ament_add_pytest_test(test_audit_mbes_loop_candidates test/test_audit_mbes_loop_candidates.py)
  ament_add_pytest_test(test_publish_mbes_loop_audit_markers test/test_publish_mbes_loop_audit_markers.py)
  ament_add_pytest_test(test_plot_mbes_loop_audit test/test_plot_mbes_loop_audit.py)
  ament_add_pytest_test(test_run_mbes_loop_benchmark test/test_run_mbes_loop_benchmark.py)
endif()

ament_package()
