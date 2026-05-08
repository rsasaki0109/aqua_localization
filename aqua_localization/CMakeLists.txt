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
  scripts/convert_scalar_pressure_bag.py
  scripts/bench_fjord_1.sh
  scripts/diagnose_yaw_frame.py
  scripts/plot_trajectories.py
  scripts/make_synthetic_sonar_bag.py
  scripts/make_demo_gif.py
  scripts/convert_tank_dataset_bag.py
  scripts/rerun_export.py
  scripts/lichtblick_screenshot.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(test_inspect_bag_topics test/test_inspect_bag_topics.py)
  ament_add_pytest_test(test_record_odometry_format test/test_record_odometry_format.py)
  ament_add_pytest_test(test_compare_trajectories test/test_compare_trajectories.py)
  ament_add_pytest_test(test_convert_scalar_pressure_bag test/test_convert_scalar_pressure_bag.py)
  ament_add_pytest_test(test_diagnose_yaw_frame test/test_diagnose_yaw_frame.py)
  ament_add_pytest_test(test_plot_trajectories test/test_plot_trajectories.py)
  ament_add_pytest_test(test_record_status_format test/test_record_status_format.py)
  ament_add_pytest_test(test_make_synthetic_sonar_bag test/test_make_synthetic_sonar_bag.py)
endif()

ament_package()
