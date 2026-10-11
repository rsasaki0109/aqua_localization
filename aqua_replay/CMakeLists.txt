cmake_minimum_required(VERSION 3.8)
project(aqua_replay)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(aqua_msgs REQUIRED)
find_package(class_loader REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(offline_replay
  src/offline_replay.cpp
  src/message_registry.cpp
)
target_compile_features(offline_replay PUBLIC cxx_std_17)
target_include_directories(offline_replay PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(offline_replay yaml-cpp)
ament_target_dependencies(offline_replay
  ament_index_cpp
  aqua_msgs
  class_loader
  geometry_msgs
  nav_msgs
  rclcpp
  rclcpp_components
  rosbag2_cpp
  rosbag2_storage
  rosgraph_msgs
  sensor_msgs
  std_msgs
  tf2_msgs
  visualization_msgs
)

install(TARGETS offline_replay
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/make_synthetic_mbes_bag.py
  scripts/compare_replay_summaries.py
  scripts/dvl_dead_reckoning_baseline.py
  scripts/evaluate_replay_trajectories.py
  scripts/make_stereo_feature_track.py
  scripts/run_beach_pond_benchmark.py
  scripts/run_tank_benchmark.py
  scripts/run_tank_live.py
  scripts/run_tank_robustness.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY scenarios
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_gtest(test_reorder_buffer test/test_reorder_buffer.cpp)
  target_include_directories(test_reorder_buffer PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)

  # Humble's rclcpp (16.x) rejects intra-process publishers with
  # transient_local durability, which the pose graph and MBES loop-closure
  # nodes use, so the full-pipeline replay needs Iron or newer.
  if(rclcpp_VERSION VERSION_GREATER_EQUAL "21.0.0")
    ament_add_pytest_test(test_offline_replay_determinism
      test/test_offline_replay_determinism.py
      TIMEOUT 600
      ENV
        AQUA_REPLAY_BIN=$<TARGET_FILE:offline_replay>
        AQUA_REPLAY_SCRIPTS=${CMAKE_CURRENT_SOURCE_DIR}/scripts
        AQUA_REPLAY_SCENARIOS=${CMAKE_CURRENT_SOURCE_DIR}/scenarios
    )
  else()
    message(STATUS "rclcpp ${rclcpp_VERSION}: skipping full-pipeline replay determinism test")
  endif()
endif()

ament_package()
