cmake_minimum_required(VERSION 3.8)
project(aqua_graph)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  # GTSAM is unusably slow without optimization.
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(GTSAM_UNSTABLE REQUIRED)
find_package(aqua_imu_loc REQUIRED)
find_package(aqua_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(aqua_graph_core
  src/factors.cpp
  src/estimator.cpp
  src/synthetic_trajectory.cpp
)
target_compile_features(aqua_graph_core PUBLIC cxx_std_17)
target_include_directories(aqua_graph_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(aqua_graph_core gtsam gtsam_unstable Eigen3::Eigen)
set_target_properties(aqua_graph_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(aqua_graph_component SHARED src/aqua_graph_node.cpp)
target_compile_features(aqua_graph_component PUBLIC cxx_std_17)
target_link_libraries(aqua_graph_component aqua_graph_core aqua_imu_loc::pressure_depth_converter)
ament_target_dependencies(aqua_graph_component
  aqua_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  rclcpp_components
  sensor_msgs
)
rclcpp_components_register_node(aqua_graph_component
  PLUGIN "aqua_graph::AquaGraphNode"
  EXECUTABLE aqua_graph_node
)

install(TARGETS aqua_graph_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(TARGETS aqua_graph_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 GTSAM GTSAM_UNSTABLE)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_aqua_graph_factors test/test_factors.cpp)
  target_link_libraries(test_aqua_graph_factors aqua_graph_core)

  ament_add_gtest(test_aqua_graph_estimator_synthetic test/test_estimator_synthetic.cpp
    TIMEOUT 300)
  target_link_libraries(test_aqua_graph_estimator_synthetic aqua_graph_core)
endif()

ament_package()
