cmake_minimum_required(VERSION 3.8)
project(aqua_pose_graph)

if(POLICY CMP0072)
  cmake_policy(SET CMP0072 NEW)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(aqua_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(g2o REQUIRED)

add_library(aqua_pose_graph_lib
  src/pose_graph.cpp
)
target_include_directories(aqua_pose_graph_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(aqua_pose_graph_lib
  Eigen3::Eigen
  g2o::core
  g2o::solver_eigen
  g2o::types_slam3d
)
ament_target_dependencies(aqua_pose_graph_lib
  rclcpp
)

add_executable(pose_graph_node
  src/pose_graph_node.cpp
)
target_link_libraries(pose_graph_node aqua_pose_graph_lib)
ament_target_dependencies(pose_graph_node
  aqua_msgs
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

install(TARGETS pose_graph_node
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS aqua_pose_graph_lib
  EXPORT aqua_pose_graph_libExport
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_pose_graph
    test/test_pose_graph.cpp
  )
  target_link_libraries(test_pose_graph aqua_pose_graph_lib)
endif()

ament_export_include_directories(include)
ament_export_libraries(aqua_pose_graph_lib)
ament_package()
