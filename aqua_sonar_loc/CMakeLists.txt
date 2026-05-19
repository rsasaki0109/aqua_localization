cmake_minimum_required(VERSION 3.8)
project(aqua_sonar_loc)

if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
  set(CMAKE_POLICY_DEFAULT_CMP0144 NEW)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(aqua_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters registration)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(sonar_cloud_preprocessor src/sonar_cloud_preprocessor.cpp)
target_compile_features(sonar_cloud_preprocessor PUBLIC cxx_std_17)
target_include_directories(sonar_cloud_preprocessor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(sonar_cloud_preprocessor sensor_msgs)

add_library(scan_matcher src/scan_matcher.cpp)
target_compile_features(scan_matcher PUBLIC cxx_std_17)
target_include_directories(scan_matcher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(scan_matcher sonar_cloud_preprocessor ${PCL_LIBRARIES})
ament_target_dependencies(scan_matcher geometry_msgs pcl_conversions sensor_msgs)

add_executable(sonar_loc_node src/sonar_loc_node.cpp)
target_compile_features(sonar_loc_node PUBLIC cxx_std_17)
target_link_libraries(sonar_loc_node sonar_cloud_preprocessor scan_matcher)
ament_target_dependencies(sonar_loc_node aqua_msgs nav_msgs rclcpp sensor_msgs)

add_executable(mbes_loop_closure_node src/mbes_loop_closure_node.cpp)
target_compile_features(mbes_loop_closure_node PUBLIC cxx_std_17)
target_include_directories(mbes_loop_closure_node PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(mbes_loop_closure_node ${PCL_LIBRARIES})
ament_target_dependencies(
  mbes_loop_closure_node
  aqua_msgs
  geometry_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
)

install(TARGETS sonar_cloud_preprocessor scan_matcher
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(TARGETS sonar_loc_node mbes_loop_closure_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_sonar_cloud_preprocessor test/test_sonar_cloud_preprocessor.cpp)
  target_link_libraries(test_sonar_cloud_preprocessor sonar_cloud_preprocessor)

  ament_add_gtest(test_scan_matcher test/test_scan_matcher.cpp)
  target_link_libraries(test_scan_matcher scan_matcher)

  ament_add_gtest(test_sonar_loc_node_runtime test/test_sonar_loc_node_runtime.cpp)
  target_link_libraries(test_sonar_loc_node_runtime sonar_cloud_preprocessor scan_matcher)
  ament_target_dependencies(
    test_sonar_loc_node_runtime
    aqua_msgs
    nav_msgs
    rclcpp
    sensor_msgs
  )
endif()

ament_package()
