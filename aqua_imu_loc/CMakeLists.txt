cmake_minimum_required(VERSION 3.8)
project(aqua_imu_loc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(aqua_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(additive_ukf src/additive_ukf.cpp)
target_compile_features(additive_ukf PUBLIC cxx_std_17)
target_include_directories(additive_ukf PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(pressure_depth_converter src/pressure_depth_converter.cpp)
target_compile_features(pressure_depth_converter PUBLIC cxx_std_17)
target_include_directories(pressure_depth_converter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(imu_preprocessor src/imu_preprocessor.cpp)
target_compile_features(imu_preprocessor PUBLIC cxx_std_17)
target_include_directories(imu_preprocessor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(imu_preprocessor additive_ukf)

add_library(static_bias_initializer src/static_bias_initializer.cpp)
target_compile_features(static_bias_initializer PUBLIC cxx_std_17)
target_include_directories(static_bias_initializer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(static_bias_initializer additive_ukf)

add_executable(imu_loc_node src/imu_loc_node.cpp)
target_compile_features(imu_loc_node PUBLIC cxx_std_17)
target_link_libraries(imu_loc_node additive_ukf pressure_depth_converter imu_preprocessor static_bias_initializer)
ament_target_dependencies(imu_loc_node
  aqua_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
)

add_executable(depth_to_pressure_node src/depth_to_pressure_node.cpp)
target_compile_features(depth_to_pressure_node PUBLIC cxx_std_17)
ament_target_dependencies(depth_to_pressure_node rclcpp sensor_msgs std_msgs)

add_executable(scalar_to_pressure_node src/scalar_to_pressure_node.cpp)
target_compile_features(scalar_to_pressure_node PUBLIC cxx_std_17)
ament_target_dependencies(scalar_to_pressure_node rclcpp sensor_msgs std_msgs)

install(TARGETS additive_ukf pressure_depth_converter imu_preprocessor static_bias_initializer
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(TARGETS imu_loc_node depth_to_pressure_node scalar_to_pressure_node
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

  ament_add_gtest(test_additive_ukf test/test_additive_ukf.cpp)
  target_link_libraries(test_additive_ukf additive_ukf)
  target_include_directories(test_additive_ukf PRIVATE ${EIGEN3_INCLUDE_DIRS})

  ament_add_gtest(test_pressure_depth_converter test/test_pressure_depth_converter.cpp)
  target_link_libraries(test_pressure_depth_converter pressure_depth_converter)

  ament_add_gtest(test_imu_preprocessor test/test_imu_preprocessor.cpp)
  target_link_libraries(test_imu_preprocessor imu_preprocessor)
  target_include_directories(test_imu_preprocessor PRIVATE ${EIGEN3_INCLUDE_DIRS})

  ament_add_gtest(test_static_bias_initializer test/test_static_bias_initializer.cpp)
  target_link_libraries(test_static_bias_initializer static_bias_initializer)
  target_include_directories(test_static_bias_initializer PRIVATE ${EIGEN3_INCLUDE_DIRS})

  ament_add_gtest(test_imu_loc_node_runtime test/test_imu_loc_node_runtime.cpp)
  target_link_libraries(
    test_imu_loc_node_runtime
    additive_ukf
    pressure_depth_converter
    imu_preprocessor
    static_bias_initializer
  )
  target_include_directories(test_imu_loc_node_runtime PRIVATE ${EIGEN3_INCLUDE_DIRS})
  ament_target_dependencies(
    test_imu_loc_node_runtime
    aqua_msgs
    geometry_msgs
    nav_msgs
    rclcpp
    sensor_msgs
    std_srvs
    tf2
    tf2_ros
  )

  ament_add_gtest(test_depth_to_pressure_node test/test_depth_to_pressure_node.cpp)
  ament_target_dependencies(test_depth_to_pressure_node rclcpp sensor_msgs std_msgs)

  ament_add_gtest(test_scalar_to_pressure_node test/test_scalar_to_pressure_node.cpp)
  ament_target_dependencies(test_scalar_to_pressure_node rclcpp sensor_msgs std_msgs)
endif()

ament_package()
