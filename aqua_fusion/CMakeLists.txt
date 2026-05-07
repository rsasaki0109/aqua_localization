cmake_minimum_required(VERSION 3.8)
project(aqua_fusion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(aqua_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(loosely_coupled_fuser src/loosely_coupled_fuser.cpp)
target_compile_features(loosely_coupled_fuser PUBLIC cxx_std_17)
target_include_directories(loosely_coupled_fuser PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(loosely_coupled_fuser nav_msgs rclcpp)

add_executable(fusion_node src/fusion_node.cpp)
target_compile_features(fusion_node PUBLIC cxx_std_17)
target_link_libraries(fusion_node loosely_coupled_fuser)
ament_target_dependencies(fusion_node aqua_msgs geometry_msgs nav_msgs rclcpp tf2_ros)

install(TARGETS loosely_coupled_fuser
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(TARGETS fusion_node
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

  ament_add_gtest(test_loosely_coupled_fuser test/test_loosely_coupled_fuser.cpp)
  target_link_libraries(test_loosely_coupled_fuser loosely_coupled_fuser)

  ament_add_gtest(test_fusion_node_runtime test/test_fusion_node_runtime.cpp)
  target_link_libraries(test_fusion_node_runtime loosely_coupled_fuser)
  ament_target_dependencies(
    test_fusion_node_runtime
    aqua_msgs
    geometry_msgs
    nav_msgs
    rclcpp
    tf2_ros
  )
endif()

ament_package()
