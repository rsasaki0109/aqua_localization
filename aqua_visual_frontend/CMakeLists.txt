cmake_minimum_required(VERSION 3.8)
project(aqua_visual_frontend)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs video calib3d)
find_package(aqua_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(stereo_tracker src/stereo_tracker.cpp src/stereo_rectifier.cpp)
target_compile_features(stereo_tracker PUBLIC cxx_std_17)
target_include_directories(stereo_tracker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(stereo_tracker ${OpenCV_LIBS} yaml-cpp)
set_target_properties(stereo_tracker PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(stereo_frontend_component SHARED src/stereo_frontend_node.cpp)
target_compile_features(stereo_frontend_component PUBLIC cxx_std_17)
target_link_libraries(stereo_frontend_component stereo_tracker)
ament_target_dependencies(stereo_frontend_component
  aqua_msgs message_filters rclcpp rclcpp_components sensor_msgs std_msgs)
rclcpp_components_register_node(stereo_frontend_component
  PLUGIN "aqua_visual_frontend::StereoFrontendNode"
  EXECUTABLE stereo_frontend_node
)

install(TARGETS stereo_tracker stereo_frontend_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_stereo_tracker test/test_stereo_tracker.cpp)
  target_link_libraries(test_stereo_tracker stereo_tracker)
  ament_add_gtest(test_stereo_rectifier test/test_stereo_rectifier.cpp)
  target_link_libraries(test_stereo_rectifier stereo_tracker)
endif()

ament_package()
