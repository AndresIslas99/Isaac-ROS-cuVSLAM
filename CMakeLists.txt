cmake_minimum_required(VERSION 3.8)
project(agv_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)

# ── Depth Filter Node ──
add_executable(depth_filter_node
  src/depth_filter_node.cpp
)
target_include_directories(depth_filter_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(depth_filter_node
  rclcpp
  sensor_msgs
  image_transport
  cv_bridge
  message_filters
  OpenCV
)

# ── SLAM Monitor Node ──
add_executable(slam_monitor_node
  src/slam_monitor_node.cpp
)
target_include_directories(slam_monitor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(slam_monitor_node
  rclcpp
  sensor_msgs
  std_msgs
  diagnostic_msgs
  nav_msgs
  tf2_ros
)

# ── Install ──
install(TARGETS
  depth_filter_node
  slam_monitor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  rviz
  scripts
  urdf
  DESTINATION share/${PROJECT_NAME}
)

# Make scripts executable
install(PROGRAMS
  scripts/jetson_setup.sh
  scripts/validate_slam.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
