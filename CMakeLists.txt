cmake_minimum_required(VERSION 3.8)
project(agv_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# ── Core dependencies ──
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(agv_greenhouse_msgs QUIET)
find_package(OpenCV REQUIRED)

# ── Optional: CUDA for GPU depth filter ──
include(CheckLanguage)
check_language(CUDA)
if(CMAKE_CUDA_COMPILER)
  set(CUDA_AVAILABLE TRUE)
else()
  set(CUDA_AVAILABLE FALSE)
endif()

# ── Optional: Qt5 for GUI ──
find_package(Qt5 QUIET COMPONENTS Widgets OpenGL)

# ══════════════════════════════════════════════
# Depth Filter Node
# ══════════════════════════════════════════════
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
  std_msgs
  image_transport
  cv_bridge
  message_filters
  OpenCV
)

# ══════════════════════════════════════════════
# SLAM Monitor Node
# ══════════════════════════════════════════════
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
  geometry_msgs
  visualization_msgs
  tf2_ros
  OpenCV
)

# ══════════════════════════════════════════════
# Pipeline Watchdog Node
# ══════════════════════════════════════════════
add_executable(pipeline_watchdog_node
  src/pipeline_watchdog_node.cpp
)
target_include_directories(pipeline_watchdog_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(pipeline_watchdog_node
  rclcpp
  sensor_msgs
  std_msgs
  diagnostic_msgs
  nav_msgs
)

# ══════════════════════════════════════════════
# Map Quality Node
# ══════════════════════════════════════════════
add_executable(map_quality_node
  src/map_quality_node.cpp
)
target_include_directories(map_quality_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(map_quality_node
  rclcpp
  sensor_msgs
  std_msgs
  diagnostic_msgs
  nav_msgs
)

# ══════════════════════════════════════════════
# CUDA Depth Filter Library (optional)
# ══════════════════════════════════════════════
if(CUDA_AVAILABLE)
  message(STATUS "CUDA found: Building GPU depth filter kernels")
  enable_language(CUDA)
  set(CMAKE_CUDA_STANDARD 14)
  set(CMAKE_CUDA_ARCHITECTURES 87)  # Orin AGX SM 8.7

  add_library(depth_filter_cuda SHARED
    src/cuda/depth_filter_cuda.cu
  )
  target_include_directories(depth_filter_cuda PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  set_target_properties(depth_filter_cuda PROPERTIES
    CUDA_SEPARABLE_COMPILATION ON
    POSITION_INDEPENDENT_CODE ON
  )

  # Link CUDA library to depth filter node
  target_link_libraries(depth_filter_node depth_filter_cuda)
  target_compile_definitions(depth_filter_node PRIVATE HAS_CUDA_FILTER=1)

  install(TARGETS depth_filter_cuda
    LIBRARY DESTINATION lib
  )
else()
  message(STATUS "CUDA not found: GPU depth filter disabled, using CPU fallback")
endif()

# ══════════════════════════════════════════════
# SLAM GUI Node (optional, requires Qt5)
# ══════════════════════════════════════════════
if(Qt5_FOUND)
  message(STATUS "Qt5 found: Building SLAM GUI")
  set(CMAKE_AUTOMOC ON)

  # MOC needs to find the Q_OBJECT headers
  qt5_wrap_cpp(SLAM_GUI_MOC_SRCS
    include/agv_slam/slam_gui_node.hpp
  )

  add_executable(slam_gui_node
    src/slam_gui_node.cpp
    ${SLAM_GUI_MOC_SRCS}
  )
  target_include_directories(slam_gui_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(slam_gui_node
    Qt5::Widgets
    Qt5::OpenGL
    GL
  )
  ament_target_dependencies(slam_gui_node
    rclcpp
    sensor_msgs
    std_msgs
    diagnostic_msgs
    nav_msgs
    geometry_msgs
    cv_bridge
    tf2_ros
    tf2
    OpenCV
  )

  install(TARGETS slam_gui_node
    DESTINATION lib/${PROJECT_NAME}
  )
else()
  message(STATUS "Qt5 not found: SLAM GUI disabled")
endif()

# ══════════════════════════════════════════════
# Install
# ══════════════════════════════════════════════
install(TARGETS
  depth_filter_node
  slam_monitor_node
  pipeline_watchdog_node
  map_quality_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  rviz
  scripts
  urdf
  docs
  DESTINATION share/${PROJECT_NAME}
)

# Make scripts executable
install(PROGRAMS
  scripts/jetson_setup.sh
  scripts/validate_slam.sh
  scripts/coverage_monitor.py
  scripts/session_recorder.py
  scripts/check_storage.py
  scripts/export_session.sh
  DESTINATION lib/${PROJECT_NAME}
)

# ══════════════════════════════════════════════
# Testing
# ══════════════════════════════════════════════
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  # ── C++ unit tests ──

  # RateTracker (standalone, no ROS deps)
  ament_add_gtest(test_rate_tracker test/test_rate_tracker.cpp)
  target_include_directories(test_rate_tracker PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  # TegraStats parser (standalone, no ROS deps)
  ament_add_gtest(test_tegrastats_parser test/test_tegrastats_parser.cpp)
  target_include_directories(test_tegrastats_parser PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  # SlamMonitorNode (requires ROS + node source)
  ament_add_gtest(test_slam_monitor_node
    test/test_slam_monitor_node.cpp
    src/slam_monitor_node.cpp
  )
  target_include_directories(test_slam_monitor_node PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  target_compile_definitions(test_slam_monitor_node PRIVATE AGV_SLAM_BUILDING_TESTS)
  ament_target_dependencies(test_slam_monitor_node
    rclcpp
    sensor_msgs
    std_msgs
    diagnostic_msgs
    nav_msgs
    geometry_msgs
    visualization_msgs
    tf2_ros
    tf2
    OpenCV
  )

  # ── Python unit tests ──
  ament_add_pytest_test(test_coverage_monitor test/test_coverage_monitor.py)
  ament_add_pytest_test(test_session_recorder test/test_session_recorder.py)
  ament_add_pytest_test(test_check_storage test/test_check_storage.py)

  # ── Integration test (launch_testing) ──
  add_launch_test(test/test_pipeline_launch.py
    TIMEOUT 60
  )
endif()

ament_package()
