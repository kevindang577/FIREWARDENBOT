cmake_minimum_required(VERSION 3.8)
project(firewardenbot_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# Python computer vision nodes
install(PROGRAMS
  scripts/foliage_hazard_node.py
  scripts/object_detection_node.py
  scripts/image_processing_node.py
  scripts/stereo_vision_node.py
  scripts/vision_aggregator.py
  DESTINATION lib/${PROJECT_NAME}
)

# C++ computer vision nodes
add_executable(fast_foliage_detector src/fast_foliage_detector.cpp)
ament_target_dependencies(fast_foliage_detector
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  OpenCV
)

# Install executables
install(TARGETS
  fast_foliage_detector
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
