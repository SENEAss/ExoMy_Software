cmake_minimum_required(VERSION 3.8)
project(exomy)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(object_detection exomy/object_detection.cpp)

ament_target_dependencies(object_detection
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)

install(TARGETS
  object_detection
  DESTINATION lib/${PROJECT_NAME})

ament_package()
