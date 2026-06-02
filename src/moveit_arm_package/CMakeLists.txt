cmake_minimum_required(VERSION 3.8)
project(moveit_arm_package)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(arm_controller src/ArmController.cpp)
ament_target_dependencies(arm_controller
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  moveit_msgs
)

add_executable(vision_node src/VisionNode.cpp)
ament_target_dependencies(vision_node
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)

install(TARGETS arm_controller vision_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY urdf launch rviz config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
