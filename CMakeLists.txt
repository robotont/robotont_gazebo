cmake_minimum_required(VERSION 3.16)
project(robotont_gazebo)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ros_gz_sim REQUIRED)

include_directories(
  include
)

install(
  DIRECTORY launch materials meshes urdf models worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()