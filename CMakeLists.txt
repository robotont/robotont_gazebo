cmake_minimum_required(VERSION 3.0.2)
project(robotont_gazebo)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  joy
  nav_msgs
  cmake_modules
  tf
)

catkin_package(
  CATKIN_DEPENDS
    std_msgs
    joy
    nav_msgs
    tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(gazebo_robotont_teleop_joy src/gazebo_robotont_teleop_joy.cpp) 
add_executable(gazebo_driver_node src/gazebo_driver.cpp) 
add_executable(gazebo_odom_node src/gazebo_odom.cpp) 

target_link_libraries(gazebo_robotont_teleop_joy ${catkin_LIBRARIES})
target_link_libraries(gazebo_driver_node ${catkin_LIBRARIES})
target_link_libraries(gazebo_odom_node ${catkin_LIBRARIES})
