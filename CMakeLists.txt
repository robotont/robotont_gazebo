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

install(TARGETS gazebo_driver_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY launch media meshes urdf worlds DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
