cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_rrt)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roslib
  rospy
  std_msgs
  sensor_msgs
)

# The catkin_package macro generates cmake config files for your package
catkin_package (
  CATKIN_DEPENDS roslib rospy
)

# Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)
