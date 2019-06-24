cmake_minimum_required(VERSION 2.8.3)
project(move_base_interpolate)

find_package(catkin REQUIRED COMPONENTS
  rospy
  tf
)

catkin_package(
 CATKIN_DEPENDS rospy tf
)
