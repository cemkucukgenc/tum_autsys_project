cmake_minimum_required(VERSION 2.8.3)
project(larics_motion_planning)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    rospy
    roslib
    roscpp
    std_msgs
    geometry_msgs
    trajectory_msgs
    nav_msgs
    cmake_modules
    eigen_conversions
    message_generation
    ompl
    # aerial_manipulators_control
)

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(octomap REQUIRED)
find_package(OMPL)

#add_message_files(
#  FILES
#)

add_service_files(
  FILES
  CartesianTrajectory.srv
  MultiDofTrajectory.srv
  VisualizeState.srv
  ParabolicAirdropTrajectory.srv
  SaveOctomap.srv
  GetPlantBoxInspectionPoints.srv
  CheckStateValidity.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
  trajectory_msgs
)

catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
  LIBRARIES
  CATKIN_DEPENDS
  DEPENDS
)

include_directories( 
  include ${catkin_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS} 
    ${OMPL_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS}
  )

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")



