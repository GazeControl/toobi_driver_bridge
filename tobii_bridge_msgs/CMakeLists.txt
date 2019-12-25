cmake_minimum_required(VERSION 2.8.3)
project(tobii_bridge_msgs)

############################
## Enable C++ 11 Features ##
############################
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

###########################
## setup build variables ##
###########################
set(CMake_BUILD_TYPE Debug)

set(PKG_DEPENDS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  )

set(BUILD_DEPENDS
  ${PKG_DEPENDS}
  message_generation
  )

set(RUN_DEPENDS
  ${PKG_DEPENDS}
  message_runtime
  )

#######################################
## Declare ROS messages and services ##
#######################################

find_package(catkin REQUIRED COMPONENTS ${BUILD_DEPENDS})

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#add_message_files(DIRECTORY msg
#  FILES
#  GazeCoordinate2D.msg
#  GazeCoordinate3D.msg
#  Gaze.msg
#  Fixation.msg
#  )

generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
  )

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS ${RUN_DEPENDS}
  )
