cmake_minimum_required(VERSION 2.8.3)
project(tobii_driver_bridge)

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
set(PKG_DEPENDS
  rospy
  #sensor_msgs
  #cv_bridge
  #image_transport
  #geometry_msgs
  tobii_bridge_msgs
  )

set(BUILD_DEPENDS
  ${PKG_DEPENDS}
  )

set(RUN_DEPENDS
  ${PKG_DEPENDS}
  )

#######################
## Find dependencies ##
#######################
find_package(catkin REQUIRED COMPONENTS ${BUILD_DEPENDS})

include_directories(
  ${catkin_INCLUDE_DIRS}
  )

catkin_package(
  INCLUDE_DIRS 
  CATKIN_DEPENDS ${RUN_DEPENDS}
  DEPENDS
  )
