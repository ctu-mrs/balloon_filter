cmake_minimum_required(VERSION 2.8.3)
project(balloon_filter)

set(CMAKE_CXX_FLAGS "-std=c++17 -Wall -O0 -g")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamic_reconfigure
  mrs_msgs
  mrs_lib
  pcl_ros
  message_generation
  object_detect
  )

# include Eigen3
find_package(Eigen3 REQUIRED)

generate_dynamic_reconfigure_options(
  config/FilterParams.cfg
  )

#Generate services in the 'srv' folder
add_service_files(
   FILES
   StartEstimation.srv
   AddExclusionZone.srv
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
 )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES BalloonFilter
  CATKIN_DEPENDS
  #  DEPENDS system_lib
  )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  )

## Declare a C++ executable
add_library(BalloonFilter
  src/BalloonFilter.cpp
  )

add_dependencies(BalloonFilter
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(BalloonFilter
  ${catkin_LIBRARIES}
  )
