cmake_minimum_required(VERSION 3.0.2)
project(shamazon_robot)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  move_base
  nav_msgs
  move_base_msgs
  actionlib
  gazebo_msgs
  gazebo_ros
  message_generation
  
)

# generate_messages(DEPENDENCIES std_msgs)
###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES shamazon_robot
 CATKIN_DEPENDS move_base roscpp nav_msgs gazebo_msgs std_msgs move_base_msgs actionlib gazebo_ros
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
include
  ${catkin_INCLUDE_DIRS}
)
add_executable(shamazon_robot src/shamazon_robot.cpp)
target_link_libraries(shamazon_robot ${catkin_LIBRARIES})

## Add models folder to path
install(DIRECTORY models
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

  if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(shamazon_test
  test/shamazon_test.test
  test/shamazon_test.cpp
  )
  target_link_libraries(
    shamazon_test
  ${catkin_LIBRARIES}
  )
  add_dependencies(
    shamazon_test
  ${catkin_EXPORTED_TARGETS}
  )
endif()