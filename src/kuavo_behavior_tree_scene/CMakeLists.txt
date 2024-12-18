cmake_minimum_required(VERSION 3.0.2)
project(kuavo_behavior_tree_scene)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  behaviortree_cpp_v3
)

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
  INCLUDE_DIRS 
    include
    ${catkin_INCLUDE_DIRS}
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
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

add_executable(robot_hotel_node src/robot_hotel_node.cpp)
target_link_libraries(robot_hotel_node
  ${catkin_LIBRARIES}
)

###########
## Test ##
###########
add_executable(timer_node src/example/timer_node.cpp)
add_executable(Blackboard_node src/example/Blackboard_node.cpp)
add_executable(Black_int_node src/example/Black_int_node.cpp)

target_link_libraries(timer_node
  ${catkin_LIBRARIES}
)

target_link_libraries(Blackboard_node
  ${catkin_LIBRARIES}
)

target_link_libraries(Black_int_node
  ${catkin_LIBRARIES}
)