cmake_minimum_required(VERSION 2.8.3)
project(remote_monitor)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf
  waypoint_navigator
)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generate cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES third_robot_monitor
  CATKIN_DEPENDS roscpp cv_bridge geometry_msgs nav_msgs sensor_msgs tf waypoint_navigator
  DEPENDS OpenCV Boost
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(remote_monitor_server src/monitor_server.cpp)
add_dependencies(remote_monitor_server ${catkin_EXPORTED_TARGETS})
target_link_libraries(remote_monitor_server ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(remote_monitor_client src/monitor_client.cpp)
add_dependencies(remote_monitor_client ${catkin_EXPORTED_TARGETS})
target_link_libraries(remote_monitor_client ${catkin_LIBRARIES} ${OpenCV_LIBS})

##########
## Test ##
##########
find_package(roslaunch)
roslaunch_add_file_check(test/remote_monitor_client.test.xml)
roslaunch_add_file_check(test/remote_monitor_server.test.xml)
