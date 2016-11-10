cmake_minimum_required(VERSION 2.8.3)
project(remote_monitor)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  cv_bridge
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
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
  CATKIN_DEPENDS roscpp rospy cv_bridge geometry_msgs nav_msgs sensor_msgs std_msgs tf waypoint_navigator
  DEPENDS OpenCV Boost
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(remote_monitor_server src/monitor_server.cpp)
add_dependencies(remote_monitor_server waypoint_navigator_generate_messages_cpp)
target_link_libraries(remote_monitor_server ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(remote_monitor_client src/monitor_client.cpp)
add_dependencies(remote_monitor_client waypoint_navigator_generate_messages_cpp)
target_link_libraries(remote_monitor_client ${catkin_LIBRARIES} ${OpenCV_LIBS})


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS third_robot_monitor third_robot_monitor_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_third_robot_monitor.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
