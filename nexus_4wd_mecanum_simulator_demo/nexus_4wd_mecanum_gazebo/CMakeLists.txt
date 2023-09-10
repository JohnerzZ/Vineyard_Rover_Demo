cmake_minimum_required(VERSION 3.0.2)
project(nexus_4wd_mecanum_gazebo)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  #gazebo-dev
  nav_msgs
  roscpp
  std_msgs
  std_srvs
  tf
  message_generation
)

include_directories(include ${catkin_INCLUDE_DIRS})

## Find gazebo
find_package(gazebo REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

## Find Boost
find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})


catkin_package(
    CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs tf message_runtime
    INCLUDE_DIRS include
    LIBRARIES
)

###########
## Build ##
###########
add_library(nexus_ros_force_based_move src/nexus_ros_force_based_move.cpp)
target_link_libraries(nexus_ros_force_based_move ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(TARGETS
  nexus_ros_force_based_move
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_nexus_4wd_mecanum_gazebo.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

################
## go_to_goal ##
################


include_directories(/usr/local/lib)
link_directories(/usr/local/lib)

add_executable(go_to_goal_x src/go_to_goal_x.cpp)
target_link_libraries(go_to_goal_x ${catkin_LIBRARIES})

add_executable(custom_waypoint_publisher1 src/custom_waypoint_publisher1.cpp)
target_link_libraries(custom_waypoint_publisher1 ${catkin_LIBRARIES})

add_executable(go_to_goal_x_y src/go_to_goal_x_y.cpp)
target_link_libraries(go_to_goal_x_y ${catkin_LIBRARIES})

add_executable(go_to_goal_x_y_yaw1 src/go_to_goal_x_y_yaw1.cpp)
target_link_libraries(go_to_goal_x_y_yaw1 ${catkin_LIBRARIES})

add_executable(from_file_waypoint_publisher1 src/from_file_waypoint_publisher1.cpp)
target_link_libraries(from_file_waypoint_publisher1 ${catkin_LIBRARIES})

add_executable(nav_goal_waypoint_publisher1 src/nav_goal_waypoint_publisher1.cpp)
target_link_libraries(nav_goal_waypoint_publisher1 ${catkin_LIBRARIES})

add_executable(go_to_goal_x_y_smooth src/go_to_goal_x_y_smooth.cpp)
target_link_libraries(go_to_goal_x_y_smooth ${catkin_LIBRARIES})


####################
## Python Scripts ##
####################

catkin_install_python(PROGRAMS scripts/image_saver.py scripts/udp_server.py scripts/udp_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


