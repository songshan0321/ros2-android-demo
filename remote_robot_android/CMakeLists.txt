cmake_minimum_required(VERSION 2.8.3)
project(remote_robot_android)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  serial
  roslaunch
)

roslaunch_add_file_check(launch)

# Initialise the export variables
catkin_package(
  INCLUDE_DIRS src
  # LIBRARIES blah
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs rmf_msgs serial
  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# One per executable!
add_executable(remote_robot_android src/remote_robot_android.cpp)
target_link_libraries(remote_robot_android ${catkin_LIBRARIES})


install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
