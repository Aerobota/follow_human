cmake_minimum_required(VERSION 2.8.3)
project(follow_human)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs urg_node sensor_msgs joy cv_bridge image_transport)
find_package(PCL 1.3 REQUIRED COMPONENTS common io)

#find_package(OpenCV REQUIRED)

## add demu
include_directories(/usr/local/include ${catkin_INCLUDE_DIRS} )#${OpenCV_INCLUDE_DIRS}  )

catkin_package(
  INCLUDE_DIRS
  LIBRARIES follow_human
  CATKIN_DEPENDS roscpp geometry_msgs
  DEPENDS urg_node sensor_msgs roscpp
)

###########
## Build ##
###########

add_executable(follow_human src/follow_human.cpp)
target_link_libraries(follow_human ${catkin_LIBRARIES} )#${OpenCV_LIBRARIES} )

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
#install(PROGRAMS
#  scripts/move
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Mark executables and/or libraries for installation
install(TARGETS follow_human
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark all other useful stuff for installation
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
