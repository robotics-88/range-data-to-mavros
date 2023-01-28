cmake_minimum_required(VERSION 2.8.3)
project(depth_image_to_mavlink)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_geometry
  mavros_msgs
  message_filters
  roscpp
  roslib
  std_msgs
  tf2
  tf2_msgs
  tf2_ros
  tf2_geometry_msgs
)

find_package(OpenCV REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    cv_bridge
    image_geometry
    message_filters
    std_msgs
    tf2
    tf2_msgs
    tf2_ros
    tf2_geometry_msgs
)

###########
## Build ##
###########
# c++11 support required
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else()
  message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include
)

# Depth img conversion pkgs
add_executable(depth_image_to_mavlink_node
  src/depth_image_to_mavlink_node.cpp
)
add_library(depth_image_to_mavlink 
  src/depth_image_to_mavlink.cpp
)
add_dependencies(depth_image_to_mavlink ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(depth_image_to_mavlink_node 
  depth_image_to_mavlink
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

#############
## Install ##
#############

# install nodes
install(TARGETS depth_image_to_mavlink
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install roslaunch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)

  # test all launch files
  roslaunch_add_file_check(launch)
endif()
