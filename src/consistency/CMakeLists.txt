cmake_minimum_required(VERSION 2.8.3)
project(consistency)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  rospy
  std_msgs
  roi_msgs
  sensor_msgs
  message_filters
)
find_package(OpenCV 2 REQUIRED
NO_MODULE
PATHS /home/valentin/cv/usr/local
NO_DEFAULT_PATH
)
find_package(PCL REQUIRED)

#generate_messages(
#	DEPENDENCIES sensor_msgs
#)

# find_package(cv_bridge REQUIRED)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES consistency
 # CATKIN_DEPENDS roscpp rospy std_msgs roi_msgs 
#  DEPENDS system_lib
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

link_directories(${OpenCV_LIB_DIR})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/consistency.cpp
# )
add_library(consistency src/consistency.cpp src/consistency_nodelet.cpp)


## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(consistency_node src/consistency_node.cpp src/consistency.cpp)
# add_executable(consistency_node src/consistency_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(consistency_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  # ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
)
# target_link_libraries(${PROJECT_NAME} roi_msgs)
# target_link_libraries(consistency_node ${PROJECT_NAME} roi_msgs boost_system boost_filesystem boost_signals)
# target_link_libraries(${PROJECT_NAME} roi_msgs_node ${catkin_LIBRARIES})
# target_link_libraries(consistency_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
# target_link_libraries(roi_msgs)



