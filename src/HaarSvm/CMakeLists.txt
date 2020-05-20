cmake_minimum_required(VERSION 2.8.3)
project(HaarSvm)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dynamic_reconfigure
  image_transport
  nodelet
  # opencv2
  roi_msgs
  roscpp
  rospy
)
find_package(OpenCV 2 REQUIRED)
message("OpenCV version: ${OpenCV_VERSION}")
#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(${OpenCV_INCLUDE_DIRS}
	/home/valentin/cv/usr/local/include
	/home/valentin/cv/usr/local/include/opencv2
)
set(${OpenCV_LIB_DIR}
  /home/valentin/cv/usr/local/lib
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES HaarSvm
#  CATKIN_DEPENDS cv_bridge dynamic_reconfigure image_transport nodelet opencv2 roi_msgs roscpp rospy
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
  ${OpenCV_INCLUDE_DIRS}
)
link_directories(${OpenCV_LIB_DIR})

add_library(${PROJECT_NAME} src/haarsvm.cpp src/haarsvm_nodelet.cpp)
target_link_libraries(${PROJECT_NAME} 
${catkin_LIBRARIES}
${OpenCV_LIBS}
# roi_msgs 
)

add_executable(HaarSvm8 src/haarsvm_node.cpp)
target_link_libraries(HaarSvm8 
 ${PROJECT_NAME}
 ${catkin_LIBRARIES}
 ${OpenCV_LIBS} 
 roi_msgs 
 boost_system 
 boost_signals)


