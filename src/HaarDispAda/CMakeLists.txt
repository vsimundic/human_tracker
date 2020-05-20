cmake_minimum_required(VERSION 2.8.3)
project(HaarDispAda)

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
# set(OpenCV_DIR "/home/valentin/opencv")
find_package(OpenCV 2 REQUIRED)
message("OpenCV version: ${OpenCV_VERSION}")
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
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
#  LIBRARIES HaarDispAda
#  CATKIN_DEPENDS cv_bridge dynamic_reconfigure image_transport nodelet opencv2 roi_msgs roscpp rospy
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/HaarDispAda.cpp
# )

link_directories(${OpenCV_LIB_DIR})

add_library(${PROJECT_NAME} src/haardispada.cpp src/haardispada_nodelet.cpp)

add_executable(HaarDispAda174 src/haardispada_node.cpp)

target_link_libraries(HaarDispAda174 ${PROJECT_NAME} ${catkin_LIBRARIES} boost_system boost_signals ${OpenCV_LIBS})

