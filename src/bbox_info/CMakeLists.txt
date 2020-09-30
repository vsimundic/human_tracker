cmake_minimum_required(VERSION 2.8.3)
project(bbox_info)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dynamic_reconfigure
  # image_geometry
  image_transport
  # opencv2
  roi_msgs
  roscpp
)
find_package(OpenCV 3 REQUIRED)
message("OpenCV version: ${OpenCV_VERSION}")

find_package(PCL REQUIRED)
message("PCL version: ${PCL_VERSION}")
message("PCL INCLUDE PATH: ${PCL_INCLUDE_DIRS}")
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
#  LIBRARIES bbox_info
#  CATKIN_DEPENDS cv_bridge dynamic_reconfigure image_geometry image_transport opencv2 roi_msgs roscpp
#  DEPENDS system_lib
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
include_directories(${Boost_INCLUDE_DIRS})

link_directories(${OpenCV_LIB_DIR})
link_directories(${PCL_LIBRARY_DIRS})


add_definitions(${PCL_DEFINITIONS})
#target_link_libraries(pcd_write_test ${PCL_LIBRARIES})


add_library(${PROJECT_NAME} 
  src/bbox_info.cpp
)
add_executable(bbox_info_node src/bbox_info_node.cpp)
add_dependencies(bbox_info_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(bbox_info_node ${Boost_LIBRARIES})
target_link_libraries(bbox_info_node 
${PROJECT_NAME} 
${catkin_LIBRARIES} 
${OpenCV_LIBS}
${PCL_LIBRARIES}
)
