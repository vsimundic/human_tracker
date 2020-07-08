cmake_minimum_required(VERSION 2.8.3)
project(find_distance)

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
  pcl_ros
  roi_msgs
  roscpp
)

find_package(OpenCV 2 REQUIRED)
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
#  LIBRARIES find_distance
#  CATKIN_DEPENDS cv_bridge dynamic_reconfigure image_geometry image_transport opencv2 pcl_ros roi_msgs roscpp
#  DEPENDS system_lib
)


include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)
# include_directories(${Boost_INCLUDE_DIRS})
link_directories(${OpenCV_LIB_DIR})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_library(${PROJECT_NAME} 
  src/find_distance.cpp
)

add_executable(find_distance_node src/find_distance_node.cpp)
add_dependencies(find_distance_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(find_distance_node ${Boost_LIBRARIES})
target_link_libraries(find_distance_node 
${PROJECT_NAME} 
${catkin_LIBRARIES} 
${OpenCV_LIBS}
${PCL_LIBRARIES}
)



