cmake_minimum_required(VERSION 2.8.3)
project(roiPlayer)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
# ADD_DEFINITIONS(-DQT_NO_KEYWORDS)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  qt_build
  roi_msgs
  roscpp
  sensor_msgs
)
find_package(OpenCV 2 REQUIRED
NO_MODULE
PATHS /home/valentin/cv/usr/local
NO_DEFAULT_PATH
)

set(${OpenCV_INCLUDE_DIRS}
	/home/valentin/cv/usr/local/include
	/home/valentin/cv/usr/local/include/opencv2
)
set(${OpenCV_LIB_DIR}
  /home/valentin/cv/usr/local/lib
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES roiPlayer
#  CATKIN_DEPENDS cv_bridge image_transport qt_build roi_msgs roscpp
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}

)

link_directories(${OpenCV_LIB_DIR})
add_executable(${PROJECT_NAME}_node src/main.cpp)
# add_executable(roiPlayer ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})

# add_boost_directories()
## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${OpenCV_LIBRARIES}
)
# target_link_libraries(roiPlayer ${QT_LIBRARIES} boost_system boost_filesystem boost_signals)
include_directories(${Boost_INCLUDE_DIRS})


# file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
# file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
# file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/roiPlayer/*.hpp)

# QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
# QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
# QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

# file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)


