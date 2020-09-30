cmake_minimum_required(VERSION 2.8.3)
project(roiViewer)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  qt_build
  roi_msgs
  roscpp
)
find_package(OpenCV 3 REQUIRED
# NO_MODULE
# PATHS /home/valentin/cv/usr/local
# NO_DEFAULT_PATH
)

#set the default path for built executables to the "bin" directory
# set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory

# set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# set(${OpenCV_INCLUDE_DIRS}
# 	/home/valentin/cv/usr/local/include
# 	/home/valentin/cv/usr/local/include/opencv2
# )
# set(${OpenCV_LIB_DIR}
#   /home/valentin/cv/usr/local/lib
# )
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES roiViewer
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
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${OpenCV_LIBRARIES}
)
include_directories(${Boost_INCLUDE_DIRS})
# rosbuild_include(qt_build qt-ros)
# rosbuild_prepare_qt4(QtCore QtGui) # Add the appropriate components to the component list here


#############################################################################
# Sections
#############################################################################

# file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
# file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
# file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/roiViewer/*.hpp)

# QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
# QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
# QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

#############################################################################
# Sources
#############################################################################

# file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

#############################################################################
# Binaries
#############################################################################

# rosbuild_add_executable(roiViewer ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
# rosbuild_add_boost_directories()
# target_link_libraries(roiViewer ${QT_LIBRARIES} boost_system boost_filesystem boost_signals)

# rosbuild_link_boost(roiViewer thread signals system)


