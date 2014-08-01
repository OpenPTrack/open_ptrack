cmake_minimum_required(VERSION 2.8.3)
project(opt_calibration)
set(CMAKE_BUILD_TYPE Release)#RelWithDebInfo)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS cmake_modules roscpp rospy calibration_common kinect camera_info_manager cv_bridge image_transport tf_conversions)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Boost REQUIRED)
find_package(Eigen REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)

###################################################
## Declare things to be passed to other projects ##
###################################################
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES
  CATKIN_DEPENDS roscpp calibration_common camera_info_manager cv_bridge image_transport
  DEPENDS eigen pcl opencv2
)

###########
## Build ##
###########
## Specify additional locations of header files
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${BOOST_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

## Declare a cpp library
add_executable(calibration_initializer apps/calibration_initializer.cpp)
target_link_libraries(calibration_initializer ${catkin_LIBRARIES})

add_executable(calibration_saver apps/calibration_saver.cpp)
target_link_libraries(calibration_saver ${catkin_LIBRARIES})

#add_executable(opt_calibration src/multicamera_calibration.cpp
#  include/open_ptrack/opt_calibration/multicamera_calibration.h
#)

add_executable(opt_calibration
  src/opt_calibration.cpp   include/open_ptrack/opt_calibration/opt_calibration.h
  src/calibration_node.cpp  include/open_ptrack/opt_calibration/calibration_node.h
)

## Add dependencies to the executable
## Specify libraries to link a library or executable target against
target_link_libraries(opt_calibration
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)

