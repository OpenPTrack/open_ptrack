cmake_minimum_required(VERSION 2.8.3)
project(tracking)
find_package(catkin REQUIRED COMPONENTS roscpp rosconsole image_transport tf tf_conversions bayes pcl_ros detection opt_msgs)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIB_DIR})

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS} include ${catkin_INCLUDE_DIRS})

catkin_package(
   INCLUDE_DIRS 
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp bayes opencv2 pcl_ros detection tf tf_conversions opt_msgs
)

add_library(${PROJECT_NAME} src/munkres.cpp src/kalman_filter.cpp src/detection_source.cpp src/track.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

