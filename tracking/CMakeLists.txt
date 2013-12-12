cmake_minimum_required(VERSION 2.8.3)
project(tracking)
find_package(catkin REQUIRED COMPONENTS roscpp rosconsole image_transport tf)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIB_DIR})

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS} include ${catkin_INCLUDE_DIRS})

catkin_package(
   INCLUDE_DIRS 
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp 
)

add_library(munkres src/munkres.cpp)
target_link_libraries(munkres ${catkin_LIBRARIES})

