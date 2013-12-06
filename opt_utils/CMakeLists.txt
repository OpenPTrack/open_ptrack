cmake_minimum_required(VERSION 2.8.3)
project(opt_utils)
find_package(catkin REQUIRED COMPONENTS roscpp rosconsole image_transport cv_bridge)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIB_DIR})

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
   INCLUDE_DIRS 
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp
)

add_executable(roi_viewer src/main.cpp)
target_link_libraries(roi_viewer boost_system boost_filesystem boost_signals ${catkin_LIBRARIES} ${OpenCV_LIBS})
