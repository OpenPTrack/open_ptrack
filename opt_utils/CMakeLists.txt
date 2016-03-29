cmake_minimum_required(VERSION 2.8.3)
project(opt_utils)
set(CMAKE_BUILD_TYPE RelWithDebInfo)
find_package(catkin REQUIRED COMPONENTS cmake_modules roscpp rosconsole image_transport cv_bridge opt_msgs pcl_ros pcl_conversions tf_conversions dynamic_reconfigure)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIB_DIR})

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS} include ${catkin_INCLUDE_DIRS})

find_package(PCL 1.7 REQUIRED)
include_directories(BEFORE ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Dynamic reconfigure support
generate_dynamic_reconfigure_options(cfg/HeatmapBuilder.cfg)

include_directories(cfg/cpp)

add_library(json src/json.cpp)
target_link_libraries(json ${catkin_LIBRARIES})

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME} json
   CATKIN_DEPENDS roscpp
)

add_library(${PROJECT_NAME} src/conversions.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(roi_viewer apps/roi_viewer.cpp)
target_link_libraries(roi_viewer boost_system boost_filesystem boost_signals ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS})

#add_executable(tracking_viewer apps/tracking_viewer.cpp)
#target_link_libraries(tracking_viewer boost_system boost_filesystem boost_signals ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(ros2udp_converter apps/ros2udp_converter.cpp src/udp_messaging.cpp)# src/json.cpp)
target_link_libraries(ros2udp_converter ${catkin_LIBRARIES} json)

add_executable(udp_listener apps/udp_listener.cpp src/udp_messaging.cpp)
target_link_libraries(udp_listener ${catkin_LIBRARIES})

add_executable(heatmap_builder apps/heatmap_builder.cpp)
add_dependencies(heatmap_builder ${PROJECT_NAME}_gencfg)
target_link_libraries(heatmap_builder ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(cloud_refinement apps/cloud_refinement.cpp)
target_link_libraries(cloud_refinement ${catkin_LIBRARIES} ${PCL_LIBRARIES})
