cmake_minimum_required(VERSION 2.8.3)
project(detection)
find_package(catkin REQUIRED COMPONENTS roscpp pcl_ros pcl_conversions nodelet rosconsole image_transport opt_utils)

find_package(VTK REQUIRED)
if(VTK_FOUND)
	include_directories(${VTK_INCLUDE_DIRS})
	link_directories(${VTK_LIBRARY_DIRS})
	add_definitions(${VTK_DEFINITIONS})
	include (${VTK_USE_FILE})
endif() 

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIB_DIR})

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS} include ${catkin_INCLUDE_DIRS})

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp pcl_ros pcl_conversions opt_utils opt_msgs
)


add_library(${PROJECT_NAME} src/detection_source.cpp src/detection.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(ground_based_people_detector apps/ground_based_people_detector_node.cpp)
target_link_libraries(ground_based_people_detector ${PROJECT_NAME} ${catkin_LIBRARIES} vtkHybrid vtkRendering)

add_library(haar_disp_ada src/haardispada.cpp apps/haardispada_nodelet.cpp)
target_link_libraries(haar_disp_ada ${catkin_LIBRARIES})

add_executable(HaarDispAda174 apps/haardispada_node.cpp)
target_link_libraries(HaarDispAda174 haar_disp_ada ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS} boost_system boost_signals)
