cmake_minimum_required(VERSION 2.8.3)
project(detection)
set(CMAKE_BUILD_TYPE RelWithDebInfo)
find_package(catkin REQUIRED COMPONENTS cmake_modules roscpp rospy pcl_ros pcl_conversions nodelet rosconsole image_transport opt_utils opt_msgs dynamic_reconfigure)

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
add_definitions(${OpenCV_DEFINITIONS})

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS} include ${catkin_INCLUDE_DIRS})

find_package(PCL 1.7 REQUIRED)
include_directories(BEFORE ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
MESSAGE("PCL_INCLUDE_DIRS\n${PCL_INCLUDE_DIRS}\n")
MESSAGE("PCL_LIBRARY_DIRS\n${PCL_LIBRARY_DIRS}\n")
MESSAGE("PCL_DEFINITIONS\n${PCL_DEFINITIONS}\n")
MESSAGE("PCL_COMMON_LIBRARIES\n${PCL_COMMON_LIBRARIES}\n")
if (NOT PCL_FOUND)
    MESSAGE(FATAL_ERROR "PCL not found.\n")
endif (NOT PCL_FOUND)

# Dynamic reconfigure support
generate_dynamic_reconfigure_options(cfg/HaarDispAdaDetector.cfg cfg/GroundBasedPeopleDetector.cfg)

include_directories(cfg/cpp)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp pcl_ros pcl_conversions opt_utils opt_msgs
)


add_library(${PROJECT_NAME} src/detection_source.cpp src/detection.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(ground_based_people_detector apps/ground_based_people_detector_node.cpp)
SET_TARGET_PROPERTIES(ground_based_people_detector PROPERTIES LINK_FLAGS -L${PCL_LIBRARY_DIRS})
target_link_libraries(ground_based_people_detector ${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES} vtkHybrid vtkRendering)

add_executable(ground_based_people_detector_sr apps/ground_based_people_detector_node_sr.cpp)
SET_TARGET_PROPERTIES(ground_based_people_detector_sr PROPERTIES LINK_FLAGS -L${PCL_LIBRARY_DIRS})
target_link_libraries(ground_based_people_detector_sr ${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES} vtkHybrid vtkRendering)

add_library(haar_disp_ada src/haardispada.cpp apps/haardispada_nodelet.cpp)
target_link_libraries(haar_disp_ada ${catkin_LIBRARIES})

add_executable(HaarDispAda174 apps/haardispada_node.cpp)
target_link_libraries(HaarDispAda174 haar_disp_ada ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS} boost_system boost_signals)

add_executable(HaarDispAda174_sr apps/haardispada_node_sr.cpp)
target_link_libraries(HaarDispAda174_sr haar_disp_ada ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS} boost_system boost_signals)
