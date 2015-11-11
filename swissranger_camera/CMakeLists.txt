cmake_minimum_required(VERSION 2.8.3)
project(swissranger_camera)
set(CMAKE_BUILD_TYPE RelWithDebInfo)
IF(EXISTS "/usr/lib/libmesasr.so")
	set(SWISSRANGER_ENABLED 1)
ELSE()
	set(SWISSRANGER_ENABLED 0)
ENDIF()
LIST(APPEND CMAKE_CXXFLAGS "-DSWISSRANGER_ENABLED")
find_package(catkin REQUIRED COMPONENTS cmake_modules roscpp tf camera_info_manager image_transport dynamic_reconfigure driver_base sensor_msgs cv_bridge)

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

generate_dynamic_reconfigure_options(cfg/SwissRanger.cfg)

include_directories(include ${catkin_INCLUDE_DIRS} cfg/cpp)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES swissranger_utility
   CATKIN_DEPENDS roscpp tf camera_info_manager image_transport dynamic_reconfigure driver_base
)
add_library(swissranger_utility src/swissranger_camera/utility.cpp
	include/swissranger_camera/utility.h)
target_link_libraries(swissranger_utility ${catkin_LIBRARIES})

IF(SWISSRANGER_ENABLED)
	add_executable(${PROJECT_NAME} src/sr.cpp src/dev_sr.cpp include/sr.h)
	add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)
	target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} mesasr)

	#add_library(swissranger_utility src/swissranger_camera/utility.cpp include/swissranger_camera/utility.h)
	#target_link_libraries(swissranger_utility ${catkin_LIBRARIES})

	add_executable(image_publisher_sr apps/main_image_publisher_sr.cpp)
	target_link_libraries(image_publisher_sr swissranger_utility ${catkin_LIBRARIES})
ENDIF(SWISSRANGER_ENABLED)
