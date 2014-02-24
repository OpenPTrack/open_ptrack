cmake_minimum_required(VERSION 2.8.3)
project(swissranger_camera)
set(CMAKE_BUILD_TYPE RelWithDebInfo)
find_package(catkin REQUIRED COMPONENTS roscpp tf camera_info_manager image_transport dynamic_reconfigure driver_base)

include_directories(include ${catkin_INCLUDE_DIRS} cfg/cpp)

generate_dynamic_reconfigure_options(cfg/SwissRanger.cfg)

catkin_package(
   INCLUDE_DIRS 
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp tf camera_info_manager image_transport dynamic_reconfigure driver_base
)

add_executable(${PROJECT_NAME} src/sr.cpp src/dev_sr.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} mesasr)
