cmake_minimum_required(VERSION 2.8.3)
project(opt_msgs)
find_package(catkin REQUIRED COMPONENTS roscpp message_generation)

add_message_files(FILES Rois.msg RoiRect.msg HumanEntry.msg HumanEntries.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
   INCLUDE_DIRS 
   LIBRARIES opt_msgs
   CATKIN_DEPENDS roscpp
)

include_directories(include include/open_ptrack/${PROJECT_NAME}/)

add_library(roi_msgs src/overlap.cpp )

target_link_libraries(roi_msgs ${catkin_LIBRARIES})
