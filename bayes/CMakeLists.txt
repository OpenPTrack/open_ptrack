cmake_minimum_required(VERSION 2.8.3)
project(bayes)
set(CMAKE_BUILD_TYPE RelWithDebInfo)
find_package(catkin REQUIRED COMPONENTS roscpp )

link_libraries(/usr/lib/liblapack.so.3gf)

include_directories(include include/open_ptrack/bayes include/open_ptrack/bayes/filters)

catkin_package(
   INCLUDE_DIRS include include/open_ptrack/bayes include/open_ptrack/bayes/filters
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp 
)

add_library(bayes src/bayesFlt.cpp
                                 src/bayesFltAlg.cpp
                                 src/CIFlt.cpp
                                 src/covFlt.cpp
                                 src/infFlt.cpp
                                 src/infRtFlt.cpp
                                 src/itrFlt.cpp
                                 src/matSup.cpp
                                 src/SIRFlt.cpp
                                 src/UDFlt.cpp
                                 src/UdU.cpp
                                 src/unsFlt.cpp)
target_link_libraries(bayes ${catkin_LIBRARIES})

