cmake_minimum_required(VERSION 2.8.3)
project(bayes)
find_package(catkin REQUIRED COMPONENTS roscpp )

link_libraries(/usr/lib/liblapack.so.3gf)

include_directories(include include/filter)

catkin_package(
   INCLUDE_DIRS 
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
                                 src/unsFlt.cpp
                                 include/random.hpp)
target_link_libraries(bayes ${catkin_LIBRARIES})

