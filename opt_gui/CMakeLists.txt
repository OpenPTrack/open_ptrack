##############################################################################
# CMake
##############################################################################
INCLUDE(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
IF(COMPILER_SUPPORTS_CXX11)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ELSEIF(COMPILER_SUPPORTS_CXX0X)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
ELSE()
  MESSAGE(ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
ENDIF()
cmake_minimum_required(VERSION 2.8.0)
project(opt_gui)

##############################################################################
# Catkin
##############################################################################

# qt_build provides the qt cmake glue, roscpp the comms for a default talker
find_package(catkin REQUIRED COMPONENTS qt_build roscpp sensor_msgs image_transport cv_bridge opt_msgs)

set(QML_IMPORT_PATH "${QML_IMPORT_PATH};${CATKIN_GLOBAL_LIB_DESTINATION}" )
set(QML_IMPORT_PATH2 "${QML_IMPORT_PATH};${CATKIN_GLOBAL_LIB_DESTINATION}" )
include_directories(${catkin_INCLUDE_DIRS} include)
# Use this to define what the package will export (e.g. libs, headers).
# Since the default here is to produce only a binary, we don't worry about
# exporting anything.
#add_message_files(FILES Roi_object.msg Roi_object_array.msg)
#generate_messages(DEPENDENCIES std_msgs geometry_msgs sensor_msgs)

catkin_package(

   CATKIN_DEPENDS qt_build roscpp sensor_msgs image_transport cv_bridge
)

#generate_messages()
##############################################################################
# Qt Environment
##############################################################################

# this comes from qt_build's qt-ros.cmake which is automatically
# included via the dependency ca ll in package.xml
#rosbuild_prepare_qt4(QtCore Qtgui QtQml QtQuick) # Add the appropriate components to the component list here
find_package(Qt5 COMPONENTS Core Gui Qml Quick REQUIRED)

##############################################################################
# Sections
##############################################################################

file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/opt_gui/*.hpp include/component/*.hpp)

QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT5_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp src/component/*.cpp)

##############################################################################
# Binaries
##############################################################################

add_executable(opt_gui ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
qt5_use_modules(opt_gui Quick Core)
target_link_libraries(opt_gui ${QT_LIBRARIES} ${catkin_LIBRARIES})
target_include_directories(opt_gui PUBLIC include ${catkin_INCLUDE_DIRS})
add_dependencies(opt_gui ${catkin_EXPORTED_TARGETS})
install(TARGETS opt_gui RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
