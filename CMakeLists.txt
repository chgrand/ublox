cmake_minimum_required(VERSION 2.6)
    
#Déclaration du projet
project(ublox)
#set(PROJECT_BINARY_DIR .)
set(EXECUTABLE_OUTPUT_PATH bin/)
set(LIBRARY_OUTPUT_PATH lib/)
#set(CMAKE_FILES_DIRECTORY build/)


#find_package(bullet)
#include_directories("/opt/include/")
#include_directories("/opt/include/bullet/")
#link_directories("/opt/lib")

add_library(gnss
	src/gps_time.cpp)

include_directories(src)
add_executable(ublox 
  src/ubx_protocol.cpp
  src/ublox_rover.cpp)

add_executable(ublox_base 
  src/ubx_protocol.cpp
  src/ublox_base.cpp)

target_link_libraries(ublox_base
  gnss)

#set_target_properties(hylos3 PROPERTIES COMPILE_FLAGS "-std=gnu++11" LINK_FLAGS "") #-std=c++11 or -std=gnu++11

#add_executable(osg_vrml src/osg_vrml.cpp)
#target_link_libraries(osg_vrml osg osgViewer )
#target_link_libraries(osg_vrml BulletDynamics LinearMath)
#target_link_libraries(osg_vrml BulletCollision GIMPACTUtils ConvexDecomposition)
#target_link_libraries(osg_vrml HACD BulletMultiThreaded MiniCL)
#set_target_properties(osg_vrml PROPERTIES COMPILE_FLAGS "-m32" LINK_FLAGS "-m32")
#
