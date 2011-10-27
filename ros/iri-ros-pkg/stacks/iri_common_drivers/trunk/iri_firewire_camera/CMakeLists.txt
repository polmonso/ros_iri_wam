cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set(PROJECT_NAME firewire_camera_driver_node)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

# added to include support for dynamic reconfiguration
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()
# end dynamic reconfiguration

FIND_PACKAGE(iriutils REQUIRED)
FIND_PACKAGE(firewire REQUIRED)

FIND_LIBRARY(raw1394_LIBRARY
    NAMES raw1394
    PATHS /usr/lib /usr/local/lib)
FIND_LIBRARY(dc1394_LIBRARY
    NAMES dc1394
    PATHS /usr/lib /usr/local/lib)

IF (raw1394_INCLUDE_DIR AND raw1394_LIBRARY AND dc1394_INCLUDE_DIR AND dc1394_LIBRARY)
   SET(firewire_ready TRUE)
ENDIF (raw1394_INCLUDE_DIR AND raw1394_LIBRARY AND dc1394_INCLUDE_DIR AND dc1394_LIBRARY)

INCLUDE_DIRECTORIES(${iriutils_INCLUDE_DIR} ${dc1394_INCLUDE_DIR} ${raw1394_INCLUDE_DIR} ${firewire_INCLUDE_DIR} ./include)

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(${PROJECT_NAME} src/firewire_camera_driver.cpp src/firewire_camera_driver_node.cpp)

TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${dc1394_LIBRARY} ${raw1394_LIBRARY} ${firewire_LIBRARY} ${iriutils_LIBRARY})
