cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set(PROJECT_NAME pmdcamera_driver_node)

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

INCLUDE_DIRECTORIES(${iriutils_INCLUDE_DIR} ./include)

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(${PROJECT_NAME} src/pmdcamera_driver.cpp src/pmdcamera_driver_node.cpp src/dev_camcube.cpp src/pmdcamera_exceptions.cpp)

#target_link_libraries(${PROJECT_NAME} ${iriutils_LIBRARY} usb pmdaccess2)
target_link_libraries(${PROJECT_NAME} ${iriutils_LIBRARY} pmdaccess2)

rosbuild_add_executable(pmdCameraTfPublisher src/pmdCameraTfPublisher.cpp)
target_link_libraries(pmdCameraTfPublisher ${iriutils_LIBRARY})
rosbuild_add_executable(kinectCameraTfPublisher src/kinectCameraTfPublisher.cpp)
target_link_libraries(kinectCameraTfPublisher ${iriutils_LIBRARY})
rosbuild_add_executable(kinectCameraWamTfPublisher src/kinectCameraWamTfPublisher.cpp)
target_link_libraries(kinectCameraWamTfPublisher ${iriutils_LIBRARY})
