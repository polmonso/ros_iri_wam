cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

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
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)

FIND_PACKAGE(iriutils REQUIRED)

#rosbuild_add_executable(teleop_teo_iri_joy src/teleop_teo_joy.cpp)
#TARGET_LINK_LIBRARIES(teleop_teo_iri_joy ${iriutils_LIBRARY})

rosbuild_add_executable(teleop_teo_iri_wiimote src/teleop_teo_wiimote.cpp)
TARGET_LINK_LIBRARIES(teleop_teo_iri_wiimote ${iriutils_LIBRARY})

#rosbuild_add_executable(teleop_obstacle_filter src/teleop_obstacle_filter.cpp)
#TARGET_LINK_LIBRARIES(teleop_obstacle_filter ${iriutils_LIBRARY})

#rosbuild_add_executable(send_obstacle_message test/send_obstacle_message.cpp)
#TARGET_LINK_LIBRARIES(send_obstacle_message ${iriutils_LIBRARY})

MESSAGE(STATUS)
MESSAGE(STATUS "IMPORTANT NOTE: This driver is not well tested and probably incomplete")
MESSAGE(STATUS "It can break your system or in the worst case, eat your cat")
MESSAGE(STATUS "Plase, be sure of contact with the maintainers before using it:")
MESSAGE(STATUS " To: <jrivero@iri.upc.edu>")
MESSAGE(STATUS " CC: <labrobotica@iri.upc.edu>")
MESSAGE(STATUS)

