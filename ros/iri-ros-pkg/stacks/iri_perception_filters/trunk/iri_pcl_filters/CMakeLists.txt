cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

FIND_PACKAGE(iriutils REQUIRED)
INCLUDE_DIRECTORIES(${IRIUTILS_INCLUDE_DIR})

FIND_PACKAGE(comm REQUIRED)
INCLUDE_DIRECTORIES(${comm_INCLUDE_DIR})

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

#rosbuild_add_executable(pcl_bb_filter src/bounding_box.cpp)
rosbuild_add_executable(pcl_je_filter src/jump_edge.cpp)

#rosbuild_add_executable(pcl_leaf_poi src/leaf_poi.cpp)
#rosbuild_add_library(segmentation src/segmentation.cpp src/matrices3.cpp src/geometria.cpp)

#rosbuild_add_executable(pcl_leaf_poi src/leaf_poi.cpp)
#rosbuild_add_library(segmentation src/segmentation.cpp src/matrices3.cpp src/geometria.cpp)
#target_link_libraries(pcl_leaf_poi segmentation)

rosbuild_add_executable(mofa_bridge src/mofa_bridge.cpp)
target_link_libraries(mofa_bridge ${iriutils_LIBRARY} ${comm_LIBRARY} )
rosbuild_add_executable(dummy_mofa_bridge src/dummy_mofa_bridge.cpp)
target_link_libraries(dummy_mofa_bridge ${iriutils_LIBRARY} ${comm_LIBRARY} )



