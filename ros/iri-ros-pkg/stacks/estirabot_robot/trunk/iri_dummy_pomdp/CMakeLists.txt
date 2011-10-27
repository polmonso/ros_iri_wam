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
INCLUDE_DIRECTORIES(${iriutils_INCLUDE_DIR})
FIND_PACKAGE(comm REQUIRED)
INCLUDE_DIRECTORIES(${comm_INCLUDE_DIR})
FIND_PACKAGE(pomdpplanner REQUIRED)
INCLUDE_DIRECTORIES(${pomdpplanner_INCLUDE_DIR})


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
rosbuild_add_executable(dummy_pomdp src/dummy_pomdp.cpp)
target_link_libraries(dummy_pomdp ${comm_LIBRARY} ${iriutils_LIBRARY} )
rosbuild_add_executable(pomdp src/pomdp.cpp)
target_link_libraries(pomdp ${pomdpplanner_LIBRARY} ${comm_LIBRARY} ${iriutils_LIBRARY} )
rosbuild_add_executable(fixed_strategy src/fixed_strategy.cpp)
target_link_libraries(fixed_strategy ${comm_LIBRARY} ${iriutils_LIBRARY} )
rosbuild_add_executable(pomdp_modeler src/pomdp_modeler.cpp)
target_link_libraries(pomdp_modeler ${pomdpplanner_LIBRARY} )
