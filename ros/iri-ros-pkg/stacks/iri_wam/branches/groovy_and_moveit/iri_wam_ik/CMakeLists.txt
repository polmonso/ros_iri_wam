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

#Eigen required    
find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})

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

rosbuild_add_executable(wam_ikac src/wam_ikac.cpp)
rosbuild_add_executable(kuka_ikac src/kuka_ikac.cpp)
#rosbuild_add_executable(wam_ikj src/wam_ikj.cpp)
#rosbuild_add_executable(wam_ik_kdl src/wam_ik_kdl.cpp)

#MESSAGE(STATUS ${wamik_LIBRARY})
#TARGET_LINK_LIBRARIES(wam_ik ${wamik_LIBRARY})

MESSAGE(STATUS)
MESSAGE(STATUS "IMPORTANT NOTE: This code is not well tested and probably incomplete")
MESSAGE(STATUS "It can break your system and, certainly, eat your cat")
MESSAGE(STATUS "Please, be sure of contact with the maintainers before using it:")
MESSAGE(STATUS " To: <pmonso@iri.upc.edu>")
MESSAGE(STATUS " CC: <labrobotica@iri.upc.edu>")
MESSAGE(STATUS)
