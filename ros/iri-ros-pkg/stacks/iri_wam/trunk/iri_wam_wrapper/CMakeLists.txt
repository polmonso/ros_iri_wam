cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set(PROJECT_NAME wam_driver_node)

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
FIND_PACKAGE(comm)
FIND_PACKAGE(CWamDriver REQUIRED)

INCLUDE_DIRECTORIES(${iriutils_INCLUDE_DIR} ${comm_INCLUDE_DIR} ${CWamDriver_INCLUDE_DIR} ./include)

FIND_PACKAGE(Eigen REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIRS})
ADD_DEFINITIONS(${EIGEN_DEFINITIONS})

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(${PROJECT_NAME} src/wam_driver.cpp src/wam_driver_node.cpp)
rosbuild_add_executable(simple_trajectory test/simple_trajectory/simple_trajectory.cpp)
rosbuild_add_executable(simple_80_points_trajectory test/simple_trajectory/simple_80_points_trajectory.cpp)
rosbuild_add_executable(lwpr_trajectory test/lwpr_trajectory/lwpr_trajectory.cpp)

target_link_libraries(${PROJECT_NAME} ${iriutils_LIBRARY} ${comm_LIBRARY} ${cwamdriver_LIBRARY})
target_link_libraries(simple_trajectory ${iriutils_LIBRARY} ${comm_LIBRARY} ${cwamdriver_LIBRARY})
target_link_libraries(simple_80_points_trajectory ${iriutils_LIBRARY} ${comm_LIBRARY} ${cwamdriver_LIBRARY})
target_link_libraries(lwpr_trajectory ${iriutils_LIBRARY} ${comm_LIBRARY} ${cwamdriver_LIBRARY})
