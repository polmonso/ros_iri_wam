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

FIND_PACKAGE(Eigen2 REQUIRED)
FIND_PACKAGE(iriutils REQUIRED)
FIND_PACKAGE(comm)
FIND_PACKAGE(Xenomai REQUIRED)


INCLUDE_DIRECTORIES(${EIGEN2_INCLUDE_DIR})
ADD_DEFINITIONS(${EIGEN2_DEFINITIONS})
FIND_PACKAGE(wamdriver REQUIRED)
INCLUDE_DIRECTORIES(${iriutils_INCLUDE_DIR} ${comm_INCLUDE_DIR} ${wamDriver_INCLUDE_DIR} ${XENOMAI_INCLUDE_DIR} ./include)

#common commands for building c++ executables and libraries
rosbuild_add_executable(${PROJECT_NAME} src/wam_driver.cpp src/wam_driver_node.cpp)
rosbuild_add_executable(test_simple_trajectory test/simple_trajectory/simple_trajectory.cpp)
#rosbuild_add_executable(test_simple_80_points_trajectory test/simple_trajectory/simple_80_points_trajectory.cpp)
#rosbuild_add_executable(test_lwpr_trajectory test/lwpr_trajectory/lwpr_trajectory.cpp)
#rosbuild_add_executable(test_simple_cartesian_move test/simple_cartesian_move/simple_cartesian.cpp)
#rosbuild_add_executable(test_simple_dmp_trajectory test/dmp_trajectory/dmp_simple_trajectory.cpp)

target_link_libraries(${PROJECT_NAME} ${iriutils_LIBRARY} ${comm_LIBRARY} ${cwamdriver_LIBRARY} ${EIGEN2_LIBRARY} ${XENOMAI_LIBRARY_NATIVE} ${XENOAMI_LIBRARY_XENOMAI} ${XENOMAI_LIBRARY_RDTM})
target_link_libraries(test_simple_trajectory ${iriutils_LIBRARY} ${comm_LIBRARY} ${cwamdriver_LIBRARY})
#target_link_libraries(test_simple_80_points_trajectory ${iriutils_LIBRARY} ${comm_LIBRARY} ${cwamdriver_LIBRARY})
#target_link_libraries(test_lwpr_trajectory ${iriutils_LIBRARY} ${comm_LIBRARY} ${cwamdriver_LIBRARY})
#target_link_libraries(test_simple_cartesian_move ${iriutils_LIBRARY} ${comm_LIBRARY} ${cwamdriver_LIBRARY})
#target_link_libraries(test_simple_dmp_trajectory ${iriutils_LIBRARY} ${comm_LIBRARY} ${cwamdriver_LIBRARY})
