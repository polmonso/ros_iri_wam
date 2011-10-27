cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set(PROJECT_NAME segway_rmp400_odom)

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

rosbuild_add_executable(${PROJECT_NAME} src/segway_rmp400_odom_2D.cpp)
rosbuild_add_executable(segway_rmp400_odom_2D_displacement_based
                        src/segway_rmp400_odom_2D_displacement_based.cpp)
rosbuild_add_executable(segway_rmp400_odom_2D_full_displacement_based
                        src/segway_rmp400_odom_2D_full_displacement_based.cpp)
rosbuild_add_executable(segway_rmp400_odom_3D src/segway_rmp400_odom_3D.cpp)
rosbuild_add_executable(segway_rmp400_odom_3Dexpmaps src/segway_rmp400_odom_3Dexpmaps.cpp)

rosbuild_add_executable(rosbag_status_generator tools/rosbag_status_generator.cpp)

target_link_libraries(${PROJECT_NAME} ${iriutils_LIBRARY})
target_link_libraries(segway_rmp400_odom_2D_displacement_based ${iriutils_LIBRARY})
target_link_libraries(segway_rmp400_odom_2D_full_displacement_based ${iriutils_LIBRARY})
target_link_libraries(segway_rmp400_odom_3D ${iriutils_LIBRARY})
target_link_libraries(segway_rmp400_odom_3Dexpmaps ${iriutils_LIBRARY})
target_link_libraries(rosbag_status_generator ${iriutils_LIBRARY})

