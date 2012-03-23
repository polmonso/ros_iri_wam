cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set(ROS_BUILD_TYPE Debug)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#rosbuild_genmsg()


### MOVE ARM INTERFACE ####
rosbuild_add_executable(move_in_joints src/move_in_joints.cpp)
#rosbuild_add_executable(move_in_joints src/move_in_joint.cpp)
#rosbuild_add_executable(ompl_test src/test_ompl.cpp)
rosbuild_add_executable(move_in_pose src/move_in_pose.cpp)
rosbuild_add_executable(simple_move_joints src/simple_move_joints.cpp)



### MOVE ARM SOURCE ####
rosbuild_add_executable(iri_wam_move_arm src/move_arm_simple_action.cpp)
rosbuild_link_boost(iri_wam_move_arm thread)
