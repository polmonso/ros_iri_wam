# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

# compile CXX with /usr/bin/c++
CXX_FLAGS = -O2 -g -fPIC -I/home/irojas/ros/iri-ros-pkg/iri_wam/iri_wam_gazebo/include -I/opt/ros/electric/stacks/simulator_gazebo/gazebo_plugins/cfg/cpp -I/opt/ros/electric/stacks/simulator_gazebo/gazebo_plugins/include -I/opt/ros/electric/stacks/simulator_gazebo/gazebo_plugins/msg_gen/cpp/include -I/opt/ros/electric/stacks/simulator_gazebo/gazebo/include -I/usr/include/libxml2 -I/opt/ros/electric/stacks/simulator_gazebo/gazebo/gazebo/include -I/opt/ros/electric/stacks/simulator_gazebo/gazebo/msg_gen/cpp/include -I/opt/ros/electric/stacks/simulator_gazebo/gazebo/srv_gen/cpp/include -I/opt/ros/electric/stacks/physics_ode/parallel_quickstep/include -I/opt/ros/electric/stacks/simulator_gazebo/gazebo_msgs/msg_gen/cpp/include -I/opt/ros/electric/stacks/simulator_gazebo/gazebo_msgs/srv_gen/cpp/include -I/opt/ros/electric/stacks/driver_common/driver_base/include -I/opt/ros/electric/stacks/driver_common/driver_base/msg/cpp -I/opt/ros/electric/stacks/driver_common/driver_base/msg_gen/cpp/include -I/opt/ros/electric/stacks/diagnostics/self_test/include -I/opt/ros/electric/stacks/diagnostics/self_test/srv/cpp -I/opt/ros/electric/stacks/diagnostics/diagnostic_updater/include -I/opt/ros/electric/stacks/common_msgs/diagnostic_msgs/msg_gen/cpp/include -I/opt/ros/electric/stacks/common_msgs/diagnostic_msgs/srv_gen/cpp/include -I/opt/ros/electric/stacks/arm_navigation/kinematics_msgs/include -I/opt/ros/electric/stacks/arm_navigation/kinematics_msgs/msg_gen/cpp/include -I/opt/ros/electric/stacks/arm_navigation/kinematics_msgs/srv_gen/cpp/include -I/opt/ros/electric/stacks/control/control_msgs/include -I/opt/ros/electric/stacks/control/control_msgs/msg/cpp -I/opt/ros/electric/stacks/control/control_msgs/srv/cpp -I/opt/ros/electric/stacks/control/control_msgs/msg_gen/cpp/include -I/opt/ros/electric/stacks/common_msgs/trajectory_msgs/msg_gen/cpp/include -I/opt/ros/electric/stacks/common_msgs/geometry_msgs/msg_gen/cpp/include -I/opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp/include -I/opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp/msg_gen/cpp/include -I/opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp/srv_gen/cpp/include -I/opt/ros/electric/stacks/common_msgs/actionlib_msgs/msg_gen/cpp/include -I/opt/ros/electric/stacks/arm_navigation/planning_environment/include -I/opt/ros/electric/stacks/common/actionlib/include -I/opt/ros/electric/stacks/common/actionlib/msg_gen/cpp/include -I/opt/ros/electric/stacks/arm_navigation/arm_navigation_msgs/include -I/opt/ros/electric/stacks/arm_navigation/arm_navigation_msgs/msg_gen/cpp/include -I/opt/ros/electric/stacks/arm_navigation/arm_navigation_msgs/srv_gen/cpp/include -I/opt/ros/electric/stacks/arm_navigation/collision_space/include -I/opt/ros/electric/stacks/arm_navigation/planning_models/include -I/opt/ros/electric/stacks/physics_ode/opende/opende/include -I/opt/ros/electric/stacks/physics_ode/opende/threadpool -I/opt/ros/electric/stacks/perception_pcl/pcl_ros/include -I/opt/ros/electric/stacks/perception_pcl/pcl_ros/cfg/cpp -I/opt/ros/electric/stacks/nodelet_core/nodelet_topic_tools/include -I/opt/ros/electric/stacks/nodelet_core/nodelet/include -I/opt/ros/electric/stacks/nodelet_core/nodelet/srv_gen/cpp/include -I/opt/ros/electric/stacks/bond_core/bondcpp/include -I/opt/ros/electric/stacks/bond_core/bond/msg_gen/cpp/include -I/opt/ros/electric/stacks/bond_core/smclib/include -I/opt/ros/electric/stacks/driver_common/dynamic_reconfigure/include -I/opt/ros/electric/stacks/driver_common/dynamic_reconfigure/msg/cpp -I/opt/ros/electric/stacks/driver_common/dynamic_reconfigure/srv/cpp -I/opt/ros/electric/stacks/driver_common/dynamic_reconfigure/msg_gen/cpp/include -I/opt/ros/electric/stacks/driver_common/dynamic_reconfigure/srv_gen/cpp/include -I/opt/ros/electric/stacks/arm_navigation/robot_self_filter/include -I/opt/ros/electric/stacks/filters/include -I/opt/ros/electric/stacks/arm_navigation/geometric_shapes/include -I/opt/ros/electric/stacks/perception_pcl/pcl/include/pcl-1.1 -I/usr/include/vtk-5.4 -I/usr/lib/openmpi/include -I/usr/lib/openmpi/include/openmpi -I/usr/include/tcl8.5 -I/usr/include/python2.6 -I/usr/lib/jvm/default-java/include -I/usr/include/freetype2 -I/opt/ros/electric/stacks/perception_pcl/pcl/msg_gen/cpp/include -I/opt/ros/electric/stacks/common_msgs/sensor_msgs/include -I/opt/ros/electric/stacks/common_msgs/sensor_msgs/msg_gen/cpp/include -I/opt/ros/electric/stacks/common_msgs/sensor_msgs/srv_gen/cpp/include -I/opt/ros/electric/stacks/perception_pcl/cminpack/include -I/opt/ros/electric/stacks/perception_pcl/flann/include -I/opt/ros/electric/stacks/visualization/rviz/src -I/opt/ros/electric/stacks/visualization_common/ogre_tools/src -I/opt/ros/electric/stacks/visualization_common/ogre/ogre/include -I/opt/ros/electric/stacks/visualization_common/ogre/ogre/include/OGRE -I/opt/ros/electric/stacks/visualization_common/ogre/ogre/include/OGRE/RTShaderSystem -I/opt/ros/electric/stacks/ros_comm/messages/std_srvs/srv_gen/cpp/include -I/opt/ros/electric/stacks/common_msgs/nav_msgs/msg_gen/cpp/include -I/opt/ros/electric/stacks/common_msgs/nav_msgs/srv_gen/cpp/include -I/opt/ros/electric/stacks/laser_pipeline/laser_geometry/include -I/usr/include/eigen3 -I/opt/ros/electric/stacks/visualization/wxpropgrid/propgrid_install/include -I/opt/ros/electric/stacks/rx/rxtools/src -I/usr/include/yaml-cpp -I/opt/ros/electric/stacks/robot_model/urdf/include -I/opt/ros/electric/stacks/robot_model/urdf_parser/include -I/opt/ros/electric/stacks/robot_model/collada_parser/include -I/opt/ros/electric/stacks/robot_model/urdf_interface/include -I/opt/ros/electric/stacks/robot_model/colladadom/include -I/opt/ros/electric/stacks/robot_model/colladadom/include/1.5 -I/opt/ros/electric/stacks/robot_model/resource_retriever/include -I/opt/ros/electric/stacks/image_common/image_transport/include -I/opt/ros/electric/stacks/pluginlib/include -I/opt/ros/electric/stacks/pluginlib -I/opt/ros/electric/stacks/visualization/interactive_markers/include -I/opt/ros/electric/stacks/common_msgs/visualization_msgs/msg_gen/cpp/include -I/opt/ros/electric/stacks/geometry/tf/include -I/opt/ros/electric/stacks/geometry/tf/msg_gen/cpp/include -I/opt/ros/electric/stacks/geometry/tf/srv_gen/cpp/include -I/opt/ros/electric/stacks/geometry/angles/include -I/opt/ros/electric/stacks/ros_comm/tools/rosbag/include -I/opt/ros/electric/stacks/ros_comm/tools/topic_tools/include -I/opt/ros/electric/stacks/ros_comm/tools/topic_tools/srv_gen/cpp/include -I/opt/ros/electric/stacks/ros_comm/utilities/message_filters/include -I/opt/ros/electric/stacks/ros_comm/tools/rostest/include -I/opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp_serialization/include -I/opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp_traits/include -I/opt/ros/electric/stacks/ros_comm/utilities/xmlrpcpp/src -I/opt/ros/electric/stacks/ros_comm/tools/rosconsole/include -I/opt/ros/electric/stacks/ros_comm/utilities/rostime/include -I/opt/ros/electric/stacks/ros_comm/utilities/cpp_common/include -I/opt/ros/electric/stacks/ros_comm/messages/rosgraph_msgs/msg_gen/cpp/include -I/opt/ros/electric/stacks/ros_comm/messages/std_msgs/include -I/opt/ros/electric/stacks/ros_comm/messages/std_msgs/msg_gen/cpp/include -I/opt/ros/electric/ros/core/roslib/msg_gen/cpp/include -I/opt/ros/electric/ros/core/roslib/include -I/opt/ros/electric/ros/tools/rospack -I/opt/ros/electric/ros/tools/rospack/include -I/opt/ros/electric/stacks/bullet/include   -DROS_PACKAGE_NAME='"iri_wam_gazebo"'

CXX_DEFINES = -Dwam_gazebo_EXPORTS -DBOOST_CB_DISABLE_DEBUG

# TARGET_FLAGS = -DdDOUBLE -pthread -D__NOTWXPYTHON__ -DSWIG_TYPE_TABLE=_wxPython_table -DWXP_USE_THREAD=1 -DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread

