// Copyright (C) 2010-2011 Institut de Robotica i Informatica Industrial, CSIC-UPC.
// Author Jose Luis Rivero
// All rights reserved.
//
// This file is part of iri-ros-pkg
// iri-ros-pkg is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program. If not, see <http://www.gnu.org/licenses/>.
// 
// IMPORTANT NOTE: This code has been generated through a script from the 
// iri_ros_scripts. Please do NOT delete any comments to guarantee the correctness
// of the scripts. ROS topics can be easly add by using those scripts. Please
// refer to the IRI wiki page for more information:
// http://wikiri.upc.es/index.php/Robotics_Lab

#ifndef SEGWAY_RMP200_NODE_GUARD_INCLUDE_SEGWAY_RMP200_STATUS_H
#define SEGWAY_RMP200_NODE_GUARD_INCLUDE_SEGWAY_RMP200_STATUS_H

#include <iridrivers/segway_rmp200.h>
#include <iri_segway_rmp_msgs/SegwayRMP200Status.h>

namespace segway_rmp200_node 
{
   inline iri_segway_rmp_msgs::SegwayRMP200Status
   build_ROS_status_msg(const TSegwayRMP200Status segway_status)
    {
        iri_segway_rmp_msgs::SegwayRMP200Status ROS_status;

        ROS_status.left_wheel_velocity      = segway_status.left_wheel_velocity;
        ROS_status.right_wheel_velocity     = segway_status.right_wheel_velocity;
        ROS_status.pitch_angle              = segway_status.pitch_angle;
        ROS_status.pitch_rate               = segway_status.pitch_rate;
        ROS_status.roll_angle               = segway_status.roll_angle;
        ROS_status.roll_rate                = segway_status.roll_rate;
        ROS_status.yaw_rate                 = segway_status.yaw_rate;
        ROS_status.left_wheel_displacement  = segway_status.left_wheel_displ;
        ROS_status.right_wheel_displacement = segway_status.right_wheel_displ;
        ROS_status.forward_displacement     = segway_status.forward_displ;
        ROS_status.yaw_displacement         = segway_status.yaw_displ;
        ROS_status.uptime                   = segway_status.uptime;
        ROS_status.left_motor_torque        = segway_status.left_torque;
        ROS_status.right_motor_torque       = segway_status.right_torque;
        ROS_status.ui_battery               = segway_status.ui_battery;
        ROS_status.powerbase_battery        = segway_status.powerbase_battery;
        ROS_status.operation_mode           = segway_status.operation_mode;
        ROS_status.gain_schedule            = segway_status.gain_schedule;

        return ROS_status;
    }
}

#endif
