// Copyright (C) 2011 Institut de Robotica i Informatica Industrial, CSIC-UPC.
// Author Joan Perez
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
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef _base_odometry_h_
#define _base_odometry_h_

#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/Transform.h>

namespace odometry_core
{
  /**
   * @class BaseOdometry
   * @brief Provides an interface for mobile platforms which want to publish an odometry message.
   */
  template <class MsgType>
  class BaseOdometry
  {
    public:
      /**
       * @brief Returns the current Pose of the robot
       * @return last computed robot position
       */
      virtual geometry_msgs::PoseWithCovariance getPose(void) = 0;

      /**
       * @brief Returns the current Twist of the robot
       * @return last computed robot twist
       */
      virtual geometry_msgs::TwistWithCovariance getTwist(void) = 0;

      /**
       * @brief Returns the current Transform of the robot
       * @return last computed robot transform
       */
      virtual geometry_msgs::Transform getTransform(void) = 0;

      /**
       * @brief Returns last odometry computation time.
       * @return time when last odometry was computed
       */
      virtual ros::Time getOdomTime(void) = 0;

      /**
       * @brief Computes odometry based on the new platform message.
       * @param msg self defined platform status message
       */
      virtual void computeOdometry(const MsgType& msg) = 0;

      /**
       * @brief  Virtual destructor for the interface
       */
      virtual ~BaseOdometry(){}

    protected:
      /**
       * @brief  Virtual destructor for the interface
       */
      BaseOdometry(){}
  };
};

#endif
