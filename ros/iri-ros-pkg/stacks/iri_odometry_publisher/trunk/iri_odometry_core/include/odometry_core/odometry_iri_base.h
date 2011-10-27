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

#ifndef _iri_base_odometry_h_
#define _iri_base_odometry_h_

#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/Transform.h>

#include "odometry_base.h"

namespace odometry_core
{
  /**
   * @class IriBaseOdometry
   * @brief This base class, implements the BaseOdometry interface providing common attributes for odometry computation.
   */
  template <class MsgType>
  class IriBaseOdometry : public odometry_core::BaseOdometry<MsgType>
  {
    protected:
      geometry_msgs::PoseWithCovariance pose_;
      geometry_msgs::TwistWithCovariance twist_;
      geometry_msgs::Transform transform_;
      ros::Time current_time_, last_time_;

    public:
      /**
       * @brief  Virtual destructor for the interface
       */
      IriBaseOdometry()
      {
      }

      /**
       * @brief Returns current pose message.
       * @return last computed robot position
       */
      geometry_msgs::PoseWithCovariance getPose(void) { return pose_; }

      /**
       * @brief Returns current twist message.
       * @return last computed robot twist
       */
      geometry_msgs::TwistWithCovariance getTwist(void) { return twist_; }

      /**
       * @brief Returns current transform message.
       * @return last computed robot transform
       */
      geometry_msgs::Transform getTransform(void) { return transform_; }

      /**
       * @brief Returns last odometry computation time.
       * @return time when last odometry was computed
       */
      ros::Time getOdomTime(void) { return current_time_; }
      
      /**
       * @brief Sets last message time value.
       */
      void setLastTime(ros::Time t) { last_time_ = t; }
      
      /**
       * @brief Computes odometry based on the new platform message.
       * @param msg self defined platform status message
       */
      virtual void computeOdometry(const MsgType& msg) = 0;

      /**
       * @brief  Virtual destructor for the interface
       */
      virtual ~IriBaseOdometry(){}
  };
};

#endif
