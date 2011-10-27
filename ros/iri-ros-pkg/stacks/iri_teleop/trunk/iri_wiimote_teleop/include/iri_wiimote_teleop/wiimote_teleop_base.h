// Copyright (C) 2010-2011 Institut de Robotica i Informatica Industrial, CSIC-UPC.
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

#ifndef _WIIMOTE_TELEOP_H
#define _WIIMOTE_TELEOP_H

#include <joystick_core/iri_base_joystick_node.h>
#include <wiimote/State.h>

class WiimoteTeleop : public iri_base_joystick::IriBaseJoystick
{
  public:
    WiimoteTeleop();
    ~WiimoteTeleop();
    
  protected:
    geometry_msgs::Twist generateTwist();
    void useButton(const unsigned int & index);
    virtual void useExtraButton(const unsigned int & index);
    void useAxes(const float & axe_value, const unsigned int & index);
};

#endif