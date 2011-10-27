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

#ifndef _IRI_BASE_JOYSTICK_H
#define _IRI_BASE_JOYSTICK_H

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

namespace iri_base_joystick
{

/**
 * \brief IRI ROS Base Joystick
 *
 * This class defines common behavior to all kinds of joysticks. For each 
 * joy::Joy message received from the joystick, current state inlcuding buttons
 * and axes is updated to determine if there is a new user event. If current
 * state is different from last one, the adequate buttons and axes trigger the
 * defined action. Once all joystick state has been analyzed, a new 
 * geometry_msgs::Twist is generated and published through its topic.
 *
 * Buttons and axes behavior as well as twist computation, are left to be
 * implemented on derivated classes.
 */
class IriBaseJoystick
{
  protected:
   /**
    * \brief node handle communication object
    *
    * This node handle is going to be used to create topics and services within
    * the node namespace. Additional node handles can be instantatied if 
    * additional namespaces are needed.
    */
    ros::NodeHandle nh_;

   /**
    * \brief twist publisher
    *
    * Platform geometry_msgs::Twist command publisher.
    */
    ros::Publisher  twist_pub_;

   /**
    * \brief joystick subscriber
    *
    * Joystick joy::Joy button and axes commands subscriber.
    */
    ros::Subscriber joy_sub_;
    
   /**
    * \brief translational velocity
    *
    * Translational platform velocity in [m/s] to be published with twist.
    */
    float vT_;
    
   /**
    * \brief rotational velocity
    *
    * Rotational platform velocity in [rad/s] to be published with twist.
    */
    float vR_;
    
   /**
    * \brief last pressed joysting buttons
    *
    * Vector with indexes to last joystick pressed buttons to avoid multiple 
    * commands without releasing a button.
    */
    std::vector<unsigned int> prev_buttons_;
    
   /**
    * \brief last used joystick axes
    *
    * Vector with indexes to last used axes to avoid multiple 
    * commands without releasing a button.
    */
    std::vector<unsigned int> prev_axes_;

  public:
   /**
    * \brief constructor
    *
    * Class attributes are initialized and topic communications defined.
    */
    IriBaseJoystick(void);
    
   /**
    * \brief destructor
    *
    * Left in blank, no need to free any dynamic memory.
    */
    ~IriBaseJoystick(void);

  protected:
   /**
    * \brief check pressed buttons
    *
    * This function receives a vector with the state from all joystick buttons.
    * If no button has been pushed, returns false, otherwise returns a vector
    * with the indexes for the pressed buttons.
    *
    * \param buttons vector with the current state of each button
    * \param index vector with the indexes for each pushed button
    * \return bool true if at least one button has been pressed, false otherwise
    */
    static bool check_pressed_button_callback(const std::vector<int> & buttons, std::vector<unsigned int> & index);

   /**
    * \brief check used axes
    *
    * This function receives a vector with the state from all joystick axes.
    * If none of the axes has been used, returns false, otherwise returns a 
    * vector with the indexes for the used axes.
    *
    * \param axes vector with the current state of each axe
    * \param index vector with the indexes for each used axe
    * \return bool true if at least one axe has been updated, false otherwise
    */
    static bool check_movement_axes_callback(const std::vector<float> & axes, std::vector<unsigned int> & index);
    
   /**
    * \brief compare index vectors
    *
    * This function given to indexes vectors compares whether both vectors have
    * the same data. If the vectors are equal returns true, otherwise returns
    * false.
    *
    * \param a first index vector to be compared
    * \param b second index vector to be compared
    * \return bool true if both vectors are equal, false otherwise
    */
    static bool compareIndexVectors(const std::vector<unsigned int> & a, const std::vector<unsigned int> & b);
    
   /**
    * \brief joystick message callback
    *
    * This callback is called whenever a new joystick message arrives. First of
    * all checks whether any button or axe has modified its state from respect
    * the previous iteration. If there are new events, calls the corresponding
    * button or axes abstract function to trigger the appropiate action. After
    * all, computes new twist message to be sent.
    *
    * \param joy first index vector to be compared
    */
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    
   /**
    * \brief generate twist
    *
    * This abstract function should fill up a twist message with current
    * translational and rotational velocities (vT, vR).
    *
    * \return Twist twist message with last updated state
    */
    virtual geometry_msgs::Twist generateTwist() = 0;
    
   /**
    * \brief use button
    *
    * This abstract function receives a single button index and updates current
    * state according to the behavior defined on the derived class.
    *
    * \param index joystick button index
    */
    virtual void useButton(const unsigned int & index) = 0;
    
   /**
    * \brief use button
    *
    * This abstract function receives the value of a single axe and its index
    * and triggers corresponding action defined on derived class.
    *
    * \param index joystick button index
    */
    virtual void useAxes(const float & axe_value, const unsigned int & index) = 0;

};

template <class Joystick>
int main(int argc, char **argv, std::string node_name)
{
  ROS_DEBUG("Teleop Joystick %s Launched", node_name.c_str());

  // ROS initialization
  ros::init(argc, argv, node_name);

  Joystick joystick;
  ros::spin();

  return 0;
}

}
#endif
