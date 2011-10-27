#include "gamepad_teleop_node.h"

GamepadTeleop::GamepadTeleop()
{
}

GamepadTeleop::~GamepadTeleop()
{
}

geometry_msgs::Twist GamepadTeleop::generateTwist()
{
  geometry_msgs::Twist twist;

  twist.linear.x  = vT_;
  twist.angular.z = vR_;

  return twist;
}

void GamepadTeleop::useButton(const unsigned int & index)
{
  switch(index)
  {
    case stop_button_:
      ROS_DEBUG("STOP");
      vT_ = 0;
      vR_ = 0;
      break;
  }
}

void GamepadTeleop::useAxes(const float & axe_value, const unsigned int & index)
{
  switch(index)
  {
    case x_axis_id_:
      if(axe_value > 0.0)
      {
        ROS_DEBUG("RIGHT");
        vR_ -= 0.1;
      }
      else
      {
        ROS_DEBUG("LEFT");
        vR_ += 0.1;
      }
      break;

    case y_axis_id_:
      if (axe_value < 0.0)
      {
        ROS_DEBUG("UP");
        vT_ += 0.1;
      }
      else
      {
        ROS_DEBUG("DOWN");
        vT_ -= 0.1;
      }
      break;
  }
}

int main(int argc,char *argv[])
{
  return iri_base_joystick::main<GamepadTeleop>(argc, argv, "gamepad_teleop_node");
}
