#include "iri_wiimote_teleop/wiimote_teleop_base.h"

WiimoteTeleop::WiimoteTeleop()
{
}

WiimoteTeleop::~WiimoteTeleop()
{
}

geometry_msgs::Twist WiimoteTeleop::generateTwist()
{
  geometry_msgs::Twist twist;

  twist.linear.x  = vT_;
  twist.angular.z = vR_;

  return twist;
}

void WiimoteTeleop::useButton(const unsigned int & index)
{
  switch(index)
  {
    case wiimote::State::MSG_BTN_A:
      ROS_DEBUG("STOP!");
      vT_ = 0.0;
      vR_ = 0.0;
      break;

    case wiimote::State::MSG_BTN_LEFT:
      ROS_DEBUG("left!");
      vR_ += 0.1;
      break;

    case wiimote::State::MSG_BTN_RIGHT:
      ROS_DEBUG("right!");
      vR_ -= 0.1;
      break;

    case wiimote::State::MSG_BTN_UP:
      ROS_DEBUG("forward!");
      vT_ += 0.1;
      break;

    case wiimote::State::MSG_BTN_DOWN:
      ROS_DEBUG("backward!");
      vT_ -= 0.1;
      break;

    default:
      useExtraButton(index);
      break;
  }
}

void WiimoteTeleop::useExtraButton(const unsigned int & index)
{
  switch(index)
  {
    case wiimote::State::MSG_BTN_1:
      ROS_DEBUG("Button 1!");
      break;
    case wiimote::State::MSG_BTN_2:
      ROS_DEBUG("Button 2!");
      break;
    case wiimote::State::MSG_BTN_B:
      ROS_DEBUG("Button B!");
      break;
    case wiimote::State::MSG_BTN_PLUS:
      ROS_DEBUG("Button +!");
      break;
    case wiimote::State::MSG_BTN_MINUS:
      ROS_DEBUG("Button -!");
      break;
    case wiimote::State::MSG_BTN_HOME:
      ROS_DEBUG("Button Caseta!");
      break;
  }
}

void WiimoteTeleop::useAxes(const float & axe_value, const unsigned int & index)
{
}

