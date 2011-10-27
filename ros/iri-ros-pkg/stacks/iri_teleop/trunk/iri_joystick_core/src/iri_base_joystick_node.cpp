#include "joystick_core/iri_base_joystick_node.h"

namespace iri_base_joystick
{

IriBaseJoystick::IriBaseJoystick() :
  nh_()
{
  vT_ = 0.0f;
  vR_ = 0.0f;

  twist_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  joy_sub_   = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &IriBaseJoystick::joyCallback, this);
}

IriBaseJoystick::~IriBaseJoystick()
{
}

bool IriBaseJoystick::check_pressed_button_callback(const std::vector<int> & buttons, std::vector<unsigned int> & index)
{
  if (buttons.size() == 0)
    return false;

  bool new_data = false;
  for (unsigned int i=0; i<buttons.size(); i++)
  {
    if (buttons[i] != 0)
    {
      new_data = true;
      index.push_back(i);
    }
  }
    
  return new_data;
}

bool IriBaseJoystick::check_movement_axes_callback(const std::vector<float> & axes, std::vector<unsigned int> & index)
{
  if (axes.size() == 0)
    return false;

  bool new_data = false;
  for (unsigned int i=0; i<axes.size(); i++)
  {
    if (axes[i] != 0.0)
    {
      new_data = true;
      index.push_back(i);
    }
  }
    
  return new_data;
}

bool IriBaseJoystick::compareIndexVectors(const std::vector<unsigned int> & a, const std::vector<unsigned int> & b)
{
  if( a.size() != b.size() )
    return false;

  return std::equal(a.begin(), a.end(), b.begin());;
}

void IriBaseJoystick::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  std::vector<unsigned int> axes_index, buttons_index;
  bool axes_mov    = check_movement_axes_callback(joy->axes,  axes_index);
  bool buttons_mov = check_pressed_button_callback(joy->buttons, buttons_index);

  // AXES
  if (axes_mov)
  {
    if( !compareIndexVectors(axes_index, prev_axes_) )
      for(unsigned int i=0; i<axes_index.size(); i++)
        useAxes(joy->axes[axes_index[i]], axes_index[i]);
  }
  else
    prev_axes_.clear();

  // BUTTONS
  if (buttons_mov)
  {
    if( !compareIndexVectors(buttons_index, prev_buttons_) )
      for(unsigned int i=0; i<buttons_index.size(); i++)
        useButton(buttons_index[i]);

    prev_buttons_ = buttons_index;
  }
  else
    prev_buttons_.clear();

  // Publish Twist
  geometry_msgs::Twist msg = generateTwist();
  twist_pub_.publish(msg);
}

}