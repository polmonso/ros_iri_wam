#include "iri_joystick_driver_node.h"
#include <iridrivers/exceptions.h>
#include <iridrivers/genius_g_12x.h>
#include <iridrivers/joystick_exceptions.h>

#include <joy/Joy.h>

IriJoystickDriverNode::IriJoystickDriverNode(ros::NodeHandle &nh) :
    iri_base_driver::IriBaseNodeDriver<IriJoystickDriver>(nh),
    joy_mov_pub_(),
    joy_mov_pub_name_("joy"),
    total_axes_events_(0),
    total_button_events_(0),
    total_joy_msg_sent_(0)
{
    try
    {
        driver_.openDriver();
        driver_.startDriver();
    }
    catch (CJoystickException &e)
    {
        ROS_ERROR("Driver Node dixit: '%s'",e.what().c_str());
    }
    catch (CException &e)
    {
        ROS_FATAL("Unexpected error: '%s'",e.what().c_str());
    }

    // Start publishing in the topic
    joy_mov_pub_ = node_handle_.advertise<joy::Joy>(joy_mov_pub_name_,1);
    ROS_DEBUG("Publishing topic active at: '%s'",joy_mov_pub_name_.c_str());
}

void
IriJoystickDriverNode::wrapper_move_joystick_callback(void * param, unsigned int axis_id, float value)
{
    IriJoystickDriverNode * self = (IriJoystickDriverNode *) param;

    self->move_joystick_callback(axis_id, value);
}

void
IriJoystickDriverNode::move_joystick_callback(unsigned int axis_id, float value)
{
    joy::Joy joy_msg;

    ROS_DEBUG("Joystick movement. Axis: '%d' - Value: '%f' ", axis_id, value);
    total_axes_events_++;

    joy_msg.axes.resize(2);
    joy_msg.axes[1] = 0.0;
    joy_msg.axes[2] = 0.0;

    joy_msg.axes[axis_id] = value;
    publish_joy_msg(joy_msg);
}

void
IriJoystickDriverNode::publish_joy_msg(const joy::Joy & msg)
{
    joy_mov_pub_.publish(msg);
    total_joy_msg_sent_++;
}

void
IriJoystickDriverNode::wrapper_button_callback(void * param, unsigned int button_id, bool level)
{
    IriJoystickDriverNode * self = (IriJoystickDriverNode *) param;

    self->button_callback(button_id, level);
}

void
IriJoystickDriverNode::button_callback(unsigned int button_id, bool level)
{
    joy::Joy joy_msg;
    int total_buttons = driver_.get_num_buttons();

    ROS_DEBUG("Button pressed: '%i'", button_id+1);
    total_button_events_++;

    // We do not use the 0 for a button id, so conversion to position 
    // in the array to button id is: pos +1
    joy_msg.buttons.resize(total_buttons+1);

    for (int i = 0; i < total_buttons + 1; i++)
        joy_msg.buttons[i] = 0;

    // level is not always 1 in callback, so please check
    if (! level)
        return;

    joy_msg.buttons[button_id+1] = 1;
    publish_joy_msg(joy_msg);
}

void
IriJoystickDriverNode::mainNodeThread()
{
}

void
IriJoystickDriverNode::postNodeOpenHook(void)
{
    int total_buttons = driver_.get_num_buttons();

    // Enable joystick axes. Need to be every time the joystick is make
    // and open since it restart the callbacks.
    driver_.enable_position_change_callback(y_axis,
                                         wrapper_move_joystick_callback,
                                         this);
    driver_.enable_position_change_callback(x_axis,
                                         wrapper_move_joystick_callback,
                                         this);

    for (int i = 0; i < total_buttons; i++)
        driver_.enable_button_callback(i, wrapper_button_callback, this);
}

void
IriJoystickDriverNode::joystick_check(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
    // TODO I: posible check if fail to open via variable in
    // node_driver if fail to open the device.
    stat.summary(0, "OK");

    // TODO II: implement get_device in driver node
    stat.add("Physical device",                driver_.get_device());
    stat.add("Buttons in device",              driver_.get_num_buttons());
    stat.add("Axes in device",                 driver_.get_num_axes());
    stat.add("Publishing movements topic",     joy_mov_pub_.getTopic());
    stat.add("Subscribers to movements topic", joy_mov_pub_.getNumSubscribers());
    stat.add("Total axes events generated",    total_axes_events_);
    stat.add("Total buttons events generated", total_button_events_);
    stat.add("Total joy msg published",        total_joy_msg_sent_);
}

void
IriJoystickDriverNode::addNodeDiagnostics(void)
{
  diagnostic_.add("General joystick check", this, &IriJoystickDriverNode::joystick_check);
}

void
IriJoystickDriverNode::addNodeOpenedTests(void)
{
}

void
IriJoystickDriverNode::addNodeStoppedTests(void)
{
}

void
IriJoystickDriverNode::addNodeRunningTests(void)
{
}

void
IriJoystickDriverNode::reconfigureNodeHook(int level)
{
}

IriJoystickDriverNode::~IriJoystickDriverNode()
{
    driver_.stopDriver();
    driver_.closeDriver();
}

/* main function */
int main(int argc,char *argv[])
{
  return driver_base::main<IriJoystickDriverNode>(argc,argv,"iri_joystick_driver_node");
}
