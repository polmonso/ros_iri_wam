#include "iri_joystick_driver.h"
#include <iridrivers/exceptions.h>

IriJoystickDriver::IriJoystickDriver() :
    dev_("/dev/input/js0"),
    pub_channel_name_("joy")
{
  //setDriverId(driver string id);
    pid_t pid = getpid();
    std::ostringstream s;
    s << "iri_joystick" << pid;

    try
    {
        jh_  = new CJoystick(s.str());
    }
    catch(CException &e)
    {
        ROS_FATAL("'%s'",e.what().c_str());
    }
}

bool IriJoystickDriver::openDriver(void)
{
    try
    {
        jh_->open(dev_);
        ROS_DEBUG("Joystick found at '%s'", dev_.c_str());
    }
    catch(CException &e)
    {
        ROS_ERROR("Open failed with: '%s'",e.what().c_str());
        return false;
    }

    return true;
}

bool IriJoystickDriver::closeDriver(void)
{
    try
    {
        jh_->close();
    }
    catch (CException & e)
    {
        ROS_ERROR("'%s'",e.what().c_str());
        return false;
    }

    return true;
}

bool IriJoystickDriver::startDriver(void)
{
  return true;
}

bool IriJoystickDriver::stopDriver(void)
{
  return true;
}

void IriJoystickDriver::config_update(const Config& new_cfg, uint32_t level)
{
  this->lock();

  //update driver with new_cfg data

  // save the current configuration
  this->config_=new_cfg;

  this->unlock();
}

void
IriJoystickDriver::enable_position_change_callback(int axis_id,
                         void (* callback) (void * param, unsigned int axis_id, float value),
                         void * param)
{
    jh_->enable_position_change_callback(axis_id, callback, param);
}

void
IriJoystickDriver::enable_button_callback(int button_id,
                         void (* callback) (void * param, unsigned int button_id, bool level),
                         void * param)
{
    jh_->enable_button_callback(button_id, callback, param);
}

int
IriJoystickDriver::get_num_buttons() const
{
    return jh_->get_num_buttons();
}

int
IriJoystickDriver::get_num_axes() const
{
    return jh_->get_num_axis();
}

std::string
IriJoystickDriver::get_device() const
{
    return dev_;
}

IriJoystickDriver::~IriJoystickDriver()
{
    delete(jh_);
}
