#include "segway_rmp200_driver.h"
#include "mutexexceptions.h"
#include <ros/console.h>

using namespace segway_rmp200_node;

SegwayRmp200Driver::SegwayRmp200Driver()
{
  ftdi_server_   = CFTDIServer::instance();
  serial_number_ = "NULL";
  segway_        = new CSegwayRMP200();
}

bool SegwayRmp200Driver::openDriver(void)
{
  try
  {
    this->lock();
    if(this->serial_number_.compare("NULL") == 0)
    {
      ROS_DEBUG("No segway serial provided, attempting auto-connect");
      segway_->connect();
      serial_number_ = segway_->get_serial();
    }
    else
    {
      std::vector<int> ftdi_devs;
      std::string serial;
      unsigned int ii;

      ftdi_devs = ftdi_server_->get_ids_by_description(CSegwayRMP200::description);
      for(ii=0; ii<ftdi_devs.size(); ii++)
      {
        serial = ftdi_server_->get_serial_number(ftdi_devs.at(ii));
        if(serial == serial_number_)
        {
          ROS_DEBUG("Segway serial provided matches FTDI serial list");
          segway_->connect(serial);
          break;
        }
      }
      if(ii == ftdi_devs.size())
      {
        serial_number_ = "NULL";
        this->unlock();
        return false;
      }
    }

    setSegwayEvents();

    this->unlock();
  }
  catch(CException &e)
  {
    this->unlock();
    ROS_FATAL("'%s'",e.what().c_str());
    return false;
  }

  ROS_INFO("Segway %s successfully connected", serial_number_.c_str());
  return true;
}

bool SegwayRmp200Driver::closeDriver(void)
{
  try
  {
    this->lock();
    ROS_DEBUG("Stopping and closing platform");
    events_.clear();
    segway_->close();
//     delete segway_;
//     segway_ = NULL;
    this->unlock();
  }
  catch (CException & e)
  {
    ROS_ERROR("'%s'", e.what().c_str());
    return false;
  }

  return true;
}

bool SegwayRmp200Driver::startDriver(void)
{
  return true;
}

bool SegwayRmp200Driver::stopDriver(void)
{
  return true;
}

void SegwayRmp200Driver::config_update(Config& new_cfg, uint32_t level)
{
  this->lock();
    
  try
  {
    if(this->getState() > SegwayRmp200Driver::CLOSED)
    {
      segway_->set_velocity_scale_factor(new_cfg.velocity_scale_factor);
      segway_->set_acceleration_scale_factor(new_cfg.acceleration_scale_factor);
      segway_->set_turnrate_scale_factor(new_cfg.turn_rate_scale_factor);
      segway_->set_currentlimit_scale_factor(new_cfg.current_limit_scale_factor);
      segway_->set_gain_schedule((gain)new_cfg.gain_schedule);
      segway_->set_operation_mode((op_mode)new_cfg.operation_mode);
      if(new_cfg.balance_lockout)
        segway_->lock_balance();
      else
        segway_->unlock_balance();
    }
    else
    {
      // save the desired serial number
      serial_number_ = new_cfg.serial_number;
    }

    if( segway_ != NULL)
    {
      new_cfg.gain_schedule  = (int)segway_->get_gain_schedule();
      new_cfg.operation_mode = (int)segway_->get_operation_mode();
      new_cfg.serial_number  = segway_->get_serial();
    }
  }
  catch(CException &e)
  {
    ROS_ERROR("'%s'", e.what().c_str());
  }

  // save the current configuration
  config_ = new_cfg;

  this->unlock();
}

SegwayRmp200Driver::~SegwayRmp200Driver()
{
  try
  {
    if(segway_ != NULL)
      delete segway_;
    segway_ = NULL;
  }
  catch(CMutexException &e)
  {
    ROS_ERROR("'%s'", e.what().c_str());
  }
}

iri_segway_rmp_msgs::SegwayRMP200Status
SegwayRmp200Driver::get_status(void)
{
    iri_segway_rmp_msgs::SegwayRMP200Status status;

    if (segway_ != NULL)
        status = build_ROS_status_msg(segway_->get_status());

    return status;
}

float SegwayRmp200Driver::get_pitch_angle(void)
{
  if(segway_!=NULL)
  {
    return segway_->get_pitch_angle();
  }
  else 
    return 0.0;
}

float SegwayRmp200Driver::get_pitch_rate(void)
{
  if(segway_!=NULL)
  {
    return segway_->get_pitch_rate();
  }
  else 
    return 0.0;
}

float SegwayRmp200Driver::get_roll_angle(void)
{
  if(segway_!=NULL)
  {
    return segway_->get_roll_angle();
  }
  else 
    return 0.0;
}

float SegwayRmp200Driver::get_roll_rate(void)
{
  if(segway_!=NULL)
  {
    return segway_->get_roll_rate();
  }
  else 
    return 0.0;
}

float SegwayRmp200Driver::get_left_wheel_velocity(void)
{
  if(segway_!=NULL)
  {
    return segway_->get_left_wheel_velocity();
  }
  else 
    return 0.0;
}

float SegwayRmp200Driver::get_right_wheel_velocity(void)
{
  if(segway_!=NULL)
  {
    return segway_->get_right_wheel_velocity();
  }
  else 
    return 0.0;
}

float SegwayRmp200Driver::get_yaw_rate(void)
{
  if(segway_!=NULL)
  {
    return segway_->get_yaw_rate();
  }
  else 
    return 0.0;
}

float SegwayRmp200Driver::get_uptime(void)
{
  if(segway_!=NULL)
  {
    return segway_->get_uptime();
  }
  else 
    return 0.0;
}

float SegwayRmp200Driver::get_left_wheel_displacement(void)
{
  if(segway_!=NULL)
  {
    return segway_->get_left_wheel_displacement();
  }
  else 
    return 0.0;
}

float SegwayRmp200Driver::get_right_wheel_displacement(void)
{
  if(segway_!=NULL)
  {
    return segway_->get_right_wheel_displacement();
  }
  else 
    return 0.0;
}

float SegwayRmp200Driver::get_forward_displacement(void)
{
  if(segway_!=NULL)
  {
    return segway_->get_forward_displacement();
  }
  else 
    return 0.0;
}

float SegwayRmp200Driver::get_yaw_displacement(void)
{
  if(segway_!=NULL)
  {
    return segway_->get_yaw_displacement();
  }
  else 
    return 0.0;
}

float SegwayRmp200Driver::get_left_motor_torque(void)
{
  if(segway_!=NULL)
  {
    return segway_->get_left_motor_torque();
  }
  else 
    return 0.0;
}

float SegwayRmp200Driver::get_right_motor_torque(void)
{
  if(segway_!=NULL)
  {
    return segway_->get_right_motor_torque();
  }
  else 
    return 0.0;
}

op_mode SegwayRmp200Driver::get_operation_mode(void)
{
  if(segway_!=NULL)
  {
    return segway_->get_operation_mode();
  }
  else 
    return (op_mode)-1;
}

gain SegwayRmp200Driver::get_gain_schedule(void)
{
  if(segway_!=NULL)
  {
    return segway_->get_gain_schedule();
  }
  else 
    return (gain)-1;
}

float SegwayRmp200Driver::get_ui_battery_voltage(void)
{
  if(segway_!=NULL)
  {
    return segway_->get_ui_battery_voltage();
  }
  else 
    return 0.0;
}

float SegwayRmp200Driver::get_powerbase_battery_voltage(void)
{
  if(segway_!=NULL)
  {
    return segway_->get_powerbase_battery_voltage();
  }
  else 
    return 0.0;
}

void SegwayRmp200Driver::reset(void)
{
  if(segway_!=NULL)
  {
    segway_->reset();
  }
}

void SegwayRmp200Driver::move_platform(float vT, float vR)
{
  if(segway_!=NULL)
  {
    segway_->move(vT,vR);
  }
}

void SegwayRmp200Driver::stop_platform(void)
{
  if(segway_!=NULL)
  {
    segway_->stop();
  }
}

const std::list<std::string> & SegwayRmp200Driver::getSegwayEvents() const
{
  return events_;
}

void SegwayRmp200Driver::setSegwayEvents(void)
{
  // keep in mind that the order in of the event son the events list fix their
  // priority. The first event on the list will be checked first, and if it is 
  // active, the other will not be checked.
  events_.push_back(segway_->get_cable_disconnected_event());    
  events_.push_back(segway_->get_power_off_event());    
  events_.push_back(segway_->get_no_heartbeat_event());    
  events_.push_back(segway_->get_new_status_event());
}

const std::string & SegwayRmp200Driver::getSegwayEventName(const unsigned int & ev)
{
  static const std::string names[5] = 
  {
    std::string("NO_USB"),
    std::string("POWER_OFF"),
    std::string("NO_HEART_BEAT"),
    std::string("NEW_STATUS"),
    std::string("UNKNOWN")
  };

  if (ev <= 3)
    return names[ev];
  else
    return names[4];
}

