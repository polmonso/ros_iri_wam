#include "segway_rmp200_driver_node.h"

using namespace segway_rmp200_node;

SegwayRmp200DriverNode::SegwayRmp200DriverNode(ros::NodeHandle &nh) try : 
  iri_base_driver::IriBaseNodeDriver<SegwayRmp200Driver>(nh)
{
  event_server_ = CEventServer::instance();

  //init class attributes if necessary
  this->loop_rate_ = 50;//in [Hz]

  // [init publishers]
  this->status_publisher_   = this->public_node_handle_.advertise<iri_segway_rmp_msgs::SegwayRMP200Status>("status", 100);

  // [init subscribers]
  this->cmd_platform_subscriber_ = this->public_node_handle_.subscribe("cmd_vel", 100, &SegwayRmp200DriverNode::cmd_platform_callback, this);
  
  // [init services]
  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]
}
catch (CException & e)
{
  ROS_FATAL("'%s'",e.what().c_str());
}

void SegwayRmp200DriverNode::mainNodeThread(void)
{
  //lock access to driver if necessary
  this->driver_.lock();

  segway_event_id_ = event_server_->wait_first(driver_.getSegwayEvents());

  switch(segway_event_id_)
  {
    case SegwayRmp200Driver::NO_USB:
      ROS_WARN("The USB cable has been disconnected");
      driver_.goClosed();
      break;

    case SegwayRmp200Driver::POWER_OFF:
      ROS_WARN("The segway platform power is off");
      break;

    case SegwayRmp200Driver::NO_HEART_BEAT:
      ROS_WARN("No heart beat detected");
      break;

    case SegwayRmp200Driver::NEW_STATUS:
      //ROS_DEBUG("New Status message!");
      updateSegwayStatus();
      status_publisher_.publish(this->segway_status_msg_);
      break;
  }
  
  // [fill msg Header if necessary]

  // [fill msg structures]
  
  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]

  //unlock access to driver if previously blocked
  this->driver_.unlock();
}

void SegwayRmp200DriverNode::updateSegwayStatus()
{
  this->segway_status_msg_ = this->driver_.get_status();
}

/*  [subscriber callbacks] */
void SegwayRmp200DriverNode::cmd_platform_callback(const geometry_msgs::Twist::ConstPtr& msg) 
{
  ROS_DEBUG("SegwayRmp200DriverNode::cmd_platform_callback");

  //lock access to driver if necessary 
  this->driver_.lock();

  if(this->driver_.isRunning())
  {
    double vT = msg->linear.x;//sqrt(msg->linear.x*msg->linear.x + msg->linear.y*msg->linear.y);
    double vR = msg->angular.z;
    ROS_INFO("New Command Received: vT=%f vR=%f", vT, vR);

    //request platform movement
    this->driver_.move_platform(vT, vR);
  }
  else
  {
    ROS_WARN("Segway driver has not been started yet");
  }

  //unlock access to driver if previously blocked 
  this->driver_.unlock(); 
}

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void SegwayRmp200DriverNode::postNodeOpenHook(void)
{
}

void SegwayRmp200DriverNode::check_configuration(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  switch(segway_event_id_)
  {
    case SegwayRmp200Driver::NO_USB:
      stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "USB Cable disconected.");
      break;

    case SegwayRmp200Driver::POWER_OFF:
      stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Platform is powered off.");
      break;

    case SegwayRmp200Driver::NO_HEART_BEAT:
      stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "No heart beat received.");
      break;

    case SegwayRmp200Driver::NEW_STATUS:
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Everything is working properly.");
      break;
  }
  stat.add("Current Segway event: ", driver_.getSegwayEventName(segway_event_id_));
}

void SegwayRmp200DriverNode::ui_battery_check(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  float ui_battery=0.0;

  ui_battery = this->driver_.get_ui_battery_voltage();
  
  if(ui_battery>=6.0)
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "UI battery level is okay.");

  else if(ui_battery>4.0 && ui_battery<6.0)
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "UI battery level is low.");

  else
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "UI battery extremelly low, please recharge it as soon as possible");

  stat.add("UI battery level: ", ui_battery);
}

void SegwayRmp200DriverNode::pb_battery_check(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  float pb_battery=0.0;

  pb_battery = this->driver_.get_powerbase_battery_voltage();
  
  if(pb_battery>=70.0)
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Powerbase battery level is okay.");
  
  else if(pb_battery>60.0 && pb_battery<70.0)
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Powerbase battery level is low.");
  
  else
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Powerbase battery extremelly low, please recharge it as soon as possible");
  
  stat.add("Powerbase battery level: ", pb_battery);
}

void SegwayRmp200DriverNode::addNodeDiagnostics(void)
{
  this->diagnostic_.add("Error event check", this, &SegwayRmp200DriverNode::check_configuration);
  this->diagnostic_.add("UI battery level check", this, &SegwayRmp200DriverNode::ui_battery_check);
  this->diagnostic_.add("Powerbase battery level check", this, &SegwayRmp200DriverNode::pb_battery_check);
}

void SegwayRmp200DriverNode::addNodeOpenedTests(void)
{
}

void SegwayRmp200DriverNode::addNodeStoppedTests(void)
{
}

void SegwayRmp200DriverNode::addNodeRunningTests(void)
{
}

void SegwayRmp200DriverNode::reconfigureNodeHook(int level)
{
}

SegwayRmp200DriverNode::~SegwayRmp200DriverNode()
{
  //[free dynamic memory]
}

/* main function */
int main(int argc,char *argv[])
{
  return driver_base::main<SegwayRmp200DriverNode>(argc,argv,"segway_rmp200_driver_node");
}

