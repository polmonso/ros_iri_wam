#include "iri_wam_controller/iri_wam_controller.h"
#include <pluginlib/class_list_macros.h>

namespace iri_wam_controller_ns {


/// Controller initialization in non-realtime
bool IriWamControllerClass::init(pr2_mechanism_model::RobotState *robot,
                            ros::NodeHandle &n)
{
  std::string joint_name;
  if (!n.getParam("joint_name", joint_name))
  {
    ROS_ERROR("No joint given in namespace: '%s')",
              n.getNamespace().c_str());
    return false;
  }

  joint_state_ = robot->getJointState(joint_name);
  if (!joint_state_)
  {
    ROS_ERROR("IriWamController could not find joint named '%s'",
              joint_name.c_str());
    return false;
  }

  // advertise service to set amplitude
  amplitude_ = 0.5;
  srv_ = n.advertiseService("set_amplitude", 
                            &IriWamControllerClass::setAmplitude, this);

  // copy robot pointer so we can access time
  robot_ = robot;

  // construct pid controller
  if (!pid_controller_.init(ros::NodeHandle(n, "pid_parameters"))){
    ROS_ERROR("IriWamController could not construct PID controller for joint '%s'",
              joint_name.c_str());
    return false;
  }

  return true;
}


/// Controller startup in realtime
void IriWamControllerClass::starting()
{
  init_pos_ = joint_state_->position_;
  time_of_last_cycle_ = robot_->getTime();
  pid_controller_.reset();
}


/// Controller update loop in realtime
void IriWamControllerClass::update()
{
  double desired_pos = init_pos_ + amplitude_ * sin(ros::Time::now().toSec());
  double current_pos = joint_state_->position_;
  
  ros::Duration dt = robot_->getTime() - time_of_last_cycle_;
  time_of_last_cycle_ = robot_->getTime();

  joint_state_->commanded_effort_ = pid_controller_.updatePid(current_pos-desired_pos, dt);
}


/// Controller stopping in realtime
void IriWamControllerClass::stopping()
{}

/// Service call to set amplitude of sin
bool IriWamControllerClass::setAmplitude(iri_wam_controllers::SetAmplitude::Request& req,
                                     iri_wam_controllers::SetAmplitude::Response& resp)
{
  if (fabs(req.amplitude) < 2.0){
    amplitude_ = req.amplitude;
    ROS_INFO("IriWamcontroller: set amplitude too %f", req.amplitude);
  }
  else
    ROS_WARN("IriWamController: requested amplitude %f too large", req.amplitude);

  resp.amplitude = amplitude_;
  return true;
}



} // namespace

/// Register controller to pluginlib
PLUGINLIB_DECLARE_CLASS(iri_wam_controllers,IriWamController, 
                         iri_wam_controller_ns::IriWamControllerClass, 
                         pr2_controller_interface::Controller)

