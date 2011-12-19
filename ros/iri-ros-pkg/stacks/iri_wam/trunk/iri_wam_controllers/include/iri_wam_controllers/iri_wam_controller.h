#include <pr2_controller_interface/controller.h>
#include <pr2_mechanism_model/joint.h>
#include <ros/ros.h>
#include <iri_wam_controllers/SetAmplitude.h>
#include <control_toolbox/pid.h>


namespace iri_wam_controller_ns{

class IriWamControllerClass: public pr2_controller_interface::Controller
{
private:
  bool setAmplitude(iri_wam_controllers::SetAmplitude::Request& req,
                    iri_wam_controllers::SetAmplitude::Response& resp);

  pr2_mechanism_model::JointState* joint_state_;
  double init_pos_;
  double amplitude_;
  ros::ServiceServer srv_;
  control_toolbox::Pid pid_controller_;
  pr2_mechanism_model::RobotState *robot_;
  ros::Time time_of_last_cycle_;

public:
  virtual bool init(pr2_mechanism_model::RobotState *robot,
                   ros::NodeHandle &n);
  virtual void starting();
  virtual void update();
  virtual void stopping();
};
} 
