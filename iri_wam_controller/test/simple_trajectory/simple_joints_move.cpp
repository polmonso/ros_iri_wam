#include <ros/ros.h>
#include <iri_wam_common_msgs/joints_move.h>
#include <actionlib/client/simple_action_client.h>

class RobotArm
{
private:
  // Action client for the joint trajectory action 
  // used to trigger the arm movement action
  ros::NodeHandle n;
  ros::ServiceClient client;
  std::string srv_name_;

public:
  //! Initialize the action client and wait for action server to come up
  RobotArm():srv_name_("/iri_wam/iri_wam_controller/joints_move")
  {
    client = n.serviceClient<iri_wam_common_msgs::joints_move>(this->srv_name_.c_str());
  }

  //! Clean up the action client
  ~RobotArm()
  {

  }

  //! Sends the command to start a given joints position
  bool startJointsMove(iri_wam_common_msgs::joints_move srv)
  {
	  if (!this->client.call(srv))
	  {
		  ROS_ERROR("Failed to call service %s", this->srv_name_.c_str());
		  return false;
	  }
	return true;
  }

  //! Generates a joints_move service message to Home Position
  //  (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
  iri_wam_common_msgs::joints_move armAtHomePosition()
  {
    //our Joints move position variable
    iri_wam_common_msgs::joints_move srv;
    srv.request.joints.resize(7);
    srv.request.joints[0] = 0.0;
    srv.request.joints[1] = 0.0;
    srv.request.joints[2] = 0.0;
    srv.request.joints[3] = 0.0;
    srv.request.joints[4] = 0.0;
    srv.request.joints[5] = 0.0;
    srv.request.joints[6] = 0.0;
    srv.request.velocity = 0.2;
    srv.request.acceleration = 0.2;

    return srv;
  }

  iri_wam_common_msgs::joints_move armAtInitialPosition()
  {
    //our Joints move position variable
    iri_wam_common_msgs::joints_move srv;
    srv.request.joints.resize(7);
    srv.request.joints[0] = 0.0;
    srv.request.joints[1] = 0.0;
    srv.request.joints[2] = 0.0;
    srv.request.joints[3] = 2.0;
    srv.request.joints[4] = 0.0;
    srv.request.joints[5] = 0.0;
    srv.request.joints[6] = 0.0;
    srv.request.velocity = 1.5;
    srv.request.acceleration = 1.5;

    return srv;
  }
 
};

int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "robot_driver");

  RobotArm arm;

  // Start the trajectory
  arm.startJointsMove(arm.armAtHomePosition());
  arm.startJointsMove(arm.armAtInitialPosition());
}

