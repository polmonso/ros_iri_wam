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

public:
  //! Initialize the action client and wait for action server to come up
  RobotArm() 
  {
    client = n.serviceClient<iri_wam_common_msgs::joints_move>("/iri_wam_controller/joints_move");
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
		  ROS_ERROR("Failed to call service add_two_ints");
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
    srv.request.velocities.resize(7);
    srv.request.accelerations.resize(7);
    srv.request.joints[0] = 0.0;
    srv.request.joints[1] = 0.0;
    srv.request.joints[2] = 0.0;
    srv.request.joints[3] = 0.0;
    srv.request.joints[4] = 0.0;
    srv.request.joints[5] = 0.0;
    srv.request.joints[6] = 0.0;
    srv.request.velocities[0] = 0.0;
    srv.request.velocities[1] = 0.0;
    srv.request.velocities[2] = 0.0;
    srv.request.velocities[3] = 0.0;
    srv.request.velocities[4] = 0.0;
    srv.request.velocities[5] = 0.0;
    srv.request.velocities[6] = 0.0;
    srv.request.accelerations[0] = 0.0;
    srv.request.accelerations[1] = 0.0;
    srv.request.accelerations[2] = 0.0;
    srv.request.accelerations[3] = 0.0;
    srv.request.accelerations[4] = 0.0;
    srv.request.accelerations[5] = 0.0;
    srv.request.accelerations[6] = 0.0;

    return srv;
  }

  iri_wam_common_msgs::joints_move armAtInitialPosition()
  {
    //our Joints move position variable
    iri_wam_common_msgs::joints_move srv;
    srv.request.joints.resize(7);
    srv.request.velocities.resize(7);
    srv.request.accelerations.resize(7);
    srv.request.joints[0] = 0.0;
    srv.request.joints[1] = 0.0;
    srv.request.joints[2] = 0.0;
    srv.request.joints[3] = 2.0;
    srv.request.joints[4] = 0.0;
    srv.request.joints[5] = 0.0;
    srv.request.joints[6] = 0.0;
    srv.request.velocities[0] = 0.0;
    srv.request.velocities[1] = 0.0;
    srv.request.velocities[2] = 0.0;
    srv.request.velocities[3] = 0.0;
    srv.request.velocities[4] = 0.0;
    srv.request.velocities[5] = 0.0;
    srv.request.velocities[6] = 0.0;
    srv.request.accelerations[0] = 0.0;
    srv.request.accelerations[1] = 0.0;
    srv.request.accelerations[2] = 0.0;
    srv.request.accelerations[3] = 0.0;
    srv.request.accelerations[4] = 0.0;
    srv.request.accelerations[5] = 0.0;
    srv.request.accelerations[6] = 0.0;

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

