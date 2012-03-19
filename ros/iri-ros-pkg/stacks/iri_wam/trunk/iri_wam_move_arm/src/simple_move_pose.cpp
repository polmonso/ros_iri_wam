#include <ros/ros.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

//typedef actionlib::SimpleActionClient< pr2_controllers_msgs::JointTrajectoryAction > TrajClient;

class RobotArm
{
private:
  // Action client for the joint trajectory action 
  // used to trigger the arm movement action
  //TrajClient* traj_client_;
  ros::Duration time_move;
  std::vector<double> pos;
  ros::Publisher pub_poseStamped;
public:
  //! Initialize the action client and wait for action server to come up
  RobotArm() 
  {
    // tell the action client that we want to spin a thread by default
    //traj_client_ = new TrajClient("iri_ros_controller/joint_trajectory_action", true);

    // wait for action server to come up
  // // while(!traj_client_->waitForServer(ros::Duration(5.0))){
   //   ROS_INFO("Waiting for the joint_trajectory_action server");
   // }
  }

  //! Clean up the action client
  ~RobotArm()
  {
   // delete traj_client_;
  }

  //! Sends the command to start a given trajectory
  void startTrajectory(geometry_msgs::PoseStamped goal)
  {
    // When to start the trajectory: 1s from now
  
  }


  geometry_msgs::PoseStamped armExtensionTrajectory()
  {
    //our goal variable
    geometry_msgs::Pose pose;
	pose.position.x=pos[0];
	pose.position.y=pos[1];
	pose.position.z=pos[2];
	pose.orientation.x=pos[3];
	pose.orientation.y=pos[4];
	pose.orientation.z=pos[5];
	pose.orientation.w=pos[6];
	geometry_msgs::PoseStamped goal;
	goal.pose=pose;
	goal.header.frame_id="wam_fk/wam0";
	goal.header.stamp=time_move;
	
    //we are done; return the goal
    return goal;
  }

  //! Returns the current state of the action
  /*actionlib::SimpleClientGoalState getState()
  {
    return traj_client_->getState();
  }*/
  void getTrajectory(int argc, char** argv)
  {
	 if(argc < 8 || argc > 9) 
	 {
	   ROS_FATAL("Error: The numbers of parameters out of bound"); 
	   exit(1);
	 }
	 pos.resize(7);
	 for(int i =1; i <=7; ++i) pos[i-1]=strtod(argv[i],NULL);

	 time_move= (argc == 9)?ros::Duration(strtod(argv[8],NULL)):ros::Duration(3.0);
	 
  }
 
};

int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "simple_move_node");
  RobotArm arm;
  // Start the trajectory
  arm.getTrajectory(argc,argv);
  arm.startTrajectory(arm.armExtensionTrajectory());
  // Wait for trajectory completion
  while(!arm.getState().isDone() && ros::ok())
  {
    usleep(50000);
  }
   bool success = (arm.getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
  if(success)
   ROS_INFO("Action finished: %s",arm.getState().toString().c_str());
  else
   ROS_INFO("Action failed: %s",arm.getState().toString().c_str());
}
