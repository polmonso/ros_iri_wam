#include <ros/ros.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient< pr2_controllers_msgs::JointTrajectoryAction > TrajClient;

class SimpleMoveJoints
{
private:
  // Action client for the joint trajectory action 
  // used to trigger the arm movement action
  TrajClient* traj_client_;
  ros::Duration time_move;
  std::vector<double> pos;

public:
  //! Initialize the action client and wait for action server to come up
  SimpleMoveJoints() 
  {
    // tell the action client that we want to spin a thread by default
    traj_client_ = new TrajClient("iri_ros_controller/joint_trajectory_action", true);

    // wait for action server to come up
    while(!traj_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the joint_trajectory_action server");
    }
  }

  //! Clean up the action client
  ~SimpleMoveJoints()
  {
    delete traj_client_;
  }

  //! Sends the command to start a given trajectory
  void startTrajectory(pr2_controllers_msgs::JointTrajectoryGoal goal)
  {
    // When to start the trajectory: 1s from now
    goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    traj_client_->sendGoal(goal);
  }


  pr2_controllers_msgs::JointTrajectoryGoal armExtensionTrajectory()
  {
    //our goal variable
    pr2_controllers_msgs::JointTrajectoryGoal goal;

    // First, the joint names, which apply to all waypoints
    goal.trajectory.joint_names.push_back("j1_joint");
    goal.trajectory.joint_names.push_back("j2_joint");
    goal.trajectory.joint_names.push_back("j3_joint");
    goal.trajectory.joint_names.push_back("j4_joint");
    goal.trajectory.joint_names.push_back("j5_joint");
    goal.trajectory.joint_names.push_back("j6_joint");
    goal.trajectory.joint_names.push_back("j7_joint");

    // We will have two waypoints in this goal trajectory
    goal.trajectory.points.resize(1);

    // Positions
    goal.trajectory.points[0].positions.resize(7);
    goal.trajectory.points[0].velocities.resize(7);
    goal.trajectory.points[0].accelerations.resize(7);
    for(int i=0; i < 7; ++i)
    {
	  goal.trajectory.points[0].positions[i] = pos[i];	
	  goal.trajectory.points[0].velocities[i] = 0.0;
      goal.trajectory.points[0].accelerations[i] = 0.0;
	}
	
    // To be reached 1 second after starting along the trajectory
    goal.trajectory.points[0].time_from_start = time_move;

    //we are done; return the goal
    return goal;
  }

  //! Returns the current state of the action
  actionlib::SimpleClientGoalState getState()
  {
    return traj_client_->getState();
  }
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
  SimpleMoveJoints arm;
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
