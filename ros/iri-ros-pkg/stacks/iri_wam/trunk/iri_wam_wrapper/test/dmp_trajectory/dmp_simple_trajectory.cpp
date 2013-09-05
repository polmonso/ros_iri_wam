#include <ros/ros.h>
#include <iri_wam_common_msgs/DMPTrackerAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient< iri_wam_common_msgs::DMPTrackerAction > TrajClient;

class RobotArm
{
private:
  // Action client for the joint trajectory action 
  // used to trigger the arm movement action
  TrajClient* traj_client_;
    ros::NodeHandle nh;
    ros::Publisher pub;
public:
  //! Initialize the action client and wait for action server to come up
  RobotArm() 
  {
    // tell the action client that we want to spin a thread by default
    traj_client_ = new TrajClient("/wam_wrapper/DMPTracker", true);

    //initialize the topic to send new goals
    
    pub = nh.advertise<trajectory_msgs::JointTrajectoryPoint>("/wam_wrapper/DMPTrackerNewGoal", 1);

    // wait for action server to come up
    while(!traj_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the joint_trajectory_action server");
    }
  }

  //! Clean up the action client
  ~RobotArm()
  {
    delete traj_client_;
  }

  //! Sends the command to start a given trajectory
  void startTrajectory()
  {
    iri_wam_common_msgs::DMPTrackerGoal DMPgoal;
    
    DMPgoal.initial.positions.resize(7);   
    DMPgoal.initial.positions[0] = -0.11976;
    DMPgoal.initial.positions[1] = -1.84794;
    DMPgoal.initial.positions[2] = 0.285349;
    DMPgoal.initial.positions[3] = 2.84315;
    DMPgoal.initial.positions[4] = -0.310117;
    DMPgoal.initial.positions[5] = -1.21896;
    DMPgoal.initial.positions[6] = 0.0192133;
     
    DMPgoal.goal.positions.resize(7); 
    DMPgoal.goal.positions[0] = 0.160557;
    DMPgoal.goal.positions[1] = -1.91039;
    DMPgoal.goal.positions[2] = -0.664568;
    DMPgoal.goal.positions[3] = 2.9184;
    DMPgoal.goal.positions[4] = -0.5448;
    DMPgoal.goal.positions[5] = -0.812694;
    DMPgoal.goal.positions[6] = -0.471291;
     
    // When to start the trajectory: 1s from now
    //goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    traj_client_->sendGoal(DMPgoal);
    
    for (int i=0;i<10;i++) {
      sleep(0.5);
      trajectory_msgs::JointTrajectoryPoint DMPnew_goal;
      DMPnew_goal.positions.resize(7);   
      DMPnew_goal.positions[0] = -0.11976;
      DMPnew_goal.positions[1] = -1.84794;
      DMPnew_goal.positions[2] = 0.285349-i*0.02;
      DMPnew_goal.positions[3] = 2.84315;
      DMPnew_goal.positions[4] = -0.310117;
      DMPnew_goal.positions[5] = -1.21896;
      DMPnew_goal.positions[6] = 0.0192133;
      
      pub.publish(DMPnew_goal);
    }     
  }

  //! Returns the current state of the action
  actionlib::SimpleClientGoalState getState()
  {
    return traj_client_->getState();
  }
 
};

int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "robot_driver");

  RobotArm arm;
  // Start the trajectory
  arm.startTrajectory();
  // Wait for trajectory completion
  while(!arm.getState().isDone() && ros::ok())
  {
    usleep(50000);
  }
}
