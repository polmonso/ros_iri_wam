#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient< control_msgs::FollowJointTrajectoryAction > TrajClient;

class RobotArm
{
private:
  // Action client for the joint trajectory action 
  // used to trigger the arm movement action
  TrajClient* traj_client_;

public:
  //! Initialize the action client and wait for action server to come up
  RobotArm() 
  {
    // tell the action client that we want to spin a thread by default
    //traj_client_ = new TrajClient("/wam_wrapper/follow_joint_trajectory", true);
    traj_client_ = new TrajClient("/iri_wam_controller/follow_joint_trajectory", true);

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
  void startTrajectory(control_msgs::FollowJointTrajectoryGoal goal)
  {
    // When to start the trajectory: 1s from now
    goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    traj_client_->sendGoal(goal);
  }

  //! Generates a simple trajectory with two waypoints, used as an example
  /*! Note that this trajectory contains two waypoints, joined together
      as a single trajectory. Alternatively, each of these waypoints could
      be in its own trajectory - a trajectory can have one or more waypoints
      depending on the desired application.
  */
  control_msgs::FollowJointTrajectoryGoal armExtensionTrajectory()
  {
    //our goal variable
    control_msgs::FollowJointTrajectoryGoal goal;
    
    // frame_id
    goal.trajectory.header.frame_id = "/wam_link_footprint";
    // joint names
    goal.trajectory.joint_names.resize(7);
    goal.trajectory.joint_names[0] = "wam_joint_1";
    goal.trajectory.joint_names[1] = "wam_joint_2";
    goal.trajectory.joint_names[2] = "wam_joint_3";
    goal.trajectory.joint_names[3] = "wam_joint_4";
    goal.trajectory.joint_names[4] = "wam_joint_5";
    goal.trajectory.joint_names[5] = "wam_joint_6";
    goal.trajectory.joint_names[6] = "wam_joint_7";

    // Waypoints in this goal trajectory
    goal.trajectory.points.resize(5);

    // First trajectory point
    // Positions
    int ind = 0;
    goal.trajectory.points[ind].positions.resize(7);
    goal.trajectory.points[ind].positions[0] = 0.0;
    goal.trajectory.points[ind].positions[1] = 0.0;
    goal.trajectory.points[ind].positions[2] = 0.0;
    goal.trajectory.points[ind].positions[3] = 0.0;
    goal.trajectory.points[ind].positions[4] = 0.0;
    goal.trajectory.points[ind].positions[5] = 0.0;
    goal.trajectory.points[ind].positions[6] = 0.0;

    ind += 1;
    goal.trajectory.points[ind].positions.resize(7);
    goal.trajectory.points[ind].positions[0] =  0.49259;
    goal.trajectory.points[ind].positions[1] =  0.212558;
    goal.trajectory.points[ind].positions[2] = -0.169996;
    goal.trajectory.points[ind].positions[3] =  2.09482;
    goal.trajectory.points[ind].positions[4] = -0.14391;
    goal.trajectory.points[ind].positions[5] =  0.919756;
    goal.trajectory.points[ind].positions[6] = -0.0255835;

    ind += 1;
    goal.trajectory.points[ind].positions.resize(7);
    goal.trajectory.points[ind].positions[0] =  1.67741;
    goal.trajectory.points[ind].positions[1] =  0.357458;
    goal.trajectory.points[ind].positions[2] = -0.273308;
    goal.trajectory.points[ind].positions[3] =  1.74678;
    goal.trajectory.points[ind].positions[4] = -0.113625;
    goal.trajectory.points[ind].positions[5] =  1.08573;
    goal.trajectory.points[ind].positions[6] = -2.00137;

    ind += 1;
    goal.trajectory.points[ind].positions.resize(7);
    goal.trajectory.points[ind].positions[0] =  1.67752;
    goal.trajectory.points[ind].positions[1] =  0.581419;
    goal.trajectory.points[ind].positions[2] = -0.327541;
    goal.trajectory.points[ind].positions[3] =  1.31991;
    goal.trajectory.points[ind].positions[4] = -0.099946;
    goal.trajectory.points[ind].positions[5] =  1.26008;
    goal.trajectory.points[ind].positions[6] = -2.00137;

    ind += 1;
    goal.trajectory.points[ind].positions.resize(7);
    goal.trajectory.points[ind].positions[0] = 1.67748;
    goal.trajectory.points[ind].positions[1] = 0.280732;
    goal.trajectory.points[ind].positions[2] = -0.297391;
    goal.trajectory.points[ind].positions[3] = 2.00713;
    goal.trajectory.points[ind].positions[4] = -0.0907737;
    goal.trajectory.points[ind].positions[5] = 0.802098;
    goal.trajectory.points[ind].positions[6] = -2.00106;

    return goal;
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
  arm.startTrajectory(arm.armExtensionTrajectory());
  // Wait for trajectory completion
  while(!arm.getState().isDone() && ros::ok())
  {
    usleep(50000);
  }
}

