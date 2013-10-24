#include <ros/ros.h>
#include <iri_wam_common_msgs/LWPRTrajectoryReturningForceEstimationAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient< iri_wam_common_msgs::LWPRTrajectoryReturningForceEstimationAction > TrajClient;

class RobotArm
{
    private:
        TrajClient* traj_client_;
        const std::string model_filename_;
        const std::string points_filename_;

    public:
        //! Initialize the action client and wait for action server to come up
        RobotArm(const std::string model_filename, const std::string points_filename) :
            model_filename_(model_filename),
            points_filename_(points_filename)
        {
            // tell the action client that we want to spin a thread by default
            traj_client_ = new TrajClient("/wam_wrapper/lwpr_trajectory", true);

            // wait for action server to come up
            while(!traj_client_->waitForServer(ros::Duration(5.0))){
              ROS_INFO("Waiting for the lwpr_trajectory_action server");
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
            iri_wam_common_msgs::LWPRTrajectoryReturningForceEstimationGoal goal;
            goal.model_filename  = model_filename_;
            goal.points_filename = points_filename_; 

            traj_client_->sendGoal(goal);
        }

        //! Returns the current state of the action
        actionlib::SimpleClientGoalState getState()
        {
            return traj_client_->getState();
        }

        double get_result()
        {
            return traj_client_->getResult()->force;
        }
};

int main(int argc, char** argv)
{
    if (argc != 3)
    {
        std::cerr << "Usage: " << argv[0] << "<server_model_file_path> <server_point_file_path>" << std::endl;
        return 0;
    }

    // Init the ROS node
    ros::init(argc, argv, "robot_driver");

    RobotArm arm(argv[1], argv[2]); 
    // Start the trajectory
    arm.startTrajectory();
    // Wait for trajectory completion
    while(!arm.getState().isDone() && ros::ok())
    {
        usleep(50000);
    }

    std::cout << "Estimated force result: " << arm.get_result() << std::endl;
}
