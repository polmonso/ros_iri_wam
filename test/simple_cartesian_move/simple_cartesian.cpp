#include <ros/ros.h>
#include "geometry_msgs/Pose.h"
#include "iri_wam_common_msgs/pose_move.h"

int main(int argc, char** argv)
{
    // Init the ROS node
    ros::init(argc, argv, "robot_driver");

    if (argc != 11) {
        std::cerr << "Usage: " << argv[0]
                  << " /wrapper/pose_move pos.x pos.y. pos.z quat.x quat.y quat.z quat.w vel acc (vel recomended 0.1) (acc recomended 0.2)" << std::endl;
        return 1;
    }

    iri_wam_common_msgs::pose_move pose_srv;

    int p = 2; // initial argument to start building the pose
    pose_srv.request.pose.position.x    = atof(argv[p]);
    pose_srv.request.pose.position.y    = atof(argv[p+1]);
    pose_srv.request.pose.position.z    = atof(argv[p+2]);
    pose_srv.request.pose.orientation.x = atof(argv[p+3]);
    pose_srv.request.pose.orientation.y = atof(argv[p+4]);
    pose_srv.request.pose.orientation.y = atof(argv[p+5]);
    pose_srv.request.pose.orientation.z = atof(argv[p+6]);
    pose_srv.request.vel = atof(argv[p+7]);
    pose_srv.request.acc = atof(argv[p+8]);

    if (ros::service::call(argv[1], pose_srv))
    {
        std::cout << "Move done" << std::endl;
    }

    return 0;
}
