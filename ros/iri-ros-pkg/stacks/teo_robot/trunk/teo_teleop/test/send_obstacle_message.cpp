/* vim: set sw=4 sts=4 et foldmethod=syntax : */

#include <ros/ros.h>
#include <teo_teleop/RearFrontObstacleDetection.h>

#include <iostream>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sender");
    ros::NodeHandle nh_;
    ros::Publisher obstacle_pub = nh_.advertise<teo_teleop::RearFrontObstacleDetection>
                                 ("/teo/segway/rear_front_obstacles", 1000);
    ros::Rate loop_rate(10);

    teo_teleop::RearFrontObstacleDetection msg;
    char c;

    std::cout << "Press f = front or r = rear. 0 to finish" << std::endl;

    while (std::cin.get(c)) 
    {
        if (c == '\n')
            continue;

        msg.stamp = ros::Time::now();

        switch (c)
        {
            case 'f':
                msg.front = 1;
                msg.rear  = 0;
                ROS_INFO("Sending obstacle in front");
                break;
            case 'r':
                msg.rear  = 1;
                msg.front = 0;
                ROS_INFO("Sending obstacle in rear");
                break;
            case '0':
                return 0;
                break;
            default:
                ROS_INFO("Sending message with no obstacles detected");
        }

        obstacle_pub.publish(msg);
    }
}
