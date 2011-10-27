/* vim: set sw=4 sts=4 et foldmethod=syntax : */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <teo_teleop/RearFrontObstacleDetection.h>

class TeleopObstacleFilter
{
    public:
        TeleopObstacleFilter();

    private:
        bool allow_movement(const geometry_msgs::Twist & movement);
        bool exist_valid_detection_now();
        bool is_movement_forward(const geometry_msgs::Twist & movement);
        bool is_movement_backwards(const geometry_msgs::Twist & movement);
        bool is_obstacle_in_front();
        bool is_obstacle_in_rear();

        /* callbacks */
        void movement_callback(const geometry_msgs::Twist::ConstPtr & movement);
        void obstacle_detection_callback(const teo_teleop::RearFrontObstacleDetection::ConstPtr & obstacle);

        teo_teleop::RearFrontObstacleDetection last_obstacle_detection_;
        // security margin to keep detection after the message arrival. 
        // In seconds
        ros::Duration detection_margin_;

        ros::NodeHandle nh_;
        ros::Subscriber vel_input_sub_;
        ros::Subscriber obstacle_sub_;
        ros::Publisher vel_output_pub_;
};

TeleopObstacleFilter::TeleopObstacleFilter() :
    detection_margin_(ros::Duration(10.0))
{

    vel_input_sub_ = nh_.subscribe<geometry_msgs::Twist>
                    ("/teo/segway/cmd_vel_desired", 1, & TeleopObstacleFilter::movement_callback, this);
    obstacle_sub_  = nh_.subscribe<teo_teleop::RearFrontObstacleDetection>
                   ("/teo/segway/rear_front_obstacles", 1, &TeleopObstacleFilter::obstacle_detection_callback, this);
    vel_output_pub_ = nh_.advertise<geometry_msgs::Twist>
                   ("/teo/segway/cmd_vel", 1);
}

bool
TeleopObstacleFilter::allow_movement(const geometry_msgs::Twist & movement)
{
    if (is_movement_forward(movement))
        if (is_obstacle_in_front())
            return false;

    if (is_movement_backwards(movement))
        if (is_obstacle_in_rear())
            return false;

    return true;
}

bool
TeleopObstacleFilter::is_movement_forward(const geometry_msgs::Twist & movement)
{
    return (movement.linear.x > 0.0);
}

bool
TeleopObstacleFilter::is_movement_backwards(const geometry_msgs::Twist & movement)
{
    return (movement.linear.x < 0.0);
}

bool
TeleopObstacleFilter::exist_valid_detection_now()
{
    ros::Duration diff = last_obstacle_detection_.stamp + detection_margin_ - ros::Time::now();
    ROS_INFO("Time difference since last detection: %f", diff.toSec());

    if (diff.toSec() > 0)
        return true;

    return false;
}

bool
TeleopObstacleFilter::is_obstacle_in_front()
{
    if (exist_valid_detection_now())
        if (last_obstacle_detection_.front == 1)
            return true;

    return false;
}

bool 
TeleopObstacleFilter::is_obstacle_in_rear()
{
    if (exist_valid_detection_now())
        if (last_obstacle_detection_.rear == 1)
            return true;

    return false;
}

void
TeleopObstacleFilter::movement_callback(const geometry_msgs::Twist::ConstPtr & movement)
{
    ROS_INFO("Movement command recieved");

    if (allow_movement(* movement)) {
        ROS_INFO("Movement command sent");
        vel_output_pub_.publish(movement);
    } else {
        ROS_INFO("Movement command was blocked");
    }
}

void 
TeleopObstacleFilter::obstacle_detection_callback(const teo_teleop::RearFrontObstacleDetection::ConstPtr & obstacle)
{
    last_obstacle_detection_ = * obstacle;
    ROS_INFO("Obstacle detection message recieved");
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "teleop_obstacle_filter");
    TeleopObstacleFilter teleop_obstacle_filter;

    ros::spin();
}
