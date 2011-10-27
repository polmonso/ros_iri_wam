#include "segway_rmp400_odom_2D_displacement_based.h"

SegwayRmp400Odom::SegwayRmp400Odom() :
  accum_th_(0.0)
{ }

SegwayRmp400Odom::~SegwayRmp400Odom()
{ }

void
SegwayRmp400Odom::config_update(const Config& new_cfg, uint32_t level)
{
  config_ = new_cfg;
}

//robot frame
  //  x:fwd, y:left-side, z:up
void
SegwayRmp400Odom::computeOdometry(const iri_segway_rmp_msgs::SegwayRMP400Status & msg)
{
    current_time_ = ros::Time::now();

    // calculate the delta T
    double dt = (current_time_ - last_time_).toSec();

    double delta_fwd_displ = (msg.rmp200[0].forward_displacement +
                              msg.rmp200[1].forward_displacement) / 2;

    //left turn -> positive. in rads
    //  double vth = - ((msg.rmp200[0].yaw_rate + msg.rmp200[1].yaw_rate) / 2);
    double vth0 = (msg.rmp200[0].left_wheel_displacement - msg.rmp200[0].right_wheel_displacement) / 0.6;
    double vth1 = (msg.rmp200[1].left_wheel_displacement - msg.rmp200[1].right_wheel_displacement) / 0.6;
    double vth = (vth0 + vth1) / 2;

    // First, we will only use the half of time. See hearder file for info.
    accum_th_ += vth * dt/2;

    double xx = delta_fwd_displ * cos(accum_th_);
    double yy = delta_fwd_displ * sin(accum_th_);

    //update second half theta
    accum_th_ += vth * dt/2;

    // Calculate velocity from delta X position / time
    double vx = (xx - pose_.pose.position.x) / dt;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(accum_th_);

    transform_.translation.x += xx;
    transform_.translation.y += yy;
    transform_.translation.z  = 0.0;
    transform_.rotation       = odom_quat;

    pose_.pose.position.x += xx;
    pose_.pose.position.y += yy;
    pose_.pose.position.z  = 0.0;
    pose_.pose.orientation = odom_quat;

    twist_.twist.linear.x  = vx;
    twist_.twist.linear.y  = 0.0;
    twist_.twist.linear.z  = 0.0;
    twist_.twist.angular.x = 0.0;
    twist_.twist.angular.y = 0.0;
    twist_.twist.angular.z = vth;

    //update last time
    last_time_ = current_time_;
}

int main(int argc, char *argv[])
{
  return algorithm_base::main< GenericOdometry<SegwayRmp400Odom, iri_segway_rmp_msgs::SegwayRMP400Status> >(argc, argv, "segway_rmp400_odom_2D_displacement_based");
}
