#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_assembler/AssembleScans.h>
#include <sensor_msgs/point_cloud_conversion.h>

using namespace laser_assembler;
    // Need to track if we've called the timerCallback at least once
//////////////////////////////////////////////////////////////////////////////////////////////////////////
class PeriodicSnapshotter
{

public:

  PeriodicSnapshotter(double scan_time)
  {

    scan_time_=scan_time;
    // Create a publisher for the clouds that we assemble
    pub_ = n_.advertise<sensor_msgs::PointCloud2> ("snap/assembled_cloud", 1);

    // Create the service client for calling the assembler
    client_ = n_.serviceClient<AssembleScans>("floor_assembler");

    // Start the timer that will trigger the processing loop (timerCallback)
    timer_ = n_.createTimer(ros::Duration(scan_time), &PeriodicSnapshotter::timerCallback, this);

    // Need to track if we've called the timerCallback at least once
    first_time_ = true;
  }

  void timerCallback(const ros::TimerEvent& e)
  {
    // We don't want to build a cloud the first callback, since we we
    //   don't have a start and end time yet
    if (first_time_)
    {
      first_time_ = false;
      return;
    }
    // Populate our service request based on our timer callback times
    AssembleScans srv;
    srv.request.begin = e.last_real;
    srv.request.end   = e.current_real;

    // Make the service call
    if (client_.call(srv))
    {
      ROS_DEBUG("Published Cloud with %u points", (uint32_t)(srv.response.cloud.points.size())) ;
      sensor_msgs::convertPointCloudToPointCloud2(srv.response.cloud, cloud_floor_laser_); 	

      pub_.publish(cloud_floor_laser_);
    }
    else
    {
      ROS_ERROR("Error making service call\n") ;
    }
  }

private:
  ros::NodeHandle n_;
  ros::Publisher pub_;
  ros::ServiceClient client_;
  ros::Timer timer_;
  bool first_time_;
  sensor_msgs::PointCloud2 cloud_floor_laser_;
  double scan_time_;



} ;



int main(int argc, char **argv)
{
  ros::init(argc, argv, "periodic_snapshotter");
  ros::NodeHandle n;
  double scan_time_; //How long snap should wait until assemble a new cloud (in seconds)
  ROS_INFO("Waiting for [build_cloud] to be advertised");
  ros::service::waitForService("floor_assembler");
  ROS_INFO("Found floor_assembled! Starting the snapshotter");
  n.param<double>("/snap/scan_time_", scan_time_, 0.5);
  PeriodicSnapshotter snapshotter(scan_time_);
  ros::spin();
  return 0;
}
