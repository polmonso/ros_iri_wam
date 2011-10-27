//Extract the current point cloud to a matlab file in the format
//X1..Xn
//Y1..Yn
//Z1..Zn
// the idea is to use a bag file, stop in the interesting frame an being able to save that pointcloud

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <boost/foreach.hpp>

//int nc=176; //tamany de la matriu
//int nr=144;

int nImages = 0;
std::ofstream rimage;

int callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  int nc=msg->width;
  int nr=msg->height;
  std::stringstream rimagefilename;
  rimagefilename << "frameTOFDepth" << std::setw(2) << std::setfill('0') << nImages << ".txt";
  std::cout << rimagefilename.str() << std::endl;
  rimage.open (rimagefilename.str().c_str(), std::ios::out);
  if (!rimage){
    std::cerr << "Unable to open the file:\n\t" << rimagefilename << std::endl;
    return 1; // terminate with error
  }

  for (int ii=0;ii<nr;ii++) {
    for (int jj=0;jj<nc;jj++){
      int index2 = (ii)*nc +(jj);
      float *pstep2 = (float*)&msg->data[(index2) * msg->point_step];
      //TODO: es pot direccionar pel nom del camp???  msg->data[(index2) * msg->point_step].x
      rimage << ii << " " << jj << " " << pstep2[2]*1000 << std::endl;
    }
  }

  rimage.close();
  
  //  ros::shutdown();
  nImages++;
  return 0;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle nh;
  // to create a subscriber, you can do this (as above):
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/input", 1, callback);
  std::cout << "ini " << std::endl;
  ros::spin();
  std::cout << "end " << std::endl;
  return 0;
}
