#include "tcm3_compass_driver_node.h"
#include <cmath>
#include <numeric>
//#include "geometry.h"

Tcm3CompassDriverNode::Tcm3CompassDriverNode(ros::NodeHandle &nh) : iri_base_driver::IriBaseNodeDriver<Tcm3CompassDriver>(nh)
{
  //init class attributes if necessary
  this->loop_rate_ = 30;//driver_.config_.frequency;//in [Hz]

  // [init publishers]
  this->tcm3_publisher_ = this->public_node_handle_.advertise<iri_common_drivers_msgs::compass3axis>("tcm3", 100);

  data_counter_ = 0;
  vheading_.reserve(10);
  vpitch_.reserve(10);
  vroll_.reserve(10);
  covariance_buffer_length_ = 10;

  // [init subscribers]

  // [init services]

  // [init clients]

  // [init action servers]

  // [init action clients]
}

void Tcm3CompassDriverNode::mainNodeThread(void)
{
  //lock access to driver if necessary
  this->driver_.lock();

  driver_.getData(heading_,pitch_,roll_,distortion_); // [º]

  if(vheading_.size() <= covariance_buffer_length_)
  {
    vheading_.push_back(heading_);
    vpitch_.push_back(pitch_);
    vroll_.push_back(roll_);
  }else{
    vheading_.pop_back();
    vpitch_.pop_back();
    vroll_.pop_back();
    vheading_.insert(vheading_.begin(),heading_);
    vpitch_.insert(vpitch_.begin(),pitch_);
    vroll_.insert(vroll_.begin(),roll_);
  }

  //data_counter_ = data_counter_ > 9 ? 0 : data_counter_+1 ;

  // [fill msg Header if necessary]
  // [fill msg structures]

  // header
  // -------------------------------------------------------------
  compass3axis_msg_.header.stamp = ros::Time::now();
  compass3axis_msg_.header.frame_id = driver_.config_.frame_id;

  // angles
  // -------------------------------------------------------------
  compass3axis_msg_.angles[0] = heading_;// * M_PI/180.0;
  compass3axis_msg_.angles[1] = pitch_;// * M_PI/180.0 ;
  compass3axis_msg_.angles[2] = roll_;// * M_PI/180.0;

  // covariances
  // -------------------------------------------------------------
  // USING LAST LECTURES TO CALCULATE THE COVARIANCE
  //covarianceMatrix(vheading_,vpitch_,vroll_,cov);
  //vector <double> cov(9,0);
  //for(uint i=0;i<9;i++) compass3axis_msg_.covariances[i] = cov[i];
  // USING DATASHEET INFO
  for(uint i=0;i<9;i++) compass3axis_msg_.covariances[i] = 999;

  double sigma_head = 0.0, sigma_pitch = 0.0, sigma_roll = 0.0;

  if(abs(pitch_) < 70.0 || abs(roll_) < 70.0)
    sigma_head = 0.5;
  else
    sigma_head = 0.8;

  sigma_pitch = 0.2;

  if(abs(pitch_) < 65.0)
    sigma_roll = 0.2;
  else if(abs(pitch_) >= 65.0 && abs(pitch_) < 80.0)
    sigma_roll = 0.5;
  else
    sigma_roll = 1.0;

  compass3axis_msg_.covariances[0]=pow(sigma_head,2);
  compass3axis_msg_.covariances[4]=pow(sigma_pitch,2);
  compass3axis_msg_.covariances[8]=pow(sigma_roll,2);
  // alarms
  // -------------------------------------------------------------
  double limit = 80.0 * M_PI/180.0;
  compass3axis_msg_.alarms[0] = distortion_;
  compass3axis_msg_.alarms[1] = (pitch_ > -limit && pitch_ < limit ) ? false: true;
  compass3axis_msg_.alarms[2] = (roll_  > -limit && roll_  < limit ) ? false: true;

  // [fill srv structure and make request to the server]

  // [fill action structure and make request to the action server]

  // [publish messages]
  this->tcm3_publisher_.publish(compass3axis_msg_);



  //unlock access to driver if previously blocked
  this->driver_.unlock();
}

/*  [subscriber callbacks] */

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void Tcm3CompassDriverNode::postNodeOpenHook(void)
{
}

void Tcm3CompassDriverNode::addNodeDiagnostics(void)
{
}

void Tcm3CompassDriverNode::addNodeOpenedTests(void)
{
}

void Tcm3CompassDriverNode::addNodeStoppedTests(void)
{
}

void Tcm3CompassDriverNode::addNodeRunningTests(void)
{
}

void Tcm3CompassDriverNode::reconfigureNodeHook(int level)
{
}

Tcm3CompassDriverNode::~Tcm3CompassDriverNode()
{
  // [free dynamic memory]
}

/* main function */
int main(int argc,char *argv[])
{
  return driver_base::main<Tcm3CompassDriverNode>(argc,argv,"tcm3_compass_driver_node");
}

void Tcm3CompassDriverNode::covarianceMatrix(vector <double> x,
                                                        vector <double> y,
                                                        vector <double> z,
                                                        vector <double> & matrix)
{
  matrix[0] = covariance(x,x);
  matrix[1] = covariance(x,y);
  matrix[2] = covariance(x,z);
  matrix[3] = covariance(y,x);
  matrix[4] = covariance(y,y);
  matrix[5] = covariance(y,z);
  matrix[6] = covariance(z,x);
  matrix[7] = covariance(z,y);
  matrix[8] = covariance(z,z);
}

double Tcm3CompassDriverNode::covariance(vector <double> x, vector <double> y)
{
  double sum=0.0, x_avg=0.0, y_avg=0.0;

  x_avg = std::accumulate(x.begin(), x.end(), 0)/(double)x.size();
  y_avg = std::accumulate(y.begin(), y.end(), 0)/(double)y.size();

  for(uint i=0;i<x.size();i++)
    sum += (x[i]-x_avg)*(y[i]-y_avg);

  return ( sum/(double)x.size() );
}





