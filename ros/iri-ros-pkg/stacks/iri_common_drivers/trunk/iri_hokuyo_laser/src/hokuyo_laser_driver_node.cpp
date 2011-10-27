#include "hokuyo_laser_driver_node.h"
#include <cmath>

HokuyoLaserDriverNode::HokuyoLaserDriverNode(ros::NodeHandle &nh) : iri_base_driver::IriBaseNodeDriver<HokuyoLaserDriver>(nh)
{
  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]

  // [init publishers]
  this->scan_publisher = this->public_node_handle_.advertise<sensor_msgs::LaserScan>("scan", 100);

  // [init subscribers]

  // [init services]

  // [init clients]

  // [init action servers]

  // [init action clients]
}

void HokuyoLaserDriverNode::mainNodeThread(void)
{
  //lock access to driver if necessary
  this->driver_.lock();

  // [fill msg Header if necessary]
  //<publisher_name>.header.stamp = ros::Time::now();
  //<publisher_name>.header.frame_id = "<publisher_topic_name>";

  // [fill msg structures]
  //this->LaserScan_msg.data = my_var;
  CEventServer *event_server;
  event_server = CEventServer::instance();
  THokuyo_scan aux_scan;
  std::list<std::string> evnts;
  evnts.push_back(this->driver_.get_new_scan_available_event());

  // Get Scan
  try
  {
    event_server->wait_all(evnts);
    this->driver_.getScan(aux_scan);
  }
  catch(CException &e)
  {
    cout << e.what() << endl;
  }

  // Build message
  // header
    ros::Time t_laser( aux_scan.t.seconds(), aux_scan.t.nanoseconds() );
    this->LaserScan_msg.header.stamp    = t_laser;                       // [s]
    this->LaserScan_msg.header.frame_id = this->driver_.config_.frame_id;   // string
  // configuration
    this->LaserScan_msg.angle_min       = this->driver_.config_.angle_min;  // [rad]
    this->LaserScan_msg.angle_max       = this->driver_.config_.angle_max;  // [rad]
    this->LaserScan_msg.angle_increment = this->driver_.angle_increment;    // [rad]
    this->LaserScan_msg.time_increment  = this->driver_.time_increment;     // [s]
    this->LaserScan_msg.scan_time       = this->driver_.scan_time;          // [s]
    this->LaserScan_msg.range_min       = this->driver_.range_min;          // [m]
    this->LaserScan_msg.range_max       = this->driver_.range_max;          // [m]
  // ranges
    vector<float> franges;
    // Unit conversion: [mm] (int) -> [m] (float)
    for(uint i=0;i<aux_scan.distance.size();i++)
      franges.push_back((float)(aux_scan.distance[i])/1000);
    this->LaserScan_msg.ranges          = franges;                        // [m]
  // intensities
    vector<float> fintens(aux_scan.intensity.begin(), aux_scan.intensity.end());
    this->LaserScan_msg.intensities     = fintens;                        // relative

  // [fill srv structure and make request to the server]

  // [fill action structure and make request to the action server]

  // [publish messages]
  this->scan_publisher.publish(this->LaserScan_msg);

  //unlock access to driver if previously blocked
  this->driver_.unlock();
}

/*  [subscriber callbacks] */

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void HokuyoLaserDriverNode::postNodeOpenHook(void)
{
}

void HokuyoLaserDriverNode::addNodeDiagnostics(void)
{
}

void HokuyoLaserDriverNode::addNodeOpenedTests(void)
{
}

void HokuyoLaserDriverNode::addNodeStoppedTests(void)
{
}

void HokuyoLaserDriverNode::addNodeRunningTests(void)
{
}

void HokuyoLaserDriverNode::reconfigureNodeHook(int level)
{
}

HokuyoLaserDriverNode::~HokuyoLaserDriverNode()
{
  //[free dynamic memory]
}

/* main function */
int main(int argc,char *argv[])
{
  return driver_base::main<HokuyoLaserDriverNode>(argc,argv,"hokuyo_laser_driver_node");
}
