#include "iri_laser_people_detection_alg_node.h"

IriLaserPeopleDetectionAlgNode::IriLaserPeopleDetectionAlgNode(void)
{
  //init library object
  myPeopleDetector = new cPeopleDetector();
  
  //init class attributes if necessary
  this->loop_rate_ = 10;//in [Hz]

  // [init publishers]
  this->people_publisher_ = this->public_node_handle_.advertise<visualization_msgs::MarkerArray>("people_array", 100);
  
  // [init subscribers]
  this->scan_subscriber_ = this->public_node_handle_.subscribe("scan", 100, &IriLaserPeopleDetectionAlgNode::scan_callback, this);
  
  // [init services]
  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]
}

IriLaserPeopleDetectionAlgNode::~IriLaserPeopleDetectionAlgNode(void)
{
  // [free dynamic memory]
}

void IriLaserPeopleDetectionAlgNode::mainNodeThread(void)
{
  // [fill msg structures]
  //this->MarkerArray_msg.data = my_var;
  
  scan_mutex_.enter();
  this->myPeopleDetector->iteration(currentScan.ranges,currentScan.angle_min,currentScan.angle_max,currentScan.angle_increment);
  this->getPeople(this->MarkerArray_msg_);
  scan_mutex_.exit();
    
  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]
  this->people_publisher_.publish(this->MarkerArray_msg_);
}

/*  [subscriber callbacks] */
void IriLaserPeopleDetectionAlgNode::scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg) 
{ 
  //ROS_INFO("IriLaserPeopleDetectionAlgNode::scan_callback: New Message Received"); 

  //use appropiate mutex to shared variables if necessary 
  //this->driver_.lock(); 
  this->scan_mutex_.enter(); 

  //std::cout << msg->data << std::endl; 
  currentScan.header.seq = msg->header.seq;
  currentScan.header.stamp = msg->header.stamp;
  currentScan.header.frame_id = msg->header.frame_id;
  currentScan.angle_min = msg->angle_min;
  currentScan.angle_max = msg->angle_max;
  currentScan.angle_increment = msg->angle_increment;
  currentScan.time_increment = msg->time_increment; 
  currentScan.scan_time = msg->scan_time;
  currentScan.range_min = msg->range_min;
  currentScan.range_max = msg->range_max;
  currentScan.ranges = msg->ranges;
  currentScan.intensities = msg->intensities;
  //unlock previously blocked shared variables 
  //this->driver_.unlock(); 
  this->scan_mutex_.exit(); 
}

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void IriLaserPeopleDetectionAlgNode::node_config_update(Config &config, uint32_t level)
{
}

void IriLaserPeopleDetectionAlgNode::addNodeDiagnostics(void)
{
}

void IriLaserPeopleDetectionAlgNode::getPeople(visualization_msgs::MarkerArray & people)
{
  vector<vector<float> > peopleList;
  unsigned int newsize;		// number of people in this iteration
  unsigned int lastsize;		// size of array in previous iteration (number of people + deleted markers)
  //int lastsizeReal;	// number of people in previous iteration
  
  myPeopleDetector->getPeopleLocation(&peopleList);
  
  newsize = peopleList.size();
  lastsize = people.markers.size();

  if(newsize>lastsize){			// If size has to increase, 
    people.markers.resize(newsize);	// do a resize
  }					// If it decreases, we have to keep the size to delete the extra markers,
					// and it will resize on the next iteration
  
  if(lastsizeReal<lastsize){				// If some markers were deleted in the last iteration,
   people.markers.resize(max(lastsizeReal,newsize)); 	// resize to remain ones, or to newsize if it's greater (size increased)
  }
  
  for(unsigned int i=newsize;i<lastsize; i++){				// For decreased size (newsize<lastsize) 
      people.markers[i].action= visualization_msgs::Marker::DELETE; 	// delete extra markers
      //people.markers[i].color.a = 0; // hide them (action: ADD/MODIFY)
  }
  
  lastsizeReal=newsize;	// Real number of markers in the iteration (after delete extra ones)
   
  for(unsigned int i=0;i<newsize; i++){
    
    if(peopleList[i][0]!=0 && peopleList[i][1]!=0 && peopleList[i][2]!=0 ){
        
    people.markers[i].scale.x = 0.1;//0.4;
    people.markers[i].scale.y = 0.1;//0.4;
    people.markers[i].scale.z = 0.1;//+2*peopleList[i][2];
    
    people.markers[i].pose.position.x = peopleList[i][1];
    people.markers[i].pose.position.y = -peopleList[i][0];
    people.markers[i].pose.position.z = people.markers[i].scale.z/2;

    people.markers[i].header.frame_id = currentScan.header.frame_id;
    people.markers[i].header.stamp    = currentScan.header.stamp;

    //people.markers[i].header.stamp = ros::Time();
    //people.markers[i].ns = "my_people";
    people.markers[i].id = i;
    people.markers[i].type = visualization_msgs::Marker::CYLINDER;
    people.markers[i].action = visualization_msgs::Marker::ADD;
    
    people.markers[i].pose.orientation.x = 0.0;
    people.markers[i].pose.orientation.y = 0.0;
    people.markers[i].pose.orientation.z = 0.0;
    people.markers[i].pose.orientation.w = 1.0;
    
    people.markers[i].color.a = 0.75;
    people.markers[i].color.r = 1.0;
    people.markers[i].color.g = 0.0;
    people.markers[i].color.b = 0.0;
    }
  }

}



/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<IriLaserPeopleDetectionAlgNode>(argc, argv, "iri_laser_people_detection_alg_node");
}
