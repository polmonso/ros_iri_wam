#include "iri_people_tracking_alg_node.h"

IriPeopleTrackingAlgNode::IriPeopleTrackingAlgNode(void)
{
  //init class attributes if necessary
  this->loop_rate_ = 10;//in [Hz]

  // [init publishers]
  this->peopleSet_publisher_ = this->public_node_handle_.advertise<iri_people_tracking::peopleTrackingArray>("peopleSet", 100);
  this->particleSet_publisher_ = this->public_node_handle_.advertise<visualization_msgs::MarkerArray>("particleSet", 100);
  
  // [init subscribers]
  this->platformOdometry_subscriber_ = this->public_node_handle_.subscribe("odom", 100, &IriPeopleTrackingAlgNode::platformOdometry_callback, this);
  this->laserDetection_subscriber_ = this->public_node_handle_.subscribe("people_array", 100, &IriPeopleTrackingAlgNode::laserDetection_callback, this);
  
  // [init services]
  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]
}

IriPeopleTrackingAlgNode::~IriPeopleTrackingAlgNode(void)
{
  // [free dynamic memory]
}

void IriPeopleTrackingAlgNode::mainNodeThread(void)
{
	std::list<CpeopleTrackingObservation> peopleList;
	std::list<CpeopleTrackingObservation>::iterator iiP;
	double headingAngle;
	geometry_msgs::Quaternion headingQuaternion;
	double markerSize;
	
	//single iteration of the tracker
	std::cout << "Detection Set:"; tracker.printDetectionSet();
	
	//prior
	this->platformOdometry_mutex_.enter(); 
	tracker.propagateFilters(odometry);
	this->platformOdometry_mutex_.exit(); 
//std::cout << "debug: " << __LINE__ << std::endl;	
	tracker.propagateFilters();
	//std::cout << "Prior peopleSet: "; tracker.printPeopleSet();

	//posterior
	this->laserDetection_mutex_.enter(); 	
	tracker.negotiateNewFilters();
//std::cout << "debug: " << __LINE__ << std::endl;	
	tracker.correctFilters();
	std::cout << "Posterior People Set: "; tracker.printPeopleSet();
	tracker.negotiateDeleteFilters();
//std::cout << "debug: " << __LINE__ << std::endl;	
	this->laserDetection_mutex_.exit(); 	
	
	//get results & resampling
	tracker.getTrackingResult(peopleList);
//std::cout << "debug: " << __LINE__ << std::endl;	
	tracker.resampleFilters();
	std::cout << "*********************************" << std::endl;
		
	// [fill msg structures]
	unsigned int ii = 0;
	this->peopleTrackingArray_msg_.peopleSet.resize(peopleList.size());
	this->MarkerArray_msg_.markers.clear();
	this->MarkerArray_msg_.markers.resize(peopleList.size());
	
	// update header
	this->peopleTrackingArray_msg_.header.seq++;
	this->laserDetection_mutex_.enter();
	this->peopleTrackingArray_msg_.header.frame_id = people_detector_frame_id_;
	this->peopleTrackingArray_msg_.header.stamp    = people_detector_stamp_;
	this->laserDetection_mutex_.exit();
	
	for (iiP=peopleList.begin();iiP!=peopleList.end();iiP++)
	{
		//peopleTrackingArray_msg_
		this->peopleTrackingArray_msg_.peopleSet[ii].targetId = iiP->id;
		this->peopleTrackingArray_msg_.peopleSet[ii].x = iiP->point.getX();
		this->peopleTrackingArray_msg_.peopleSet[ii].y = iiP->point.getY();
		this->peopleTrackingArray_msg_.peopleSet[ii].vx = iiP->velocities.getX();
		this->peopleTrackingArray_msg_.peopleSet[ii].vy = iiP->velocities.getY();
		this->peopleTrackingArray_msg_.peopleSet[ii].covariances[0] = iiP->point.getCxx();
		this->peopleTrackingArray_msg_.peopleSet[ii].covariances[1] = iiP->point.getCxy();
		this->peopleTrackingArray_msg_.peopleSet[ii].covariances[2] = 0.0;
		this->peopleTrackingArray_msg_.peopleSet[ii].covariances[3] = 0.0;
		this->peopleTrackingArray_msg_.peopleSet[ii].covariances[4] = iiP->point.getCxy();
		this->peopleTrackingArray_msg_.peopleSet[ii].covariances[5] = iiP->point.getCyy();
		this->peopleTrackingArray_msg_.peopleSet[ii].covariances[6] = 0.0;
		this->peopleTrackingArray_msg_.peopleSet[ii].covariances[7] = 0.0;
		this->peopleTrackingArray_msg_.peopleSet[ii].covariances[8] = 0.0;
		this->peopleTrackingArray_msg_.peopleSet[ii].covariances[9] = 0.0;
		this->peopleTrackingArray_msg_.peopleSet[ii].covariances[10] = 0.0; //iiP->cvx;//to do 
		this->peopleTrackingArray_msg_.peopleSet[ii].covariances[11] = 0.0; //iiP->cvxy;//to do 
		this->peopleTrackingArray_msg_.peopleSet[ii].covariances[12] = 0.0;
		this->peopleTrackingArray_msg_.peopleSet[ii].covariances[13] = 0.0;
		this->peopleTrackingArray_msg_.peopleSet[ii].covariances[14] = 0.0; //iiP->cvxy;//to do
		this->peopleTrackingArray_msg_.peopleSet[ii].covariances[15] = 0.0; //iiP->cvy;//to do
		
		//MarkerArray_msg_
		this->MarkerArray_msg_.markers[ii].header = this->peopleTrackingArray_msg_.header;
		this->MarkerArray_msg_.markers[ii].id = iiP->id;
		this->MarkerArray_msg_.markers[ii].type = visualization_msgs::Marker::ARROW;
		this->MarkerArray_msg_.markers[ii].action = visualization_msgs::Marker::ADD;

		this->MarkerArray_msg_.markers[ii].pose.position.x = iiP->point.getX();
		this->MarkerArray_msg_.markers[ii].pose.position.y = iiP->point.getY();
		this->MarkerArray_msg_.markers[ii].pose.position.z = 0.2;

		//headingAngle = atan2(iiP->velocities.getY(), iiP->velocities.getX());
		headingAngle = iiP->heading;
		headingQuaternion = tf::createQuaternionMsgFromYaw(headingAngle);
		this->MarkerArray_msg_.markers[ii].pose.orientation.x = headingQuaternion.x;
		this->MarkerArray_msg_.markers[ii].pose.orientation.y = headingQuaternion.y;
		this->MarkerArray_msg_.markers[ii].pose.orientation.z = headingQuaternion.z;
		this->MarkerArray_msg_.markers[ii].pose.orientation.w = headingQuaternion.w;

		//matker size, color and duration
		markerSize = 0.6;
		this->MarkerArray_msg_.markers[ii].scale.x = markerSize/2.0;//0.6;
		this->MarkerArray_msg_.markers[ii].scale.y = markerSize*3.0;//0.6;
		this->MarkerArray_msg_.markers[ii].scale.z = markerSize/2.0;//0.6;
		this->MarkerArray_msg_.markers[ii].color.a = 0.75;
		this->MarkerArray_msg_.markers[ii].color.r = 1.0;
		this->MarkerArray_msg_.markers[ii].color.g = 0.5;
		this->MarkerArray_msg_.markers[ii].color.b = 0.0;
		this->MarkerArray_msg_.markers[ii].lifetime = ros::Duration(0.1f);

		ii++;
	}
	
	// [fill srv structure and make request to the server]
	
	// [fill action structure and make request to the action server]

	// [publish messages]
	this->peopleSet_publisher_.publish(this->peopleTrackingArray_msg_);
	this->particleSet_publisher_.publish(this->MarkerArray_msg_);
}

/*  [subscriber callbacks] */
void IriPeopleTrackingAlgNode::platformOdometry_callback(const nav_msgs::Odometry::ConstPtr& msg) 
{ 
	double vx,vy,vz,vTrans;
	double tLast, dT;
	
	//ROS_INFO("::platformOdometry_callback: New Message Received"); 

	//use appropiate mutex to shared variables if necessary 
	//this->driver_.lock(); 
	this->platformOdometry_mutex_.enter(); 
	
	//std::cout << msg->data << std::endl; 
	tLast = odometry.getTimeStamp();
	odometry.setTimeStamp();
	dT = odometry.getTimeStamp() - tLast;
	vx = msg->twist.twist.linear.x; 
	vy = msg->twist.twist.linear.y;
	vz = msg->twist.twist.linear.z;
	vTrans = sqrt(vx*vx+vy*vy+vz*vz);  //odometry observation considers only forward velocity
	odometry.acumulateToDeltas( dT*vTrans , dT*msg->twist.twist.angular.z ); //accumulates translational / rotational (only heading) displacement

	//unlock previously blocked shared variables 
	//this->driver_.unlock(); 
	this->platformOdometry_mutex_.exit(); 
}

void IriPeopleTrackingAlgNode::laserDetection_callback(const visualization_msgs::MarkerArray::ConstPtr& msg) 
{ 
	CpeopleDetectionObservation * newDetection;
	unsigned int ii;

	//ROS_INFO("::laserDetection_callback: New Message Received"); 

	//use appropiate mutex to shared variables if necessary 
	//this->driver_.lock(); 
	this->laserDetection_mutex_.enter(); 
	
	//std::cout << msg->data << std::endl; 

	//deletes previous detections
	tracker.resetDetectionSet();

  if(msg->markers.size()>0)
  {
    people_detector_frame_id_ = msg->markers[0].header.frame_id;
    //std::cout << "*******************************" << people_detector_frame_id_ << std::endl;
    people_detector_stamp_    = msg->markers[0].header.stamp;
  }

	//sets current (received) detections
	for (ii=0; ii<msg->markers.size(); ii++)
	{
		newDetection = new CpeopleDetectionObservation();
		newDetection->point.setXY(msg->markers[ii].pose.position.x, msg->markers[ii].pose.position.y);
		newDetection->point.setCov(DEFAULT_DET_COV_XY,DEFAULT_DET_COV_XY,0.0);
		tracker.addDetection(*newDetection);
	}

	//unlock previously blocked shared variables 
	//this->driver_.unlock(); 
	this->laserDetection_mutex_.exit(); 
}

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void IriPeopleTrackingAlgNode::node_config_update(Config &config, uint32_t level)
{
}

void IriPeopleTrackingAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<IriPeopleTrackingAlgNode>(argc, argv, "iri_people_tracking_alg_node");
}
