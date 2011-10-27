#include "localization3d_alg_node.h"

Localization3dAlgNode::Localization3dAlgNode(void)
{
	sleep(1);//time to allow tf to be ready
		
	//init class attributes if necessary
	this->loop_rate_ = 40;//in [Hz]
	infoGain = 0;

	// [init publishers]
	this->tf_publisher_ = this->public_node_handle_.advertise<tf::tfMessage>("odom_to_map", 100);
	this->particleSet_publisher_ = this->public_node_handle_.advertise<geometry_msgs::PoseArray>("particleSet", 100);
	this->position_publisher_ = this->public_node_handle_.advertise<geometry_msgs::PoseWithCovarianceStamped>("position", 100);
	
	// [init subscribers]
	this->platformData_subscriber_ = this->public_node_handle_.subscribe("platform_data", 100, &Localization3dAlgNode::platformData_callback, this);
	this->platformOdometry_subscriber_ = this->public_node_handle_.subscribe("platform_odom", 100, &Localization3dAlgNode::platformOdometry_callback, this);
	this->laser1_subscriber_ = this->public_node_handle_.subscribe("laser1/scan", 100, &Localization3dAlgNode::laser1_callback, this);
	this->laser2_subscriber_ = this->public_node_handle_.subscribe("laser2/scan", 100, &Localization3dAlgNode::laser2_callback, this);
	this->laser3_subscriber_ = this->public_node_handle_.subscribe("laser3/scan", 100, &Localization3dAlgNode::laser3_callback, this);
	
	// [init services]
	
	// [init clients]
	
	// [init action servers]
	
	// [init action clients]
	
	//initializes odometry time stamp to now
	odometry.setTimeStamp(); 
	
	//gets user parameters
	//std::cout << "CONFIG = " << alg_.config_.num_particles << std::endl; //doesn't work beacuse config_update is not yet called
	this->public_node_handle_.getParam("num_particles", numberOfParticles);
	this->public_node_handle_.getParam("mapFile", mapFileName);
	this->public_node_handle_.getParam("floorGridFile", gridFileName);
	this->public_node_handle_.getParam("initX", initX);
	this->public_node_handle_.getParam("initY", initY);
	this->public_node_handle_.getParam("initH", initH);
	this->public_node_handle_.getParam("l1_frameName",rDevConfig[0].frameName);
	this->public_node_handle_.getParam("l1_typeId",rDevConfig[0].deviceType);
	if (rDevConfig[0].deviceType == 0)
	{
		this->public_node_handle_.getParam("l1_nRays",rDevConfig[0].nRays);
		this->public_node_handle_.getParam("l1_aperture",rDevConfig[0].aperture);
		this->public_node_handle_.getParam("l1_rMin",rDevConfig[0].rangeMin);
		this->public_node_handle_.getParam("l1_rMax",rDevConfig[0].rangeMax);
	}
	this->public_node_handle_.getParam("l2_frameName",rDevConfig[1].frameName);
	this->public_node_handle_.getParam("l2_typeId",rDevConfig[1].deviceType);
	if (rDevConfig[1].deviceType == 0)
	{
		this->public_node_handle_.getParam("l2_nRays",rDevConfig[1].nRays);
		this->public_node_handle_.getParam("l2_aperture",rDevConfig[1].aperture);
		this->public_node_handle_.getParam("l2_rMin",rDevConfig[1].rangeMin);
		this->public_node_handle_.getParam("l2_rMax",rDevConfig[1].rangeMax);
	}
	this->public_node_handle_.getParam("l3_frameName",rDevConfig[2].frameName);
	this->public_node_handle_.getParam("l3_typeId",rDevConfig[2].deviceType);
	if (rDevConfig[2].deviceType == 0)
	{
		this->public_node_handle_.getParam("l3_nRays",rDevConfig[2].nRays);
		this->public_node_handle_.getParam("l3_aperture",rDevConfig[2].aperture);
		this->public_node_handle_.getParam("l3_rMin",rDevConfig[2].rangeMin);
		this->public_node_handle_.getParam("l3_rMax",rDevConfig[2].rangeMax);
	}

	//prints user's config
	printUserConfiguration();

	//init on-board laser mounting positions
	initDevicePositions();
	
	//Allocates memory to run the filter 
	pFilter = new CbasicPF(gridFileName, mapFileName);
	for (unsigned int ii=0; ii<MAX_RANGE_DEVICES; ii++)
	{
		if(rDevConfig[ii].frameName != "" )//device has been configured
		{
			if(rDevConfig[ii].deviceType == 0 ) //device type not specified -> use paremetric description
			{
				pFilter->addRangeModel(2, rDevConfig[ii].nRays, rDevConfig[ii].aperture, rDevConfig[ii].aperture/(double)rDevConfig[ii].nRays,rDevConfig[ii].rangeMin,rDevConfig[ii].rangeMax,ii+1);
			}
			else //device type has been specified
			{
				pFilter->addRangeModel(rDevConfig[ii].deviceType,ii+1);
			}
		}
	}
	//pFilter->addRangeModel(LEUZE_RS4,0);
	//pFilter->addRangeModel(HOKUYO_UTM30LX_180DEG,1);
	
	//initializes the filter with an initial estimate
	pFilter->trackingInit(numberOfParticles, initX, initY, initH);
	
}

Localization3dAlgNode::~Localization3dAlgNode(void)
{
	// [free dynamic memory]
	delete pFilter;
}

void Localization3dAlgNode::initDevicePositions()
{
	tf::TransformListener tfListener;
	tf::StampedTransform laserWRTbase;

	//get device mounting point with respect to the platform (base link)
	for (unsigned int ii=0; ii<MAX_RANGE_DEVICES; ii++)
	{
		if(rDevConfig[ii].frameName != "" )
		{
			tfListener.waitForTransform("base_link", rDevConfig[ii].frameName, ros::Time(0), ros::Duration(1.0));
			tfListener.lookupTransform("base_link", rDevConfig[ii].frameName, ros::Time(0), laserWRTbase);
			laserObs[ii].mountingPosition.setXYZ(laserWRTbase.getOrigin().x(),laserWRTbase.getOrigin().y(),laserWRTbase.getOrigin().z());
			laserObs[ii].mountingPosition.setQuaternion(laserWRTbase.getRotation().getW(),laserWRTbase.getRotation().getX(),laserWRTbase.getRotation().getY(),laserWRTbase.getRotation().getZ());
			//laserObs[ii].mountingPosition.printPosition();
		}
	}
	
	//fixed way
	//frontLaser.mountingPosition.setFullPose(0.3, 0.0, 0.45, 0.0, 0.0, 0.0); //to do: use tf server to get laser position wrt robot platform	
	//backLaser.mountingPosition.setFullPose(-0.3, 0.0, 0.45, M_PI, 0.0, M_PI); //to do: use tf server to get laser position wrt robot platform
	//vertLaser.mountingPosition.setFullPose(0.35, 0.0, 0.82, 0.0, 0.0, -M_PI/2); //to do: use tf server to get laser position wrt robot platform	
}

void Localization3dAlgNode::printUserConfiguration()
{
	std::cout << "******************* PARTICLE FILTER CONFIGURATION ***********************" << std::endl;
	std::cout << "Number Of Particles = \t " << numberOfParticles << std::endl;
	std::cout << "Map File = \t" << mapFileName << std::endl;
	std::cout << "Grid File = \t" << gridFileName << std::endl;
	std::cout << "Init (X,Y,H) = \t" << "(" << initX << "," << initY << "," << initH << ")" << std::endl;
	
	for (unsigned int ii=0; ii<MAX_RANGE_DEVICES; ii++)
	{
		if(rDevConfig[ii].frameName != "" )
		{
			std::cout << "RANGE DEVICE " << ii << std::endl;
			std::cout << "\t Frame Name = \t" << rDevConfig[ii].frameName << std::endl;
			std::cout << "\t Type = \t" << rDevConfig[ii].deviceType << std::endl;
			std::cout << "\t Num Rays = \t" << rDevConfig[ii].nRays << std::endl;
			std::cout << "\t Aperture = \t" << rDevConfig[ii].aperture << std::endl;
			std::cout << "\t Min Range = \t" << rDevConfig[ii].rangeMin << std::endl;
			std::cout << "\t Max Range = \t" << rDevConfig[ii].rangeMax << std::endl;
		}
	}
	std::cout << "*************************************************************************" << std::endl;
}

void Localization3dAlgNode::mainNodeThread(void)
{
	double qReal, qi, qj, qk; //aux variables to fill messages
	unsigned int ii;
	Cparticle3d *particlePtr;
	tf::Quaternion quat;
	
	//to access to config:
	//std::cout << "CONFIG 2 = " << alg_.config_.num_particles << std::endl;
	
	//process filter iteration
// std::cout << "mainNodeThread():" << __LINE__ << std::endl;	
	
	//increments iteration index
	pFilter->incrementIterationId();
// std::cout << "mainNodeThread():" << __LINE__ << std::endl;
// pFilter->checkPset();//debugging "nan" values
	
	//propagation
	this->platformOdometry_mutex_.enter();
 	//odometry.printObservation();
	pFilter->propagatePset(odometry);
	//priorTime = ros::Time::now();
	priorTime = odoTime;
	this->platformOdometry_mutex_.exit(); 
// std::cout << "mainNodeThread():" << __LINE__ << std::endl;		
// pFilter->checkPset();//debugging "nan" values
	
	//sets estimate time stamp just after propagation
	locEstimate.setTimeStamp();
	//this->PoseWithCovarianceStamped_msg_.header.stamp = ros::Time::now();
// std::cout << "mainNodeThread():" << __LINE__ << std::endl;		
// pFilter->checkPset();//debugging "nan" values

	//platform inclinometer correction
	this->platformData_mutex_.enter();
	//pFilter->correctPset(inclinometers);
	this->platformData_mutex_.exit(); 
// std::cout << "mainNodeThread():" << __LINE__ << std::endl;		
// pFilter->checkPset();//debugging "nan" values
	
	//laser 1 correction
	this->laser1_mutex_.enter();
	pFilter->correctPset(laserObs[0], 1);
	this->laser1_mutex_.exit(); 
// std::cout << "mainNodeThread():" << __LINE__ << std::endl;		
// pFilter->checkPset();//debugging "nan" values

	//laser 2 correction
	this->laser2_mutex_.enter();
	pFilter->correctPset(laserObs[1], 2);
	this->laser2_mutex_.exit(); 
// std::cout << "mainNodeThread():" << __LINE__ << std::endl;		
// pFilter->checkPset();//debugging "nan" values

	//laser 3 correction
	this->laser3_mutex_.enter();
	pFilter->correctPset(laserObs[2], 3);
	this->laser3_mutex_.exit(); 
// std::cout << "mainNodeThread():" << __LINE__ << std::endl;	
// pFilter->checkPset();//debugging "nan" values
	
	//normalize particle set
	pFilter->normalizePset();
// std::cout << "mainNodeThread():" << __LINE__ << std::endl;		
// pFilter->checkPset();//debugging "nan" values

	//computes Info gain for this iteration
	infoGain += pFilter->computeKLDivergence();
	//std::cout << "IG = " << infoGain << std::endl;

	//set estimate
	pFilter->setEstimate(locEstimate);
	locEstimate.position.printPosition();
// std::cout << "mainNodeThread():" << __LINE__ << std::endl;			
// pFilter->checkPset();//debugging "nan" values

	//resampling
	//pFilter->regularizedResamplingPset();
	pFilter->lowVarianceResamplingPset();
// std::cout << "mainNodeThread():" << __LINE__ << std::endl;		
// pFilter->checkPset();//debugging "nan" values

	// [fill msg structures]
	
	//fill particle cloud
	this->PoseArray_msg_.header.seq = pFilter->getIterationId();
	//this->PoseArray_msg_.header.stamp.sec = locEstimate.getTimeStampSeconds();
	//this->PoseArray_msg_.header.stamp.nsec = locEstimate.getTimeStampNanoSeconds();
	this->PoseArray_msg_.header.stamp = priorTime;
	this->PoseArray_msg_.header.frame_id = mapFrame;
	for(ii=0; ii<pFilter->getNumParticles(); ii++)
	{
		particlePtr = pFilter->getParticle(ii);
		PoseArray_msg_.poses.resize(ii+1);
		PoseArray_msg_.poses.at(ii).position.x = particlePtr->getX();
		PoseArray_msg_.poses.at(ii).position.y = particlePtr->getY();
		PoseArray_msg_.poses.at(ii).position.z = particlePtr->getZ();
// 		particlePtr->getQuaternion(qReal, qi, qj, qk);
// 		PoseArray_msg_.poses.at(ii).orientation.x = qi;
// 		PoseArray_msg_.poses.at(ii).orientation.y = qj;
// 		PoseArray_msg_.poses.at(ii).orientation.z = qk;
// 		PoseArray_msg_.poses.at(ii).orientation.w = qReal;
		quat = tf::createQuaternionFromRPY( locEstimate.position.getR(),locEstimate.position.getP(),locEstimate.position.getH() );
		PoseArray_msg_.poses.at(ii).orientation.x = quat.getX();
		PoseArray_msg_.poses.at(ii).orientation.y = quat.getY();
		PoseArray_msg_.poses.at(ii).orientation.z = quat.getZ();
		PoseArray_msg_.poses.at(ii).orientation.w = quat.getW();	
	}
// std::cout << "mainNodeThread():" << __LINE__ << std::endl;

	//fill localization pose estimate
	this->PoseWithCovarianceStamped_msg_.header.seq = pFilter->getIterationId();
	//this->PoseWithCovarianceStamped_msg_.header.stamp.sec = locEstimate.getTimeStampSeconds();
	//this->PoseWithCovarianceStamped_msg_.header.stamp.nsec = locEstimate.getTimeStampNanoSeconds();
	this->PoseWithCovarianceStamped_msg_.header.stamp = priorTime;
	this->PoseWithCovarianceStamped_msg_.header.frame_id = mapFrame;
	this->PoseWithCovarianceStamped_msg_.pose.pose.position.x = locEstimate.position.getX();
	this->PoseWithCovarianceStamped_msg_.pose.pose.position.y = locEstimate.position.getY();
	this->PoseWithCovarianceStamped_msg_.pose.pose.position.z = locEstimate.position.getZ();
	
	locEstimate.position.getQuaternion(qReal, qi, qj, qk);
	this->PoseWithCovarianceStamped_msg_.pose.pose.orientation.x = qi;
	this->PoseWithCovarianceStamped_msg_.pose.pose.orientation.y = qj;
	this->PoseWithCovarianceStamped_msg_.pose.pose.orientation.z = qk;
	this->PoseWithCovarianceStamped_msg_.pose.pose.orientation.w = qReal;

/*	quat = tf::createQuaternionFromRPY( locEstimate.position.getR(),locEstimate.position.getP(),locEstimate.position.getH() );
	this->PoseWithCovarianceStamped_msg_.pose.pose.orientation.x = quat.getX();
	this->PoseWithCovarianceStamped_msg_.pose.pose.orientation.y = quat.getY();
	this->PoseWithCovarianceStamped_msg_.pose.pose.orientation.z = quat.getZ();
	this->PoseWithCovarianceStamped_msg_.pose.pose.orientation.w = quat.getW();*/
	
	this->PoseWithCovarianceStamped_msg_.pose.covariance[0] = locEstimate.cxx;
	this->PoseWithCovarianceStamped_msg_.pose.covariance[1] = locEstimate.cxy;
	this->PoseWithCovarianceStamped_msg_.pose.covariance[6] = locEstimate.cxy;
	this->PoseWithCovarianceStamped_msg_.pose.covariance[7] = locEstimate.cyy;
	this->PoseWithCovarianceStamped_msg_.pose.covariance[14] = locEstimate.czz;
	this->PoseWithCovarianceStamped_msg_.pose.covariance[21] = locEstimate.chh;
	this->PoseWithCovarianceStamped_msg_.pose.covariance[28] = locEstimate.cpp;
	this->PoseWithCovarianceStamped_msg_.pose.covariance[35] = locEstimate.crr;	
// std::cout << "mainNodeThread():" << __LINE__ << std::endl;	
	
	// [fill srv structure and make request to the server]
	
	// [fill action structure and make request to the action server]

	// [publish messages]
	this->particleSet_publisher_.publish(this->PoseArray_msg_);
	this->position_publisher_.publish(this->PoseWithCovarianceStamped_msg_);
// std::cout << "mainNodeThread():" << __LINE__ << std::endl;	
	
	//Publish provided tf Transforms: odom wrt map
	tf::Pose mapToBase;
	tf::poseMsgToTF(this->PoseWithCovarianceStamped_msg_.pose.pose, mapToBase);
	tf::Stamped<tf::Pose> baseToMap(mapToBase.inverse(),ros::Time(0),baseFrame);
	tf::Stamped<tf::Pose> odomToMap;
	this->tfListener.transformPose(odomFrame, baseToMap, odomToMap);
	this->tfBroadcaster.sendTransform(tf::StampedTransform(odomToMap.inverse(),priorTime,mapFrame,odomFrame));

// std::cout << "mainNodeThread():" << __LINE__ << std::endl;	
// std::cout << "mainNodeThread(): END OF ITERATION" << std::endl << std::endl;	

}

/*  [subscriber callbacks] */
void Localization3dAlgNode::platformData_callback(const iri_segway_rmp_msgs::SegwayRMP200Status::ConstPtr& msg) 
{ 
	//ROS_INFO("Localization3dAlgNode::platformData_callback: New Message Received"); 
	
	//use appropiate mutex to shared variables if necessary 
	//this->driver_.lock(); 
	this->platformData_mutex_.enter(); 

	//std::cout << msg->data << std::endl; 
	inclinometers.markAsNew();
	inclinometers.setTimeStamp();//to do: it would be better to get ts from the message
	//inclinometers.setTimeStamp(msg->header.stamp.sec,msg->header.stamp.nsec);
	inclinometers.pitch = msg->pitch_angle;
	inclinometers.roll = msg->roll_angle;
	inclinometers.setMagneticDistortion(); //absolute heading is not provided by segway, so we indicate it by alarming 
	inclinometers.markAsCorrect(); //to do: we should ckeck for correctness before mark the observation as correct
	
	//unlock previously blocked shared variables 
	//this->driver_.unlock(); 
	this->platformData_mutex_.exit(); 
}
void Localization3dAlgNode::platformOdometry_callback(const nav_msgs::Odometry::ConstPtr& msg) 
{ 
	double vx,vy,vz,vTrans;
	double tLast, dT;
		
	//ROS_INFO("Localization3dAlgNode::platformOdometry_callback: New Message Received"); 

	//use appropiate mutex to shared variables if necessary 
	//this->driver_.lock(); 
	this->platformOdometry_mutex_.enter(); 

	//std::cout << msg->data << std::endl; 
	odometry.markAsNew();
	tLast = odometry.getTimeStamp(); //gets last time stamp
	//odometry.setTimeStamp(msg->header.stamp.sec,msg->header.stamp.nsec); //get ts from the message
	odometry.setTimeStamp(); //
	dT = odometry.getTimeStamp() - tLast; //computes elapsed time between consecutive readings
	vx = msg->twist.twist.linear.x; 
	vy = msg->twist.twist.linear.y;
	vz = msg->twist.twist.linear.z;
	vTrans = sqrt(vx*vx+vy*vy+vz*vz);  //odometry observation considers only forward velocity
	odometry.accumDeltaTrans(dT*vTrans); //accumulates translational displacement
	odometry.accumDeltaH(dT*msg->twist.twist.angular.z); //accumulates heading rotation
	odometry.accumDeltaP(dT*msg->twist.twist.angular.y); //accumulates pitch rotation
	odometry.accumDeltaR(dT*msg->twist.twist.angular.x); //accumulates roll rotation
	odometry.markAsCorrect();//to do: we should ckeck for correctness before mark the observation as correct
	//tf::poseMsgToTF(msg->pose.pose, odoPose);//keeps odoPose for tf broadcaster
	odoTime = msg->header.stamp;
	//odoTime = ros::Time::now();

	//unlock previously blocked shared variables 
	//this->driver_.unlock(); 
	this->platformOdometry_mutex_.exit(); 
}
void Localization3dAlgNode::laser1_callback(const sensor_msgs::LaserScan::ConstPtr& msg) 
{ 
	unsigned int ii;

	ROS_INFO("Localization3dAlgNode::laser1_callback: New Message Received"); 

	//use appropiate mutex to shared variables if necessary 
	//this->driver_.lock(); 
	this->laser1_mutex_.enter(); 

	//std::cout << msg->data << std::endl; 
	laserObs[0].ranges.clear(); //erase previous range data 
	laserObs[0].markAsNew();
	laserObs[0].setTimeStamp(msg->header.stamp.sec,msg->header.stamp.nsec);//laserObs[1].setTimeStamp();
	laserObs[0].numPoints = msg->ranges.size();
	laserObs[0].aperture = fabs(msg->angle_max - msg->angle_min);
	laserObs[0].rmin = msg->range_min;
	laserObs[0].rmax = msg->range_max;
	if (msg->angle_min > msg->angle_max) //angles are provided clockwise from an overhead point of view
	{
		for(ii=0; ii<laserObs[0].numPoints; ii++) laserObs[0].ranges.push_back(msg->ranges.at(ii));
	}
	else //angles are provided counterclockwise from an overhead point of view
	{
		for(ii=0; ii<laserObs[0].numPoints; ii++) laserObs[0].ranges.push_back(msg->ranges.at(laserObs[0].numPoints-1-ii));
	}
	laserObs[0].markAsCorrect();//to do: we should ckeck for correctness before mark the observation as correct
	//laserObs[1].printObservation();//debug: check received data
	
	//unlock previously blocked shared variables 
	//this->driver_.unlock(); 
	this->laser1_mutex_.exit(); 
}
void Localization3dAlgNode::laser2_callback(const sensor_msgs::LaserScan::ConstPtr& msg) 
{ 
	unsigned int ii;

	ROS_INFO("Localization3dAlgNode::laser2_callback: New Message Received"); 

	//use appropiate mutex to shared variables if necessary 
	//this->driver_.lock(); 
	this->laser2_mutex_.enter(); 

	//std::cout << msg->data << std::endl; 
	laserObs[1].ranges.clear(); //erase previous range data 
	laserObs[1].markAsNew();
	laserObs[1].setTimeStamp(msg->header.stamp.sec,msg->header.stamp.nsec);//laserObs[1].setTimeStamp();
	laserObs[1].numPoints = msg->ranges.size();
	laserObs[1].aperture = fabs(msg->angle_max - msg->angle_min);
	laserObs[1].rmin = msg->range_min;
	laserObs[1].rmax = msg->range_max;
	if (msg->angle_min > msg->angle_max) //angles are provided clockwise from an overhead point of view
	{
		for(ii=0; ii<laserObs[1].numPoints; ii++) laserObs[1].ranges.push_back(msg->ranges.at(ii));
	}
	else //angles are provided counterclockwise from an overhead point of view
	{
		for(ii=0; ii<laserObs[1].numPoints; ii++) laserObs[1].ranges.push_back(msg->ranges.at(laserObs[1].numPoints-1-ii));
	}
	laserObs[1].markAsCorrect();//to do: we should ckeck for correctness before mark the observation as correct
	//laserObs[1].printObservation();//debug: check received data
	
	//unlock previously blocked shared variables 
	//this->driver_.unlock(); 
	this->laser2_mutex_.exit(); 
}
void Localization3dAlgNode::laser3_callback(const sensor_msgs::LaserScan::ConstPtr& msg) 
{ 
	unsigned int ii;
	
	//ROS_INFO("Localization3dAlgNode::laserFront_callback: New Message Received"); 

	//use appropiate mutex to shared variables if necessary 
	//this->driver_.lock(); 
	this->laser3_mutex_.enter(); 

	//std::cout << msg->data << std::endl; 
	laserObs[2].ranges.clear(); //erase previous range data 
	laserObs[2].markAsNew();
	laserObs[2].setTimeStamp(msg->header.stamp.sec,msg->header.stamp.nsec);//laserObs[1].setTimeStamp();
	laserObs[2].numPoints = msg->ranges.size();
	laserObs[2].aperture = fabs(msg->angle_max - msg->angle_min);
	laserObs[2].rmin = msg->range_min;
	laserObs[2].rmax = msg->range_max;
	if (msg->angle_min > msg->angle_max) //angles are provided clockwise from an overhead point of view
	{
		for(ii=0; ii<laserObs[2].numPoints; ii++) laserObs[2].ranges.push_back(msg->ranges.at(ii));
	}
	else //angles are provided counterclockwise from an overhead point of view
	{
		for(ii=0; ii<laserObs[2].numPoints; ii++) laserObs[2].ranges.push_back(msg->ranges.at(laserObs[2].numPoints-1-ii));
	}
	laserObs[2].markAsCorrect();//to do: we should ckeck for correctness before mark the observation as correct
	//laserObs[2].printObservation();//debug: check received data
	
	//unlock previously blocked shared variables 
	//this->driver_.unlock(); 
	this->laser3_mutex_.exit(); 
}

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */


void Localization3dAlgNode::node_config_update(Config &config, uint32_t level)
{
	/*
	config_mutex.enter();
	std::cout << "CONFIG = " << this->config_.num_particles << std::endl;
	pFilter->trackingInit(this->config_.num_particles,initX,initY,initH);
	config_mutex.exit();
	*/
}

void Localization3dAlgNode::addNodeDiagnostics(void)
{
}

// void Localization3dAlgNode::setLaserObservation(const sensor_msgs::LaserScan::ConstPtr& msg, ClaserObservation & laserData)
// {
// 	laserData.
// }

/* main function */
int main(int argc,char *argv[])
{
	//initialize the filter (to do)
	
	//initialize glut (faramotics objects)
	glutInit(&argc, argv);

	//run the thread
	return algorithm_base::main<Localization3dAlgNode>(argc, argv, "localization3d_alg_node");
}
