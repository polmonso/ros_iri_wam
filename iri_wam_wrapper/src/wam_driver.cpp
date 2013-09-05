#include "wam_driver.h"
#include "wam_packet.h" // from wam low level driver

using namespace std;
using namespace XmlRpc;

WamDriver::WamDriver() :
    force_request_(new ForceRequest)
{
    ros::NodeHandle nh("~");

    XmlRpc::XmlRpcValue r_name;
    XmlRpc::XmlRpcValue ip;
    XmlRpc::XmlRpcValue port;
    XmlRpc::XmlRpcValue rate;

    nh.getParam("wam_ip",ip);
    this->wamserver_ip=(std::string)ip;

    if(!nh.hasParam("robot_name"))
    {
        ROS_WARN_STREAM("Robot name not defined. Default: " << "iri_wam");
        robot_name_="iri_wam";
    } 
    else
    {
        nh.getParam("robot_name", r_name);
        robot_name_=(std::string)r_name;
    }
    if(!nh.hasParam("port"))
    {
        ROS_WARN_STREAM("Port not defined. Defaults: "<<4321);
        this->server_port=4321;
    } 
    else
    {
        nh.getParam("port",port);
        this->server_port=(int)port;
    }
    if(!nh.hasParam("refresh_rate"))
    {
        ROS_WARN_STREAM("Refresh Rate not defined. Defaults: "<<100);
        this->state_refresh_rate=100;
    } 
    else
    {
        nh.getParam("refresh_rate",rate);
        this->state_refresh_rate=(int)rate;
    }

    ROS_INFO_STREAM("Robot Name: "   << this->robot_name_);
    ROS_INFO_STREAM("IP Address: "   << this->wamserver_ip);
    ROS_INFO_STREAM("Port Server: "  << this->server_port);
    ROS_INFO_STREAM("Refresh Rate: " << this->state_refresh_rate);
}

bool WamDriver::openDriver(void)
{
    //setDriverId(driver string id);
    string input;
    //setDriverId(driver string id);
    try{
        if(this->state_ != OPENED){
            this->wam = new CWamDriver(this->wamserver_ip, this->server_port, this->state_refresh_rate);
            wam->open();
            this->state_ = OPENED;
            ROS_INFO("Wam opened, press shift+idle and enter.");
            getchar();

            wam->create(); 
            ROS_INFO("Wam created, press shift+activate and press enter.");
            getchar(); 
            wam->activate();
            return true;
        }else{
            ROS_ERROR("WAM was already opened!");
            return false;
        }
    }catch(CException &e){
        ROS_ERROR("%s",e.what().c_str());
        return false;
    }
}

bool WamDriver::closeDriver(void)
{
    wam->close();
    ROS_INFO("[APP] Wait until wam gets home, idle the wam and press enter.");
    getchar();
    this->state_ = CLOSED;
    return true;
}

bool WamDriver::startDriver(void)
{
    try{
        wam->setGravityCompensation(1.1);
        ROS_INFO("Switching to Running and going to default position.");
        wam->goToDefaultPosition();
        ROS_INFO("Waiting final position reach");
        wam->waitTillMotionDone();
        ROS_INFO("All is ready to work now!");
        wam->setGravityCompensation(1.1);
        this->state_ = RUNNING;
        return true;
    }catch(CException &e){
        ROS_ERROR("%s",e.what().c_str());
        return false;
    }
    return true;
}

bool WamDriver::stopDriver(void)
{
    //return to home but keep waiting for messages
    wam->home();
    wam->waitTillMotionDone();
    this->state_ = OPENED;
    return true;
}

void WamDriver::config_update(const Config& new_cfg, uint32_t level)
{
  this->lock();
  //update driver with new_cfg data
  if(isRunning()){
    // save the current configuration
    this->config_=new_cfg;
  }else{
    ROS_ERROR("Driver is not running");
  }
  this->unlock();
}

WamDriver::~WamDriver()
{
    std::cout << "WamDriver destructor" << std::endl;
}

std::string WamDriver::get_robot_name() {

    return this->robot_name_;
}

int WamDriver::get_num_joints() {
    if(this->wam!=NULL) {
        return this->wam->getNumAngles();
    } 
    return 0;
}

bool WamDriver::is_moving() {
    if (this->wam != NULL) {
        return this->wam->isMoving();
    }
    return false;
}

void WamDriver::wait_move_end() {
    if(this->wam!=NULL) {
        this->wam->waitTillMotionDone();
    } 
}

void WamDriver::get_pose(std::vector<double> *pose) {
    if(this->wam!=NULL) {
        this->wam->getCartesianPose(pose);
    }
}

void WamDriver::get_joint_angles(std::vector<double> *angles) {
    if(this->wam!=NULL) {
        this->wam->getJointAngles(angles);
    } 
}

bool WamDriver::is_joints_move_request_valid(const std::vector<double> & angles){
    // Check number of joints sent to the robot
    if (static_cast<signed int>(angles.size()) != get_num_joints()) {
        ROS_ERROR("Invalid request to move. Joint vector size is %i while the robot has %i joints", 
                (int) angles.size(), get_num_joints());
        return false;
    }

    // Check valid values of those angles
    for (std::vector<double>::const_iterator it = angles.begin(); it != angles.end(); it++) {
        // Until std::isNan (c++11 feature) reach compilers, we use the Nan propiety
        // of not returning true when comparing against itself
        if (* it != * it) {
            ROS_ERROR("Invalid request to move. Joint vector contain a Nan value.");
            return false;
        }
    }

    return true;
}

void WamDriver::move_in_joints(std::vector<double> *angles, std::vector<double>* vels, std::vector<double>* accs){
    uint16_t errormask = 0x00;

    if (this->wam!=NULL) {
        if (! is_joints_move_request_valid(* angles)) {
            ROS_ERROR("Joints angles were not valid. Refuse to move.");
            return;
        }

        // Check if there are vels and accs
        if ((vels == NULL) || (accs == NULL)) {
            wam->moveInJoints(&errormask, angles);
        }
        else {
            wam->moveInJoints(&errormask, angles, vels, accs);
        }
        if(errormask > 0x00){
            string err_msg = wam->errorToString(errormask);
            ROS_ERROR("%s",err_msg.c_str());
            errormask = 0x00;
        }
    }
}

void
WamDriver::move_in_cartesian_pose(const geometry_msgs::Pose pose,const double vel,const double acc)
{
    if (this->wam == NULL)
        return;

    std::vector<double> low_level_pose;
    low_level_pose.push_back(pose.position.x);
    low_level_pose.push_back(pose.position.y);
    low_level_pose.push_back(pose.position.z);
    low_level_pose.push_back(pose.orientation.x);
    low_level_pose.push_back(pose.orientation.y);
    low_level_pose.push_back(pose.orientation.z);
    low_level_pose.push_back(pose.orientation.w);

    this->wam->moveInCartesianPose(&low_level_pose, vel, acc);
}

void
WamDriver::hold_current_position(bool on)
{
  if(this->wam!=NULL){
    this->wam->holdCurrentPosition(on);
  }
}

void
WamDriver::move_trajectory_in_joints(const trajectory_msgs::JointTrajectory & trajectory)
{
    uint16_t errormask = 0x00;
    WAMJointTrajectory low_level_trajectory;

    for (std::vector<trajectory_msgs::JointTrajectoryPoint>::const_iterator it = trajectory.points.begin();
         it != trajectory.points.end(); it++) {
            if (! is_joints_move_request_valid(it->positions)) {
                ROS_ERROR("Joints angles were not valid. Refuse to move.");
                return;
            }

            low_level_trajectory.push_back(it->positions);
    }

    this->wam->moveTrajectoryInJoints(&errormask, &low_level_trajectory);
}

void
WamDriver::move_trajectory_learnt_and_estimate_force(const std::string model_filename,
                                                     const std::string points_filename)
{
    // TODO: implement error handling
    force_request_->init();
    double response = this->wam->moveTrajectoryLearntAndEstimateForce(model_filename, points_filename);
    force_request_->success_response(response);

    return;
}

void
WamDriver::start_dmp_tracker(const std::vector<double> * initial, const std::vector<double> * goal)
{
    uint16_t errormask = 0x00;

    if (this->wam!=NULL) {
        if (! is_joints_move_request_valid(* initial)) {
            ROS_ERROR("Initial joints angles were not valid. Refuse to move.");
            return;
        }
        if (! is_joints_move_request_valid(* goal)) {
            ROS_ERROR("Goal joints angles were not valid. Refuse to move.");
            return;
        }
std::vector<double> nc_initial, nc_goal;
nc_initial= *initial;
nc_goal = *goal;
        ROS_DEBUG("Send values %f %f %f %f %f %f %f",nc_initial[0],nc_initial[0],nc_initial[0],nc_initial[0],nc_initial[0],nc_initial[0],nc_initial[0]);
        ROS_DEBUG("Send values %f %f %f %f %f %f %f",nc_goal[0],nc_goal[0],nc_goal[0],nc_goal[0],nc_goal[0],nc_goal[0],nc_goal[0]);
        this->wam->trackGoalDMP(&errormask, &nc_initial, &nc_goal);
       
        if(errormask > 0x00){
            string err_msg = wam->errorToString(errormask);
            ROS_ERROR("%s",err_msg.c_str());
            errormask = 0x00;
        }
   }
  /*
  uint16_t errormask = 0x00;
    std::vector<double> low_level_initial, low_level_goal;    
    for (std::vector<trajectory_msgs::JointTrajectoryPoint>::const_iterator it = initial.positions.begin();
         it != initial.positions.end(); it++) {
        low_level_initial.push_back(it->positions);
    }
    for (std::vector<trajectory_msgs::JointTrajectoryPoint>::const_iterator it = goal.positions.begin();
         it != goal.positions.end(); it++) {
        low_level_goal.push_back(it->positions);
    }
    */
 }
 
void 
WamDriver::dmp_tracker_new_goal(const std::vector<double> * new_goal)
{
    uint16_t errormask = 0x00;

    if (this->wam!=NULL) { 
        if (! is_joints_move_request_valid(* new_goal)) {
            ROS_ERROR("Initial joints angles were not valid. Refuse to move.");
            return;
        }
        
	std::vector<double> nc_new_goal;
	nc_new_goal = *new_goal;

	this->wam->trackGoalDMPNewGoal(&errormask, &nc_new_goal);
	
	if(errormask > 0x00){
            string err_msg = wam->errorToString(errormask);
            ROS_ERROR("%s",err_msg.c_str());
            errormask = 0x00;
        }
    }
 }


