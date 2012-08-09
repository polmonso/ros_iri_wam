#include "wam_driver.h"
#include "wam_packet.h" // from wam low level driver

using namespace std;
using namespace XmlRpc;

WamDriver::WamDriver()
{
 ros::NodeHandle nh("~");

 XmlRpc::XmlRpcValue ip;
 XmlRpc::XmlRpcValue port;
 XmlRpc::XmlRpcValue rate;

 nh.getParam("wam_ip",ip);
 this->wamserver_ip=(std::string)ip;
 
 if(!nh.hasParam("port"))
 {
  ROS_WARN_STREAM("Port not defined, Defaults: "<<4321);
  this->server_port=4321;
 } 
 else
 {
  nh.getParam("port",port);
  this->server_port=(int)port;
 }
 if(!nh.hasParam("refresh_rate"))
 {
  ROS_WARN_STREAM("Refresh Rate not defined, Defaults: "<<100);
  this->state_refresh_rate=100;
 } 
 else
 {
 nh.getParam("refresh_rate",rate);
 this->state_refresh_rate=(int)rate;
 }
 
 ROS_INFO_STREAM("IP Address: "<<this->wamserver_ip);
 ROS_INFO_STREAM("Port Server: "<<this->server_port);
 ROS_INFO_STREAM("Refresh Rate: "<<this->state_refresh_rate);
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
    //gravity and go to position?
    wam->setGravityCompensation(1.1);
    ROS_INFO("Switching to Running and going to default position.");
    wam->goToDefaultPosition();
    ROS_INFO("Waiting final position reach");
    wam->waitTillMotionDone();
    ROS_INFO("Done!");
//fix waitTillMotionDone
    sleep(1);
    ROS_INFO("arbitrary wait Done!");
    wam->goToDefaultPosition();
    ROS_INFO("Waiting final position reach");
    wam->waitTillMotionDone();
    ROS_INFO("Done!");
//fix waitTillMotionDone
    sleep(1);
    ROS_INFO("arbitrary wait Done!");
    //something i wanna try
    wam->holdCurrentPosition(false);
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
}

int WamDriver::get_num_joints(){

  if(this->wam!=NULL){
    return this->wam->getNumAngles();
  } 
  return 0;
}

bool
WamDriver::is_moving()
{
    if (this->wam != NULL)
        return this->wam->isMoving();

    return false;
}

void WamDriver::wait_move_end(){

  if(this->wam!=NULL){
    this->wam->waitTillMotionDone();
  } 
}

void WamDriver::get_pose(std::vector<double> *pose){

  if(this->wam!=NULL){
    this->wam->getCartesianPose(pose);
//        pose->clear();
//        pose->push_back(0);
//        pose->push_back(1);
//        pose->push_back(0);
//        pose->push_back(0.3);
//        pose->push_back(1);
//        pose->push_back(0);
//        pose->push_back(0);
//        pose->push_back(0.3);
//        pose->push_back(0);
//        pose->push_back(0);
//        pose->push_back(1);
//        pose->push_back(0.5);
  } 
}
void WamDriver::get_joint_angles(std::vector<double> *angles){

  if(this->wam!=NULL){
    this->wam->getJointAngles(angles);
//    angles->clear();
//    for(int i=0;i<get_num_joints();i++){
//        angles->push_back(i*0.001);
//    }
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

void WamDriver::move_in_joints(std::vector<double> *angles){
    uint16_t errormask = 0x00;

    if (this->wam!=NULL) {
        if (! is_joints_move_request_valid(* angles)) {
            ROS_ERROR("Joints angles were not valid. Refuse to move.");
            return;
        }

        wam->moveInJoints(&errormask, angles);
        if(errormask > 0x00){
            string err_msg = wam->errorToString(errormask);
            ROS_ERROR("%s",err_msg.c_str());
            errormask = 0x00;
        }
    }
}

void WamDriver::move_in_cartesian(std::vector<double> *pose, double vel, double acc){
  if(this->wam!=NULL){
    this->wam->moveInCartesian(pose, vel, acc);
  } 
}   

void WamDriver::hold_current_position(bool on){
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
    force_request.init();
    // TODO: implement the returning value from client-server
    // TODO: implement error handling
    this->wam->moveTrajectoryLearntAndEstimateForce(model_filename, points_filename);
    force_request.success_response(69); 

    return;
}



