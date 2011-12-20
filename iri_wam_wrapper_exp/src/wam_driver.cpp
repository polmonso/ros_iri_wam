#include "wam_driver.h"

using namespace std;
using namespace XmlRpc;

WamDriver::WamDriver()
{
//setDriverId(driver string id);
// this->wamserver_ip = "192.168.100.49";
//  this->wamserver_ip = "192.168.100.50";
// this->server_port = 4321;
//  this->state_refresh_rate = 100;

 ros::NodeHandle nh;
 
 nh.getParam("wam_ip",this->wamserver_ip);
 nh.param<int>("port", this->server_port, 4321);
 nh.param<int>("refresh_rate", this->state_refresh_rate, 100);


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

void WamDriver::move_in_joints(std::vector<double> *angles){
    uint16_t errormask = 0x00;

    if(this->wam!=NULL){
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
