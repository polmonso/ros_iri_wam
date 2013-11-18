#include "wam_controller_driver.h"

WamControllerDriver::WamControllerDriver(void)
{
  //get the robot name to name the links accordingly
    ros::NodeHandle nh("~");

    XmlRpc::XmlRpcValue r_name;
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
}

//TODO: fer que no calgui esperar els getchar
bool WamControllerDriver::openDriver(void)
{
    try{
        if(this->state_ != OPENED){
            this->wam_ = new wamDriver();
            wam_->open();
            this->state_ = OPENED;
            ROS_INFO("Wam opened, press shift+idle and enter.");
            getchar();

            wam_->create();
            ROS_INFO("Wam created, press shift+activate and press enter.");
            getchar();
            wam_->activate();
            return true;
        }else{
            ROS_ERROR("WAM was already opened!");
            return false;
        }
    }catch(const std::exception &e){
        //ROS_ERROR("%s",e.what().c_str());
        ROS_ERROR("Exception in openDriver");
        return false;
    }
}

bool WamControllerDriver::closeDriver(void)
{
  wam_->close();
  ROS_INFO("[APP] Wait until wam gets home, idle the wam and press enter.");
  getchar();
  this->state_ = CLOSED;
  return true;
}

bool WamControllerDriver::startDriver(void)
{
    try{
        wam_->setGravityCompensation(1.1);
        ROS_INFO("Switching to Running and going to default position.");
        wam_->goToDefaultPosition();
        ROS_INFO("Waiting final position reach");
        wam_->waitTillMotionDone();
        ROS_INFO("All is ready to work now!");
        wam_->setGravityCompensation(1.1);
        this->state_ = RUNNING;
        return true;
    }catch(const std::exception &e){
        //ROS_ERROR("%s",e.what().c_str());
        ROS_ERROR("Exception in startDriver");
        return false;
    }

  return true;
}

bool WamControllerDriver::stopDriver(void)
{
  wam_->home();
  wam_->waitTillMotionDone();
  this->state_ = OPENED;
  return true;
}

void WamControllerDriver::config_update(Config& new_cfg, uint32_t level)
{
  this->lock();
  
  // depending on current state
  // update driver with new_cfg data
  switch(this->getState())
  {
    case WamControllerDriver::CLOSED:
      break;

    case WamControllerDriver::OPENED:
      break;

    case WamControllerDriver::RUNNING:
      break;
  }

  // save the current configuration
  this->config_=new_cfg;

  this->unlock();
}

WamControllerDriver::~WamControllerDriver(void)
{
}

bool WamControllerDriver::is_joints_move_request_valid(const std::vector<double> & angles){
    // Check number of joints sent to the robot
    if (static_cast<unsigned int>(angles.size()) != get_num_joints()) {
        ROS_ERROR("Invalid request to move. Joint vector size is %d while the robot has %u joints",
                (int)angles.size(), (unsigned int)get_num_joints());
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

void 
WamControllerDriver::get_pose(std::vector<double> *pose) 
{
  if(this->wam_!=NULL)  this->wam_->getCartesianPose(pose);
}

void 
WamControllerDriver::get_joint_angles(std::vector<double> *angles) 
{
    if(this->wam_!=NULL) this->wam_->getJointAngles(angles);
}


std::string 
WamControllerDriver::get_robot_name() 
{
  return this->robot_name_;
}


//TODO:
bool
WamControllerDriver::is_moving_trajectory()
{
  if (this->wam_ != NULL) {
    return this->wam_->isMovingTrajectory();
  }
  return false;
}

//TODO
bool
WamControllerDriver::is_moving()
{
  if (this->wam_ != NULL) {
    return this->wam_->isMoving();
  }
  return false;
}

//TODO
bool
WamControllerDriver::is_joint_trajectory_result_succeeded()
{
   if (this->wam_ != NULL){
      /* Not yet implemented */
      //return this->wam_->is_joint_trajectory_result_succeeded();
      ROS_INFO("joint_trajectory_result_succeeded: TRUE hardcoded");
      return true; // Until it 
    }
    return false;

}

// TODO: get the current desired joint trajectory point from the low level driver
trajectory_msgs::JointTrajectoryPoint 
WamControllerDriver::get_desired_joint_trajectory_point()
{
  return desired_joint_trajectory_point_;
}

void 
WamControllerDriver::move_trajectory_in_joints(const trajectory_msgs::JointTrajectory & trajectory)
{
  uint16_t errormask = 0x00;
  //message with positions, no velocities are present
  if (trajectory.points.begin()->velocities.size()==0)
  {
    WAMPositionsJointTrajectory low_level_trajectory;
    std::vector<double> point_trajectory;


    for (std::vector<trajectory_msgs::JointTrajectoryPoint>::const_iterator it = trajectory.points.begin();
            it != trajectory.points.end(); it++)
    {
        if (! is_joints_move_request_valid(it->positions)) {
            ROS_ERROR("Joints angles were not valid. Refuse to move.");
            return;
        }
        point_trajectory.clear();
        point_trajectory.insert(point_trajectory.end(),it->positions.begin(), it->positions.end());
        low_level_trajectory.push_back(point_trajectory);
    }
   this->wam_->moveTrajectoryInJoints(&errormask, &low_level_trajectory);

  }
  //message with positions, velocities (and accelerations)
  else if (trajectory.points.begin()->velocities.size()==7) {
    WAMJointTrajectory low_level_trajectory;
    WAMTrajectoryPoint point_trajectory;

    for (std::vector<trajectory_msgs::JointTrajectoryPoint>::const_iterator 
         it = trajectory.points.begin(); it != trajectory.points.end(); it++) {
        if (! is_joints_move_request_valid(it->positions)) {
            ROS_ERROR("Joints angles were not valid. Refuse to move.");
            return;
        }
        // Now low_level_trajectory contains angles, velocities, accelerations and time_from_start
        point_trajectory.positions.clear();
        point_trajectory.velocities.clear();
        point_trajectory.accelerations.clear();
        point_trajectory.time_from_start.clear();
        point_trajectory.positions.insert(point_trajectory.positions.end(), 
                                          it->positions.begin(), 
                                          it->positions.end());
        point_trajectory.velocities.insert(point_trajectory.velocities.end(), 
                                           it->velocities.begin(), 
                                           it->velocities.end());
        point_trajectory.accelerations.insert(point_trajectory.accelerations.end(), 
                                              it->accelerations.begin(), 
                                              it->accelerations.end());
        point_trajectory.time_from_start.push_back(it->time_from_start.sec);
        point_trajectory.time_from_start.push_back(it->time_from_start.nsec);
        low_level_trajectory.push_back(point_trajectory);
        ROS_DEBUG("New point: %f %f %f %f %f %f %f",point_trajectory.positions[0],
                                                    point_trajectory.positions[1],
                                                    point_trajectory.positions[2],
                                                    point_trajectory.positions[3],
                                                    point_trajectory.positions[4],
                                                    point_trajectory.positions[5],
                                                    point_trajectory.positions[6]);
    }
    this->wam_->moveTrajectoryInJoints(&errormask, &low_level_trajectory);

  }
  else
    ROS_ERROR("Error in trajectory formatting: invalid velocity vector. Refuse to move.");
    
}

void
WamControllerDriver::stop_trajectory_in_joints(){
  ROS_INFO("Cancel Trajectory");
  this->wam_->stopTrajectoryInJoints();
}




