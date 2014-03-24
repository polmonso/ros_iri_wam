#include "wam_controller_driver.h"
#include "wam_exceptions.h"

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
            wam_->activate();
            this->state_ = OPENED;
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
  ROS_INFO("[APP] Wait until wam gets home, idle the wam and press enter.");
  wam_->deactivate();
  //getchar();
  this->state_ = CLOSED;
  return true;
}

bool WamControllerDriver::startDriver(void)
{
    try{
        wam_->turn_on_gravity_compensation();
        ROS_INFO("All is ready to work now!");
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
  wam_->move_to_home();
  wam_->wait_until_motion_done();
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

bool WamControllerDriver::is_joint_vector_valid(const std::vector<double> & angles)
{
    // Check number of joints sent to the robot
    if (static_cast<unsigned int>(angles.size()) != get_num_joints())
    {
        ROS_ERROR("Invalid request to move. Joint vector size is %d while the robot has %u joints", (int)angles.size(), (unsigned int)get_num_joints());
        return false;
    }
    // Check valid values of those angles
    for (std::vector<double>::const_iterator it = angles.begin(); it != angles.end(); it++)
    {
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
  if(this->wam_!=NULL)  this->wam_->get_cartesian_pose(pose);
}

void 
WamControllerDriver::get_joint_angles(std::vector<double> *angles) 
{
    if(this->wam_!=NULL) this->wam_->get_joint_angles(angles);
}

std::string 
WamControllerDriver::get_robot_name() 
{
  return this->robot_name_;
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
trajectory_msgs::JointTrajectoryPoint WamControllerDriver::get_desired_joint_trajectory_point()
{
  return desired_joint_trajectory_point_;
}

bool WamControllerDriver::move_trajectory_in_joints(const trajectory_msgs::JointTrajectory &trajectory, const bool &blocking, const bool &compliant)
{
    if (this->wam_==NULL)
    {
        throw CWamException(_HERE_, "Low level robot driver not defined");
        return false;
    } else if (trajectory.points.size() < 2) {
        throw CWamException(_HERE_, "Trajectory must contain at least 2 points");
        return false;
    }

    WAMJointTrajectory low_level_trajectory;
    WAMTrajectoryPoint point_trajectory;

    for (std::vector<trajectory_msgs::JointTrajectoryPoint>::const_iterator it = trajectory.points.begin(); it != trajectory.points.end(); it++)
    {
        if (!is_joint_vector_valid(it->positions))
        {
            throw CWamException(_HERE_, "Joint angles are not valid. Refuse to move.");
            return false;
        } else if (!is_joint_vector_valid(it->velocities)) {
            throw CWamException(_HERE_, "Joint velocities are not valid. Refuse to move.");
            return false;
        } else if (!is_joint_vector_valid(it->accelerations)) {
            throw CWamException(_HERE_, "Joint accelerations are not valid. Refuse to move.");
            return false;
        }

        // Now low_level_trajectory contains angles, velocities, accelerations and time_from_start
        point_trajectory.positions.clear();
        point_trajectory.velocities.clear();
        point_trajectory.accelerations.clear();
        point_trajectory.time_from_start = 0.0f;
        point_trajectory.positions.insert(point_trajectory.positions.end(), it->positions.begin(), it->positions.end());
        point_trajectory.velocities.insert(point_trajectory.velocities.end(), it->velocities.begin(), it->velocities.end());
        point_trajectory.accelerations.insert(point_trajectory.accelerations.end(), it->accelerations.begin(), it->accelerations.end());
        point_trajectory.time_from_start = it->time_from_start.toSec();
        low_level_trajectory.push_back(point_trajectory);
        ROS_DEBUG("New point: %f %f %f %f %f %f %f",point_trajectory.positions[0],
                point_trajectory.positions[1],
                point_trajectory.positions[2],
                point_trajectory.positions[3],
                point_trajectory.positions[4],
                point_trajectory.positions[5],
                point_trajectory.positions[6]);
    }

    try {
        this->wam_->move_trajectory_in_joints(low_level_trajectory, blocking, compliant);
    } catch(CWamException &e) {
        ROS_INFO("%s", e.what().c_str());
        return false;
    }
    return true;
}

void
WamControllerDriver::stop_trajectory_in_joints(){
  ROS_INFO("Cancel Trajectory");
  this->wam_->stop();
}

bool WamControllerDriver::move_in_joints(std::vector<double> *angles, const double &vel, const double &accel)
{
    if (this->wam_==NULL)
    {
        ROS_ERROR("Low level robot driver not defined");
        return false;
    }

    if (!this->is_joint_vector_valid(*angles))
    {
        ROS_ERROR("Joints angles are not valid. Refuse to move.");
        return false;
    }

    bool blocking(false); // Forcing NON blocking behaviour 
    try {
        this->wam_->move_in_joints(*angles, blocking, vel, accel);
    } catch(CWamException &e) {
        ROS_INFO("%s", e.what().c_str());
        return false;
    }

    return true;
}

void
WamControllerDriver::wait_move_end()
{
    if(this->wam_!=NULL)
    {
        this->wam_->wait_until_motion_done();
    }
}

void
WamControllerDriver::hold_on()
{
    if(this->wam_!=NULL)
    {
        this->wam_->hold_on_current_position();
    }
}

void
WamControllerDriver::hold_off()
{
    if(this->wam_!=NULL)
    {
        this->wam_->hold_off_current_position();
    }
}

void
WamControllerDriver::start_dmp_tracker(const std::vector<double> * initial, const std::vector<double> * goal)
{
    if (this->wam_!=NULL)
    {
        if (! is_joint_vector_valid(*initial))
        {
            ROS_ERROR("Initial joints angles were not valid. Refuse to move.");
            return;
        }
        if (! is_joint_vector_valid(*goal))
        {
            ROS_ERROR("Goal joints angles were not valid. Refuse to move.");
            return;
        }
        ROS_DEBUG("Initial joint values: %f %f %f %f %f %f %f", initial->at(0), initial->at(1), initial->at(2), initial->at(3), initial->at(4), initial->at(5), initial->at(6));
        ROS_DEBUG("Goal joint values: %f %f %f %f %f %f %f", goal->at(0), goal->at(1), goal->at(2), goal->at(3), goal->at(4), goal->at(5), goal->at(6));

        this->wam_->track_goal_dmp(initial, goal);
    }
}


void
WamControllerDriver::dmp_tracker_new_goal(const std::vector<double> * new_goal)
{
    if (this->wam_!=NULL)
    { 
        if (! is_joint_vector_valid(*new_goal))
        {
            ROS_ERROR("New goal joints angles were not valid. Refuse to move.");
            return;
        }
        ROS_DEBUG("New goal joint values: %f %f %f %f %f %f %f", new_goal->at(0), new_goal->at(1), new_goal->at(2), new_goal->at(3), new_goal->at(4), new_goal->at(5), new_goal->at(6));

        this->wam_->track_goal_dmp_new_goal(new_goal);
    }
}

