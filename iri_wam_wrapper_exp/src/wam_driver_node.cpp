#include "wam_driver_node.h"

using namespace Eigen;

WamDriverNode::WamDriverNode(ros::NodeHandle &nh) :
 iri_base_driver::IriBaseNodeDriver<WamDriver>(nh),
 action_server_(nh,"joint_trajectory_action",false)
{
  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]

    //ask wam for the numaxes
    this->JointState_msg.name.resize(7);
    this->JointState_msg.position.resize(7); 

  // [init publishers]
  this->pose_publisher = this->public_node_handle_.advertise<geometry_msgs::PoseStamped>("pose", 5);
  this->joint_states_publisher = this->public_node_handle_.advertise<sensor_msgs::JointState>("joint_states", 5);
  
  // [init subscribers]
  
  // [init services]
  this->wam_services_server_ = this->public_node_handle_.advertiseService("wam_services", &WamDriverNode::wam_servicesCallback, this);
  this->pose_move_server = this->public_node_handle_.advertiseService("pose_move", &WamDriverNode::pose_moveCallback, this);
  this->joints_move_server = this->public_node_handle_.advertiseService("joints_move", &WamDriverNode::joints_moveCallback, this);

  ROS_INFO("Wam node started"); 
  
  // [init clients]
  
  // [init action servers]
  action_server_.registerGoalCallback(boost::bind(&WamDriverNode::goalCB, this, _1));
  //action_server_.registerCancelCallback(boost::bind(&WamDriverNode::cancelCB, this, _1));
  action_server_.start();
   // action_server_.reset(new ActionExecutor(nh, "joint_trajectory_action",boost::bind(&WamDriverNode::goalCB, this, _1), boost::bind(&WamDriverNode::cancelCB, this, _1)));
  // [init action clients]
}

void WamDriverNode::mainNodeThread(void)
{
  //lock access to driver if necessary
  //this->driver_.lock();

  std::vector<double> angles(7,0.1);
  std::vector<double> pose(16,0);
  char jname[5];
  Matrix3f rmat;

  // [fill msg Header if necessary]
  this->PoseStamped_msg.header.stamp = ros::Time::now();
  this->PoseStamped_msg.header.frame_id = "wam0";

  // [fill msg structures]
  this->driver_.lock();
  this->driver_.get_pose(&pose);
  this->driver_.get_joint_angles(&angles);
  this->driver_.unlock();

  rmat << pose.at(0), pose.at(1),pose.at(2),pose.at(4),pose.at(5),pose.at(6),pose.at(8),pose.at(9),pose.at(10);

  {
    Quaternion<float> quat(rmat);

    this->PoseStamped_msg.pose.position.x = pose.at(3);
    this->PoseStamped_msg.pose.position.y = pose.at(7);
    this->PoseStamped_msg.pose.position.z = pose.at(11);
    this->PoseStamped_msg.pose.orientation.x = quat.x();
    this->PoseStamped_msg.pose.orientation.y = quat.y();
    this->PoseStamped_msg.pose.orientation.z = quat.z();
    this->PoseStamped_msg.pose.orientation.w = quat.w();
  }


  JointState_msg.header.stamp = ros::Time::now();
  for(int i=0;i<(int)angles.size();i++){
      snprintf(jname, 5, "wam%d", i);
      JointState_msg.name[i] = jname;
      JointState_msg.position[i] = angles[i];
  }

  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]
  this->joint_states_publisher.publish(this->JointState_msg);
  this->pose_publisher.publish(this->PoseStamped_msg);

  //unlock access to driver if previously blocked
  //this->driver_.unlock();
}

/*  [subscriber callbacks] */

/*  [service callbacks] */
bool WamDriverNode::wam_servicesCallback(iri_wam_common_msgs::wamdriver::Request &req, iri_wam_common_msgs::wamdriver::Response &res) 
{ 
  //lock access to driver if necessary 
  this->driver_.lock(); 

  if(this->driver_.isRunning()) 
  { 
    switch(req.call){
      case HOLDON:
        this->driver_.hold_current_position(true);
        break;
      case HOLDOFF:
        this->driver_.hold_current_position(false);
        break;
      default:
          ROS_ERROR("Invalid action id. Check wam_actions_node.h");
        break;
    }
    //do operations with req and output on res 
    //res.data2 = req.data1 + my_var; 
  } else { 
    std::cout << "ERROR: Driver is not on run mode yet." << std::endl; 
  } 

  //unlock driver if previously blocked 
  this->driver_.unlock(); 

  return true; 
}
bool WamDriverNode::joints_moveCallback(iri_wam_common_msgs::joints_move::Request &req, iri_wam_common_msgs::joints_move::Response &res) 
{ 
  //lock access to driver if necessary 
  bool result;
  this->driver_.lock();

  if(this->driver_.isRunning()){
    //do operations with req and output on res 
    //res.data2 = req.data1 + my_var; 
    std::vector <double> joints(7,0);
    for(int i=0; i< (int) req.joints.size();i++)
       joints[i] = req.joints[i];

    this->driver_.move_in_joints(&joints); //this call blocks if the wam faults. The mutex is not freed...!
    //unlock driver if previously blocked 
    this->driver_.unlock();
    this->driver_.wait_move_end();
    result = true;

  }else{
    ROS_ERROR("Driver is not running");
    result = false;
    //unlock driver if previously blocked 
    this->driver_.unlock();
  }
  return result;
}

bool WamDriverNode::pose_moveCallback(iri_wam_common_msgs::pose_move::Request &req, iri_wam_common_msgs::pose_move::Response &res) 
{ 
  bool result;
  //lock access to driver if necessary 
  this->driver_.lock();
  if(this->driver_.isRunning()){

     //do operations with req and output on res 
     //res.data2 = req.data1 + my_var; 
    Quaternion<float> quat(req.pose.orientation.w, req.pose.orientation.x, req.pose.orientation.y, req.pose.orientation.z);
    Matrix3f mat = quat.toRotationMatrix();

     std::vector <double> pose(16,0);
      ROS_INFO("Received Quat: %f %f %f %f %f %f %f\n",
                req.pose.position.x, req.pose.position.y, req.pose.position.z,
                req.pose.orientation.x, req.pose.orientation.y, req.pose.orientation.z, req.pose.orientation.w);
     pose[3] = req.pose.position.x;
     pose[7] = req.pose.position.y;
     pose[11] = req.pose.position.z;
     pose[15] = 1;
     for(int i=0; i<12; i++){
      if(i%4 != 3)
        pose[i] = mat(i/4,i%4);
     }
      ROS_INFO("Received Pose:\n %f %f %f %f \n %f %f %f %f \n %f %f %f %f\n %f %f %f %f\n",
            pose[0],pose[1],pose[2],pose[3],
            pose[4],pose[5],pose[6],pose[7],
            pose[8],pose[9],pose[10],pose[11],
            pose[12],pose[13],pose[14],pose[15]);

     this->driver_.move_in_cartesian(&pose);
     this->driver_.unlock();
     //do we want a blocking service?
     this->driver_.wait_move_end();
      result = true;

  }else{
    ROS_ERROR("Driver is not running at the moment");
    result = false;
    this->driver_.unlock();
  }

  res.success = result;
  return result;
}

/*  [action callbacks] */
void WamDriverNode::goalCB(GoalHandle gh)
{	gh.setAccepted();
	  this->driver_.lock();

	trajectory_msgs::JointTrajectory traj=gh.getGoal()->trajectory;
	for(unsigned int i=0; i < traj.points.size(); ++i)
	{
      if(this->driver_.isRunning())
      {
       this->driver_.move_in_joints(&traj.points[i].positions); //this call blocks if the wam faults. The mutex is not freed...!
       ros::Duration(5.0).sleep();
      }
      else
      {
		  ROS_FATAL("Driver is not running");
	  }
    }
    this->driver_.unlock();
    this->driver_.wait_move_end();
}
 


/*  [action requests] */

void WamDriverNode::postNodeOpenHook(void)
{
}

void WamDriverNode::addNodeDiagnostics(void)
{
}

void WamDriverNode::addNodeOpenedTests(void)
{
}

void WamDriverNode::addNodeStoppedTests(void)
{
}

void WamDriverNode::addNodeRunningTests(void)
{
}

void WamDriverNode::reconfigureNodeHook(int level)
{
}

WamDriverNode::~WamDriverNode()
{
  //[free dynamic memory]
}

/* main function */
int main(int argc,char *argv[])
{
  return driver_base::main<WamDriverNode>(argc,argv,"wam_driver_node");
}
