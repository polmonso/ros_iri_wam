#include "wam_ik_kdl.h"
using namespace Eigen;
using namespace std;

WamIkKdl::WamIkKdl() {
  //init class attributes if necessary
  //this->loop_rate = 2;//in [Hz]

  this->wam63_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0.0,0.0,0.0,0.0) ));
  this->wam63_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY), KDL::Frame::DH(0.0,0.0,0.0,0.0) ));
  this->wam63_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0.045,0.0,0.5500,0.0) ));
  this->wam63_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY), KDL::Frame::DH(-0.045,0.0,0.0,0.0) ));
  this->wam63_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0.0,0.0,0.300,0.0) ));
  this->wam63_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY), KDL::Frame::DH(0.0,0.0,0.0,0.0) ));
  this->wam63_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0.0,0.0,0.0609,0.0) ));

  this->num_joints_ = this->wam63_.getNrOfJoints();

  this->currentjoints.resize(this->num_joints_);

  //string for port names
  std::string port_name;

  // [init publishers]
  
  // [init subscribers]
  port_name = ros::names::append(ros::this_node::getName(), "joint_states"); 
  this->joint_states_subscriber = this->nh_.subscribe(port_name, 5, &WamIkKdl::joint_states_callback, this);
  
  // [init services]
  port_name = ros::names::append(ros::this_node::getName(), "move_in_xyzquat"); 
  this->move_in_xyzquat_server = this->nh_.advertiseService(port_name, &WamIkKdl::move_in_xyzquat_Callback, this);

  port_name = ros::names::append(ros::this_node::getName(), "print_ik_xyzquat"); 
  this->print_ik_xyzquat_server = this->nh_.advertiseService(port_name, &WamIkKdl::print_ik_xyzquat_Callback, this);
  
  // [init clients]
  port_name = ros::names::append(ros::this_node::getName(), "move_in_joints"); 
  move_in_joints_client = this->nh_.serviceClient<iri_wam_common_msgs::joints_move>(port_name);
  
  // [init action servers]
  
  // [init action clients]
}

/*  [subscriber callbacks] */
void WamIkKdl::joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg) { 

  for(unsigned int i=0;i<this->num_joints_;i++)
    currentjoints[i] = msg->position[i]; 

}

/*  [service callbacks] */
bool WamIkKdl::move_in_xyzquat_Callback(iri_wam_common_msgs::pose_move::Request &req, iri_wam_common_msgs::pose_move::Response &res) 
{ 

  bool result;
  Quaternion<float> quat( req.pose.orientation.w, req.pose.orientation.x, req.pose.orientation.y, req.pose.orientation.z);
  Matrix3f mat = quat.toRotationMatrix();
      ROS_INFO("Received Quat: %f %f %f %f %f %f %f\n",
                req.pose.position.x, req.pose.position.y, req.pose.position.z, 
                req.pose.orientation.x, req.pose.orientation.y, req.pose.orientation.z, req.pose.orientation.w);

  std::vector <double> pose(16,0);
  std::vector <double> joints(7,0);

  pose[3] = req.pose.position.x;
  pose[7] = req.pose.position.y;
  pose[11] = req.pose.position.z;
  pose[15] = 1;
  for(unsigned int i=0; i<12; i++){
   if(i%4 != 3){
     pose[i] = mat(i/4,i%4);
   }
  }
  ROS_INFO("move_in_xyzquat Service Received Pose:\n %f %f %f %f\n %f %f %f %f\n %f %f %f %f\n %f %f %f %f\n",
        pose[0],pose[1],pose[2],pose[3],
        pose[4],pose[5],pose[6],pose[7],
        pose[8],pose[9],pose[10],pose[11],
        pose[12],pose[13],pose[14],pose[15]);

  if(!WamIkKdl::ik(pose, currentjoints, joints)){
      ROS_ERROR("IK solution not found");
      result = false;
  } else {

    ROS_INFO("move_in_xyzquat Service computed joints:\n %f %f %f %f %f %f %f\n",
	     joints.at(0), joints.at(1), joints.at(2), joints.at(3), joints.at(4), joints.at(5), joints.at(6));
    
    move_in_joints_srv.request.joints.resize(7);
    for(unsigned int ii=0; ii < this->num_joints_; ii++)
      move_in_joints_srv.request.joints[ii] = joints.at(ii);
    
    if (this->move_in_joints_client.call(move_in_joints_srv)) { 
      ROS_INFO(" %d\n",move_in_joints_srv.response.success); 
      result = true;
    } else { 
      ROS_ERROR("Failed to call service move_in_joints"); 
      result = false;
    }
  }
  res.success = result;
  return result;
}

bool WamIkKdl::print_ik_xyzquat_Callback(iri_wam_common_msgs::wamInverseKinematics::Request &req, iri_wam_common_msgs::wamInverseKinematics::Response &res){

  bool result;
  Quaternion<float> quat(req.pose.orientation.x, req.pose.orientation.y, req.pose.orientation.z, req.pose.orientation.w);
  Matrix3f mat = quat.toRotationMatrix();

  std::vector <double> pose(16,0);
  std::vector <double> joints(this->num_joints_,0);

  pose[3] = req.pose.position.x;
  pose[7] = req.pose.position.y;
  pose[11] = req.pose.position.z;
  pose[15] = 1;
  
  for(unsigned int ii=0; ii < 12; ii++){
    if(ii%4 != 3){
      pose[ii] = mat(ii/4,ii%4);
    }
  }
  
  ROS_INFO("print_ik_xyzquat Service Received Pose:\n %f %f %f %f\n %f %f %f %f\n %f %f %f %f\n %f %f %f %f\n",
	   pose[0],pose[1],pose[2],pose[3],
	   pose[4],pose[5],pose[6],pose[7],
	   pose[8],pose[9],pose[10],pose[11],
	   pose[12],pose[13],pose[14],pose[15]);
  
  if(!WamIkKdl::ik(pose, currentjoints, joints)){
  
    ROS_ERROR("IK solution not found");
    result = false;
    
  }else{
    
    ROS_INFO("print_ik_xyzquat Service computed joints:\n %f %f %f %f %f %f %f\n",
	     joints.at(0), joints.at(1), joints.at(2), joints.at(3), joints.at(4), joints.at(5), joints.at(6));
    res.joints.position.resize(this->num_joints_);
    for(unsigned int ii=0; ii < this->num_joints_; ii++)
      res.joints.position[ii] = joints.at(ii);
    result = true;
    
  }
  return result;
}

/*  [action callbacks] */

/*  [action requests] */

bool WamIkKdl::ik(vector<double> goal_in_cartesian, vector<double> currentjoints, vector<double>& goal_in_joints){

  //fk solver
  KDL::ChainFkSolverPos_recursive fksolver(KDL::ChainFkSolverPos_recursive(this->wam63_));

  // Create joint array
  KDL::JntArray setpointJP = KDL::JntArray(this->num_joints_);
  KDL::JntArray max = KDL::JntArray(this->num_joints_); //The maximum joint positions
  KDL::JntArray min = KDL::JntArray(this->num_joints_); //The minimium joint positions

  double minjp[7] = {-2.6,-2.0,-2.8,-0.9,-4.76,-1.6,-3.0};
  double maxjp[7] = { 2.6, 2.0, 2.8, 3.2, 1.24, 1.6, 3.0};

  for(unsigned int ii=0; ii < this->num_joints_; ii++){
    max(ii) = maxjp[ii];
    min(ii) = minjp[ii];
  }

  //Create inverse velocity solver
  KDL::ChainIkSolverVel_pinv_givens iksolverv = KDL::ChainIkSolverVel_pinv_givens(this->wam63_);
  
  //Iksolver Position: Maximum 100 iterations, stop at accuracy 1e-6
  //ChainIkSolverPos_NR iksolver = ChainIkSolverPos_NR(wam63,fksolver,iksolverv,100,1e-6);
  KDL::ChainIkSolverPos_NR_JL iksolver = KDL::ChainIkSolverPos_NR_JL(this->wam63_, min, max, fksolver, iksolverv, 2000, 1e-6); //With Joints Limits

  KDL::Frame cartpos;
  KDL::JntArray jointpos = KDL::JntArray(this->num_joints_);
  KDL::JntArray currentJP = KDL::JntArray(this->num_joints_);
  
  // Copying position to KDL frame
  cartpos.p(0) = goal_in_cartesian.at(3);
  cartpos.p(1) = goal_in_cartesian.at(7);
  cartpos.p(2) = goal_in_cartesian.at(11);

  // Copying Rotation to KDL frame
  cartpos.M(0,0) = goal_in_cartesian.at(0);
  cartpos.M(0,1) = goal_in_cartesian.at(1);
  cartpos.M(0,2) = goal_in_cartesian.at(2);
  cartpos.M(1,0) = goal_in_cartesian.at(4);
  cartpos.M(1,1) = goal_in_cartesian.at(5);
  cartpos.M(1,2) = goal_in_cartesian.at(6);
  cartpos.M(2,0) = goal_in_cartesian.at(8);
  cartpos.M(2,1) = goal_in_cartesian.at(9);
  cartpos.M(2,2) = goal_in_cartesian.at(10);

  for(unsigned int ii=0; ii < this->num_joints_; ii++)
    currentJP(ii) = currentjoints.at(ii);

  // Calculate inverse kinematics to go from currentJP to cartpos. The result in jointpos
  int ret = iksolver.CartToJnt(currentJP, cartpos, jointpos);

  if (ret >= 0) {

    std::cout << " Current Joint Position: [";
    for(unsigned int ii=0; ii < this->num_joints_; ii++)
      std::cout << currentJP(ii) << " ";
    std::cout << "]"<< std::endl;

    std::cout << "Cartesian Position " << cartpos << std::endl;

    std::cout << "IK result Joint Position: [";
    for(unsigned int ii=0; ii < this->num_joints_; ii++)
      std::cout << jointpos(ii) << " ";
    std::cout << "]"<< std::endl;

    goal_in_joints.clear();
    goal_in_joints.resize(this->num_joints_);
    for(unsigned int ii=0; ii < this->num_joints_; ii++)
      goal_in_joints[ii] = jointpos(ii);

  } else {

    std::cout << "Error: could not calculate inverse kinematics :(" << std::endl;
    return false;

  }

  return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "wam_ik_kdl");
    WamIkKdl wam_ik_kdl;
    ros::spin();
}
