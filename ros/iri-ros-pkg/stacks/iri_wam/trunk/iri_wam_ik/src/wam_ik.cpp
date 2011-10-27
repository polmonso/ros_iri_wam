#include "wam_ik.h"
using namespace Eigen;
using namespace std;

WamIK::WamIK() {
  //init class attributes if necessary
  //this->loop_rate = 2;//in [Hz]

  this->currentjoints.resize(7);

  //string for port names
  std::string port_name;

  // [init publishers]
  
  // [init subscribers]
  port_name = ros::names::append(ros::this_node::getName(), "joint_states"); 
  this->joint_states_subscriber = this->nh_.subscribe(port_name, 5, &WamIK::joint_states_callback, this);
  
  // [init services]
  port_name = ros::names::append(ros::this_node::getName(), "pose_move"); 
  this->pose_move_server = this->nh_.advertiseService(port_name, &WamIK::pose_moveCallback, this);
  port_name = ros::names::append(ros::this_node::getName(), "wamik"); 
  this->wamik_server = this->nh_.advertiseService(port_name, &WamIK::wamikCallback, this);
  
  // [init clients]
  port_name = ros::names::append(ros::this_node::getName(), "joint_move"); 
  joint_move_client = this->nh_.serviceClient<iri_wam_common_msgs::joints_move>(port_name);
  
  // [init action servers]
  
  // [init action clients]
}

/*  [subscriber callbacks] */
void WamIK::joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg) { 

  for(int i=0;i<7;i++)
    currentjoints[i] = msg->position[i]; 

}

/*  [service callbacks] */
bool WamIK::pose_moveCallback(iri_wam_common_msgs::pose_move::Request &req, iri_wam_common_msgs::pose_move::Response &res) 
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
  for(int i=0; i<12; i++){
   if(i%4 != 3){
     pose[i] = mat(i/4,i%4);
   }
  }
  ROS_INFO("wamik Service Received Pose:\n %f %f %f %f\n %f %f %f %f\n %f %f %f %f\n %f %f %f %f\n",
        pose[0],pose[1],pose[2],pose[3],
        pose[4],pose[5],pose[6],pose[7],
        pose[8],pose[9],pose[10],pose[11],
        pose[12],pose[13],pose[14],pose[15]);

  if(!WamIK::ik(pose, currentjoints, joints)){
      ROS_ERROR("IK solution not found. Requested pose:\n %f %f %f %f\n %f %f %f %f\n %f %f %f %f\n %f %f %f %f\n",
        pose[0],pose[1],pose[2],pose[3],
        pose[4],pose[5],pose[6],pose[7],
        pose[8],pose[9],pose[10],pose[11],
        pose[12],pose[13],pose[14],pose[15]);
      result = false;
  }else{

      ROS_INFO("wamik Service computed joints:\n %f %f %f %f %f %f %f\n", joints.at(0), joints.at(1), joints.at(2), joints.at(3), joints.at(4), joints.at(5), joints.at(6));
    
      joint_move_srv.request.joints.resize(7);
      for(int i=0;i<7;i++)
        joint_move_srv.request.joints[i] = joints.at(i);
    
      if (this->joint_move_client.call(joint_move_srv)) 
      { 
        ROS_INFO(" %d\n",joint_move_srv.response.success); 
        result = true;
      } 
      else 
      { 
        ROS_ERROR("Failed to call service joint_move"); 
        result = false;
      }
  }
  res.success = result;
  return result;
}

bool WamIK::wamikCallback(iri_wam_common_msgs::wamInverseKinematics::Request &req, iri_wam_common_msgs::wamInverseKinematics::Response &res){
  bool result;
  Quaternion<float> quat(req.pose.orientation.x, req.pose.orientation.y, req.pose.orientation.z, req.pose.orientation.w);
  Matrix3f mat = quat.toRotationMatrix();

  std::vector <double> pose(16,0);
  std::vector <double> joints(7,0);

  pose[3] = req.pose.position.x;
  pose[7] = req.pose.position.y;
  pose[11] = req.pose.position.z;
  pose[15] = 1;
  for(int i=0; i<12; i++){
   if(i%4 != 3){
     pose[i] = mat(i/4,i%4);
   }
  }
  ROS_INFO("wamik Service Received Pose:\n %f %f %f %f\n %f %f %f %f\n %f %f %f %f\n %f %f %f %f\n",
        pose[0],pose[1],pose[2],pose[3],
        pose[4],pose[5],pose[6],pose[7],
        pose[8],pose[9],pose[10],pose[11],
        pose[12],pose[13],pose[14],pose[15]);

  if(!WamIK::ik(pose, currentjoints, joints)){
      ROS_ERROR("IK solution not found");
      result = false;
  }else{

      ROS_INFO("wamik Service computed joints:\n %f %f %f %f %f %f %f\n", joints.at(0), joints.at(1), joints.at(2), joints.at(3), joints.at(4), joints.at(5), joints.at(6));
    
      res.joints.position.resize(7);
      for(int i=0;i<7;i++)
        res.joints.position[i] = joints.at(i);
    
  }
  return result;
}

/*  [action callbacks] */

/*  [action requests] */

bool WamIK::ik(vector<double> pose, vector<double> currentjoints, vector<double>& joints){
    //ik solver

  double posec[16];
  double nextangles[7], currentangles[7] = {0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.0};
  for(int i=0;i<16;i++)
     posec[i] = pose.at(i); 

  for(int i=0;i<7;i++)
     currentangles[i] = currentjoints.at(i);

  if(!wam_ik(posec, currentangles, nextangles))
      return false;

  joints.clear();
  joints.resize(7);
  for(int i=0;i<7;i++)
     joints[i] = nextangles[i];

  return true;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "wam_ik");
    WamIK wamik;
    ros::spin();
}
