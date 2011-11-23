#include "wam_fk.h"
using namespace Eigen;

WamFK::WamFK() {
  //init class attributes if necessary
  //this->loop_rate = 2;//in [Hz]
  //TODO support for other configurations
  int numtransforms = 8;
  
  //wambase_ = ros::names::append(ros::this_node::getName(), "wambase"); 
  wambase_ = "wambase"; 
  wam0_ = ros::names::append(ros::this_node::getName(), "wam0"); 
  wam1_ = ros::names::append(ros::this_node::getName(), "wam1"); 
  wam2_ = ros::names::append(ros::this_node::getName(), "wam2"); 
  wam3_ = ros::names::append(ros::this_node::getName(), "wam3"); 
  wam4_ = ros::names::append(ros::this_node::getName(), "wam4"); 
  wam5_ = ros::names::append(ros::this_node::getName(), "wam5"); 
  wam6_ = ros::names::append(ros::this_node::getName(), "wam6"); 
  wam7_ = ros::names::append(ros::this_node::getName(), "wam7"); 
  

  //string for port names
  std::string port_name;

  // [init publishers]
  port_name = ros::names::append(ros::this_node::getName(), "tf"); 
  this->tf_publisher = this->nh_.advertise<tf::tfMessage>(port_name, 5);

  this->transforms.resize(numtransforms);
  this->tfMessage_msg.transforms.resize(numtransforms);
  
  // [init subscribers]
  port_name = ros::names::append(ros::this_node::getName(), "joint_states"); 
  this->joint_states_subscriber = this->nh_.subscribe(port_name, 5, &WamFK::joint_states_callback, this);
  
  // [init services]
  
  // [init clients]
  
  // [init action servers]
  
  // [init action clients]
    ROS_INFO("starting fknode");
}

void WamFK::tfPub(void)
{
    if(transforms.size() == 0){
        ROS_ERROR("joint states hasn't published yet");
        return;
    }

    for(int i=0;i<transforms.size();i++){
      this->tfMessage_msg.transforms[i].header.stamp = ros::Time::now();
      fillTfmessage(i,transforms[i]);
    }

    this->tfMessage_msg.transforms[0].header.frame_id = wambase_;
    this->tfMessage_msg.transforms[0].child_frame_id = wam0_;
    this->tfMessage_msg.transforms[1].header.frame_id = wam0_;
    this->tfMessage_msg.transforms[1].child_frame_id = wam1_;
    this->tfMessage_msg.transforms[2].header.frame_id = wam1_;
    this->tfMessage_msg.transforms[2].child_frame_id = wam2_;
    this->tfMessage_msg.transforms[3].header.frame_id = wam2_;
    this->tfMessage_msg.transforms[3].child_frame_id = wam3_;
    this->tfMessage_msg.transforms[4].header.frame_id = wam3_;
    this->tfMessage_msg.transforms[4].child_frame_id = wam4_;
    this->tfMessage_msg.transforms[5].header.frame_id = wam4_;
    this->tfMessage_msg.transforms[5].child_frame_id = wam5_;
    this->tfMessage_msg.transforms[6].header.frame_id = wam5_;
    this->tfMessage_msg.transforms[6].child_frame_id = wam6_;
    this->tfMessage_msg.transforms[7].header.frame_id = wam6_;
    this->tfMessage_msg.transforms[7].child_frame_id = wam7_;
  
    this->tf_publisher.publish(this->tfMessage_msg);
  
}

/*  [subscriber callbacks] */
void WamFK::joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg) 
{ 

    ROS_DEBUG("joint states callback with angles %f %f %f %f %f %f %f",
         msg->position[0],msg->position[1],msg->position[2],
         msg->position[3],msg->position[4],msg->position[5],msg->position[6]);
    
    transforms[0] << 1,0,0,0.22,
                     0,1,0,0.14,
                     0,0,1,0.346,
                     0,0,0,1;

    dhMatrix(0,-M_PI/2,0,msg->position[0], transforms[1]); 
    dhMatrix(0,M_PI/2,0,msg->position[1], transforms[2]); 
    dhMatrix(0.045,-M_PI/2,0.55,msg->position[2], transforms[3]); 
    dhMatrix(-0.045,M_PI/2,0,msg->position[3], transforms[4]); 
    dhMatrix(0,-M_PI/2,0.3,msg->position[4], transforms[5]); 
    dhMatrix(0,M_PI/2,0,msg->position[5], transforms[6]); 
    dhMatrix(0,0,0.0609,msg->position[6], transforms[7]); 
   
}

void WamFK::fillTfmessage(int joint, const Eigen::Matrix4f T){

      Quaternion<float> quat(T.block<3,3>(0,0));
      this->tfMessage_msg.transforms[joint].transform.translation.x = T(0,3);
      this->tfMessage_msg.transforms[joint].transform.translation.y = T(1,3);
      this->tfMessage_msg.transforms[joint].transform.translation.z = T(2,3);
      this->tfMessage_msg.transforms[joint].transform.rotation.x = quat.x();
      this->tfMessage_msg.transforms[joint].transform.rotation.y = quat.y();
      this->tfMessage_msg.transforms[joint].transform.rotation.z = quat.z();
      this->tfMessage_msg.transforms[joint].transform.rotation.w = quat.w();

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wam_fk");
    WamFK wam_fk;
    ros::Rate loop_rate(10); 
    while(ros::ok()){
      wam_fk.tfPub();
      ros::spinOnce();
      loop_rate.sleep(); 
    }
}

void WamFK::forwardKinematics(const sensor_msgs::JointState::ConstPtr& msg,geometry_msgs::Pose& pose){

  Matrix4f Tfk(Matrix4f::Identity());
  Matrix4f T_aux(Matrix4f::Identity());
  dhMatrix(0,-M_PI/2,0,msg->position[0], T_aux); 
  Tfk *= T_aux;
  dhMatrix(0,M_PI/2,0,msg->position[1], T_aux); 
  Tfk *= T_aux;
  dhMatrix(0.045,-M_PI/2,0.55,msg->position[2], T_aux); 
  Tfk *= T_aux;
  dhMatrix(-0.045,M_PI/2,0,msg->position[3], T_aux); 
  Tfk *= T_aux;
  dhMatrix(0,-M_PI/2,0.3,msg->position[4], T_aux); 
  Tfk *= T_aux;
  dhMatrix(0,M_PI/2,0,msg->position[5], T_aux); 
  Tfk *= T_aux;
  dhMatrix(0,0,0.0609,msg->position[6], T_aux); 
  Tfk *= T_aux;

  Quaternion<float> quat(Tfk.block<3,3>(0,0));
  pose.orientation.x = quat.x(); ; 
  pose.orientation.y = quat.y(); ; 
  pose.orientation.z = quat.z(); ; 
  pose.orientation.w = quat.w(); ; 
  pose.position.x = Tfk(1,3);
  pose.position.y = Tfk(2,3);
  pose.position.z = Tfk(3,3);

}

template<typename T>
void WamFK::quatToMatrix(Eigen::Matrix<T,4,4>& m,
                            const Eigen::Quaternion<T> qrot){

    m.template block<3,3>(0,0) = qrot.toRotationMatrix().transpose();
    m.template block<3,1>(0,3) = -m.template block<3,3>(0,0) * m.template block<3,1>(0,3);

}

void WamFK::dhMatrix(float a, float alpha, float d, float theta, Matrix4f& T){

 T.setIdentity(); 
 T << cos(theta),-sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta),
      sin(theta), cos(theta)*cos(alpha),-cos(theta)*sin(alpha), a*sin(theta),
               0,            sin(alpha),            cos(alpha),            d,
               0,                     0,                     0,            1;


}
