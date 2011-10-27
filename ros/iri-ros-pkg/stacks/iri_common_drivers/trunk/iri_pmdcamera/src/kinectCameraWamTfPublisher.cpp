//This can be used to publish a TF for the point cloud and visualize in rviz
//this example attaches the TF to the WAM 

#include <tf/transform_broadcaster.h>
//#include <tf/transform_listener.h>
//#include "tf/transform_datatypes.h"
#include "tf/tfMessage.h"

class kinectCameraWamTfPublisher {
  public:
  kinectCameraWamTfPublisher();
  ~kinectCameraWamTfPublisher();
  void mainLoop();
  
  ros::Publisher tf_publisher_;
//  tf::TransformListener listener;
  tf::tfMessage tfMessage_msg_;
  ros::NodeHandle nh_;

};

kinectCameraWamTfPublisher::kinectCameraWamTfPublisher() {
  
  tfMessage_msg_.transforms.resize(1);
  
  //string for port names
  std::string port_name;
  
//  port_name = ros::names::append(ros::this_node::getName(), "tf"); 
//  tf_publisher_ = nh_.advertise<tf::tfMessage>(port_name, 5);
  tf_publisher_ = nh_.advertise<tf::tfMessage>("tf", 5);
}

kinectCameraWamTfPublisher::~kinectCameraWamTfPublisher() {
  
}

void kinectCameraWamTfPublisher::mainLoop() {
    // [publish messages]
  // PM kinect
  //  this->tfMessage_msg_.transforms[0].header.stamp = ros::Time::now();
  //  this->tfMessage_msg_.transforms[0].header.frame_id = "wambase";
  //  this->tfMessage_msg_.transforms[0].child_frame_id = "openni_rgb_frame";
  //  this->tfMessage_msg_.transforms[0].transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0,1.25,3.14); 
  //  this->tfMessage_msg_.transforms[0].transform.translation.x = 1.13; 
  //  this->tfMessage_msg_.transforms[0].transform.translation.y = 0.15; 
  //  this->tfMessage_msg_.transforms[0].transform.translation.z = 0.80; 

  // WAM kinect
  this->tfMessage_msg_.transforms[0].header.stamp = ros::Time::now();
  this->tfMessage_msg_.transforms[0].header.frame_id = "wam7";
  //this->tfMessage_msg_.transforms[0].child_frame_id = "openni_rgb_optical_frame";
  this->tfMessage_msg_.transforms[0].child_frame_id = "openni_camera";
  //this->tfMessage_msg_.transforms[0].transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0,0,-1.57); 
  this->tfMessage_msg_.transforms[0].transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0,-1.57,0); 
  this->tfMessage_msg_.transforms[0].transform.translation.x = 0.0; 
  this->tfMessage_msg_.transforms[0].transform.translation.y = 0.0; 
  this->tfMessage_msg_.transforms[0].transform.translation.z = 0.1; 
  
//    tf::tfMessage tfMessage_msgaux(this->tfMessage_msg_);
//  this->tf_publisher.publish(tfMessage_msgaux);
  tf_publisher_.publish(tfMessage_msg_);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "kinect_Tf");
    kinectCameraWamTfPublisher kinectCameraWamTfPublisher_;
    ros::Rate loop_rate(10); 
    while(ros::ok()){
      kinectCameraWamTfPublisher_.mainLoop();
      ros::spinOnce();
      loop_rate.sleep(); 
    }
}
