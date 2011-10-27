//This can be used to publish a TF for the point cloud and visualize in rviz
//this example attaches the TF to the WAM 

#include <tf/transform_broadcaster.h>
//#include <tf/transform_listener.h>
//#include "tf/transform_datatypes.h"
#include "tf/tfMessage.h"

class kinectCameraTfPublisher {
  public:
  kinectCameraTfPublisher();
  ~kinectCameraTfPublisher();
  void mainLoop();
  
  ros::Publisher tf_publisher_;
//  tf::TransformListener listener;
  tf::tfMessage tfMessage_msg_;
  ros::NodeHandle nh_;

};

kinectCameraTfPublisher::kinectCameraTfPublisher() {
  
  tfMessage_msg_.transforms.resize(1);
  
  //string for port names
  std::string port_name;
  
//  port_name = ros::names::append(ros::this_node::getName(), "tf"); 
//  tf_publisher_ = nh_.advertise<tf::tfMessage>(port_name, 5);
  tf_publisher_ = nh_.advertise<tf::tfMessage>("tf", 5);
}

kinectCameraTfPublisher::~kinectCameraTfPublisher() {
  
}

void kinectCameraTfPublisher::mainLoop() {
    // [publish messages]
    // PM kinect
    this->tfMessage_msg_.transforms[0].header.stamp = ros::Time::now();
    this->tfMessage_msg_.transforms[0].header.frame_id = "wambase";
    this->tfMessage_msg_.transforms[0].child_frame_id = "openni_rgb_frame";
    
    this->tfMessage_msg_.transforms[0].transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0,1.25,3.14); 
    this->tfMessage_msg_.transforms[0].transform.translation.x = 1.13; 
    this->tfMessage_msg_.transforms[0].transform.translation.y = 0.15; 
    this->tfMessage_msg_.transforms[0].transform.translation.z = 0.80; 
  
//    tf::tfMessage tfMessage_msgaux(this->tfMessage_msg_);
//  this->tf_publisher.publish(tfMessage_msgaux);
  tf_publisher_.publish(tfMessage_msg_);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "kinect_Tf");
    kinectCameraTfPublisher kinectCameraTfPublisher_;
    ros::Rate loop_rate(10); 
    while(ros::ok()){
      kinectCameraTfPublisher_.mainLoop();
      ros::spinOnce();
      loop_rate.sleep(); 
    }
}
