//This can be used to publish a TF for the point cloud and visualize in rviz
//this example attaches the TF to the WAM 

#include <tf/transform_broadcaster.h>
#include "tf/tfMessage.h"

class pmdCameraTfPublisher {
  public:
  pmdCameraTfPublisher();
  ~pmdCameraTfPublisher();
  void mainLoop();
  
  ros::Publisher tf_publisher_;
  tf::tfMessage tfMessage_msg_;
  ros::NodeHandle nh_;

};

pmdCameraTfPublisher::pmdCameraTfPublisher() {
  
  tfMessage_msg_.transforms.resize(1);
  
  // [publishers]
  tf_publisher_ = nh_.advertise<tf::tfMessage>("tf", 5);
}

pmdCameraTfPublisher::~pmdCameraTfPublisher() {
  
}

void pmdCameraTfPublisher::mainLoop() {
  // [publish messages]
  this->tfMessage_msg_.transforms[0].header.stamp = ros::Time::now();
  this->tfMessage_msg_.transforms[0].header.frame_id = "wam7";
  this->tfMessage_msg_.transforms[0].child_frame_id = "camcube3_frame";
  this->tfMessage_msg_.transforms[0].transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(-0.1284,-0.0090,-1.5220); 
  
  this->tfMessage_msg_.transforms[0].transform.translation.x = -0.0861; 
  this->tfMessage_msg_.transforms[0].transform.translation.y = -0.0030; 
  this->tfMessage_msg_.transforms[0].transform.translation.z =  0.1285; 

  tf_publisher_.publish(tfMessage_msg_);

  // static tf::TransformBroadcaster br;
  // br.sendTransform (tf::StampedTransform (tf::Transform (tf::Quaternion (0, 0, 0, 1), tf::Vector3 (0, 0, 0.5)), ros::Time::now (), "wam7", "camcube3_frame"));



}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pmd_Tf");
    pmdCameraTfPublisher pmdCameraTfPublisher_;
    ros::Rate loop_rate(10); 
    while(ros::ok()){
      pmdCameraTfPublisher_.mainLoop();
      ros::spinOnce();
      loop_rate.sleep(); 
    }
}
