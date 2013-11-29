//This can be used to publish a TF for the point cloud and visualize in rviz
//this example attaches the TF to the WAM 

#include <tf/transform_broadcaster.h>
#include "tf/tfMessage.h"

class Color2CamboardTfPublisher {
  public:
  Color2CamboardTfPublisher();
  ~Color2CamboardTfPublisher();
  void mainLoop();
  
  ros::Publisher tf_publisher_;
  tf::tfMessage tfMessage_msg_;
  ros::NodeHandle nh_;

};

Color2CamboardTfPublisher::Color2CamboardTfPublisher() {
  
  tfMessage_msg_.transforms.resize(1);
  
  //string for port names
  std::string port_name;
  tf_publisher_ = nh_.advertise<tf::tfMessage>("tf", 5);
}

Color2CamboardTfPublisher::~Color2CamboardTfPublisher() {
  
}

void
Color2CamboardTfPublisher::mainLoop()
{
  // [publish messages]
  this->tfMessage_msg_.transforms[0].header.stamp = ros::Time::now();
  this->tfMessage_msg_.transforms[0].header.frame_id = "camboard_frame";
  this->tfMessage_msg_.transforms[0].child_frame_id = "color_camera_frame";
  this->tfMessage_msg_.transforms[0].transform.rotation = tf::createQuaternionMsgFromRollPitchYaw( 0, 0, 0 ); 
  
  this->tfMessage_msg_.transforms[0].transform.translation.x = 0.0003921; 
  this->tfMessage_msg_.transforms[0].transform.translation.y = -0.0592096; 
  this->tfMessage_msg_.transforms[0].transform.translation.z = -0.0208921; 
  
  //    tf::tfMessage tfMessage_msgaux(this->tfMessage_msg_);
  //  this->tf_publisher.publish(tfMessage_msgaux);
  tf_publisher_.publish(tfMessage_msg_);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "color2camboard_Tf");
  Color2CamboardTfPublisher Color2CamboardTfPublisher_;
  ros::Rate loop_rate(10); 
  while(ros::ok()){
    Color2CamboardTfPublisher_.mainLoop();
    ros::spinOnce();
    loop_rate.sleep(); 
  }
}