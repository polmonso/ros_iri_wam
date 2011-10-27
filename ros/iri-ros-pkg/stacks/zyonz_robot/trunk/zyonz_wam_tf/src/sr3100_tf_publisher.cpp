//This can be used to publish a TF for the point cloud and visualize in rviz
//this example attaches the TF to the WAM 

#include "tf/transform_broadcaster.h"
#include "tf/tfMessage.h"

class sr3100CameraTfPublisher {
  public:
  sr3100CameraTfPublisher();
  ~sr3100CameraTfPublisher();
  void mainLoop();
  
  ros::Publisher tf_publisher_;
  tf::tfMessage tfMessage_msg_;
  ros::NodeHandle nh_;

};

sr3100CameraTfPublisher::sr3100CameraTfPublisher() {
  
  tfMessage_msg_.transforms.resize(1);
  tf_publisher_ = nh_.advertise<tf::tfMessage>("tf", 5);

}

sr3100CameraTfPublisher::~sr3100CameraTfPublisher() {
  
}

void sr3100CameraTfPublisher::mainLoop() {
  
  // [publish messages]
  this->tfMessage_msg_.transforms[0].header.stamp = ros::Time::now();
  this->tfMessage_msg_.transforms[0].header.frame_id = "wam7";
  this->tfMessage_msg_.transforms[0].child_frame_id = "/camera";

  this->tfMessage_msg_.transforms[0].transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(-0.0040, -0.0175, -1.5711); 
  this->tfMessage_msg_.transforms[0].transform.translation.x = -0.0060545; 
  this->tfMessage_msg_.transforms[0].transform.translation.y = 0.0066072; 
  this->tfMessage_msg_.transforms[0].transform.translation.z = 0.1165950; 

  tf_publisher_.publish(tfMessage_msg_);

}

int main(int argc, char** argv) {

    ros::init(argc, argv, "sr3100_Tf");
    sr3100CameraTfPublisher sr3100CameraTfPublisher_;
    ros::Rate loop_rate(10); 
    while(ros::ok()){
      sr3100CameraTfPublisher_.mainLoop();
      ros::spinOnce();
      loop_rate.sleep(); 
    }

}
