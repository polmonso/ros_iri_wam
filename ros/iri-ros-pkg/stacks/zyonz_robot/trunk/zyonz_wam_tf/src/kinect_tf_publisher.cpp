//This can be used to publish a TF for the point cloud and visualize in rviz
//this example attaches the TF to the WAM 

#include "tf/transform_broadcaster.h"
#include "tf/tfMessage.h"

class kinectCameraTfPublisher {
  public:
  kinectCameraTfPublisher();
  ~kinectCameraTfPublisher();
  void mainLoop();
  
  ros::Publisher tf_publisher_;
  tf::tfMessage tfMessage_msg_;
  ros::NodeHandle nh_;

};

kinectCameraTfPublisher::kinectCameraTfPublisher() {
  
  tfMessage_msg_.transforms.resize(1);
  tf_publisher_ = nh_.advertise<tf::tfMessage>("tf", 5);

}

kinectCameraTfPublisher::~kinectCameraTfPublisher() {
  
}

void kinectCameraTfPublisher::mainLoop() {

  // [publish messages]
  this->tfMessage_msg_.transforms[0].header.stamp = ros::Time::now();
  this->tfMessage_msg_.transforms[0].header.frame_id = "wam0"; //not wam0?
  this->tfMessage_msg_.transforms[0].child_frame_id = "openni_rgb_frame";
  
  //TODO read from parameter server
  btMatrix3x3 cam2wambase(0.0307485, 0.948154, -0.316320, 0.999456, 0.0253980, -0.0210247, -0.0119007, -0.316795, -0.948420);
  double yaw, pitch, roll;
  cam2wambase.getEulerYPR(yaw, pitch, roll);

  this->tfMessage_msg_.transforms[0].transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw); 
  this->tfMessage_msg_.transforms[0].transform.translation.x =  0.851029; 
  this->tfMessage_msg_.transforms[0].transform.translation.y = -0.0173444; 
  this->tfMessage_msg_.transforms[0].transform.translation.z =  0.228769; 
  
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
