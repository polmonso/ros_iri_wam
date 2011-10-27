//This can be used to publish a TF for the point cloud and visualize in rviz
//this example attaches the TF to the WAM 

#include <tf/transform_broadcaster.h>

void callback() {
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(-0.0690, 0.0051, 0.1501));
  transform.setRotation(tf::Quaternion( -1.5187, -0.0593, -0.1859 )); //tf::Quaternion(Yaw, Pitch, Roll);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "wam7", "camcube3_frame"));
}

int main(int argc, char** argv) {
  ros::Time::init();
  ros::init(argc, argv, "camcube3_Tf_broadcaster");
  ros::Rate loop_rate(10);
  while(ros::ok()){
    callback();
    ros::spinOnce();
    loop_rate.sleep(); 
  }

  return 0;
}
