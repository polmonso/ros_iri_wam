#include "wam_tutorial.h"

/* wam_tutorial node code */

WamTutorial::WamTutorial() {
  //init class attributes if necessary
  this->gocenter = true;
  this->targetid = 0;

  this->numtargets = 10;
  this->tfMessage_msg.transforms.resize(numtargets+2);
  this->angles.resize(5);
  this->angles[0] = 0.0;
  this->angles[1] = M_PI/4.0;
  this->angles[2] = 3.0*M_PI/4.0;
  this->angles[3] = 9.0/8.0*M_PI;
  this->angles[4] = 13.0/8.0*M_PI;
  //string for port names
  std::string port_name;

  // [init publishers]
  port_name = ros::names::append(ros::this_node::getName(), "tf"); 
  this->tf_publisher = this->nh_.advertise<tf::tfMessage>(port_name, 5);
  
  // [init subscribers]
  port_name = ros::names::append(ros::this_node::getName(), "pose"); 
  this->pose_subscriber = this->nh_.subscribe(port_name, 5, &WamTutorial::pose_callback, this);
  port_name = ros::names::append(ros::this_node::getName(), "joint_states"); 
  this->joint_states_subscriber = this->nh_.subscribe(port_name, 5, &WamTutorial::joint_states_callback, this);
  
  // [init services]
  
  // [init clients]
  port_name = ros::names::append(ros::this_node::getName(), "pose_move2"); 
  pose_move2_client = this->nh_.serviceClient<iri_wam_common_msgs::pose_move>(port_name);

  port_name = ros::names::append(ros::this_node::getName(), "pose_move"); 
  pose_move_client = this->nh_.serviceClient<iri_wam_common_msgs::pose_move>(port_name);
  port_name = ros::names::append(ros::this_node::getName(), "joints_move"); 
  joint_move_client = this->nh_.serviceClient<iri_wam_common_msgs::joints_move>(port_name);
  
  // [init action servers]
  
  // [init action clients]
}

void WamTutorial::mainLoop(void)
{
  //lock access to driver if necessary
  //this->driver_.lock();
  tf::StampedTransform transform;
  geometry_msgs::TransformStamped msg;
  std::ostringstream oss;
  std::ostringstream oss2;
  std::ostringstream oss3;
  std::string targetname("target");
  // [fill msg Header if necessary]
  //<publisher_name>.header.stamp = ros::Time::now();
  //<publisher_name>.header.frame_id = "<publisher_topic_name>";

  // [fill msg structures]
  //this->tfMessage_msg.data = my_var;
  

  //joint_move_srv.request.resize(7);
  //joint_move_srv.request.joints[0] = 0.0; 
  //joint_move_srv.request.joints[1] = 0.0; 
  //joint_move_srv.request.joints[2] = 0.2; 
  //joint_move_srv.request.joints[3] = 0.0; 
  //joint_move_srv.request.joints[4] = 0.0; 
  //joint_move_srv.request.joints[5] = 0.0; 
  //joint_move_srv.request.joints[6] = 0.0; 
  //if (joint_move_client.call(joint_move_srv)) { 
  //  ROS_INFO(joint_move_srv.response.success);
  //} else { 
  //  ROS_ERROR("Failed to call service joint_move"); 
  //}
  
  // [fill action structure and make request to the action server]

  // [publish messages]
  this->tfMessage_msg.transforms[0].header.stamp = ros::Time::now();
  this->tfMessage_msg.transforms[0].header.frame_id = "wam0";
  this->tfMessage_msg.transforms[0].child_frame_id = "central";
  this->tfMessage_msg.transforms[0].transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0); 

  this->tfMessage_msg.transforms[0].transform.translation.x = 0.55; 
  this->tfMessage_msg.transforms[0].transform.translation.y = 0.2; 
  this->tfMessage_msg.transforms[0].transform.translation.z = 0.1; 

  this->tfMessage_msg.transforms[1].header.stamp = ros::Time::now();
  this->tfMessage_msg.transforms[1].header.frame_id = "central";
  this->tfMessage_msg.transforms[1].child_frame_id = "target0b";
  this->tfMessage_msg.transforms[1].transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0); 

  this->tfMessage_msg.transforms[1].transform.translation.x = 0.0; 
  this->tfMessage_msg.transforms[1].transform.translation.y = 0.0; 
  this->tfMessage_msg.transforms[1].transform.translation.z = 0.3; 

  for(uint i=2;i<tfMessage_msg.transforms.size();i+=2){
    this->tfMessage_msg.transforms[i].header.stamp = ros::Time::now();
    this->tfMessage_msg.transforms[i].header.frame_id = "central";
    std::ostringstream oss;
    oss << targetname << i/2;
    this->tfMessage_msg.transforms[i].child_frame_id = oss.str();
    this->tfMessage_msg.transforms[i].transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(angles[(i+1)/2%5],0,0); 
    this->tfMessage_msg.transforms[i].transform.translation.x = 0.0; 
    this->tfMessage_msg.transforms[i].transform.translation.y = 0.0; 
    this->tfMessage_msg.transforms[i].transform.translation.z = 0.0; 

    this->tfMessage_msg.transforms[i+1].header.stamp = ros::Time::now();
    this->tfMessage_msg.transforms[i+1].header.frame_id = oss.str();
    oss << "b";
    this->tfMessage_msg.transforms[i+1].child_frame_id = oss.str();
    this->tfMessage_msg.transforms[i+1].transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0); 

    this->tfMessage_msg.transforms[i+1].transform.translation.x = 0.0; 
    this->tfMessage_msg.transforms[i+1].transform.translation.y = 0.0; 
    this->tfMessage_msg.transforms[i+1].transform.translation.z = 0.3; 
  } 

  tf::tfMessage tfMessage_msgaux(this->tfMessage_msg);
  this->tf_publisher.publish(tfMessage_msgaux);

  oss3 << targetname << targetid << "b";
  calculate_error(oss3.str());

    if(gocenter){
      gocenter = false;
      ROS_INFO("Going to central");
      try{
        listener.lookupTransform("wam0", "central", ros::Time(0), transform);
        tf::transformStampedTFToMsg(transform,msg);
        //Sembla increible, pero els valors estan desplaçats...
        //!--> y = x, z = y, w = z i x = w.
        pose_move_srv.request.pose.orientation.y = msg.transform.rotation.x;
        pose_move_srv.request.pose.orientation.z = msg.transform.rotation.y;
        pose_move_srv.request.pose.orientation.w = msg.transform.rotation.z;
        pose_move_srv.request.pose.orientation.x = msg.transform.rotation.w;
        
        pose_move_srv.request.pose.position.x = msg.transform.translation.x;
        pose_move_srv.request.pose.position.y = msg.transform.translation.y;
        pose_move_srv.request.pose.position.z = msg.transform.translation.z;
        std::cout << "moving to:" << msg << std::endl;
        sleep(1);
        //kaijen ik
        if (pose_move2_client.call(pose_move_srv)) { 
          ROS_INFO("pose_move2: %d", pose_move_srv.response.success); 
          calculate_error("central");
        } else { 
          ROS_ERROR("Failed to call service pose_move2"); 
        }
      } catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
      } 
    }else{ 
      gocenter = true;
      targetid++;
      targetid = targetid%6;
      oss2 << targetname << targetid << "b";
      ROS_INFO("Going to %s",oss2.str().c_str());
      try{
        listener.lookupTransform("wam0", oss2.str(), ros::Time(0), transform);

//change this with http://www.ros.org/wiki/tf/Overview/Using%20Published%20Transforms

        tf::transformStampedTFToMsg(transform,msg);
        pose_move_srv.request.pose.orientation.y = msg.transform.rotation.x;
        pose_move_srv.request.pose.orientation.z = msg.transform.rotation.y;
        pose_move_srv.request.pose.orientation.w = msg.transform.rotation.z;
        pose_move_srv.request.pose.orientation.x = msg.transform.rotation.w;
        
        pose_move_srv.request.pose.position.x = msg.transform.translation.x;
        pose_move_srv.request.pose.position.y = msg.transform.translation.y;
        pose_move_srv.request.pose.position.z = msg.transform.translation.z;
          std::cout << "moving to:" << msg << std::endl;
        sleep(1);
        //kaijen ik
        if (pose_move2_client.call(pose_move_srv)) { 
          ROS_INFO("pose_move2: %d", pose_move_srv.response.success); 
        } else { 
          ROS_ERROR("Failed to call service pose_move2"); 
        }
      } catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
      } 
    }
  // [fill srv structure and make request to the server]
  
//  pose_move_srv.request.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.785,0.785,0.785); 
//
//  pose_move_srv.request.pose.position.x = 0.4; 
//  pose_move_srv.request.pose.position.y = 0.2; 
//  pose_move_srv.request.pose.position.z = 0.1; 

//  if (pose_move_client.call(pose_move_srv)) { 
//    ROS_INFO("pose_move: %d", pose_move_srv.response.success); 
//  } else { 
//    ROS_ERROR("Failed to call service pose_move"); 
//  }

  //unlock access to driver if previously blocked
  //this->driver_.unlock();
}

/*  [subscriber callbacks] */
void WamTutorial::pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) 
{ 
  //std::cout << msg->data << std::endl; 

}
void WamTutorial::joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg) 
{ 

  //std::cout << msg->data << std::endl; 

}

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void WamTutorial::calculate_error(std::string target) {

  tf::StampedTransform transform;
  geometry_msgs::TransformStamped msg;
  try{
      listener.lookupTransform("wam7", target, ros::Time(0), transform);
      tf::transformStampedTFToMsg(transform,msg);
      std::cout << "transform error " << msg << std::endl;
  } catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  } 
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "wam_tut");
    WamTutorial wam_tutorial;
    ros::Rate loop_rate(10); 
    while(ros::ok()){
      wam_tutorial.mainLoop();
      ros::spinOnce();
      loop_rate.sleep(); 
    }
}
