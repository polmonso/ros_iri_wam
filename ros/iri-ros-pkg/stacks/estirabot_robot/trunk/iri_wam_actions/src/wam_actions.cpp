#include "wam_actions.h"

/* wam_actions node code */

WamActions::WamActions() {
  //init class attributes if necessary
  transform_grasping_point.setOrigin(tf::Vector3(0.5, 0.0, 0.5));
  transform_grasping_point.setRotation(tf::Quaternion(0, 0, 0, -sqrt(2)/2));
  destination_counter = 0;

  //string for port names

  // [init publishers]
  
  // [init subscribers]
  
  // [init services]
  this->wam_action_server = this->node_handle_.advertiseService("/wam_actions/wam_action", &WamActions::wam_actionCallback, this);
  
  // [init clients]
  obj_filter_client = this->node_handle_.serviceClient<iri_wam_common_msgs::compute_obj_grasp_pose>("/wam_actions/obj_filter");
  barrett_hand_cmd_client = this->node_handle_.serviceClient<iri_wam_common_msgs::bhand_cmd>("/wam_actions/bhand_cmd");
  pose_move_client = this->node_handle_.serviceClient<iri_wam_common_msgs::pose_move>("/wam_actions/pose_move");

  // [init action servers]
  
  // [init action clients]
}

void WamActions::mainLoop(void)
{
  //lock access to driver if necessary
  //this->driver_.lock();

  // [fill msg Header if necessary]
//  tf_broadcaster_mutex.enter();
//  tf_br.sendTransform(tf::StampedTransform(transform_grasping_point, ros::Time::now(), "openni_depth_optical_frame", "graspingPoint"));
//  tf_broadcaster_mutex.exit();

  // [fill msg structures]
  
  // [fill srv structure and make request to the server]
  
  // [fill action structure and make request to the action server]

  // [publish messages]

  //unlock access to driver if previously blocked
  //this->driver_.unlock();
}

/*  [subscriber callbacks] */

/*  [service callbacks] */
bool WamActions::wam_actionCallback(iri_wam_common_msgs::wamaction::Request &req, iri_wam_common_msgs::wamaction::Response &res) 
{ 
    double x=0.5,y,z;
    double x2=0.5,y2,z2;
    int zone = ANYZONE;
    int zone2 = ANYZONE;
    geometry_msgs::Pose grasping_pose;

    ROS_DEBUG("Received action %d at zone %d with grasp %d\n",req.action,req.zone,req.hand); 
    switch(req.zone){
        case ANYZONEMSG:
            zone = ANYZONE;
            zone2 = ANYZONE;
            break;
        case AZONEMSG:
            zone = AZONE;
            zone2 = BZONE;
            break;
        case BZONEMSG:
            zone = BZONE;
            zone2 = AZONE;
            break;
        default:
            ROS_ERROR("Unrecognized zone request");
            zone = ANYZONE;
            zone2 = ANYZONE;
            break;
    }

    switch(req.hand){
        case STRAIGHT:
            barrett_hand_cmd_srv.request.bhandcmd = "GM 1500"; 
            if (barrett_hand_cmd_client.call(barrett_hand_cmd_srv)) 
            { 
              ROS_DEBUG("bh %s success %d", barrett_hand_cmd_srv.request.bhandcmd.c_str(), barrett_hand_cmd_srv.response.success); 
            } else { 
              ROS_ERROR("Failed to call service barrett_hand_cmd"); 
              return false;
            }
            barrett_hand_cmd_srv.request.bhandcmd = "SO"; 
            if (barrett_hand_cmd_client.call(barrett_hand_cmd_srv)) 
            { 
              ROS_DEBUG("bh %s success %d", barrett_hand_cmd_srv.request.bhandcmd.c_str(), barrett_hand_cmd_srv.response.success); 
            } else { 
              ROS_ERROR("Failed to call service barrett_hand_cmd"); 
              return false;
            }
            break;
        case ISOMETRIC:
            ROS_INFO("ISOTAKE");
            barrett_hand_cmd_srv.request.bhandcmd = "SM 1000"; 
            if (barrett_hand_cmd_client.call(barrett_hand_cmd_srv)) { 
              ROS_DEBUG("bh %s success %d", barrett_hand_cmd_srv.request.bhandcmd.c_str(), barrett_hand_cmd_srv.response.success); 
            } else { 
              ROS_ERROR("Failed to call service barrett_hand_cmd"); 
              return false;
            }
            break;
        case PEG:
            ROS_INFO("PEG");
            barrett_hand_cmd_srv.request.bhandcmd = "SM 1600"; 
            if (barrett_hand_cmd_client.call(barrett_hand_cmd_srv)) { 
              ROS_DEBUG("bh %s success %d", barrett_hand_cmd_srv.request.bhandcmd.c_str(), barrett_hand_cmd_srv.response.success); 
            } else { 
              ROS_ERROR("Failed to call service barrett_hand_cmd"); 
              return false;
            }
            barrett_hand_cmd_srv.request.bhandcmd = "3M 18000"; 
            if (barrett_hand_cmd_client.call(barrett_hand_cmd_srv)) 
            { 
              ROS_DEBUG("bh %s success %d", barrett_hand_cmd_srv.request.bhandcmd.c_str(), barrett_hand_cmd_srv.response.success); 
            } else { 
              ROS_ERROR("Failed to call service barrett_hand_cmd"); 
              return false;
            }
            break;
        default:
            ROS_ERROR("Unrecognized hand request");
            break;
    }

    switch(req.action){
        case TAKEHIGH:
            ROS_INFO("TAKE");
            requestGraspingPoint(zone,x,y,z); 
            take3D(x,y,z);
            go_to_kinect();
            get_destination(zone2,x2,y2,z2);
            go_to(x2, y2, z2+0.2);
            drop(x2, y2, z2);
            go_to_kinect();
            break;
        case TAKELOW:
            ROS_INFO("TAKELOW");
            requestGraspingPoint(zone,x,y,z); 
            take3D(x,y,z-0.1);
            go_to_kinect();
            get_destination(zone2,x2,y2,z2);
            go_to(x2, y2, z2+0.3);
            drop(x2, y2, z2);
            go_to_kinect();
            break;
        case FOLDLOW:
            ROS_INFO("FOLDLOW");
            requestGraspingPoint(zone,x,y,z); 
            ROS_INFO("take %f %f %f",x,y,z);
            take3D(x,y,z);
            ROS_INFO("goto %f %f %f",x,y,z+0.2);
            go_to(x,y, z+0.2);
            if(check_box(NOX,y+0.3,NOZ) && y+0.3 < 0){
                ROS_INFO("goto %f %f %f",x,y+0.3,z+0.2);
                go_to(x,y+0.3, z+0.2);
            }else{
                ROS_INFO("Outside box, going the other way");
                ROS_INFO("goto %f %f %f",x,y-0.3,z+0.2);
                go_to(x,y-0.3, z+0.2);
            }
            drop();
            go_to_kinect();
            break;
        case FOLDHIGH:
            ROS_INFO("FOLDHIGH");
            requestGraspingPoint(zone,x,y,z); 
            take3D(x,y,z);
            go_to(0.4,-0.3, 0.3);
            go_to(0.4,-0.3, -0.1);
            go_to(0.4,-0.15, -0.2);
            go_to(0.4,-0.3, -0.3);
            drop();
            go_to_kinect();
            break;
        case SPREAD:
            ROS_INFO("SPREAD");
            requestGraspingPoint(zone,x,y,z); 
            take3D(x,y,z);
            //retrieve point far away in the direction from the cloth center to where I am.
            if(zone == AZONE){
                go_to(0.30, -0.4, z+0.1);
                drop(0.30, -0.40, z+0.05);
            }else{
                go_to(0.30, 0.4, z+0.1);
                drop(0.30, 0.40, z+0.05);
            }
            go_to_kinect();
            break; 
        case CHANGEPILE:
            ROS_INFO("CHANGE PILE");
            requestGraspingPoint(zone,x,y,z); 
            take3D(x,y,z);
            go_to_kinect();
            if(zone == AZONE)
                drop(0.5,0.3, 0.2); 
            else
                drop(0.5,-0.3, 0.2);
            go_to_kinect();
            break;
        case FLIP:
            ROS_INFO("FLIP");
            requestGraspingPoint(zone,x,y,z); 
            take3D(x,y,z);
            go_to_kinect();
        
            grasping_pose.position.x = 0.5;
            grasping_pose.position.y = 0;
            grasping_pose.position.z = 0.2;
            grasping_pose.orientation.x = 0.0;
            grasping_pose.orientation.y = 1.0;
            grasping_pose.orientation.z = 0.0;
            grasping_pose.orientation.w = 0.0;
            pose_move_srv.request.pose = grasping_pose;
        
            if (pose_move_client.call(pose_move_srv)) { 
              ROS_DEBUG("pose_move flip success %d", pose_move_srv.response.success);
            } else { 
              ROS_ERROR("Failed to call service pose_move");
              return false;
            }

            grasping_pose.position.x = x-0.05;
            grasping_pose.position.y = y;
            grasping_pose.position.z = z+0.18;
            grasping_pose.orientation.x = 0.0;
            grasping_pose.orientation.y = 1.0;
            grasping_pose.orientation.z = 0.0;
            grasping_pose.orientation.w = 0.0;
            pose_move_srv.request.pose = grasping_pose;
        
            if(pose_move_client.call(pose_move_srv)) { 
              ROS_DEBUG("pose_move flip back to position success%d", pose_move_srv.response.success);
            } else { 
              ROS_ERROR("Failed to call service pose_move");
              return false;
            }

            barrett_hand_cmd_srv.request.bhandcmd = "GM 5000"; 
            if (barrett_hand_cmd_client.call(barrett_hand_cmd_srv)) 
            { 
              ROS_DEBUG("bh2 torque open success %d", barrett_hand_cmd_srv.response.success); 
            } else { 
              ROS_ERROR("Failed to call service barrett_hand_cmd"); 
              return false;
            }
            go_to_kinect();
            break;
        case SHAKE:
            ROS_INFO("SHAKE");
            requestGraspingPoint(zone,x,y,z); 
            take3D(x,y,z);
            go_to(x,y,z+0.3);
            go_to(x,y-0.2,z+0.4);
            go_to(x,y+0.2,z+0.3);
            drop(x,y,z+0.2);
            go_to_kinect();
            break;
        case MOVEAWAY:
            ROS_INFO("MOVEAWAY");
            requestGraspingPoint(zone,x,y,z); 
            take3D(x,y,z);
            go_to_kinect();
            drop(-0.1,0.6,0.15);
            go_to_kinect();
            break;
        case DUMMY:
            ROS_INFO("DUMMY");
            requestGraspingPoint(zone,x,y,z); 
            break;
        default:
            ROS_ERROR("Invalid action id. Check wam_actions_node.h");
            break;
    }
    return true; 
}

bool WamActions::check_box(double x, double y, double z){
    return (x > 0.33 && x < 0.80 && y > -0.4 && y < 0.4 && z > -0.36 && z < 0.8);
}
bool WamActions::fit_box(double& x, double& y, double& z){

    if(!check_box(x,y,z)){
      ROS_WARN("Grasping point %f %f %f outside working box [0.33, 0.8] [-0.35, 0.4] [-0.37, 0.8] fitting to box",x,y,z);
      if(x<=0.33) x = 0.33;
      else if(x>=0.8) x = 0.8;
      if(y<=-0.4) y = -0.4;
      else if(y>=0.4) y = 0.4;
      if(z<=-0.36) z = -0.36;
      else if(z>=0.8) z = 0.8;
      ROS_WARN("fit with %f %f %f",x,y,z);
    }
    //TODO check if difference is too high instead of autorize move all the time
    return true;
}

/*  [action callbacks] */

/*  [action requests] */
bool WamActions::go_to(double x, double y, double z){

    geometry_msgs::Pose grasping_pose;
    if(!fit_box(x,y,z))
        return false;

    grasping_pose.position.x = x-0.05;
    grasping_pose.position.y = y;
    grasping_pose.position.z = z+0.18;
    grasping_pose.orientation.x = 1.0;
    grasping_pose.orientation.y = 0.0;
    grasping_pose.orientation.z = 0.0;
    grasping_pose.orientation.w = 0.0;
    pose_move_srv.request.pose = grasping_pose;

    if (pose_move_client.call(pose_move_srv)) { 
      ROS_DEBUG("pose_move go_to %f %f %f success %d",x,y,z, pose_move_srv.response.success);
      return true;
    } else { 
      ROS_ERROR("Failed to call service pose_move");
      return false;
    }
    return true;
}

bool WamActions::go_to_kinect(){

    geometry_msgs::Pose grasping_pose;

    grasping_pose.position.x = 0.5;
    grasping_pose.position.y = 0;
    grasping_pose.position.z = 0.5;
    grasping_pose.orientation.x = 1.0;
    grasping_pose.orientation.y = 0.0;
    grasping_pose.orientation.z = 0.0;
    grasping_pose.orientation.w = 0.0;
    pose_move_srv.request.pose = grasping_pose;

    if (pose_move_client.call(pose_move_srv)) { 
      ROS_DEBUG("pose_move kinect %f %f %f success %d",0.5,0.0,0.5, pose_move_srv.response.success);
      return true;
    } else { 
      ROS_ERROR("Failed to call service pose_move");
      return false;
    }
}

bool WamActions::requestGraspingPoint(int filterID, double &x, double &y, double &z){

    geometry_msgs::PoseStamped pose_msg;
    tf::StampedTransform transform_aux;
    //Call obj_filter node
    obj_filter_srv.request.filterID = filterID; 
    if(obj_filter_client.call(obj_filter_srv)) 
    { 
      ROS_DEBUG("Grasping point %f %f %f wrinkleness %f", obj_filter_srv.response.graspPose.pose.position.x,obj_filter_srv.response.graspPose.pose.position.y,obj_filter_srv.response.graspPose.pose.position.z, obj_filter_srv.response.wrinkleness); 
      pose_msg = obj_filter_srv.response.graspPose;
    } else { 
      ROS_ERROR("Failed to call service obj_filter"); 
      ROS_ERROR("defaulting");
      //default position for debugging the structure
      pose_msg.pose.position.x = 0.5;
      pose_msg.pose.position.y = 0.0;
      pose_msg.pose.position.z = 0.0;
      pose_msg.pose.orientation.x = 1;
      pose_msg.pose.orientation.y = 0;
      pose_msg.pose.orientation.z = 0;
      pose_msg.pose.orientation.w = 0;
      return false;
    }

    transform_grasping_point.setOrigin(tf::Vector3(pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z));
    transform_grasping_point.setRotation(tf::Quaternion( pose_msg.pose.position.x, pose_msg.pose.orientation.y,
                                                   pose_msg.pose.orientation.z, pose_msg.pose.orientation.w));

    //ensure tf is published before tf lookup
    //tf_br.sendTransform(tf::StampedTransform(transform_grasping_point, ros::Time::now(), "openni_depth_optical_frame", "graspingPoint"));

    ROS_DEBUG("t_aux %f %f %f   %f %f %f %f",transform_grasping_point.getOrigin().x(),transform_grasping_point.getOrigin().y(),transform_grasping_point.getOrigin().z(),transform_grasping_point.getRotation().getAxis().x(),transform_grasping_point.getRotation().getAxis().y(),transform_grasping_point.getRotation().getAxis().z(),transform_grasping_point.getRotation().getAxis().w());

    //maybe wait for a while so the tf updates the data?
    sleep(2); 
    //retrieve target pose from tf tree
    try{
      tf_grasp.lookupTransform("/wam0", "/graspingPoint", ros::Time(0), transform_aux);
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s",ex.what());
      return false;
    }

    //execute action 
    x = transform_aux.getOrigin().x();
    y = transform_aux.getOrigin().y(); 
    z = transform_aux.getOrigin().z();
    ROS_DEBUG("Received 3D position %f %f %f\n",x,y,z);
    return true;

}

bool WamActions::take3D(double x, double y,double z){
    
    if(!fit_box(x,y,z))
        return false;
    //execute move 
    geometry_msgs::Pose grasping_pose;

//assuming hand opened and convinent for finger 3 blocked
//    barrett_hand_cmd_srv.request.bhandcmd = "GTO"; 
//    if (barrett_hand_cmd_client.call(barrett_hand_cmd_srv)) 
//    { 
//      ROS_DEBUG("bh take3d success %d", barrett_hand_cmd_srv.response.success); 
//    } else { 
//      ROS_ERROR("Failed to call service barrett_hand_cmd"); 
//      return false;
//    }

    //switch to camera coordinates
    //transform to world coordinates
    grasping_pose.position.x = x-0.05;
    grasping_pose.position.y = y;
    grasping_pose.position.z = z+0.17;
    grasping_pose.orientation.x = sqrt(2)/2;
    grasping_pose.orientation.y = sqrt(2)/2;
    grasping_pose.orientation.z = 0.0;
    grasping_pose.orientation.w = 0.0;
    pose_move_srv.request.pose = grasping_pose;

    if (pose_move_client.call(pose_move_srv)) { 
      ROS_DEBUG("pose_move success %d", pose_move_srv.response.success);
    } else { 
      ROS_ERROR("Failed to call service pose_move");
      return false;
    }

    barrett_hand_cmd_srv.request.bhandcmd = "GM 18000"; 
    if (barrett_hand_cmd_client.call(barrett_hand_cmd_srv)) 
    { 
      ROS_DEBUG("bh2 success %d", barrett_hand_cmd_srv.response.success); 
    } else { 
      ROS_ERROR("Failed to call service barrett_hand_cmd"); 
      return false;
    }
    return true;
}

bool WamActions::drop(){
    barrett_hand_cmd_srv.request.bhandcmd = "GM 5000"; 
    if (barrett_hand_cmd_client.call(barrett_hand_cmd_srv)) 
    { 
      ROS_DEBUG("bh_drop success %d", barrett_hand_cmd_srv.response.success); 
    } else { 
      ROS_ERROR("Failed to call service barrett_hand_cmd"); 
      return false;
    }
    return true;
}

bool WamActions::drop(double x, double y, double z){

    geometry_msgs::Pose droppose;
    //switch to camera coordinates
    //transform to world coordinates
    droppose.position.x = x-0.05;
    droppose.position.y = y;
    droppose.position.z = z+0.18;
    droppose.orientation.x = sqrt(2)/2;
    droppose.orientation.y = sqrt(2)/2;
    droppose.orientation.z = 0.0;
    droppose.orientation.w = 0.0;
    pose_move_srv.request.pose = droppose;
    if (pose_move_client.call(pose_move_srv)) { 
      ROS_DEBUG("pose_move success %d", pose_move_srv.response.success);
    } else { 
      ROS_ERROR("Failed to call service pose_move");
      return false;
    }
    drop();

    return true; 

}

bool WamActions::fold(){

  return false;
}

bool WamActions::flip(){

  return false;
}

void WamActions::get_destination(int destination_zone,  double& x, double& y, double& z){
    if(destination_zone == AZONE){
        this->destination_counter++;
        this->destination_counter = this->destination_counter%6;
        switch(destination_counter){
            case 0:
                x = 0.50;
                y = -0.27;
                z = 0.0; 
                break;
            case 1:
                x = 0.55;
                y = -0.27;
                z = 0.0; 
                break;
            case 2:
                x = 0.55;
                y = -0.35;
                z = 0.0; 
                break;
            case 3:
                x = 0.50;
                y = -0.35;
                z = 0.0; 
                break;
            case 4:
                x = 0.45;
                y = -0.35;
                z = 0.0; 
                break;
            case 5:
                x = 0.45;
                y = -0.27;
                z = 0.0; 
                break;
            default:
                x = 0.50;
                y = -0.27;
                z = 0.0; 
                break;
        }
    }else if(destination_zone == BZONE){
        destination_counter++;
        destination_counter = destination_counter%6;
        switch(destination_counter){
            case 0:
                x = 0.50;
                y = 0.3;
                z = 0.0; 
                break;
            case 1:
                x = 0.55;
                y = 0.3;
                z = 0.0; 
                break;
            case 2:
                x = 0.55;
                y = 0.4;
                z = 0.0; 
                break;
            case 3:
                x = 0.50;
                y = 0.4;
                z = 0.0; 
                break;
            case 4:
                x = 0.45;
                y = 0.4;
                z = 0.0; 
                break;
            case 5:
                x = 0.45;
                y = 0.3;
                z = 0.0; 
                break;
            default:
                x = 0.50;
                y = 0.3;
                z = 0.0; 
                break;
        }
    }else{
        //TODO check xyz zone and mirror it
        x = 0.50;
        y = -0.3;
        z = 0.0; 
    }
}


/* main function */
int main(int argc,char *argv[])
{
    ros::init(argc, argv, "wam_actions");
    WamActions wam_actions;
    ros::Rate loop_rate(10); 
    while(ros::ok()){
      wam_actions.mainLoop();
      ros::spinOnce();
      loop_rate.sleep(); 
    }
}
