#include "dummy_pomdp.h"

DummyPomdp::DummyPomdp(){

  //init class attributes if necessary
    this->execution_flag = false;

  //string for port names

  // [init publishers]
  this->focused_obj_label_publisher = this->node_handle_.advertise<std_msgs::Int32>("/dummy_pomdp/focused_obj_label", 5);
  
  // [init subscribers]
  
  // [init services]
  this->execution_flow_control_server = this->node_handle_.advertiseService("/dummy_pomdp/execution_flow_control", &DummyPomdp::execution_flow_controlCallback, this);
  
  // [init clients]
  obs_client = this->node_handle_.serviceClient<iri_wam_common_msgs::obs_request>("/dummy_pomdp/obs_client");
  wam_action_client = this->node_handle_.serviceClient<iri_wam_common_msgs::wamaction>("/dummy_pomdp/wam_action");

  // [init action servers]
  
  // [init action clients]
}

void DummyPomdp::mainLoop(void){

    execution_mutex.enter();
    if(execution_flag){
        execution_mutex.exit();
        if(obs_client.call(obs_request_srv)) 
        { 
          ROS_INFO("Service result: %d", obs_request_srv.response.num_objects); 
        } else { 
          ROS_ERROR("Failed to call service observation client"); 
          return;
        }
    
        //process observation
        //update state
        //retrieve action 

        this->ObjLabel_msg.data = 1;
        //publish focused object
        focused_obj_label_publisher.publish(this->ObjLabel_msg);
        
        //request action
        wam_actions_client_srv.request.action = 0;
        if(wam_action_client.call(wam_actions_client_srv)) 
        { 
          ROS_INFO("Service result: %d", wam_actions_client_srv.response.success); 
        } else { 
          ROS_ERROR("Failed to call service wam actions service"); 
          return;
        }

    }else{
        execution_mutex.exit();
        //sleep
        ROS_INFO("Awaiting trigger");
        sleep(5);

    }

}

bool DummyPomdp::execution_flow_controlCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){

    //stop execution when called
    execution_mutex.enter();
    this->execution_flag = !this->execution_flag;
    execution_mutex.exit();
    //TODO free execution mutex
    return true;

}

/* main function */
int main(int argc,char *argv[])
{
    ros::init(argc, argv, "dummy_pomdp");
    DummyPomdp dummy_pomdp;
    ros::Rate loop_rate(10); 
    while(ros::ok()){
      dummy_pomdp.mainLoop();
      ros::spinOnce();
      loop_rate.sleep(); 
    }
}
