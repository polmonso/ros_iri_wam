#include "fixed_strategy.h"

FixedStrategy::FixedStrategy(){

  //init class attributes if necessary
    this->execution_flag = false;

  //string for port names

  // [init publishers]
  this->focused_obj_label_publisher = this->node_handle_.advertise<std_msgs::Int32>("/planner/focused_obj_label", 5);
  
  // [init subscribers]
  
  // [init services]
  this->execution_flow_control_server = this->node_handle_.advertiseService("/planner/execution_flow_control", &FixedStrategy::execution_flow_controlCallback, this);
  
  // [init clients]
  obs_client = this->node_handle_.serviceClient<iri_wam_common_msgs::obs_request>("/planner/obs_client");
  wam_action_client = this->node_handle_.serviceClient<iri_wam_common_msgs::wamaction>("/planner/wam_action");

  // [init action servers]
  
  // [init action clients]
}

void FixedStrategy::mainLoop(void){

    if(execution_flag){
        ROS_INFO("Starting Loop");
        if(obs_client.call(obs_request_srv)) 
        { 
          ROS_INFO("Service result: %d objects at A, %d objects at B, total %d", obs_request_srv.response.num_objectsA,obs_request_srv.response.num_objectsB,obs_request_srv.response.num_objects); 
        } else { 
          ROS_ERROR("Failed to call service observation client"); 
          return;
        }
    
        //process observation
        if(obs_request_srv.response.num_objectsB > 1){
            //bring some back
            //unused on a fixed strategy
            this->ObjLabel_msg.data = 1; //1 = A, 2 = B
            focused_obj_label_publisher.publish(this->ObjLabel_msg);
            
            wam_actions_client_srv.request.action = CHANGEPILE;
            wam_actions_client_srv.request.zone = B;
            if(wam_action_client.call(wam_actions_client_srv)) { 
              ROS_INFO("Service result: %d", wam_actions_client_srv.response.success); 
            } else { 
              ROS_ERROR("Failed to call service wam actions service"); 
              return;
            }
        }else if(obs_request_srv.response.num_objectsB == 1){
            //hooray! take it out
            wam_actions_client_srv.request.action = MOVEAWAY;
            if(wam_action_client.call(wam_actions_client_srv)) { 
              ROS_INFO("Service result: %d", wam_actions_client_srv.response.success); 
            } else { 
              ROS_ERROR("Failed to call service wam actions service"); 
              return;
            }
        }else if(obs_request_srv.response.num_objectsB == 0){
            wam_actions_client_srv.request.action = CHANGEPILE;
            wam_actions_client_srv.request.zone = A;
            if(wam_action_client.call(wam_actions_client_srv)) { 
              ROS_INFO("Service result: %d", wam_actions_client_srv.response.success); 
            } else { 
              ROS_ERROR("Failed to call service wam actions service"); 
              return;
            }
        }


    }else{
        //sleep
        ROS_INFO("Awaiting trigger");
        sleep(5); 
    }

}

bool FixedStrategy::execution_flow_controlCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){

    //stop execution when called
    this->execution_flag = !this->execution_flag;
    //TODO free execution mutex
    return true;

}

/* main function */
int main(int argc,char *argv[])
{
    ros::init(argc, argv, "planner");
    FixedStrategy fixed_strategy;
    ros::Rate loop_rate(10); 
    while(ros::ok()){
      fixed_strategy.mainLoop();
      ros::spinOnce();
      loop_rate.sleep(); 
    }
}
