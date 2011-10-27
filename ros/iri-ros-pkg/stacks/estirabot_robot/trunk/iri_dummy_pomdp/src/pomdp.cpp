#include "pomdp.h"

Pomdp::Pomdp(){

  int numstates, numactions, numobservations;
  //init class attributes if necessary
  this->actioncount = 0;

  std::string pomdpfile("data/pomdpfile.txt");
  if(node_handle_.getParam("pomdpfile", pomdpfile)) {
    ROS_INFO("pomdpfile global path found, using %s", pomdpfile.c_str());
  }else{
    ROS_WARN("pomdpfile global path not found, using default %s", pomdpfile.c_str());
  }

  srand( time(NULL) ); //we do this at policy too...  
  Policy::getDimensions(pomdpfile,&numstates,&numactions,&numobservations, 1); 
  myPolicy = new Policy(pomdpfile,numactions,numstates,numobservations);

  //init class attributes if necessary
  this->execution_flag = false;

  //string for port names

  // [init publishers]
  this->focused_obj_label_publisher = this->node_handle_.advertise<std_msgs::Int32>("/planner/focused_obj_label", 5);
  
  // [init subscribers]
  
  // [init services]
  this->execution_flow_control_server = this->node_handle_.advertiseService("/planner/execution_flow_control", &Pomdp::execution_flow_controlCallback, this);
  
  // [init clients]
  obs_client = this->node_handle_.serviceClient<iri_wam_common_msgs::obs_request>("/planner/obs_client");
  wam_action_client = this->node_handle_.serviceClient<iri_wam_common_msgs::wamaction>("/planner/wam_action");

  // [init action servers]
  
  // [init action clients]

}

void Pomdp::mainLoop(void){

    int action, wam_action, obs, state;
    int zone, hand;
    if(execution_flag){ 
        state = myPolicy->mostProbableState();
        ROS_INFO("Most probable state %s with probability %f", myPolicy->domain.state2string(state).c_str(), myPolicy->getStateProbability(state) );
        if(!myPolicy->domain.isFinal(state) || myPolicy->getStateProbability(state) < 0.45){ 
            myPolicy->logState(LOGFILE);
            //request action
            action = myPolicy->getBestAction();
            ROS_INFO("Best action selected %s", myPolicy->domain.action2string(action).c_str());
            pomdp_action2wam_action(action, wam_action, zone, hand);

            wam_actions_client_srv.request.action = wam_action;
            wam_actions_client_srv.request.zone = zone;
            wam_actions_client_srv.request.hand = hand;
            if(wam_action_client.call(wam_actions_client_srv)) 
            { 
              ROS_INFO("Service result: %d", wam_actions_client_srv.response.success); 
            } else { 
              ROS_ERROR("Failed to call service wam actions service"); 
              return;
            } 
    
            //request observation
            if(obs_client.call(obs_request_srv)) 
            { 
              ROS_INFO("Service result: %d objects, %d on A, %d on B", obs_request_srv.response.num_objects,obs_request_srv.response.num_objectsA,obs_request_srv.response.num_objectsB); 
            } else { 
              ROS_ERROR("Failed to call service observation client"); 
              return;
            }
            obs = myPolicy->domain.get_observation(obs_request_srv.response.num_objectsA, obs_request_srv.response.num_objectsB, MAXNUMB);
              ROS_INFO("Pomdp equivalent Observation is %s (%d)", myPolicy->domain.obs2string(obs).c_str(),obs); 
        
            //process observation
            //update state
            myPolicy->transformState(action,obs);
            ROS_INFO("State transformed");
    
            this->ObjLabel_msg.data = 1;
            //publish focused object
            focused_obj_label_publisher.publish(this->ObjLabel_msg);
         }else{
             //sleep
             ROS_INFO("Final state reached with probability %f", myPolicy->getStateProbability(myPolicy->mostProbableState()));
             myPolicy->logState(LOGFILE);
             sleep(5); 
         } 
    }else{
        //sleep
        ROS_INFO("Awaiting trigger");
        sleep(5); 
    } 
}

bool Pomdp::execution_flow_controlCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){

    //stop execution when called
    this->execution_flag = !this->execution_flag;
    return true; 
}

void Pomdp::pomdp_action2wam_action(int pomdp_action, int& wam_action, int& zone, int& hand){
    wam_action = pomdp_action;
    if(wam_action < 5)
        zone = AZONEMSG;
    else
        zone = BZONEMSG;
    wam_action = pomdp_action%5;

    switch(wam_action){
        case 0:
            wam_action = TAKEHIGH;
            hand = STRAIGHT;
            break;
        case 1:
            wam_action = TAKELOW;
            hand = STRAIGHT;
            break;
        case 2:
            wam_action = TAKEHIGH;
            hand = ISOMETRIC;
            break;
        case 3:
            wam_action = TAKELOW;
            hand = ISOMETRIC;
            break;
        case 4:
            wam_action = MOVEAWAY;
            hand = STRAIGHT;
            break;
        default:
            ROS_ERROR("Unrecognized pomdp_action2wam_action equivalent");
            break;
    }

}

/* main function */
int main(int argc,char *argv[])
{
    ros::init(argc, argv, "pomdp");
    Pomdp pomdp;
    ros::Rate loop_rate(10); 
    while(ros::ok()){
      pomdp.mainLoop();
      ros::spinOnce();
      loop_rate.sleep(); 
    }
}
