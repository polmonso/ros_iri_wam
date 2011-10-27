#include "pomdp_modeler.h"

PomdpModeler::PomdpModeler(){

  //init class attributes if necessary
  this->experiments_per_action = 20;

  this->modelactions = 2*2;
  //this->modelactions = 2*3*5;
  if (!this->node_handle_.getParam("/pomdp_planner/numactions", numactions)){
    //this->numactions = 3*2*6; // hand (3) / zone (2) / actions (6)
    this->numactions = 2*2*3; // hand (2) / zone (2) / actions (3)
    //this->node_handle_.setParam("/pompd_planner/numstates",numactions);
  }
  if (!this->node_handle_.getParam("/pomdp_planner/numstates", numstates)){
    this->numstates = 5*5*2; // pilea (5) / pileb (5) / wrinkle (2)
    //this->node_handle_.setParam("/pompd_planner/numstates",numstates);
  }
  if (!this->node_handle_.getParam("/pomdp_planner/numobservations", numobservations)){
    this->numobservations = 5*5; // pilea (5) / pileb (5)
    //this->node_handle_.setParam("/pompd_planner/numobservations",numobservations);
  }

  std::string pomdpfile("data/pomdpfile.txt");
  if(node_handle_.getParam("/pomdp_planner/pomdpfile", pomdpfile)) {
    ROS_INFO("pomdpfile global path found, using %s", pomdpfile.c_str());
  }else{
    ROS_WARN("pomdpfile global path not found, using default %s", pomdpfile.c_str());
  }

    ROS_INFO("Creating modeler with %d states %d actions %d observations", this->numactions, this->numstates, this->numobservations);
    this->modeler = new Modeler(pomdpfile,this->numactions,this->numstates,this->numobservations);

  //init class attributes if necessary
  this->execution_flag = false;

  //string for port names

  // [init publishers]
  this->focused_obj_label_publisher = this->node_handle_.advertise<std_msgs::Int32>("/planner/focused_obj_label", 5);
  
  // [init subscribers]
  
  // [init services]
  this->execution_flow_control_server = this->node_handle_.advertiseService("/planner/execution_flow_control", &PomdpModeler::execution_flow_controlCallback, this);
  
  // [init clients]
  obs_client = this->node_handle_.serviceClient<iri_wam_common_msgs::obs_request>("/planner/obs_client");
  save_pcl_client = this->node_handle_.serviceClient<iri_wam_common_msgs::wamaction>("/planner/save_pcl");
  wam_action_client = this->node_handle_.serviceClient<iri_wam_common_msgs::wamaction>("/planner/wam_action");

  // [init action servers]
  
  // [init action clients]

    ROS_INFO("Done. Awaiting state family trigger.");
}

void PomdpModeler::mainLoop(void){

    if(execution_flag){

        //request action
        int action = this->counter/this->experiments_per_action;
        int hand = action/2;
        action = action%2;
        ROS_INFO("Requesting action %d with hand %d", action, hand);
        wam_actions_client_srv.request.action = action;
        wam_actions_client_srv.request.zone = 1;
        wam_actions_client_srv.request.hand = hand;
        if(wam_action_client.call(wam_actions_client_srv)) 
        { 
          ROS_INFO("Service result: %d", wam_actions_client_srv.response.success); 
        } else { 
          ROS_ERROR("Failed to call service wam actions service"); 
          return;
        } 
        //request observation
        ROS_INFO("Requesting observation");
#ifdef REAL_OBS
        if(obs_client.call(obs_request_srv)) 
        { 
          ROS_INFO("Service result: %d", obs_request_srv.response.num_objects); 
        } else { 
          ROS_ERROR("Failed to call service observation client"); 
          return;
        }
        //publish current model progress
        //TODO state_final when on a familiy state has many equivalent candidates
        ROS_INFO("Adding Observation %d %d", obs_request_srv.response.num_objectsA, obs_request_srv.response.num_objectsB);
        //this->modeler->addObservationExperienced(action, state_final, obs_request_srv.response.num_objectsA, obs_request_srv.response.num_objectsB);
        this->obs_base_matrix[this->real_num_objects][obs_request_srv.response.num_objectsA]++; 
#else
        if(save_pcl_client.call(wam_actions_client_srv)) 
        { 
          ROS_INFO("Service result: %d", obs_request_srv.response.num_objects); 
        } else { 
          ROS_ERROR("Failed to call service observation client at action %d base state %d and counter %d",action, this->real_num_objects, this->counter); 
          return;
        }
#endif
        this->progress_msg.data = 1;
        ROS_INFO("Action %d %d %% completed [Total: %d %%]", action, (this->counter%this->experiments_per_action)*100/this->experiments_per_action, this->counter*100/(this->experiments_per_action*this->numactions));
        focused_obj_label_publisher.publish(this->progress_msg);
        this->counter++;
        if(this->counter == this->experiments_per_action*this->modelactions){
            ROS_INFO("All experiments completed. Awaiting next state family.");
            this->execution_flag = false;
        }
    }
}

//ad-hoc
//void PomdpModeler::copyStateFamilies(){
//    int numa, numb;
//    int unique_acts = (this->numactions-1)/2;
//    int equi_state;
//    for(int state_final=5;state_final<numstates;state_final++){
//        for(int act=0;act<unique_acts;act++){
//            for(int obs=0;obs<numobservations;obs++){
//               equi_state = modeler->get_equivalent_base_state(state_final);
//               modeler->setObservationProbability(act,state_final,obs,modeler->getObservationProbability(act,equi_state,obs)); 
//               //copy to equivalent actions, we need to swap observations!
//               modeler->get_num_obj(state_final,numa,numb,NUMMAX);
//               modeler->setObservationProbability(act+unique_acts,state_final,obs,obs_base_matrix[numa][numb]); 
//            }
//        }
//    }
//}

bool PomdpModeler::execution_flow_controlCallback(iri_wam_common_msgs::modeling::Request &req, iri_wam_common_msgs::modeling::Response &res){

    //TODO model matrix backup
    //reset auxiliary values:
    this->counter = 0;
    this->real_num_objects = req.stateFamily;
    //resume execution
    this->execution_flag = true;
    return true; 
}

void PomdpModeler::printObsBaseMatrix(){
    std::stringstream strs(std::stringstream::in);
    for(int i=0; i<5; i++){
        for(int j=0; j<5; j++){
            strs << obs_base_matrix[i][j] << " ";
        }
        strs << std::endl;
    }
    ROS_INFO("Matrix\n %s",strs.str().c_str()); 
}

/* main function */
int main(int argc,char *argv[])
{
    ros::init(argc, argv, "pomdp_modeler");
    PomdpModeler pomdp_modeler;
    ros::Rate loop_rate(10); 
    while(ros::ok()){
      pomdp_modeler.mainLoop();
      ros::spinOnce();
      loop_rate.sleep(); 
    }
}
