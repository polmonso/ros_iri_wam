#include "mofa_bridge.h"

MofaBridge::MofaBridge() {

  //init class attributes if necessary
  idx = 0;
  this->nc=0;
  this->nr=0;

  //init socket attributes
  this->serverip = "127.0.0.1";
  this->port = 4321;
  //create Server Socket
  this->csocket = new CSocketClient("Client socket");
  //create Event Socket
  this->eventserver = CEventServer::instance(); 

  //string for port names
  std::string pcl2_sub_name                    = "/mofa_bridge/input/pcl2/raw";
  std::string labeled_pcl2_sub_name            = "/mofa_bridge/output/pcl2/segmented";
  std::string request_observation_sub_name     = "/mofa_bridge/srv/request_observation";

  // [init subscribers]
  this->pcl2_sub = this->nh.subscribe(pcl2_sub_name, 1, &MofaBridge::pcl2_sub_callback, this);
  
  // [init publishers]
  this->labeled_pcl2_publisher = this->nh.advertise< pcl::PointCloud<pcl::PointXYZRGB> > (labeled_pcl2_sub_name, 5);

  // [init services]
  this->obs_request = this->nh.advertiseService(request_observation_sub_name, &MofaBridge::obs_request_callback, this);

  // [init clients]
  
  // [init action servers]
  
  // [init action clients]
  ROS_INFO("Starting mofaBridge Node");
  ROS_INFO("Use [ rosservice call /request_observation ] to store current range image and compute observation");
}

void MofaBridge::open(){
    TSocket_info config;
    config.address = this->serverip.c_str();
    config.port = this->port;
    ROS_INFO("[MoFA] Open\n");
#ifdef DEBUGGING
                  reloadRGB();
#endif
    
    //todo catch exception and reconnect after delay
    this->csocket->open(&config);
    this->csocket->config();
    this->connected = true;
    
    events.push_back(csocket->get_error_event_id());
    ROS_INFO("[MoFA] Error event id: %s\n", events.back().c_str());
                
    events.push_back(csocket->get_rx_event_id());
    ROS_INFO("[MoFA] Rx event id: %s\n", events.back().c_str());
                
}

/*  [subscriber callbacks] */
void MofaBridge::pcl2_sub_callback(const sensor_msgs::PointCloud2ConstPtr& msg) 
{ 
  pcl::fromROSMsg(*msg, this->pcl_xyzrgb);

  this->nc=msg->width;
  this->nr=msg->height;

}

/*  [service callback] */
bool MofaBridge::obs_request_callback(iri_wam_common_msgs::obs_request::Request &req, iri_wam_common_msgs::obs_request::Response &res){

  // [xyzRGB matlab file name]
  std::stringstream matlab_xyzrgb_fn;
  matlab_xyzrgb_fn << "last_image.dat";

  // [open xyzRGB matlab file]
  matlab_xyzrgb.open(matlab_xyzrgb_fn.str().c_str(), std::ios::out);

  // [store xyzRGB matlab file]
  pcl::PointCloud<pcl::PointXYZRGB>::iterator pt_iter = this->pcl_xyzrgb.begin();
  for(int rr = 0; rr < (int)pcl_xyzrgb.height; ++rr) {
    for(int cc = 0; cc < (int)pcl_xyzrgb.width; ++cc, ++pt_iter) {
        if(rr%2 == 0){ //burde downsampling
            if(cc%2 == 0){ //burde downsampling
               pcl::PointXYZRGB &pt = *pt_iter;
               RGBValue color;
               color.float_value = pt.rgb;
               matlab_xyzrgb << rr << " " << cc << " " << pt.x*1000 << " " << pt.y*1000 << " " << pt.z*1000 << " " << (int)color.Red << " " << (int)color.Green << " " << (int)color.Blue << std::endl;
            }
        }
    }
  }

  // [close range image file]
  matlab_xyzrgb.close();
  ROS_INFO("[MoFA] Saved XYZRGB image %s", matlab_xyzrgb_fn.str().c_str());

  int numbytes = 0;
  bool dataread = false;
  int numdata = 0;
  if(connected){

      int request = htonl(1);
      ROS_INFO("writing request");
      this->csocket->write((unsigned char*)&request,sizeof(int)); 

      ROS_INFO("[MoFA] Waiting event\n");
      try{
        //TODO this way of ensuring we're reading 4 bytes needs polishing
        while(!dataread && this->connected == true){
          int eventpos = this->eventserver->wait_first(events,4500000);
          switch(eventpos){
              case ERROR_POS: //disconnection
                  ROS_INFO("[MoFA] Socket error. Assuming disconnected. \n");
                  this->connected = false;
                  break;
              case RX_ID_POS: //rx_id
                  numdata = this->csocket->get_num_data();
                  ROS_INFO("[MoFA] New data on socket (%d Bytes)\n",numdata); 
                  if(numdata < sizeof(int)) //ensure all data is there
                    break;
                  numbytes = this->csocket->read((unsigned char *) &num_objects, sizeof(int)); 
                  dataread = true;
                  ROS_INFO("%d bytes read", numbytes);
//                  if(numbytes < sizeof(int))
//                    break;
                  num_objects = ntohl(num_objects);
                  if(this->num_objects < 0){
                      ROS_ERROR("MOFA reported an error %d",num_objects);
                  }else{
                      ROS_INFO("MOFA reported %d objects through the socket",num_objects);
                  }

                  sleep(1); //ensure the file is written?
                  reloadRGB();
                  res.num_objects = this->numobjset.size() - 1 ; 
                  res.num_objectsA = this->numobjAset.size() - 1 ; 
                  res.num_objectsB = this->numobjBset.size() - 1 ; 
                  ROS_INFO("Sending %d objects on A, %d on B of a total of %d",res.num_objectsA,res.num_objectsB,res.num_objects);
                  this->labeled_pcl2_publisher.publish(this->pcl_xyzrgb);
                  break;
              default:
                  ROS_ERROR("Unexpected event triggered by eventserver");
                  break;
          }
        }
      }catch(CEventTimeoutException &e){
          ROS_ERROR("Time out exception");
      }
  }else{
    ROS_ERROR("Mofa not connected"); 
  }

  return true;
}

void MofaBridge::reloadRGB(){
  //  int x, y, z, u, v;
    int red, green, blue;
    int label;
    RGBValue color;
    // [xyzRGB matlab file name]
    std::stringstream matlab_xyzrgb_fn;
    matlab_xyzrgb_fn << "last_image_labelled.dat";

  this->numobjset.clear();
  this->numobjAset.clear();
  this->numobjBset.clear();

  // [open xyzRGB matlab file]
  matlab_xyzrgb_in.open(matlab_xyzrgb_fn.str().c_str(), std::ios::in);

  // [store xyzRGB matlab file]
  pcl::PointCloud<pcl::PointXYZRGB>::iterator pt_iter = this->pcl_xyzrgb.begin();
  ROS_DEBUG("pcl height & width %d %d",(int)pcl_xyzrgb.height,(int)pcl_xyzrgb.width);
  for (int rr = 0; rr < (int)pcl_xyzrgb.height; ++rr) {
        for (int cc = 0; cc < (int)pcl_xyzrgb.width; ++cc, ++pt_iter) {
          pcl::PointXYZRGB &pt = *pt_iter;
            if(rr%2 == 0 && cc%2 == 0){
          
        //      matlab_xyzrgb_in >> u;
        //      matlab_xyzrgb_in >> v;
        //
        //      matlab_xyzrgb_in >> x;
        //      matlab_xyzrgb_in >> y;
        //      matlab_xyzrgb_in >> z;
        
              matlab_xyzrgb_in >> label;
        
              matlab_xyzrgb_in >> red;
              matlab_xyzrgb_in >> green;
              matlab_xyzrgb_in >> blue;
        
        //       if(label == 0){
        //         ROS_INFO("label %d RGB %d %d %d", label, red, green, blue);
        //       }
        
              //TODO more efficient ways of doing this:
              this->numobjset.insert(label); 
        
              if(cc < (int)(pcl_xyzrgb.width/2)){ 
                  this->numobjAset.insert(label); 
              }else{
                  this->numobjBset.insert(label); 
              }
        
              color.Red = red;
              color.Green = green;
              color.Blue = blue;
    
              pt.rgb = color.float_value;
           }else{
              //burde upsampling
              pt.rgb = color.float_value;
           }
        }
  }

  ROS_INFO("Without the background, %d objects on A, %d on B of a total of %d",this->numobjAset.size()-1,this->numobjBset.size()-1,this->numobjset.size()-1);
  ROS_INFO("last label was %d",label);

  std::ostream_iterator< double > output( std::cout, " " ); 
  std::cout << "A contains: ";
  std::copy( this->numobjAset.begin(), this->numobjAset.end(), output );
  std::cout << std::endl << "B contains: ";
  std::copy( this->numobjBset.begin(), this->numobjBset.end(), output );
  std::cout << std::endl << "total contains: ";
  std::copy( this->numobjset.begin(), this->numobjset.end(), output );

  // [close range image file]
  matlab_xyzrgb_in.close();
  ROS_INFO("Loaded and processed segmented pcl image %s",matlab_xyzrgb_fn.str().c_str());

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mofa_bridge");
  MofaBridge mofa_bridge;
  mofa_bridge.open();
  ros::spin();
  return 0;
}
